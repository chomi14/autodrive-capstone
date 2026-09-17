#!/usr/bin/env python3
import csv
import os
import time
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from interfaces_pkg.msg import MotionCommand


class ManualDriveCaptureNode(Node):
    """Incremental WASD drive + labeled camera capture.

    Key behavior requested for the real vehicle:
      W : signed speed += speed_step
          (more forward / less reverse / then forward)
      S : signed speed -= speed_step
          (less forward / more reverse)
      A : steering moves one step toward LEFT
      D : steering moves one step toward RIGHT

    Therefore speed and steering are independent state variables.  Repeated key
    presses accumulate until the configured limits are reached.

    Other keys:
      X or SPACE : speed=0, steering=0, DISARM
      C          : save one labeled frame
      R          : toggle automatic labeled capture while moving
      Q          : DISARM and stop this node

    The first W/S after startup only works after the Arduino startup calibration
    reports READY.  Pressing W/S also publishes /vehicle/armed=True.
    """

    WINDOW = 'manual_drive_capture'

    def __init__(self):
        super().__init__('manual_drive_capture_node')

        self.declare_parameter('image_topic', 'image_raw')
        self.declare_parameter('cmd_topic', 'topic_control_signal')
        self.declare_parameter('arm_topic', 'vehicle/armed')
        self.declare_parameter('ready_topic', 'vehicle/calibration_ready')
        self.declare_parameter('output_dir', '~/ros2_ws/datasets/manual_drive')
        self.declare_parameter('speed_step', 20)
        self.declare_parameter('steering_step', 1)
        self.declare_parameter('max_speed', 255)
        self.declare_parameter('max_steering', 7)
        self.declare_parameter('left_sign', -1)
        self.declare_parameter('auto_capture_interval', 0.20)
        self.declare_parameter('jpeg_quality', 95)
        self.declare_parameter('show_window', True)
        self.declare_parameter('dry_run', False)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.arm_topic = str(self.get_parameter('arm_topic').value)
        self.ready_topic = str(self.get_parameter('ready_topic').value)
        self.speed_step = max(1, int(self.get_parameter('speed_step').value))
        self.steering_step = max(1, int(self.get_parameter('steering_step').value))
        self.max_speed = max(1, min(255, int(self.get_parameter('max_speed').value)))
        self.max_steering = max(1, min(7, int(self.get_parameter('max_steering').value)))
        self.left_sign = -1 if int(self.get_parameter('left_sign').value) < 0 else 1
        self.auto_interval = float(self.get_parameter('auto_capture_interval').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.show_window = bool(self.get_parameter('show_window').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)

        root = Path(os.path.expanduser(str(self.get_parameter('output_dir').value))).resolve()
        self.images_dir = root / 'images'
        self.images_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = root / 'labels.csv'
        new_csv = not self.csv_path.exists()
        self.csv_file = self.csv_path.open('a', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        if new_csv:
            self.csv_writer.writerow([
                'timestamp', 'filename', 'steering', 'left_speed', 'right_speed',
                'capture_mode', 'last_key'
            ])
            self.csv_file.flush()

        self.frame_index = len(list(self.images_dir.glob('*.jpg')))
        self.latest_frame = None
        self.bridge = CvBridge()
        self.auto_capture = False
        self.last_capture_time = 0.0
        self.last_key = 'x'
        self.should_quit = False
        self.calibration_ready = False
        self.armed = False

        self.speed_cmd = 0
        self.steering_cmd = 0

        self.cmd = MotionCommand()
        self.cmd.steering = 0
        self.cmd.left_speed = 0
        self.cmd.right_speed = 0

        image_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        cmd_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.sub = self.create_subscription(Image, self.image_topic, self.on_image, image_qos)
        self.ready_sub = self.create_subscription(Bool, self.ready_topic, self.on_ready, latched_qos)
        self.pub = self.create_publisher(MotionCommand, self.cmd_topic, cmd_qos)
        self.arm_pub = self.create_publisher(Bool, self.arm_topic, latched_qos)
        self.timer = self.create_timer(0.05, self.on_timer)

        if self.show_window:
            cv2.namedWindow(self.WINDOW, cv2.WINDOW_NORMAL)

        self.publish_arm(False)

        print('\n=== INCREMENTAL MANUAL DRIVE + DATA CAPTURE ===')
        print('Focus the camera window before pressing keys.')
        print(f'W: speed +{self.speed_step} | S: speed -{self.speed_step}')
        print(f'A/D: steering ±{self.steering_step} step | range ±{self.max_steering}')
        print('X/SPACE: stop+center+disarm | C: capture | R: auto capture | Q: quit')
        print('DRY RUN: no Arduino bridge' if self.dry_run else
              'Vehicle is LOCKED until startup steering calibration becomes READY.')
        print(f'dataset: {root}')
        print('=================================================\n', flush=True)

    def on_image(self, msg: Image):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'Image conversion failed: {exc}')

    def on_ready(self, msg: Bool):
        was_ready = self.calibration_ready
        self.calibration_ready = bool(msg.data)
        if self.calibration_ready and not was_ready:
            self.get_logger().info('Steering calibration READY. Press W or S to move.')
        if not self.calibration_ready:
            self.publish_arm(False)
            self.speed_cmd = 0
            self.steering_cmd = 0
            self.publish_current_cmd('calibration_lock')

    def publish_arm(self, value: bool):
        self.armed = bool(value)
        msg = Bool()
        msg.data = self.armed
        self.arm_pub.publish(msg)

    def publish_current_cmd(self, key: str):
        self.cmd.steering = int(max(-self.max_steering, min(self.max_steering, self.steering_cmd)))
        self.cmd.left_speed = int(max(-self.max_speed, min(self.max_speed, self.speed_cmd)))
        self.cmd.right_speed = int(max(-self.max_speed, min(self.max_speed, self.speed_cmd)))
        self.last_key = key
        self.pub.publish(self.cmd)
        print(
            f'cmd key={key!r}: steer={self.cmd.steering:+d}, '
            f'speed={self.speed_cmd:+d}, armed={self.armed}, ready={self.calibration_ready}',
            flush=True,
        )

    def _arm_for_manual_motion(self) -> bool:
        if self.dry_run:
            return True
        if not self.calibration_ready:
            self.get_logger().warn('Drive key ignored: steering calibration is not READY yet')
            return False
        if not self.armed:
            self.publish_arm(True)
        return True

    def handle_key(self, key: str):
        if key == 'w':
            if not self._arm_for_manual_motion():
                return
            self.speed_cmd = min(self.max_speed, self.speed_cmd + self.speed_step)
            self.publish_current_cmd(key)

        elif key == 's':
            if not self._arm_for_manual_motion():
                return
            self.speed_cmd = max(-self.max_speed, self.speed_cmd - self.speed_step)
            self.publish_current_cmd(key)

        elif key == 'a':
            if not (self.calibration_ready or self.dry_run):
                self.get_logger().warn('Steering key ignored: calibration is not READY yet')
                return
            # Move one step in the physical LEFT direction.  With left_sign=-1,
            # repeated A presses produce 0,-1,-2,...,-7.
            self.steering_cmd += self.left_sign * self.steering_step
            self.steering_cmd = max(-self.max_steering, min(self.max_steering, self.steering_cmd))
            self.publish_current_cmd(key)

        elif key == 'd':
            if not (self.calibration_ready or self.dry_run):
                self.get_logger().warn('Steering key ignored: calibration is not READY yet')
                return
            self.steering_cmd -= self.left_sign * self.steering_step
            self.steering_cmd = max(-self.max_steering, min(self.max_steering, self.steering_cmd))
            self.publish_current_cmd(key)

        elif key in ('x', ' '):
            self.speed_cmd = 0
            self.steering_cmd = 0
            self.publish_arm(False)
            self.publish_current_cmd(key)

        elif key == 'c':
            self.last_key = key
            self.capture_frame('manual')

        elif key == 'r':
            self.last_key = key
            self.auto_capture = not self.auto_capture
            print(f'[capture] AUTO={self.auto_capture}', flush=True)

        elif key == 'q':
            self.speed_cmd = 0
            self.steering_cmd = 0
            self.publish_arm(False)
            self.publish_current_cmd(key)
            self.should_quit = True

    def capture_frame(self, mode: str):
        if self.latest_frame is None:
            print('[capture] no camera frame yet', flush=True)
            return False

        self.frame_index += 1
        ts_ns = time.time_ns()
        filename = f'{self.frame_index:08d}_{ts_ns}.jpg'
        path = self.images_dir / filename
        ok = cv2.imwrite(
            str(path), self.latest_frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )
        if not ok:
            print(f'[capture] failed: {path}', flush=True)
            return False

        self.csv_writer.writerow([
            f'{time.time():.6f}', filename, self.cmd.steering,
            self.cmd.left_speed, self.cmd.right_speed, mode, self.last_key,
        ])
        self.csv_file.flush()
        print(
            f'[capture:{mode}] {filename} steer={self.cmd.steering:+d} '
            f'speed={self.cmd.left_speed:+d}',
            flush=True,
        )
        return True

    def draw_window(self):
        if not self.show_window or self.latest_frame is None:
            return

        img = self.latest_frame.copy()
        status = (
            f'steer={self.steering_cmd:+d} speed={self.speed_cmd:+d} '
            f'ARM={"ON" if self.armed else "OFF"}'
        )
        calibration = 'CAL=READY' if self.calibration_ready else 'CAL=WAIT/LOCKED'
        auto = f'AUTO_CAPTURE={"ON" if self.auto_capture else "OFF"} {self.auto_interval:.2f}s'

        cv2.putText(img, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.62, (0, 255, 255), 2)
        cv2.putText(img, calibration, (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 255, 0) if self.calibration_ready else (0, 0, 255), 2)
        cv2.putText(img, auto, (10, 75), cv2.FONT_HERSHEY_SIMPLEX,
                    0.52, (0, 255, 0) if self.auto_capture else (160, 160, 160), 1)
        cv2.putText(img, 'W/S speed +/- | A/D steering | X stop | C/R capture',
                    (10, img.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX,
                    0.46, (255, 255, 255), 1)

        cv2.imshow(self.WINDOW, img)
        keycode = cv2.waitKeyEx(1)
        if keycode >= 0:
            self.handle_key(chr(keycode & 0xFF).lower())

    def on_timer(self):
        self.pub.publish(self.cmd)
        self.draw_window()

        moving = (self.armed or self.dry_run) and (self.cmd.left_speed != 0 or self.cmd.right_speed != 0)
        now = time.monotonic()
        if self.auto_capture and moving and (now - self.last_capture_time >= self.auto_interval):
            if self.capture_frame('auto'):
                self.last_capture_time = now

    def destroy_node(self):
        try:
            self.speed_cmd = 0
            self.steering_cmd = 0
            self.publish_arm(False)
            self.publish_current_cmd('shutdown')
        except Exception:
            pass
        try:
            self.csv_file.flush()
            self.csv_file.close()
        except Exception:
            pass
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ManualDriveCaptureNode()
    try:
        while rclpy.ok() and not node.should_quit:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
