#!/usr/bin/env python3
import csv
import math
import os
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
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

    Drive and steering keys only work after the Arduino startup calibration
    reports READY. A/D can explicitly arm stationary steering with drive PWM 0.
    Motion requests carry the serial bridge's current challenge;
    ARM display/capture follow the bridge's accepted drive_state feedback.
    """

    WINDOW = 'manual_drive_capture'

    def __init__(self):
        super().__init__('manual_drive_capture_node')

        self.declare_parameter('image_topic', 'image_raw')
        self.declare_parameter('use_rear_camera', False)
        self.declare_parameter('rear_image_topic', 'manual/rear/image_raw')
        self.declare_parameter('auto_capture', False)
        self.declare_parameter('max_pair_skew', 0.10)
        self.declare_parameter('max_frame_age', 0.50)
        self.declare_parameter('cmd_topic', 'topic_control_signal')
        self.declare_parameter('arm_topic', 'vehicle/armed')
        self.declare_parameter('ready_topic', 'vehicle/calibration_ready')
        self.declare_parameter('output_dir', '~/autodrive_dataset/manual_drive')
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
        self.use_rear_camera = bool(self.get_parameter('use_rear_camera').value)
        self.rear_image_topic = str(self.get_parameter('rear_image_topic').value)
        self.max_pair_skew = float(self.get_parameter('max_pair_skew').value)
        self.max_frame_age = float(self.get_parameter('max_frame_age').value)
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
        for name, value in (('auto_capture_interval', self.auto_interval),
                            ('max_pair_skew', self.max_pair_skew),
                            ('max_frame_age', self.max_frame_age)):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')

        root = Path(os.path.expanduser(str(self.get_parameter('output_dir').value))).resolve()
        self.images_dir = root / 'images'
        self.images_dir.mkdir(parents=True, exist_ok=True)
        if self.use_rear_camera:
            (self.images_dir / 'front').mkdir(exist_ok=True)
            (self.images_dir / 'rear').mkdir(exist_ok=True)
        # Keep the existing single-camera CSV format available unchanged.
        self.csv_path = root / ('pairs.csv' if self.use_rear_camera else 'labels.csv')
        new_csv = not self.csv_path.exists() or self.csv_path.stat().st_size == 0
        self.csv_file = self.csv_path.open('a', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        if new_csv:
            header = [
                'timestamp', 'filename', 'steering', 'left_speed', 'right_speed',
                'capture_mode', 'last_key'
            ]
            if self.use_rear_camera:
                header = ['timestamp', 'front_filename', 'rear_filename', 'steering',
                          'left_speed', 'right_speed', 'capture_mode', 'last_key',
                          'front_stamp_ns', 'rear_stamp_ns', 'pair_delta_ms']
            self.csv_writer.writerow(header)
            self.csv_file.flush()

        self.frame_index = len(list(self.images_dir.glob('front/*.jpg' if self.use_rear_camera else '*.jpg')))
        self.latest_frame = None
        self.latest_rear_frame = None
        self.frame_received_at = {}
        self.frame_stamps = {}
        self.frame_sequence = {'front': 0, 'rear': 0}
        self.saved_sequence = {'front': 0, 'rear': 0}
        self.bridge = CvBridge()
        self.auto_capture = bool(self.get_parameter('auto_capture').value)
        self.last_capture_time = 0.0
        self.last_key = 'x'
        self.should_quit = False
        self.calibration_ready = False
        self.armed = False
        self.arm_challenge = ''
        self._last_ui_heartbeat = 0.0

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
        if self.use_rear_camera:
            self.rear_sub = self.create_subscription(Image, self.rear_image_topic, self.on_rear_image, image_qos)
        self.ready_sub = self.create_subscription(Bool, self.ready_topic, self.on_ready, latched_qos)
        self.pub = self.create_publisher(MotionCommand, self.cmd_topic, cmd_qos)
        self.arm_pub = self.create_publisher(Bool, self.arm_topic, latched_qos)
        if not self.dry_run:
            self.request_pub = self.create_publisher(String, 'vehicle/arm_request', cmd_qos)
            self.heartbeat_pub = self.create_publisher(String, 'vehicle/ui_heartbeat', cmd_qos)
            self.create_subscription(String, 'vehicle/arm_challenge', self.on_arm_challenge, latched_qos)
            self.create_subscription(Bool, 'vehicle/drive_state', self.on_drive_state, latched_qos)
        self.timer = self.create_timer(0.05, self.on_timer)

        if self.show_window:
            cv2.namedWindow(self.WINDOW, cv2.WINDOW_NORMAL)

        self.publish_arm(False)

        print('\n=== INCREMENTAL MANUAL DRIVE + DATA CAPTURE ===')
        print('Focus the camera window before pressing keys.')
        print(f'W: speed +{self.speed_step} | S: speed -{self.speed_step}')
        print(f'A/D: steering ±{self.steering_step} step | range ±{self.max_steering}')
        print('X/SPACE: stop+disarm | C: capture | R: auto capture | Q: quit')
        print('DRY RUN: no Arduino bridge' if self.dry_run else
              'Vehicle is LOCKED until startup steering calibration becomes READY.')
        print(f'dataset: {root}')
        print(f'cameras: {"FRONT + REAR (paired)" if self.use_rear_camera else "FRONT"}, '
              f'AUTO_CAPTURE={self.auto_capture}, interval={self.auto_interval:.2f}s')
        print('=================================================\n', flush=True)

    def on_image(self, msg: Image):
        self._receive_image(msg, 'front')

    def on_rear_image(self, msg: Image):
        self._receive_image(msg, 'rear')

    def _receive_image(self, msg: Image, role: str):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'{role} image conversion failed: {exc}')
            return
        if role == 'front':
            self.latest_frame = frame
        else:
            self.latest_rear_frame = frame
        self.frame_received_at[role] = time.monotonic()
        self.frame_stamps[role] = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.frame_sequence[role] += 1

    def on_ready(self, msg: Bool):
        was_ready = self.calibration_ready
        self.calibration_ready = bool(msg.data)
        if self.calibration_ready and not was_ready:
            self.get_logger().info('Steering calibration READY. W/S: drive; A/D: steer, including at rest.')
        if not self.calibration_ready:
            self.publish_arm(False)
            self.speed_cmd = 0
            self.steering_cmd = 0
            self.publish_current_cmd('calibration_lock')

    def publish_arm(self, value: bool):
        if not value:
            self.armed = False
        msg = Bool()
        msg.data = bool(value)
        self.arm_pub.publish(msg)

    def _clear_motion(self, reason):
        self.speed_cmd = 0
        self.steering_cmd = 0
        self.publish_current_cmd(reason)

    def on_arm_challenge(self, msg: String):
        if self.dry_run:
            return
        if self.arm_challenge != msg.data:
            self.arm_challenge = msg.data
            self.armed = False
            # A bridge stop/restart invalidates earlier manual commands. There
            # is no automatic retry that could turn a pre-stop key into a new W.
            self._clear_motion('bridge_disarm')

    def on_drive_state(self, msg: Bool):
        if self.dry_run:
            return
        was_armed = self.armed
        self.armed = bool(msg.data)
        if was_armed and not self.armed:
            self._clear_motion('bridge_disarm')

    def publish_ui_heartbeat(self, force=False):
        if self.dry_run or self.should_quit or not self.arm_challenge:
            return
        now = time.monotonic()
        if force or now - self._last_ui_heartbeat >= 0.1:
            self.heartbeat_pub.publish(String(data=self.arm_challenge))
            self._last_ui_heartbeat = now

    def request_manual_motion(self):
        if self.dry_run:
            self.armed = True
        elif not self.armed:
            self.publish_ui_heartbeat(force=True)
            self.request_pub.publish(String(data=self.arm_challenge))

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
        if not self.arm_challenge:
            self.get_logger().warn('Drive key ignored: waiting for the serial bridge start token')
            return False
        return True

    def handle_key(self, key: str):
        key = key.lower()
        if key == 'w':
            if not self._arm_for_manual_motion():
                return
            self.speed_cmd = min(self.max_speed, self.speed_cmd + self.speed_step)
            self.publish_current_cmd(key)
            self.request_manual_motion()

        elif key == 's':
            if not self._arm_for_manual_motion():
                return
            self.speed_cmd = max(-self.max_speed, self.speed_cmd - self.speed_step)
            self.publish_current_cmd(key)
            self.request_manual_motion()

        elif key in ('a', 'd'):
            if not self._arm_for_manual_motion():
                return
            # A fresh steering key authorizes steering even before W/S. When
            # disarmed, discard any unaccepted speed request before arming.
            if not self.armed:
                self.speed_cmd = 0
            direction = self.left_sign if key == 'a' else -self.left_sign
            self.steering_cmd += direction * self.steering_step
            self.steering_cmd = max(-self.max_steering, min(self.max_steering, self.steering_cmd))
            self.publish_current_cmd(key)
            self.request_manual_motion()

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
        if self.use_rear_camera:
            return self.capture_pair(mode)
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

    def capture_pair(self, mode: str):
        now = time.monotonic()
        if any(now - self.frame_received_at.get(role, -math.inf) > self.max_frame_age
               for role in ('front', 'rear')):
            self.get_logger().warn('Capture skipped: waiting for fresh FRONT and REAR frames',
                                   throttle_duration_sec=2.0)
            return False
        delta_ms = abs(self.frame_stamps['front'] - self.frame_stamps['rear']) / 1_000_000
        if delta_ms > self.max_pair_skew * 1000:
            self.get_logger().warn(f'Capture skipped: front/rear time difference {delta_ms:.1f} ms',
                                   throttle_duration_sec=2.0)
            return False
        if mode == 'auto' and any(self.frame_sequence[role] == self.saved_sequence[role]
                                  for role in ('front', 'rear')):
            return False

        ts_ns = time.time_ns()
        pair_id = f'{self.frame_index + 1:08d}_{ts_ns}'
        names = [f'front/{pair_id}_front.jpg', f'rear/{pair_id}_rear.jpg']
        paths = [self.images_dir / name for name in names]
        # Snapshot commands before disk IO; preview overlays never touch originals.
        row = [f'{ts_ns / 1_000_000_000:.6f}', *names, self.cmd.steering,
               self.cmd.left_speed, self.cmd.right_speed, mode, self.last_key,
               self.frame_stamps['front'], self.frame_stamps['rear'], f'{delta_ms:.3f}']
        try:
            for path, frame in zip(paths, (self.latest_frame, self.latest_rear_frame)):
                if not cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]):
                    raise OSError(f'Could not save {path}')
        except (OSError, cv2.error) as exc:
            for path in paths:
                path.unlink(missing_ok=True)
            self.get_logger().error(f'Pair capture failed: {exc}')
            return False
        self.csv_writer.writerow(row)
        self.csv_file.flush()
        self.frame_index += 1
        self.saved_sequence = self.frame_sequence.copy()
        print(f'[capture:{mode}] pair={pair_id} delta={delta_ms:.1f}ms '
              f'steer={self.cmd.steering:+d} speed={self.cmd.left_speed:+d}', flush=True)
        return True

    def draw_window(self):
        if not self.show_window:
            return

        panels = []
        roles = ('front', 'rear') if self.use_rear_camera else ('front',)
        for role in roles:
            frame = self.latest_frame if role == 'front' else self.latest_rear_frame
            panel = (cv2.resize(frame, (640, 480)) if frame is not None
                     else np.zeros((480, 640, 3), dtype=np.uint8))
            fresh = time.monotonic() - self.frame_received_at.get(role, -math.inf) <= self.max_frame_age
            label = role.upper() + ('' if fresh else ' - WAIT / STALE')
            cv2.putText(panel, label, (10, 105), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 0) if fresh else (0, 0, 255), 2)
            panels.append(panel)
        img = cv2.hconcat(panels)
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
        if self.show_window:
            try:
                closed = cv2.getWindowProperty(self.WINDOW, cv2.WND_PROP_VISIBLE) < 1
            except cv2.error:
                closed = True
            if closed:
                self.handle_key('q')
                return
        self.draw_window()
        if self.should_quit:
            return
        self.publish_ui_heartbeat()

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
