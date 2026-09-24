#!/usr/bin/env python3
import subprocess
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraPublisherNode(Node):
    """Ubuntu/V4L2 camera -> sensor_msgs/Image publisher."""

    def __init__(self):
        super().__init__('camera_publisher_node')

        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('fourcc', 'YUYV')
        self.declare_parameter('buffer_size', 1)
        self.declare_parameter('reopen_after_failures', 2)
        self.declare_parameter('reopen_delay_sec', 0.20)
        self.declare_parameter('disable_dynamic_framerate', True)
        self.declare_parameter('reliability', 'reliable')
        self.declare_parameter('topic', 'image_raw')
        self.declare_parameter('frame_id', 'camera_frame')
        self.declare_parameter('show', False)

        self.device = self.get_parameter('device').value
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.fourcc = str(self.get_parameter('fourcc').value).upper()
        self.buffer_size = max(1, int(self.get_parameter('buffer_size').value))
        self.reopen_after_failures = max(
            1, int(self.get_parameter('reopen_after_failures').value)
        )
        self.reopen_delay_sec = max(
            0.0, float(self.get_parameter('reopen_delay_sec').value)
        )
        self.disable_dynamic_framerate = bool(
            self.get_parameter('disable_dynamic_framerate').value
        )
        self.reliability = str(self.get_parameter('reliability').value).lower()
        if self.reliability not in ('best_effort', 'reliable'):
            raise ValueError(
                f'reliability must be best_effort or reliable: {self.reliability!r}'
            )
        self.topic = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.show = bool(self.get_parameter('show').value)

        self.cap = None
        self.consecutive_failures = 0
        self.next_reopen_time = 0.0
        self.last_frame_time = None
        if not self.open_camera():
            raise RuntimeError(f'Cannot open camera: {self.device}')

        qos = QoSProfile(
            depth=1,
            reliability=(
                ReliabilityPolicy.BEST_EFFORT
                if self.reliability == 'best_effort'
                else ReliabilityPolicy.RELIABLE
            ),
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(Image, self.topic, qos)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / max(self.fps, 1.0), self.on_timer)
        self.frame_count = 0
        self.get_logger().info(
            f'Camera opened: device={self.device}, '
            f'requested={self.width}x{self.height}@{self.fps:.1f}, '
            f'topic=/{self.topic}, reliability={self.reliability}'
        )

    @staticmethod
    def fourcc_text(value):
        value = int(value)
        return ''.join(chr((value >> (8 * index)) & 0xFF) for index in range(4))

    def apply_v4l2_controls(self):
        if not self.disable_dynamic_framerate:
            return
        try:
            result = subprocess.run(
                [
                    'v4l2-ctl',
                    f'--device={self.device}',
                    '--set-ctrl=exposure_dynamic_framerate=0',
                ],
                capture_output=True,
                text=True,
                timeout=1.0,
                check=False,
            )
            if result.returncode != 0:
                detail = (result.stderr or result.stdout).strip()
                self.get_logger().warn(
                    f'Could not disable dynamic camera framerate: {detail}'
                )
        except (FileNotFoundError, subprocess.SubprocessError) as exc:
            self.get_logger().warn(
                f'Could not configure dynamic camera framerate: {exc}'
            )

    def open_camera(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

        # /dev/videoN or /dev/v4l/by-id/... can both be passed as a string.
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            self.get_logger().warn(
                f'V4L2 open failed for {self.device}; retrying with default backend'
            )
            cap = cv2.VideoCapture(self.device)
        if not cap.isOpened():
            cap.release()
            return False

        if len(self.fourcc) != 4:
            cap.release()
            raise ValueError(
                f'fourcc must contain exactly four characters: {self.fourcc!r}'
            )

        # Request an explicit pixel format before size/FPS so reconnects use the
        # same transport instead of depending on backend negotiation history.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, self.buffer_size)

        self.cap = cap
        self.apply_v4l2_controls()
        self.consecutive_failures = 0
        self.last_frame_time = None

        actual_width = int(round(cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
        actual_height = int(round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        actual_fourcc = self.fourcc_text(cap.get(cv2.CAP_PROP_FOURCC))
        self.get_logger().info(
            f'Camera negotiated: {actual_width}x{actual_height}@{actual_fps:.2f} '
            f'fourcc={actual_fourcc!r}, requested_buffer={self.buffer_size}'
        )
        if actual_fourcc != self.fourcc:
            self.get_logger().warn(
                f'Camera did not accept requested FOURCC {self.fourcc!r}; '
                f'using {actual_fourcc!r}'
            )
        return True

    def schedule_reopen(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        self.next_reopen_time = time.monotonic() + self.reopen_delay_sec

    def on_timer(self):
        if self.cap is None:
            if time.monotonic() < self.next_reopen_time:
                return
            if not self.open_camera():
                self.next_reopen_time = time.monotonic() + self.reopen_delay_sec
                return
            self.get_logger().warn('Camera reopened after capture failure')

        read_start = time.monotonic()
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.consecutive_failures += 1
            self.get_logger().warn(
                f'Camera frame read failed '
                f'({self.consecutive_failures}/{self.reopen_after_failures})'
            )
            if self.consecutive_failures >= self.reopen_after_failures:
                self.get_logger().warn('Reopening camera after consecutive read failures')
                self.schedule_reopen()
            return
        self.consecutive_failures = 0

        now = time.monotonic()
        read_ms = (now - read_start) * 1000.0
        expected_period = 1.0 / max(self.fps, 1.0)
        stall_warn_ms = max(100.0, expected_period * 3.0 * 1000.0)
        frame_gap_warn_ms = max(150.0, expected_period * 4.0 * 1000.0)
        if read_ms > stall_warn_ms:
            self.get_logger().warn(f'Camera read stalled for {read_ms:.1f} ms')
        if self.last_frame_time is not None:
            frame_gap_ms = (now - self.last_frame_time) * 1000.0
            if frame_gap_ms > frame_gap_warn_ms:
                self.get_logger().warn(f'Camera frame gap: {frame_gap_ms:.1f} ms')
        self.last_frame_time = now

        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)
        self.frame_count += 1

        if self.show:
            cv2.imshow('ROS2 Camera', frame)
            cv2.waitKey(1)

    def destroy_node(self):
        try:
            if self.cap is not None:
                self.cap.release()
            cv2.destroyAllWindows()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'[camera_publisher_node] ERROR: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
