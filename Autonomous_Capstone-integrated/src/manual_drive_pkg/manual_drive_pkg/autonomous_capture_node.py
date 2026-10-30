"""Passive raw-camera dataset capture; never publishes vehicle commands."""
import csv
import json
import math
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


class AutonomousCaptureNode(Node):
    def __init__(self):
        super().__init__('autonomous_capture_node')
        self.declare_parameter('image_topic', '/track/image_raw')
        self.declare_parameter('output_dir', '~/autodrive_dataset/autonomous_drive')
        self.declare_parameter('auto_capture_interval', 0.20)
        self.declare_parameter('jpeg_quality', 95)
        self.declare_parameter('start_enabled', False)
        self.interval = float(self.get_parameter('auto_capture_interval').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        if not math.isfinite(self.interval) or self.interval <= 0:
            raise ValueError('auto_capture_interval must be finite and positive')
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError('jpeg_quality must be between 1 and 100')
        # Each run gets its own CSV, including when the same output root is reused.
        session = time.strftime('%Y%m%d_%H%M%S') + f'_{time.time_ns()}'
        self.root = Path(str(self.get_parameter('output_dir').value)).expanduser() / session
        self.images_dir = self.root / 'images'
        self.csv_file = None
        self.csv_writer = None
        self.bridge = CvBridge()
        self.enabled = bool(self.get_parameter('start_enabled').value)
        self.armed = False
        self.command = (0, 0, 0)
        self.command_armed = False
        self.command_time = float('-inf')
        self.latest_image = None
        self.image_time = float('-inf')
        self.sequence = 0
        self.saved_sequence = -1
        self.count = 0
        self.last_capture_time = float('-inf')
        self.last_key = ''
        self.error = ''

        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # BEST_EFFORT accepts both camera QoS variants and cannot back-pressure
        # the camera/controller while this process is encoding or writing JPEGs.
        images = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, str(self.get_parameter('image_topic').value), self.on_image, images)
        self.create_subscription(Bool, 'vehicle/drive_state', self.on_drive_state, latched)
        self.create_subscription(String, 'vehicle/command_status', self.on_command_status, reliable)
        self.create_subscription(String, 'capture/key', self.on_key, reliable)
        self.status_pub = self.create_publisher(String, 'capture/status', latched)
        self.create_service(SetBool, '~/set_enabled', self.set_enabled)
        self.create_timer(0.05, self.on_timer)
        self.create_timer(1.0, self.publish_status)
        self.publish_status()
        self.get_logger().info(f'Raw capture ready (R: ON/OFF, C: single frame). Dataset: {self.root}')

    def on_image(self, msg):
        self.latest_image = msg
        self.image_time = time.monotonic()
        self.sequence += 1

    def on_drive_state(self, msg):
        self.armed = bool(msg.data)
        if not self.armed:
            self.command_armed = False

    def on_command_status(self, msg):
        try:
            status = json.loads(msg.data)
            command = tuple(int(status[k]) for k in ('steering', 'left_pwm', 'right_pwm'))
            armed = status['armed']
            if not isinstance(armed, bool):
                raise ValueError('armed must be a boolean')
        except (ValueError, TypeError, KeyError):
            return
        self.command = command
        self.command_armed = armed
        self.command_time = time.monotonic()

    def set_enabled(self, request, response):
        self.enable_capture(request.data)
        response.success = True
        response.message = f'Automatic raw capture {"ON" if self.enabled else "OFF"}'
        return response

    def enable_capture(self, enabled):
        self.enabled = bool(enabled)
        self.error = ''
        self.last_capture_time = float('-inf')
        self.get_logger().info(f'AUTO_CAPTURE={"ON" if self.enabled else "OFF"}')
        self.publish_status()

    def on_key(self, msg):
        key = msg.data.lower()
        if key == 'r':
            self.last_key = key
            self.enable_capture(not self.enabled)
        elif key == 'c':
            self.last_key = key
            self.capture_frame('manual')

    def moving(self, now):
        return (self.armed and self.command_armed and now - self.command_time <= 0.75
                and any(self.command[1:]))

    def on_timer(self):
        now = time.monotonic()
        if self.enabled and self.moving(now) and now - self.last_capture_time >= self.interval:
            self.capture_frame('auto')

    def capture_frame(self, mode):
        now = time.monotonic()
        msg = self.latest_image
        if msg is None or now - self.image_time > 0.75:
            if mode == 'manual':
                self.get_logger().warn('No fresh raw camera image available for capture')
            return False
        if mode == 'auto' and self.saved_sequence == self.sequence:
            return False
        try:
            # No overlays, crop, resize, BEV or detector output in the saved image.
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if self.csv_file is None:
                self.images_dir.mkdir(parents=True, exist_ok=True)
                self.csv_file = (self.root / 'labels.csv').open('w', newline='', encoding='utf-8')
                self.csv_writer = csv.writer(self.csv_file)
                self.csv_writer.writerow([
                    'timestamp', 'filename', 'steering', 'left_speed', 'right_speed',
                    'capture_mode', 'last_key', 'image_stamp_sec', 'image_stamp_nanosec',
                    'frame_id', 'bridge_armed', 'command_status_age_s',
                ])
            timestamp = time.time_ns()
            filename = f'{self.count + 1:08d}_{timestamp}.jpg'
            if not cv2.imwrite(str(self.images_dir / filename), frame,
                               [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]):
                raise OSError(f'JPEG write failed: {filename}')
            fresh = now - self.command_time <= 0.75
            command = self.command if self.armed and self.command_armed and fresh else (0, 0, 0)
            self.csv_writer.writerow([
                f'{timestamp / 1e9:.6f}', filename, *command, mode, self.last_key,
                msg.header.stamp.sec, msg.header.stamp.nanosec, msg.header.frame_id,
                self.armed, f'{now - self.command_time:.6f}' if fresh else '',
            ])
            self.csv_file.flush()
        except Exception as exc:
            self.enabled = False
            self.error = str(exc)
            self.get_logger().error(f'Raw capture disabled: {exc}')
            self.publish_status()
            return False
        self.count += 1
        self.saved_sequence = self.sequence
        self.last_capture_time = now
        self.publish_status()
        return True

    def publish_status(self):
        now = time.monotonic()
        self.status_pub.publish(String(data=json.dumps({
            'enabled': self.enabled, 'moving': self.moving(now),
            'image_fresh': self.latest_image is not None and now - self.image_time <= 0.75,
            'interval': self.interval, 'count': self.count,
            'output_dir': str(self.root), 'error': self.error,
        })))

    def destroy_node(self):
        try:
            if self.csv_file is not None:
                self.csv_file.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AutonomousCaptureNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
