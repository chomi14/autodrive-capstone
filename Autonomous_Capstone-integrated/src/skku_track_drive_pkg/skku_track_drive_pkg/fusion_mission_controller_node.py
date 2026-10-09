"""Opt-in camera + LiDAR mission; original mission executable is unchanged."""
import json
import time
from collections import deque

import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from . import lidar_camera_fusion as cfg
from .fusion_mission_core import FusionCore
from .mission_controller_node import MissionControllerNode
from .controller_liveness import run_controller

# 모든 추가 조정값은 lidar_camera_fusion.py 상단 SETTINGS에 모았습니다.
SCAN_BUFFER_SIZE = 30               # 영상 timestamp에 가장 가까운 scan 보관




class FusionMissionControllerNode(MissionControllerNode):
    def __init__(self):
        super().__init__()
        # TrackController creates cmd_pub/liveness AFTER initialize_mode().
        self.create_subscription(LaserScan, cfg.SCAN_TOPIC, self.on_scan,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                                 callback_group=self.liveness.group)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, cfg.CALIBRATION_TOPIC, self.on_calibration, qos,
                                 callback_group=self.liveness.group)
        self.create_timer(cfg.WATCHDOG_PERIOD_S, self.sensor_watchdog, callback_group=self.liveness.group)

    def initialize_mode(self):
        super().initialize_mode()
        self.declare_parameter('calibration_path', cfg.CALIBRATION_PATH)
        self.fusion_settings = cfg.load_settings(self.get_parameter('calibration_path').value)
        self.scans = deque(maxlen=SCAN_BUFFER_SIZE)
        self.current_image_stamp = 0.0
        self.current_image_received = time.monotonic()
        self.mission = FusionCore(self.mission, self)

    def on_scan(self, scan):
        stamp = scan.header.stamp.sec + scan.header.stamp.nanosec / 1e9
        self.scans.append((stamp, time.monotonic(), scan))

    def on_calibration(self, msg):
        try:
            self.fusion_settings = cfg.validate(json.loads(msg.data))
            self.publish_stop()  # Calibration adjustment requires a fresh processed frame.
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self.get_logger().error(f'Invalid calibration: {exc}')

    def on_image(self, image_msg, frame_received_at=None):
        self.current_image_stamp = self._stamp_ns(image_msg) / 1e9
        self.current_image_received = time.monotonic() if frame_received_at is None else frame_received_at
        return super().on_image(image_msg, frame_received_at)

    def scan_for_image(self):
        import numpy as np
        p, now = self.fusion_settings, time.monotonic()
        empty = np.empty((0, 2))
        if not p['confirmed']:
            return 'CALIBRATION_NOT_CONFIRMED', empty
        scans = list(self.scans)
        if not scans:
            return 'NO_SCAN', empty
        stamp, received, scan = min(scans, key=lambda item: abs(item[0] - self.current_image_stamp))
        if now - received > p['scan_timeout_s']:
            return 'SCAN_STALE', empty
        if now - self.current_image_received > p['image_timeout_s']:
            return 'IMAGE_STALE', empty
        clock_age = self.get_clock().now().nanoseconds / 1e9 - stamp
        if clock_age < -p['max_pair_delta_s'] or clock_age > p['scan_timeout_s']:
            return 'SCAN_TIMESTAMP_STALE', empty
        image_age = self.get_clock().now().nanoseconds / 1e9 - self.current_image_stamp
        if image_age < -p['max_pair_delta_s'] or image_age > p['image_timeout_s']:
            return 'IMAGE_TIMESTAMP_STALE', empty
        if not stamp or not self.current_image_stamp or abs(stamp - self.current_image_stamp) > p['max_pair_delta_s']:
            return 'PAIR_TIME_MISMATCH', empty
        xy = cfg.points(scan, p)
        return ('', xy) if len(xy) else ('NO_VALID_SCAN_POINTS', empty)

    def sensor_watchdog(self):
        p, now = self.fusion_settings, time.monotonic()
        scans = list(self.scans)
        if (not p['confirmed'] or not scans or now - scans[-1][1] > p['scan_timeout_s'] or
                now - self.current_image_received > p['image_timeout_s']):
            self.publish_stop()

    def _make_debug(self, frame, detections, *args):
        out = super()._make_debug(frame, detections, *args)
        for item in self.mission.status.get('fusion', []):
            x1, y1, x2, y2 = map(int, item['box'])
            color = (0, 0, 255) if item['camera_blocks_path'] else (0, 200, 255)
            cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
            distance = item['distance_m']
            label = f'{distance:.2f}m' if distance is not None else item['reason']
            cv2.putText(out, f'{label} n={item["points"]} path={item["camera_blocks_path"]}',
                        (max(0, x1), max(45, y1)), cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1)
        return out


def main(args=None):
    run_controller(FusionMissionControllerNode, args)
