#!/usr/bin/env python3
"""Exercise deployed camera parking nodes over DDS in isolated preview mode.

Run after colcon build, source install/setup.bash, and validate_parking_models.py.
Only dataset images are published. No camera, arm gate or serial is started.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from interfaces_pkg.msg import MotionCommand

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT/'reports/parking_training_2026-11-13'


class Probe(Node):
    def __init__(self):
        super().__init__('parking_dataset_preview_probe')
        self.bridge = CvBridge()
        self.frames, self.publishers = {}, {}
        self.valid = {'front': 0, 'rear': 0}
        self.sent = {'front': set(), 'rear': set()}
        self.commands = []
        self.status = {}
        self.arm_pub = self.create_publisher(Bool, '/parking_camera_preview/preview_enabled', 1)
        self.create_subscription(MotionCommand, '/sensors_only/topic_control_signal', self.command, 10)
        self.create_subscription(String, '/parking_camera_preview/status', self.on_status, 1)
        for role in ('front', 'rear'):
            data = json.loads((REPORT/role/'runtime_validation.json').read_text())
            frame = cv2.imread(data['example_source_images'][0])
            # Dataset rear is already 90 deg clockwise. Publish inverse-oriented
            # RAW pixels, so the actual ROS perception node performs the rotation.
            if role == 'rear':
                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
            self.frames[role] = frame
            self.publishers[role] = self.create_publisher(Image, f'/parking_camera/{role}/image_raw', qos_profile_sensor_data)
            self.create_subscription(String, f'/parking_camera_preview/{role}/observation',
                                     lambda msg, role=role: self.observation(role, msg), 10)
        self.timer = self.create_timer(.2, self.publish_frames)

    def command(self, msg):
        self.commands.append((time.monotonic(), msg.steering, msg.left_speed, msg.right_speed))

    def on_status(self, msg):
        self.status = json.loads(msg.data)

    def observation(self, role, msg):
        data = json.loads(msg.data)
        assert data['camera'] == role
        assert data['stamp_ns'] in self.sent[role], 'Observation lost original frame timestamp'
        if data['valid']:
            self.valid[role] += 1

    def publish_frames(self):
        stamp = self.get_clock().now().to_msg()
        key = stamp.sec*1000000000 + stamp.nanosec
        for role in ('front', 'rear'):
            msg = self.bridge.cv2_to_imgmsg(self.frames[role], encoding='bgr8')
            msg.header.stamp = stamp
            msg.header.frame_id = f'parking_{role}_optical'
            self.sent[role].add(key)
            self.publishers[role].publish(msg)


def until(node, predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.03)
        if predicate():
            return
    raise RuntimeError(f'Preview validation timeout: valid={node.valid}, status={node.status}')


def main():
    # Dedicated DDS domain avoids interference with concurrently running modes.
    os.environ['ROS_DOMAIN_ID'] = '113'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_LOG_DIR'] = '/tmp/codex_parking_ros_validation'
    log_path = ROOT/'datasets/parking/20261113/ros_preview.log'
    log = log_path.open('w')
    process = subprocess.Popen([
        'ros2', 'launch', 'vehicle_bringup_pkg', 'perpendicular_camera_preview.launch.py',
        f'config_file:={ROOT}/src/vehicle_bringup_pkg/config/perpendicular_camera.yaml',
        'source:=topics', 'gui:=false'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    node = None
    rclpy.init()
    try:
        node = Probe()
        until(node, lambda: min(node.valid.values()) >= 5, 60)
        until(node, lambda: node.commands and not any(node.commands[-1][1:]), 5)
        node.arm_pub.publish(Bool(data=True))
        until(node, lambda: node.commands and any(node.commands[-1][2:]), 15)
        moving = node.commands[-1]
        stopped_at = time.monotonic()
        node.arm_pub.publish(Bool(data=False))
        until(node, lambda: node.commands and node.commands[-1][0] > stopped_at and
              not any(node.commands[-1][1:]), 2)
        stopped = node.commands[-1]
        until(node, lambda: node.status.get('armed') is False, 2)
        result = dict(valid_observations=node.valid, preview_motion_command=list(moving[1:]),
                      stopped_command=list(stopped[1:]),
                      stop_latency_ms=(stopped[0]-stopped_at)*1000,
                      status=node.status, timestamp_preserved=True,
                      command_topic='/sensors_only/topic_control_signal',
                      source='Dataset frames over DDS; actual two perception nodes and controller',
                      no_hardware=True, process_log=str(log_path.relative_to(ROOT)))
        (REPORT/'ros_preview_validation.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2))
    finally:
        if node is not None:
            node.arm_pub.publish(Bool(data=False))
            node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=10)
        log.close()


if __name__ == '__main__':
    main()
