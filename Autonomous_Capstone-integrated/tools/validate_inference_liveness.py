#!/usr/bin/env python3
"""Real ROS callbacks/processes, synthetic images/detector and fake serial ONLY.

Run with ROS_DOMAIN_ID=144 after sourcing the workspace. No serial device,
camera, LiDAR, motor or firmware tool is opened. The child uses the production
track/mission pipeline/executor, replacing only YOLO's detector with a 2s stall.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

OUTPUT = Path('/tmp/autodrive_liveness_validation')
REMAPS = ['vehicle/' + name + ':=/dry_run/health_test/' + name for name in (
    'controller_heartbeat', 'arm_challenge', 'drive_state', 'ui_heartbeat',
    'tuner_heartbeat', 'command_status', 'arm_request', 'armed', 'calibration_ready', 'calibration_status')]


def ros_args(params):
    args = ['--ros-args']
    for name, value in params.items():
        args += ['-p', f'{name}:={str(value).lower() if type(value) is bool else value}']
    for remap in REMAPS:
        args += ['-r', remap]
    return args


def controller(mode, marker):
    from skku_track_drive_pkg import track_controller_node as track
    from skku_track_drive_pkg.messages import DetectionArray
    from skku_track_drive_pkg.mission_controller_node import MissionControllerNode
    from skku_track_drive_pkg.controller_liveness import run_controller
    class DelayedDetector:
        def __init__(self, *_args, **_kwargs):
            self.yolo = SimpleNamespace(task='segment', names={i: s for i, s in enumerate(
                ('lane1', 'lane2', 'obstacle', 'traffic_light'))})
            self.calls = 0
        def detect(self, _frame):
            self.calls += 1
            if self.calls in (2, 4):
                marker.write_text(str(self.calls))
                time.sleep(2.0)  # releases the GIL like a waiting native YOLO call
            return DetectionArray()
    with patch.object(track, 'YoloDetector', DelayedDetector):
        run_controller(track.TrackControllerNode if mode == 'track' else MissionControllerNode,
                       ros_args({'cmd_topic': '/dry_run/health_test/command',
                                 'image_topic': '/dry_run/health_test/image', 'start_enabled': True,
                                 'publish_debug': False, 'publish_bev_debug': False,
                                 'image_reliability': 'reliable', 'speed': 80}))


class FakeSerial:
    def __init__(self, *_args, **_kwargs):
        self.is_open = True
        self.in_waiting = 0
        self.events = []
    def write(self, payload):
        self.events.append((time.monotonic(), payload.decode().strip()))
        return len(payload)
    def close(self):
        self.is_open = False


def validate(mode):
    from vehicle_io_pkg.serial_sender_node import SerialSenderNode
    from cv_bridge import CvBridge
    from interfaces_pkg.msg import MotionCommand
    from std_msgs.msg import Bool
    rclpy.init(args=ros_args({'topic': '/dry_run/health_test/command', 'port': 'FAKE_ONLY',
                             'auto_calibrate': False, 'require_tuner_heartbeat': True,
                             'require_controller_heartbeat': True,
                             'require_perception_progress': True,
                             'perception_stop_s': 6.0,
                             'command_timeout': 0.0 if mode == 'track' else .75}))
    serial = FakeSerial()
    with patch('vehicle_io_pkg.serial_sender_node.serial.Serial', return_value=serial), \
         patch('vehicle_io_pkg.serial_sender_node.time.sleep'):
        sender = SerialSenderNode()
    runner = Node('liveness_test_runner')
    executor = SingleThreadedExecutor()
    executor.add_node(sender)
    executor.add_node(runner)
    state = {}
    runner.create_subscription(String, '/mission/status', lambda m: state.update(json.loads(m.data)), 1)
    frame_pub = runner.create_publisher(Image, '/dry_run/health_test/image', 1)
    frame = CvBridge().cv2_to_imgmsg(np.zeros((480, 640, 3), np.uint8), encoding='bgr8')
    request_pub = runner.create_publisher(String, 'vehicle/arm_request', 1)
    def ui():
        sender.on_ui_heartbeat(String(data=sender.challenge))
        sender.on_tuner_heartbeat(String(data=sender.challenge))
    runner.create_timer(.1, ui)
    process = None
    logs = []
    result = {}
    marker = OUTPUT / f'{mode}.stall'

    def pump(duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)

    def until(predicate, timeout=10):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)
        assert predicate(), f'{mode}: timed out waiting for condition; child={process.poll()}'

    def spawn():
        nonlocal process
        marker.unlink(missing_ok=True)
        log = (OUTPUT / f'{mode}.controller.{len(logs)}.log').open('w')
        logs.append(log)
        process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--controller', mode, str(marker)],
                                   stdout=log, stderr=subprocess.STDOUT)
        until(lambda: sender.last_controller_time is not None)

    def image():
        # Wait for discovery; never use a real camera publisher.
        until(lambda: frame_pub.get_subscription_count() == 1)
        frame_pub.publish(frame)

    def w():
        until(lambda: sender.delay_guard.ready(time.monotonic()))
        request_pub.publish(String(data=sender.challenge))
        until(lambda: sender.armed)
        pump(.07)

    def stopped_since(index):
        assert all(event == 'X' for _, event in serial.events[index:]), serial.events[index:]

    try:
        spawn()
        image()
        until(lambda: sender.last_command_time is not None)
        w()
        image()
        until(lambda: marker.exists() and marker.read_text() == '2')
        initial_command_time = sender.last_command_time
        initial_health_time = sender.last_controller_time
        writes = len(serial.events)
        pump(1.25)
        assert sender.armed and sender.last_command_time == initial_command_time
        assert sender.last_controller_time > initial_health_time + 1.0
        frames = serial.events[writes:]
        assert len(frames) >= 20 and all(event == 's0l80r80' for _, event in frames), frames
        max_gap = max(b[0] - a[0] for a, b in zip(frames, frames[1:]))
        assert max_gap < .5
        result['two_second_inference_delay'] = {'observed_stall_s': 1.25, 'motion_writes': len(frames),
            'max_serial_gap_s': round(max_gap, 4), 'health_independent': True, 'armed': True}

        stale_w = sender.challenge
        sender.on_arm(Bool(data=False))  # exact production S recipient
        assert serial.events[-1][1] == 'X' and not sender.armed
        writes = len(serial.events)
        request_pub.publish(String(data=stale_w))
        pump(1.0)  # delayed inference completes after S
        assert not sender.armed
        stopped_since(writes)
        until(lambda: sender.last_command_time is not None and sender.last_controller_time is not None)
        w()
        pump(1.2)  # temporary CAMERA gap, no next image
        assert sender.armed
        if mode == 'mission':
            assert state.get('system_state') == 'IMAGE_DELAY_HOLD', state
        result['s_during_delay'] = {'direct_stop': 'X', 'late_result_keeps_disarmed': True, 'fresh_w_required': True}
        result['camera_gap_s'] = 1.2

        # Intentional zero commands must stay zero throughout a later delay.
        sender.on_cmd(MotionCommand())
        writes = len(serial.events)
        pump(.8)
        assert sender.armed and all(event == 's0l0r0' for _, event in serial.events[writes:])
        result['intentional_stop_held'] = True
        image()  # third detector call, immediate
        until(lambda: sender.last_cmd.left_speed == 80)
        pump(.07)
        image()  # fourth detector call, another two-second inference
        until(lambda: marker.read_text() == '4')
        started = time.monotonic()
        process.send_signal(signal.SIGINT)
        until(lambda: not sender.armed)
        result['controller_ctrl_c_stop_s'] = round(time.monotonic() - started, 4)
        writes = len(serial.events)
        pump(.3)
        stopped_since(writes)
        until(lambda: process.poll() is not None)
        assert process.returncode == 0

        # A replacement controller/health alone cannot restore authorization.
        spawn()
        request_pub.publish(String(data=sender.challenge))
        pump(.2)
        assert not sender.armed
        image()
        until(lambda: sender.last_command_time is not None)
        assert not sender.armed
        w()
        stale_w = sender.challenge
        process.send_signal(signal.SIGSTOP)
        started = time.monotonic()
        until(lambda: not sender.armed, timeout=2)
        result['controller_sigstop_stop_s'] = round(time.monotonic() - started, 4)
        writes = len(serial.events)
        process.send_signal(signal.SIGCONT)
        pump(.3)
        stopped_since(writes)
        request_pub.publish(String(data=stale_w))
        image()  # replacement child's second call stalls for 2s
        until(lambda: sender.last_command_time is not None)
        assert not sender.armed
        w()
        process.kill()
        started = time.monotonic()
        until(lambda: not sender.armed, timeout=2)
        result['controller_sigkill_stop_s'] = round(time.monotonic() - started, 4)
        assert result['controller_sigkill_stop_s'] < sender.controller_timeout + .1
        writes = len(serial.events)
        pump(.3)
        stopped_since(writes)
        result['process_loss_bounded_and_new_w_required'] = True
        result['controller_lease_s'] = sender.controller_timeout
        return result
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGCONT)
            process.kill()
            process.wait(timeout=5)
        for log in logs:
            log.close()
            assert 'Traceback' not in Path(log.name).read_text(), Path(log.name).read_text()
        sender.close_serial()
        (OUTPUT / f'{mode}.serial_events.json').write_text(json.dumps(serial.events, indent=2))
        executor.shutdown()
        sender.destroy_node()
        runner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    if os.environ.get('ROS_DOMAIN_ID') != '144':
        raise RuntimeError('Set ROS_DOMAIN_ID=144; validation must be isolated from the vehicle')
    if len(sys.argv) > 1 and sys.argv[1] == '--controller':
        controller(sys.argv[2], Path(sys.argv[3]))
    else:
        OUTPUT.mkdir(exist_ok=True)
        results = {mode: validate(mode) for mode in ('track', 'mission')}
        (OUTPUT / 'results.json').write_text(json.dumps(results, indent=2))
        print(json.dumps(results, indent=2))
