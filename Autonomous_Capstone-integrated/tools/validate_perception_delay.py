#!/usr/bin/env python3
"""Four failure scenarios on production ROS nodes, simulated images/fake serial.

Only ROS_DOMAIN_ID=146. No camera/LiDAR/serial devices, motors or firmware tools.
The child replaces YOLO with a scripted waiting detector; the production lane,
path, control, progress, health executor and serial sender run unchanged.
"""
import json
import os
from pathlib import Path
import re
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
from std_msgs.msg import String
from sensor_msgs.msg import Image

from validate_inference_liveness import FakeSerial, ros_args

OUTPUT = Path('/tmp/autodrive_perception_hold_validation')
COMMAND = re.compile(r's(-?\d+)l(-?\d+)r(-?\d+)$')


def child(mode, directory, wait_green):
    from skku_track_drive_pkg import track_controller_node as track
    from skku_track_drive_pkg.mission_controller_node import MissionControllerNode
    from skku_track_drive_pkg.controller_liveness import run_controller
    from skku_track_drive_pkg.messages import DetectionArray
    script, marker = directory / 'script.json', directory / 'worker.json'
    class WaitingDetector:
        def __init__(self, *_args, **_kwargs):
            self.yolo = SimpleNamespace(task='segment', names={i: s for i, s in enumerate(
                ('lane1', 'lane2', 'obstacle', 'traffic_light'))})
            self.generation = 0
        def detect(self, _frame):
            action = json.loads(script.read_text())
            if action['generation'] > self.generation:
                self.generation = action['generation']
                temp = marker.with_suffix('.tmp')
                temp.write_text(json.dumps({'action': action['action'], 'started_at_s': time.monotonic()}))
                temp.replace(marker)
                if action['action'] == 'short':
                    time.sleep(2.0)
                elif action['action'] == 'hang':
                    # Worker only: the main executor, receipt and 10Hz health
                    # callback continue. No inference completion occurs here.
                    while json.loads(script.read_text())['action'] == 'hang':
                        time.sleep(.05)
            return DetectionArray()
    parameters = {'cmd_topic': '/dry_run/health_test/command', 'image_topic': '/dry_run/health_test/image',
        'start_enabled': True, 'publish_debug': False, 'publish_bev_debug': False,
        'image_reliability': 'reliable', 'speed': 100}
    if mode == 'mission':
        parameters['wait_for_green'] = int(wait_green)
    with patch.object(track, 'YoloDetector', WaitingDetector):
        run_controller(track.TrackControllerNode if mode == 'track' else MissionControllerNode, ros_args(parameters))


class Harness:
    def __init__(self, mode, case, wait_green=False):
        from vehicle_io_pkg.serial_sender_node import SerialSenderNode
        from vehicle_bringup_pkg.perception_policy import measured_policy
        from cv_bridge import CvBridge
        self.mode, self.case = mode, case
        self.directory = OUTPUT / f'{mode}_{case}'
        self.directory.mkdir(parents=True, exist_ok=True)
        self.script = self.directory / 'script.json'
        self.script.write_text(json.dumps({'generation': 0, 'action': 'normal'}))
        (self.directory / 'worker.json').unlink(missing_ok=True)
        self.policy = measured_policy(mode)
        rclpy.init(args=ros_args({'topic': '/dry_run/health_test/command', 'port': 'FAKE_ONLY',
            'auto_calibrate': False, 'require_tuner_heartbeat': True, 'require_controller_heartbeat': True,
            'require_perception_progress': True, 'command_timeout': 0.0 if mode == 'track' else .75,
            **self.policy}))
        self.port = FakeSerial()
        with patch('vehicle_io_pkg.serial_sender_node.serial.Serial', return_value=self.port), \
             patch('vehicle_io_pkg.serial_sender_node.time.sleep'):
            self.sender = SerialSenderNode()
        from rclpy.parameter import Parameter
        assert self.sender.describe_parameter('perception_stop_s').read_only
        result = self.sender.set_parameters([Parameter('perception_stop_s', value=1.0)])[0]
        assert not result.successful
        assert self.sender.get_parameter('perception_stop_s').value == self.policy['perception_stop_s']
        self.node = Node('perception_delay_test_runner')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.sender)
        self.executor.add_node(self.node)
        self.health, self.states = [], []
        self.node.create_subscription(String, 'vehicle/controller_heartbeat', self.on_health, 1)
        self.node.create_subscription(String, 'vehicle/command_status', self.on_status, 1)
        self.pub = self.node.create_publisher(Image, '/dry_run/health_test/image', 1)
        self.frame = CvBridge().cv2_to_imgmsg(np.zeros((480, 640, 3), np.uint8), encoding='bgr8')
        self.frame_timer = self.node.create_timer(1 / 15, lambda: self.pub.publish(self.frame))
        self.request = self.node.create_publisher(String, 'vehicle/arm_request', 1)
        self.node.create_timer(.1, self.ui)
        self.log = (self.directory / 'controller.log').open('w')
        self.process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--child', mode,
                                        str(self.directory), str(int(wait_green))],
                                        stdout=self.log, stderr=subprocess.STDOUT)
        self.until(lambda: self.sender.last_command_time is not None and self.sender.delay_guard.ready(time.monotonic()))
        self.w()
        self.pump(.2)

    def on_health(self, message):
        self.health.append({'at_s': time.monotonic(), **json.loads(message.data)})

    def on_status(self, message):
        self.states.append({'at_s': time.monotonic(), **json.loads(message.data)})

    def ui(self):
        self.sender.on_ui_heartbeat(String(data=self.sender.challenge))
        self.sender.on_tuner_heartbeat(String(data=self.sender.challenge))

    def pump(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.01)

    def until(self, condition, timeout=12):
        end = time.monotonic() + timeout
        while not condition() and time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.01)
        assert condition(), f'{self.mode}/{self.case}: condition timed out; check {self.directory}'

    def w(self):
        self.until(lambda: self.sender.delay_guard.ready(time.monotonic()) and self.sender.ui_alive(time.monotonic()))
        self.request.publish(String(data=self.sender.challenge))
        self.until(lambda: self.sender.armed)

    def action(self, action):
        previous = json.loads(self.script.read_text())
        temp = self.script.with_suffix('.tmp')
        temp.write_text(json.dumps({'generation': previous['generation'] + 1, 'action': action}))
        temp.replace(self.script)
        self.until(lambda: (self.directory / 'worker.json').exists() and
                   json.loads((self.directory / 'worker.json').read_text())['action'] == action)

    def motion(self, index):
        result = []
        for timestamp, data in self.port.events[index:]:
            match = COMMAND.fullmatch(data)
            if match:
                result.append((timestamp, *(int(x) for x in match.groups())))
        return result

    def long_stop(self, cause, intentional_zero=False):
        index, started, token = len(self.port.events), time.monotonic(), self.sender.challenge
        self.until(lambda: not self.sender.armed, self.policy['perception_stop_s'] + 1)
        elapsed = time.monotonic() - started
        rows = self.motion(index)
        levels = sorted(set(r[2] for r in rows), reverse=True)
        if intentional_zero:
            assert levels == [0], rows
        else:
            assert levels == [100, 0], levels
            assert all(r[1] == rows[0][1] for r in rows), 'steering changed during command hold'
            assert all(r[2:] == (100, 100) for r in rows[:-1]), 'PWM changed before terminal stop'
            assert rows[-1][2:] == (0, 0)
        assert cause in self.sender.stop_reason, self.sender.stop_reason
        assert elapsed >= self.policy['perception_stop_s'] - .3
        assert self.process.poll() is None, 'test must stop a live controller, not kill it first'
        assert self.port.events[-1][1] == 'X'
        self.request.publish(String(data=token))
        self.pump(.1)
        assert not self.sender.armed
        return {'stop_after_s': round(elapsed, 4), 'pwm_levels': levels, 'steering_preserved': True,
                'controller_still_alive': True, 'reason': self.sender.stop_reason}

    def close(self):
        if self.process.poll() is None:
            self.process.kill()
        self.process.wait(timeout=5)
        self.log.close()
        assert 'Traceback' not in (self.directory / 'controller.log').read_text()
        (self.directory / 'serial_events.json').write_text(json.dumps(self.port.events, indent=2))
        (self.directory / 'health.json').write_text(json.dumps(self.health, indent=2))
        (self.directory / 'status.json').write_text(json.dumps(self.states, indent=2))
        self.sender.close_serial()
        self.executor.shutdown()
        self.sender.destroy_node()
        self.node.destroy_node()
        rclpy.shutdown()


def validate(mode, case):
    harness = Harness(mode, case, wait_green=case == 'intentional_zero')
    h = harness
    try:
        if case == 'short':
            index, started = len(h.port.events), time.monotonic()
            h.action('short')
            h.pump(2.4)
            rows = h.motion(index)
            assert rows and all(row[2:] == (100, 100) for row in rows)
            assert h.sender.armed
            assert all(event != 'X' for _, event in h.port.events[index:])
            h.frame_timer.cancel()
            h.pump(1.0)
            assert h.sender.armed and h.sender.last_applied_cmd.left_speed == 100
            h.frame_timer.reset()
            h.pump(.3)
            return {'one_frame_delay_s': 2.0, 'short_camera_pause_s': 1.0, 'no_slowdown_or_stop': True,
                    'max_serial_gap_s': round(max(b[0] - a[0] for a, b in zip(rows, rows[1:])), 4)}
        if case in ('worker_hang', 'intentional_zero'):
            h.action('hang')
            result = h.long_stop('inference worker/result stopped', intentional_zero=case == 'intentional_zero')
            samples = [r['perception'] for r in h.health if r.get('active') and 'perception' in r]
            assert samples[-1]['camera_age_s'] < .3 and samples[-1]['worker_busy']
            h.action('resume')
            h.until(lambda: h.sender.delay_guard.ready(time.monotonic()))
        elif case == 'camera_stop':
            h.frame_timer.cancel()
            result = h.long_stop('camera input stopped')
            samples = [r['perception'] for r in h.health if r.get('active') and 'perception' in r]
            assert samples[-1]['camera_age_s'] > 5 and not samples[-1]['worker_busy']
            h.frame_timer.reset()
            h.until(lambda: h.sender.delay_guard.ready(time.monotonic()))
        else:
            h.action('hang')  # Ctrl+C must bypass even a permanently waiting worker
            index, started = len(h.port.events), time.monotonic()
            h.process.send_signal(signal.SIGINT)
            h.until(lambda: not h.sender.armed, timeout=2)
            assert 'controller shutdown' in h.sender.stop_reason
            result = {'controller_ctrl_c_stop_s': round(time.monotonic() - started, 4),
                      'immediate_x_path': h.port.events[-1][1] == 'X'}
            assert all(row[2] in (0, 100) for row in h.motion(index)), 'process exit must stop immediately'
            return result
        index = len(h.port.events)
        h.pump(.4)
        assert not h.sender.armed and all(event == 'X' for _, event in h.port.events[index:])
        h.w()
        result['recovery_needs_new_w'] = True
        return result
    finally:
        h.close()


if __name__ == '__main__':
    if os.environ.get('ROS_DOMAIN_ID') != '146':
        raise RuntimeError('Set ROS_DOMAIN_ID=146 to isolate simulation from the vehicle')
    if len(sys.argv) > 1 and sys.argv[1] == '--child':
        child(sys.argv[2], Path(sys.argv[3]), bool(int(sys.argv[4])))
    else:
        OUTPUT.mkdir(exist_ok=True)
        report = {}
        for mode in ('track', 'mission'):
            report[mode] = {}
            for case in ('short', 'worker_hang', 'camera_stop', 'process_exit', *(['intentional_zero'] if mode == 'mission' else [])):
                report[mode][case] = validate(mode, case)
                print(mode, case, json.dumps(report[mode][case]), flush=True)
        (OUTPUT / 'results.json').write_text(json.dumps(report, indent=2))
