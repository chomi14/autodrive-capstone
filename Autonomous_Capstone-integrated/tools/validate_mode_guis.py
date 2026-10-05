#!/usr/bin/env python3
"""Exercise real parameter services + GUI callbacks with HighGUI headless.

Run after sourcing this workspace. Launches only dry_run controllers on an
isolated ROS domain, never serial/sensor/arm nodes. Saves temporary YAMLs and
render previews in /tmp, not operator tuning files. GUI window functions are
stubbed; drawing, ROS clients, acceptance and atomic saves are production code.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from unittest.mock import patch

import cv2
import numpy as np
import rclpy
import yaml
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import parameter_value_to_python

from skku_track_drive_pkg.mode_parameters import MISSION, PERPENDICULAR, PARALLEL, trackbars
from skku_track_drive_pkg.track_tuner_node import TrackTunerNode
from skku_track_drive_pkg.calibration_controller_node import CALIBRATION


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = Path('/tmp/codex_four_modes_validation')
MODES = {
    'track': ('track_drive_tuning.launch.py', 'track_controller_node', {}, 'speed', 37),
    'mission': ('mission_drive_tuning.launch.py', 'mission_controller_node', MISSION, 'traffic_min_width', 42),
    'perpendicular': ('perpendicular_parking.launch.py', 'perpendicular_parking_controller_node', PERPENDICULAR, 'side_detect_m', 1.4),
    'parallel': ('parallel_parking.launch.py', 'parallel_parking_controller_node', PARALLEL, 'gap_margin_m', .35),
    'calibration': ('parking_calibration.launch.py', 'parking_calibration_controller_node', CALIBRATION, 'pwm', -60),
}


def spin_until(node, predicate, timeout=12.0):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not predicate() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
    if not predicate():
        raise RuntimeError('Parameter service/GUI validation timed out')


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '143':
        raise RuntimeError('Set ROS_DOMAIN_ID=143 to isolate validation from the vehicle')
    OUTPUT.mkdir(exist_ok=True)
    results = {}
    for mode, (filename, controller, specs, parameter, value) in MODES.items():
        log_path = OUTPUT / f'{mode}.gui_validation.log'
        save_path = OUTPUT / f'{mode}.saved.yaml'
        node = None
        with log_path.open('w') as log:
            process = subprocess.Popen(['ros2', 'launch', 'vehicle_bringup_pkg', filename,
                                        'dry_run:=true', 'gui:=false', *(['device:=cpu'] if not specs or mode == 'mission' else []),
                                        *(['speed:=23'] if mode == 'track' else [])],
                                       stdout=log, stderr=subprocess.STDOUT)
            try:
                config_filename = f'{mode}_tuning.yaml' if mode in ('track', 'mission') else 'parking_calibration.yaml' if mode == 'calibration' else f'{mode}_parking.yaml'
                config = ROOT / 'src/vehicle_bringup_pkg/config' / config_filename
                rclpy.init(args=['--ros-args', '-p', f'target_node:=/{controller}',
                                 '-p', f'save_path:={save_path}', '-p', f'loaded_config_path:={config}',
                                 '-p', 'cmd_topic:=/dry_run/topic_control_signal',
                                 '-p', f'debug_topic:={"/parking/debug_image" if mode in ("perpendicular", "parallel") else "/mission/debug_image" if mode == "mission" else "/track_debug_image"}',
                                 '-p', f'status_topic:={"/mission/status" if mode == "mission" else "/parking_calibration/status" if mode == "calibration" else "/parking/status"}',
                                 '-p', f'allow_speed_tuning:={str(mode in ("track", "mission", "calibration")).lower()}'])
                images = {}
                with patch.multiple('skku_track_drive_pkg.track_tuner_node.cv2',
                                    namedWindow=lambda *a: None, resizeWindow=lambda *a: None,
                                    createTrackbar=lambda *a: None, setTrackbarPos=lambda *a: None,
                                    getWindowProperty=lambda *a: 1, waitKey=lambda *a: -1,
                                    destroyWindow=lambda *a: None,
                                    imshow=lambda title, image: images.update({title: image.copy()})):
                    node = TrackTunerNode(node_name=f'{mode}_validation_tuner', extra_trackbars=trackbars(specs),
                                          parameter_root=controller, window_prefix=mode.capitalize(),
                                          parking=mode in ('perpendicular', 'parallel'), only_extra_controls=mode == 'calibration')
                    spin_until(node, lambda: node._ui_ready)
                    if mode != 'track':
                        spin_until(node, lambda: bool(node.mode_status))
                    if mode in ('perpendicular', 'parallel'):
                        spin_until(node, lambda: node._latest_debug_frame is not None)
                        cv2.imwrite(str(OUTPUT / f'{mode}.lidar.png'), node._latest_debug_frame)
                    if mode == 'track':
                        assert node.current_values['speed'] == 23, 'Explicit launch speed was not loaded'
                    node._on_trackbar(parameter, node.trackbars[parameter][3](value))
                    node._send_pending_parameters()
                    spin_until(node, lambda: node._set_future is None and node.current_values.get(parameter) == value)
                    request = GetParameters.Request(names=[parameter])
                    readback = node.get_client.call_async(request)
                    spin_until(node, readback.done)
                    ros_value = parameter_value_to_python(readback.result().values[0])
                    assert ros_value == value, 'Controller parameter readback does not match GUI'
                    if mode == 'calibration':
                        spin_until(node, lambda: node.mode_status.get('pwm') == value)
                    if mode == 'track':
                        # Synthetic camera frames exercise real YOLO/controller
                        # and published commands, without opening any hardware.
                        publisher = node.create_publisher(Image, '/track/image_raw', 1)
                        frame = node.bridge.cv2_to_imgmsg(np.zeros((480, 640, 3), np.uint8), encoding='bgr8')
                        frame_timer = node.create_timer(.1, lambda: publisher.publish(frame))
                        spin_until(node, lambda: node.current_left_speed == value and node.current_right_speed == value)
                        frame_timer.cancel()
                        assert node.desired_values['speed'] == node.current_values['speed'] == node.current_left_speed == node.current_right_speed
                    node.save()
                    contents = yaml.safe_load(save_path.read_text())
                    assert contents[controller]['ros__parameters'][parameter] == value
                    if mode in ('track', 'mission'):
                        node.on_serial_status(String(data=json.dumps({
                            'armed': True, 'steering': -3, 'left_pwm': 100, 'right_pwm': 100,
                            'result_age_s': 4.5, 'perception_stop_s': 6.0, 'stop_reason': '',
                        })))
                    if mode == 'mission':
                        evidence = ROOT / 'reports/tuning_analysis_2026-10-05/mission_scenarios/frames.jsonl'
                        if evidence.is_file():
                            node.mode_status = json.loads(evidence.read_text().splitlines()[-1])['mission']
                    with patch('skku_track_drive_pkg.track_tuner_node.cv2.putText', wraps=cv2.putText) as draw:
                        node._draw_status()
                        labels = [call.args[1] for call in draw.call_args_list]
                        if mode in ('track', 'mission'):
                            assert any('SERIAL command PWM L/R=100/100' in label for label in labels)
                            assert any('PERCEPTION age=4.5' in label for label in labels)
                            assert not any('SLOWDOWN' in label or 'factor=' in label for label in labels)
                        if mode == 'mission' and 'traffic_boxes' in node.mode_status:
                            assert any('alternative lane=' in label for label in labels)
                            assert any('reason=' in label for label in labels)
                            assert any('R/G/Y=' in label for label in labels)
                    cv2.imwrite(str(OUTPUT / f'{mode}.tuner.png'), images[node.tuner_window])
                    results[mode] = {'parameter': parameter, 'accepted': value, 'yaml': str(save_path),
                                     'ros_readback': ros_value,
                                     'status': node.mode_status,
                                     'command_speed': node.current_left_speed,
                                     'windows': [node.tuner_window, *node.control_windows]}
                    node.destroy_node()
                    node = None
                rclpy.shutdown()
            finally:
                if node is not None:
                    with patch('skku_track_drive_pkg.track_tuner_node.cv2.destroyWindow'):
                        node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=7)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    process.wait(timeout=5)
        if 'Traceback' in log_path.read_text():
            raise RuntimeError(f'Controller smoke failed: {log_path}')
        print(f'{mode}: real parameter service, GUI callbacks and P/save passed', flush=True)
    (OUTPUT / 'gui_validation.json').write_text(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
