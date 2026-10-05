#!/usr/bin/env python3
"""Record camera arrivals and SUCCESSFUL perception completions, no motors.

Starts ONLY sensors_only controller/camera on ROS domain 145. Records timing
JSONL, never images. Motor sender/gate never run; GUI is headless/disabled.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


def distribution(samples):
    if not samples:
        return {'count': 0}
    values = np.asarray(samples) * 1000.0
    return {'count': len(samples), **{name: round(float(np.percentile(values, p)), 3)
        for name, p in (('min_ms', 0), ('median_ms', 50), ('p95_ms', 95), ('p99_ms', 99), ('max_ms', 100))}}


class TimingObserver(Node):
    def __init__(self, mode, stream):
        super().__init__('perception_timing_observer')
        self.frames, self.results = [], []
        self.stream = stream
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image, f'/{mode}/image_raw', self.on_frame, qos)
        self.create_subscription(String, f'/{mode}_controller_node/perception_timing', self.on_result, qos)

    def on_frame(self, message):
        row = {'kind': 'frame', 'received_at_s': time.monotonic(),
               'stamp_ns': message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec}
        self.frames.append(row)
        self.stream.write(json.dumps(row) + '\n')

    def on_result(self, message):
        row = {'kind': 'result', **json.loads(message.data), 'observed_at_s': time.monotonic()}
        self.results.append(row)
        self.stream.write(json.dumps(row) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', required=True, choices=('track', 'mission'))
    parser.add_argument('--camera-device', required=True)
    parser.add_argument('--device', default='cuda:0')
    parser.add_argument('--duration', type=float, default=60)
    parser.add_argument('--warmup', type=float, default=5)
    parser.add_argument('--output', required=True)
    options = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '145':
        parser.error('Set ROS_DOMAIN_ID=145 for isolated sensor-only measurement')
    if not math.isfinite(options.duration) or options.duration < 10 or not math.isfinite(options.warmup) or options.warmup < 0:
        parser.error('duration >= 10 seconds; warmup >= 0, both finite')
    output = Path(options.output)
    output.mkdir(parents=True, exist_ok=True)
    filename = 'track_drive_tuning.launch.py' if options.mode == 'track' else 'mission_drive_tuning.launch.py'
    process = None
    rclpy.init()
    with (output / 'events.jsonl').open('w') as stream, (output / 'launch.log').open('w') as log:
        observer = TimingObserver(options.mode, stream)
        try:
            process = subprocess.Popen(['ros2', 'launch', 'vehicle_bringup_pkg', filename,
                'sensors_only:=true', 'gui:=false', f'camera_device:={options.camera_device}', f'device:={options.device}'],
                stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 30
            while not observer.results and time.monotonic() < deadline and process.poll() is None:
                rclpy.spin_once(observer, timeout_sec=.05)
            if not observer.results:
                raise RuntimeError('No successful inference; inspect launch.log. No threshold can be inferred.')
            started = time.monotonic() + options.warmup
            end = started + options.duration
            while time.monotonic() < end and process.poll() is None:
                rclpy.spin_once(observer, timeout_sec=.05)
            if process.poll() is not None:
                raise RuntimeError('Launch exited during measurement; inspect launch.log')
            frames = [r for r in observer.frames if started <= r['received_at_s'] <= end]
            results = [r for r in observer.results if started <= r['completed_at_s'] <= end]
            def gaps(rows, key, scale=1):
                return [(b[key] - a[key]) * scale for a, b in zip(rows, rows[1:])]
            summary = {'mode': options.mode, 'camera_device': options.camera_device, 'device': options.device,
                'duration_s': options.duration, 'warmup_s': options.warmup,
                'camera_identity': 'See /dev/v4l/by-id; this measurement is not automatically a vehicle-camera calibration',
                'frames': len(frames), 'successful_results': len(results),
                'frame_arrival_interval': distribution(gaps(frames, 'received_at_s')),
                'camera_publish_stamp_interval': distribution(gaps(frames, 'stamp_ns', 1e-9)),
                'completion_interval': distribution(gaps(results, 'completed_at_s')),
                'completion_basis': 'Successful YOLO + lane/path/mission/control command completion',
                'yolo_completion_interval': distribution(gaps(
                    [r for r in results if 'inference_completed_at_s' in r], 'inference_completed_at_s')),
                'inference_duration': distribution([r['inference_duration_s'] for r in results]),
                'pipeline_duration': distribution([r['pipeline_duration_s'] for r in results]),
                'ingress_to_result_age': distribution([r['result_frame_age_s'] for r in results
                                                       if 'result_frame_age_s' in r]),
                'startup_frame_arrival_interval': distribution(gaps(observer.frames[:10], 'received_at_s')),
                'startup_completion_interval': distribution(gaps(observer.results[:10], 'completed_at_s')),
                'frame_tail_age_s': end - frames[-1]['received_at_s'] if frames else None,
                'result_tail_age_s': end - results[-1]['completed_at_s'] if results else None,
                'serial_sender_started': False, 'arm_gate_started': False}
            if not frames or len(results) < 10:
                raise RuntimeError('Insufficient samples for a timing distribution')
            (output / 'summary.json').write_text(json.dumps(summary, indent=2))
            print(json.dumps(summary, indent=2), flush=True)
        finally:
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=7)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    process.wait(timeout=5)
            observer.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
