"""Sidecar recorder: reuse rosbag2 for inputs; join control CSV by source stamp.

No sensor, command or arm publishers and no serial access. Disk I/O runs in
this process, independently of the driving controller and its health callback.
"""
import csv
import hashlib
import json
from pathlib import Path
import signal
import subprocess
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String


FIELDS = ['received_ros_ns', 'received_monotonic_s', 'frame_stamp_ns', 'pipeline_duration_s',
          'inference_duration_s', 'analysis_build_ms', 'target_lane', 'path_lane', 'path_valid',
          'cte_px', 'heading_error_deg', 'error_reference_valid', 'steering', 'left_pwm', 'right_pwm',
          'mission_state', 'reason', 'lane_switch_requested', 'lane_switch_applied', 'details_json']


class TuningRecorderNode(Node):
    def __init__(self):
        super().__init__('tuning_recorder_node')
        for name, default in [('output_dir', ''), ('mode', 'track'), ('input_topic', '/track/image_raw'),
                              ('controller_root', ''), ('tuning_keys_json', '[]'),
                              ('command_topic', '/dry_run/topic_control_signal'), ('vehicle_prefix', '/dry_run/vehicle')]:
            self.declare_parameter(name, default)
        self.mode = self.get_parameter('mode').value
        roots = {'track': 'track_controller_node', 'mission': 'mission_controller_node',
                 'perpendicular': 'perpendicular_parking_controller_node',
                 'parallel': 'parallel_parking_controller_node', 'calibration': 'parking_calibration_controller_node'}
        root = self.get_parameter('controller_root').value or roots[self.mode]
        output = str(self.get_parameter('output_dir').value)
        if not output:
            raise ValueError('output_dir is required')
        self.directory = Path(output).expanduser().resolve()
        self.directory.mkdir(parents=True, exist_ok=False)
        self.csv_file = (self.directory / 'control.csv').open('w', newline='')
        self.writer = csv.DictWriter(self.csv_file, fieldnames=FIELDS)
        self.writer.writeheader()
        self.events = (self.directory / 'events.jsonl').open('w')
        self.rows = 0
        self._closed = False
        self.bag = self.dump = None
        timing = '/' + root + '/perception_timing'
        status = '/mission/status' if self.mode == 'mission' else '/parking_calibration/status' if self.mode == 'calibration' else '/parking/status'
        topics = [self.get_parameter('command_topic').value, '/parameter_events', '/video_replay_node/frame_source']
        source = self.get_parameter('input_topic').value
        if source:
            topics.append(source)
        if self.mode in ('track', 'mission'):
            topics.append(timing)
            self.create_subscription(String, timing, self.on_frame, 100)
        if self.mode != 'track':
            topics.append(status)
            self.create_subscription(String, status, self.on_status, 100)
        prefix = str(self.get_parameter('vehicle_prefix').value).rstrip('/')
        topics.extend(prefix + '/' + name for name in ('command_status', 'drive_state', 'drive_key',
                      'controller_heartbeat', 'arm_challenge', 'calibration_status'))
        self.create_subscription(String, prefix + '/command_status', lambda m: self.event('serial_command', json.loads(m.data)), 100)
        self.manifest = {'schema': 1, 'mode': self.mode, 'controller': root, 'topics': topics,
                         'started_ros_ns': self.get_clock().now().nanoseconds,
                         'timestamp_basis': 'frame_stamp_ns joins original Image/LaserScan header; received_ros_ns matches rosbag receipt clock',
                         'physical_pwm_measured': False, 'rows': 0}
        self.write_manifest()
        self.bag_log = (self.directory / 'rosbag.log').open('w')
        try:
            self.bag = subprocess.Popen(['ros2', 'bag', 'record', '-o', str(self.directory / 'input_bag'), *topics],
                                        stdout=self.bag_log, stderr=subprocess.STDOUT)
        except Exception:
            self.close()
            raise
        self.create_timer(.5, self.poll_metadata)
        self.get_logger().info(f'Recording inputs and source-stamped control evidence to {self.directory}')

    def write_manifest(self):
        (self.directory / 'manifest.json').write_text(json.dumps(self.manifest, indent=2))

    def poll_metadata(self):
        if self.dump is None and not self.manifest.get('parameter_snapshot_done') and self.manifest['controller'] in self.get_node_names():
            self.dump = subprocess.Popen(['ros2', 'param', 'dump', '/' + self.manifest['controller']],
                                         stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if self.dump is not None and self.dump.poll() is not None:
            text, error = self.dump.communicate()
            (self.directory / 'loaded_parameters.yaml').write_text(text)
            self.manifest['parameter_dump_error'] = error
            try:
                document = yaml.safe_load(text)
                values = next(iter(document.values()))['ros__parameters']
                root = self.manifest['controller']
                allowed = json.loads(self.get_parameter('tuning_keys_json').value)
                (self.directory / 'replay_tuning.yaml').write_text(yaml.safe_dump({root: {'ros__parameters':
                    {key: value for key, value in values.items() if key in allowed}}}, sort_keys=False))
                for label, key in [('model', 'model_path'), ('tuning', 'loaded_tuning_config')]:
                    path = Path(values.get(key, ''))
                    if path.is_file():
                        self.manifest[label] = {'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
                self.manifest['initial_speed'] = values.get('speed')
                self.manifest['parameter_snapshot_done'] = True
                self.manifest.pop('parameter_snapshot_unavailable', None)
            except (TypeError, AttributeError, KeyError, StopIteration):
                self.manifest['parameter_snapshot_unavailable'] = True
            self.dump = None
            self.write_manifest()
        if self.bag is not None and self.bag.poll() is not None and not self._closed:
            self.get_logger().error('rosbag exited; recording is incomplete, controller continues unchanged')
            self.manifest['bag_exited_early'] = True
            self.write_manifest()
            self.bag = None

    def event(self, kind, data):
        self.events.write(json.dumps({'kind': kind, 'received_ros_ns': self.get_clock().now().nanoseconds,
                                     'received_monotonic_s': time.monotonic(), 'data': data}) + '\n')
        self.events.flush()

    def on_frame(self, message):
        data = json.loads(message.data)
        self.event('frame_control', data)
        row = {key: data.get(key) for key in FIELDS}
        row.update(received_ros_ns=self.get_clock().now().nanoseconds,
                   received_monotonic_s=time.monotonic(), details_json=json.dumps(data))
        self.writer.writerow(row)
        self.csv_file.flush()
        self.rows += 1

    def on_status(self, message):
        data = json.loads(message.data)
        self.event('mission' if self.mode == 'mission' else self.mode, data)
        if self.mode not in ('track', 'mission'):
            self.on_frame(String(data=json.dumps({**data, 'mission_state': data.get('state'),
                'reason': data.get('waiting', data.get('note')), 'left_pwm': data.get('left_pwm', data.get('speed', data.get('pwm'))),
                'right_pwm': data.get('right_pwm', data.get('speed', data.get('pwm'))), 'steering': data.get('steering', data.get('steering_step'))})))

    def close(self):
        if self._closed:
            return
        self._closed = True
        if self.bag is not None and self.bag.poll() is None:
            self.bag.send_signal(signal.SIGINT)
            try:
                self.bag.wait(timeout=12)
            except subprocess.TimeoutExpired:
                self.bag.kill()
                self.bag.wait()
                self.manifest['bag_forced_exit'] = True
        if self.dump is not None and self.dump.poll() is None:
            self.dump.terminate()
            self.dump.wait(timeout=5)
        self.manifest['rows'] = self.rows
        self.manifest['closed_ros_ns'] = self.get_clock().now().nanoseconds if rclpy.ok() else None
        self.write_manifest()
        self.csv_file.close()
        self.events.close()
        if hasattr(self, 'bag_log'):
            self.bag_log.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TuningRecorderNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
