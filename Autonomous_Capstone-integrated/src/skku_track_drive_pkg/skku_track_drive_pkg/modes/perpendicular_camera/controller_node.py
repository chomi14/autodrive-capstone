"""ROS control adapter; the ONLY camera-parking MotionCommand producer."""
from dataclasses import fields
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from interfaces_pkg.msg import MotionCommand

from skku_track_drive_pkg.command_lease import CommandLease
from skku_track_drive_pkg.controller_liveness import ControllerLiveness, run_controller
from .configuration import load_config, save_config_atomic, validate_config
from .core import CameraParkingCore, ControlConfig
from .observations import Observation


class CameraParkingController(Node):
    def __init__(self):
        super().__init__('perpendicular_camera_controller')
        self.declare_parameter('config_file', '')
        self.declare_parameter('preview', True)
        self.declare_parameter('cmd_topic', '/dry_run/topic_control_signal')
        self.declare_parameter('session_overrides_json', '{}')
        self.preview = self.get_parameter('preview').value
        self.data = load_config(self.get_parameter('config_file').value)
        overrides = json.loads(self.get_parameter('session_overrides_json').value)
        if not isinstance(overrides, dict):
            raise ValueError('Invalid session overrides')
        for role, values in overrides.items():
            if role not in ('front', 'rear') or not isinstance(values, dict) or set(values)-{'model_path', 'device'}:
                raise ValueError('Only front/rear device and model_path session overrides are allowed')
            self.data[role].update(values)
        validate_config(self.data)
        defaults = ControlConfig(**self.data['control'])
        for item in fields(defaults):
            self.declare_parameter(item.name, getattr(defaults, item.name))
        settings = {item.name: self.get_parameter(item.name).value for item in fields(defaults)}
        self.core = CameraParkingCore(ControlConfig(**settings), require_calibration=not self.preview)
        topic = self.get_parameter('cmd_topic').value
        if self.preview and not topic.startswith(('/dry_run/', '/sensors_only/')):
            raise ValueError('Preview commands must use an isolated preview topic')
        if not self.preview and not self.core.p.calibration_confirmed:
            raise ValueError('Camera/steering calibration must be confirmed before live launch')
        self.command_lease = CommandLease(topic, self.get_name())
        self.cmd_pub = self.create_publisher(MotionCommand, topic, 1)
        self.status_pub = self.create_publisher(String, '/parking_camera/status', 1)
        self.observations = {'front': None, 'rear': None}
        self.received = {'front': 0.0, 'rear': 0.0}
        self.pipeline_seen = set()
        self.armed = False
        self.last_tick = time.monotonic()
        self.last_ros_ns = self.get_clock().now().nanoseconds
        self.core.reset(self.last_ros_ns)
        for role in ('front', 'rear'):
            self.create_subscription(String, f'/parking_camera/{role}/observation',
                                     lambda msg, role=role: self.on_observation(role, msg), 1)
        if self.preview:
            self.create_subscription(Bool, '/parking_camera/preview_enabled', self.on_arm, 1)
        else:
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Bool, 'vehicle/drive_state', self.on_arm, latched)
        self.create_service(Trigger, '~/reset', self.on_reset)
        self.create_service(Trigger, '~/save', self.on_save)
        self.add_on_set_parameters_callback(self.on_parameters)
        self.timer = self.create_timer(.05, self.on_timer)
        self.liveness = ControllerLiveness(self)

    def on_observation(self, role, message):
        try:
            obs = Observation.from_json(message.data)
            previous = self.observations[role]
            if obs.camera != role or (previous is not None and obs.stamp_ns <= previous.stamp_ns):
                return
            if obs.stamp_ns > self.get_clock().now().nanoseconds:
                return
            self.observations[role] = obs
            self.received[role] = time.monotonic()
            if obs.valid:
                self.pipeline_seen.add(role)
        except (ValueError, TypeError, KeyError) as exc:
            self.get_logger().error(f'Invalid {role} observation: {exc}')

    def on_arm(self, message):
        self.armed = bool(message.data)
        if not self.armed:
            # Requiring a fresh capture on the next W also handles S/W between
            # consecutive timer callbacks, without losing paused state/time.
            self.core.was_armed = False
            self.cmd_pub.publish(MotionCommand())

    def on_reset(self, _request, response):
        response.success = not self.armed
        response.message = 'S/disarm before reset' if self.armed else 'Camera parking reset'
        if response.success:
            self.core.reset(self.get_clock().now().nanoseconds)
            self.observations = {'front': None, 'rear': None}
            self.pipeline_seen.clear()
        return response

    def on_save(self, _request, response):
        try:
            if self.armed:
                raise ValueError('S/disarm before saving commissioning settings')
            save_config_atomic(self.data['save_path'], self.data, self.core.p)
            response.success, response.message = True, self.data['save_path']
        except (OSError, ValueError) as exc:
            response.success, response.message = False, str(exc)
        return response

    def on_parameters(self, parameters):
        try:
            if self.armed:
                raise ValueError('S/disarm before changing camera parking parameters')
            allowed = {item.name for item in fields(ControlConfig)}
            values = {name: getattr(self.core.p, name) for name in allowed}
            for parameter in parameters:
                if parameter.name not in allowed:
                    raise ValueError(f'{parameter.name} requires restart')
                values[parameter.name] = parameter.value
            self.core.p = ControlConfig(**values)
            self.core.reset(self.get_clock().now().nanoseconds)
            return SetParametersResult(successful=True)
        except (TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))

    def on_timer(self):
        now = time.monotonic()
        dt, self.last_tick = now-self.last_tick, now
        stamp = self.get_clock().now().nanoseconds
        if stamp < self.last_ros_ns:
            self.observations = {'front': None, 'rear': None}
            self.pipeline_seen.clear()
            self.core.fail('ROS_CLOCK_MOVED_BACKWARD_RESET_REQUIRED', stamp)
        self.last_ros_ns = stamp
        # Arrival-age ALSO bounds results when a simulated ROS clock is paused.
        usable = {role: obs if now-self.received[role] <= self.core.p.observation_timeout_s else None
                  for role, obs in self.observations.items()}
        ready = self.pipeline_seen == {'front', 'rear'}
        command = self.core.step(dt, stamp, self.armed and ready, **usable)
        self.cmd_pub.publish(MotionCommand(steering=command.steering,
                                           left_speed=command.left_speed, right_speed=command.right_speed))
        status = {
            'state': self.core.state, 'reason': self.core.reason if ready else 'WAIT_BOTH_PERCEPTION_PIPELINES',
            'transition': self.core.transition, 'fault': self.core.fault,
            'armed': self.armed, 'preview': self.preview,
            'calibration_confirmed': self.core.p.calibration_confirmed,
            'active_elapsed_s': self.core.elapsed, 'loss_watchdog_s': self.core.loss_elapsed,
            'seen_space': self.core.seen_space, 'seen_count': self.core.seen_count,
            'missing_count': self.core.missing_count, 'align_count': self.core.align_count,
            'errors': self.core.errors, 'steering': command.steering, 'pwm': command.left_speed,
            'observations': {role: None if obs is None else {
                'stamp_ns': obs.stamp_ns, 'frame_id': obs.frame_id, 'valid': obs.valid,
                'source_age_s': (stamp-obs.stamp_ns)/1e9,
                'arrival_age_s': now-self.received[role], 'error': obs.error,
                'space_found': obs.space_found, 'line': vars(obs.line), 'space': vars(obs.space),
                'end': vars(obs.end), 'outline': vars(obs.outline)}
                for role, obs in self.observations.items()},
        }
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))

    def destroy_node(self):
        self.liveness.close()
        if rclpy.ok():
            self.cmd_pub.publish(MotionCommand())
        self.command_lease.close()
        super().destroy_node()


def main(args=None):
    run_controller(CameraParkingController, args)
