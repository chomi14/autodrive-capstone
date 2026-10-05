"""Manual PWM/steering measurement through the existing W/S serial gate."""
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool, String
from interfaces_pkg.msg import MotionCommand

from .command_lease import CommandLease
from .controller_liveness import ControllerLiveness, run_controller
from .mode_parameters import trackbars
from .track_tuner_node import TrackTunerNode


CALIBRATION = {'pwm': (0, -255, 255, 1), 'steering_step': (0, -7, 7, 1)}


class CalibrationControllerNode(Node):
    def __init__(self):
        super().__init__('parking_calibration_controller_node')
        self.declare_parameter('cmd_topic', '/parking_calibration/command')
        self.declare_parameter('loaded_tuning_config', '')
        self.declare_parameter('publish_debug', False)
        self.declare_parameter('publish_bev_debug', False)
        for name, spec in CALIBRATION.items():
            self.declare_parameter(name, spec[0])
        self.settings = {name: self.get_parameter(name).value for name in CALIBRATION}
        error = self.validate(self.settings)
        if error:
            raise RuntimeError(error)
        self.armed = False
        self.started_at = None
        self.elapsed = 0.0
        self.command_lease = CommandLease(str(self.get_parameter('cmd_topic').value), self.get_name())
        self.cmd_pub = self.create_publisher(MotionCommand, str(self.get_parameter('cmd_topic').value), 1)
        self.status_pub = self.create_publisher(String, '/parking_calibration/status', 1)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, 'vehicle/drive_state', self.on_armed, qos)
        self.add_on_set_parameters_callback(self.on_parameters)
        self.create_timer(.05, self.tick)
        self.liveness = ControllerLiveness(self)

    @staticmethod
    def validate(settings):
        for name, value in settings.items():
            if type(value) is not int or not CALIBRATION[name][1] <= value <= CALIBRATION[name][2]:
                return f'{name} must be an integer in {CALIBRATION[name][1:3]}'
        return ''

    def on_parameters(self, parameters):
        updates = {}
        for parameter in parameters:
            if parameter.name in ('publish_debug', 'publish_bev_debug'):
                if type(parameter.value) is not bool:
                    return SetParametersResult(successful=False, reason='display settings must be bool')
            elif parameter.name in CALIBRATION:
                updates[parameter.name] = parameter.value
            else:
                return SetParametersResult(successful=False, reason='requires restart')
        if self.armed and updates:
            return SetParametersResult(successful=False, reason='S before changing calibration PWM/steering')
        error = self.validate({**self.settings, **updates})
        if error:
            return SetParametersResult(successful=False, reason=error)
        self.settings.update(updates)
        return SetParametersResult(successful=True)

    def on_armed(self, message):
        if message.data and not self.armed:
            self.started_at = time.monotonic()
            self.elapsed = 0.0
        elif self.armed and not message.data:
            self.elapsed = time.monotonic() - self.started_at
        self.armed = bool(message.data)

    def command(self):
        if not self.armed:
            return MotionCommand()
        return MotionCommand(steering=self.settings['steering_step'], left_speed=self.settings['pwm'], right_speed=self.settings['pwm'])

    def tick(self):
        processing_started = time.monotonic()
        command = self.command()
        self.cmd_pub.publish(command)
        elapsed = time.monotonic() - self.started_at if self.armed else self.elapsed
        self.status_pub.publish(String(data=json.dumps({'state': 'MANUAL_MEASUREMENT' if self.armed else 'DISARMED',
            'processing_ros_ns': self.get_clock().now().nanoseconds, 'frame_stamp_ns': None,
            'steering': command.steering, 'left_pwm': command.left_speed, 'right_pwm': command.right_speed,
            'pipeline_duration_s': time.monotonic() - processing_started,
            'pwm': self.settings['pwm'], 'steering_step': self.settings['steering_step'],
            'armed': self.armed, 'elapsed_s': round(elapsed, 3),
            'note': 'Time is arm-state duration; measure actual travel/time independently.'})))

    def destroy_node(self):
        self.liveness.close()
        if rclpy.ok():
            self.cmd_pub.publish(MotionCommand())
        self.command_lease.close()
        super().destroy_node()


def controller_main(args=None):
    run_controller(CalibrationControllerNode, args)


def tuner_main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrackTunerNode(node_name='parking_calibration_tuner_node', extra_trackbars=trackbars(CALIBRATION),
            parameter_root='parking_calibration_controller_node', window_prefix='Calibration', only_extra_controls=True)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
