#!/usr/bin/env python3

"""Bound contest motion commands before they reach the vehicle I/O gate."""

import time

import rclpy
from interfaces_pkg.msg import MotionCommand
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class MotionCommandGuard(Node):
    def __init__(self):
        super().__init__('contest_motion_command_guard')

        self.declare_parameter('input_topic', 'motion_command')
        self.declare_parameter('output_topic', 'topic_control_signal')
        self.declare_parameter('max_steering', 7)
        self.declare_parameter('max_speed', 100)
        self.declare_parameter('steering_sign', 1)
        self.declare_parameter('command_timeout', 0.5)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.max_steering = abs(int(self.get_parameter('max_steering').value))
        self.max_speed = abs(int(self.get_parameter('max_speed').value))
        self.steering_sign = -1 if int(self.get_parameter('steering_sign').value) < 0 else 1
        self.command_timeout = max(0.1, float(self.get_parameter('command_timeout').value))

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.publisher = self.create_publisher(MotionCommand, output_topic, qos)
        self.subscription = self.create_subscription(
            MotionCommand, input_topic, self.on_command, qos
        )
        self.last_command_time = time.monotonic()
        self.timeout_stop_sent = False
        self.timer = self.create_timer(0.05, self.on_watchdog)

        self.get_logger().info(
            f'Guarding {input_topic} -> {output_topic}: steering '
            f'+/-{self.max_steering}, speed +/-{self.max_speed}, '
            f'steering_sign={self.steering_sign}'
        )

    @staticmethod
    def _clamp(value: int, limit: int) -> int:
        return max(-limit, min(int(value), limit))

    def publish_stop(self):
        self.publisher.publish(MotionCommand(steering=0, left_speed=0, right_speed=0))

    def on_command(self, msg: MotionCommand):
        guarded = MotionCommand()
        guarded.steering = self._clamp(
            int(msg.steering) * self.steering_sign, self.max_steering
        )
        guarded.left_speed = self._clamp(msg.left_speed, self.max_speed)
        guarded.right_speed = self._clamp(msg.right_speed, self.max_speed)
        self.publisher.publish(guarded)
        self.last_command_time = time.monotonic()
        self.timeout_stop_sent = False

    def on_watchdog(self):
        if self.timeout_stop_sent:
            return
        if time.monotonic() - self.last_command_time > self.command_timeout:
            self.publish_stop()
            self.timeout_stop_sent = True
            self.get_logger().warning('Motion command timeout: published STOP')

    def destroy_node(self):
        self.publish_stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotionCommandGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
