#!/usr/bin/env python3

import sys
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from interfaces_pkg.msg import MotionCommand
import serial

class ControlNode(Node):
    def __init__(self):
        super().__init__('control')

        # serial port
        self.ser = serial.Serial('/dev/ttyACM0', 9600)
        self.get_logger().info("Connected to Arduino on /dev/ttyACM0 @9600")

        # 토글 플래그: True면 모션 전송, False면 제로 전송
        self.send_motion = False

        # 구독
        self.create_subscription(
            MotionCommand,
            'motion_command',
            self.motion_command_callback,
            10
        )

        # 스페이스바로 send_motion 토글
        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()

    def motion_command_callback(self, msg: MotionCommand):
        # 모드에 따라 즉시 전송
        if self.send_motion:
            cmd = f"s{msg.steering}r{msg.right_speed}l{msg.left_speed}\n"
        else:
            cmd = "s0r0l0\n"
        self.ser.write(cmd.encode())
        self.get_logger().info(f"Sent: {cmd.strip()}")

    def _keyboard_loop(self):
        """stdin에서 스페이스바를 감지해 모드를 토글."""
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while rclpy.ok():
                ch = sys.stdin.read(1)
                if ch == ' ':
                    self.send_motion = not self.send_motion
                    mode = "MOTION" if self.send_motion else "ZERO"
                    self.get_logger().info(f"Toggled to {mode} mode")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()