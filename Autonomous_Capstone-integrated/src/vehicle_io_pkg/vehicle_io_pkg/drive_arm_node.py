#!/usr/bin/env python3
"""Operator safety gate for autonomous track / mission driving.

A small OpenCV window is intentionally used instead of raw terminal stdin so
that key handling remains reliable when this node is started from ros2 launch.
Focus the window and press:
  W : ARM / start autonomous motion (only after calibration READY)
  X or SPACE : DISARM / immediate stop gate
  Q : DISARM and close this UI node
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool, String


class DriveArmNode(Node):
    WINDOW = 'vehicle_start_gate'

    def __init__(self):
        super().__init__('drive_arm_node')
        self.declare_parameter('arm_topic', 'vehicle/armed')
        self.declare_parameter('ready_topic', 'vehicle/calibration_ready')
        self.declare_parameter('status_topic', 'vehicle/calibration_status')
        self.declare_parameter('show_window', True)

        self.arm_topic = str(self.get_parameter('arm_topic').value)
        self.ready_topic = str(self.get_parameter('ready_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.show_window = bool(self.get_parameter('show_window').value)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.arm_pub = self.create_publisher(Bool, self.arm_topic, qos)
        self.create_subscription(Bool, self.ready_topic, self.on_ready, qos)
        self.create_subscription(String, self.status_topic, self.on_status, qos)

        self.ready = False
        self.armed = False
        self.status = 'Waiting for Arduino calibration...'
        self.should_quit = False

        if self.show_window:
            cv2.namedWindow(self.WINDOW, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.WINDOW, 720, 260)

        self.publish_arm(False)
        self.timer = self.create_timer(0.04, self.on_timer)

    def publish_arm(self, value: bool):
        self.armed = bool(value)
        msg = Bool()
        msg.data = self.armed
        self.arm_pub.publish(msg)

    def on_ready(self, msg: Bool):
        self.ready = bool(msg.data)
        if not self.ready and self.armed:
            self.publish_arm(False)

    def on_status(self, msg: String):
        self.status = msg.data

    def handle_key(self, key: str):
        if key == 'w':
            if not self.ready:
                self.get_logger().warn('W ignored: calibration is not READY yet')
                self.publish_arm(False)
            else:
                self.publish_arm(True)
                self.get_logger().warn('ARMED by W - autonomous motion enabled')
        elif key in ('x', ' '):
            self.publish_arm(False)
            self.get_logger().warn('DISARMED - motion stopped')
        elif key == 'q':
            self.publish_arm(False)
            self.should_quit = True

    def draw(self):
        if not self.show_window:
            return

        canvas = np.zeros((260, 720, 3), dtype=np.uint8)
        if not self.ready:
            headline = 'CALIBRATING / LOCKED'
            headline_color = (0, 0, 255)
        elif self.armed:
            headline = 'ARMED - VEHICLE MAY MOVE'
            headline_color = (0, 200, 0)
        else:
            headline = 'READY - PRESS W TO START'
            headline_color = (0, 220, 220)

        cv2.putText(canvas, headline, (24, 55), cv2.FONT_HERSHEY_SIMPLEX,
                    0.95, headline_color, 2)
        cv2.putText(canvas, 'W = start/arm    X or SPACE = stop/disarm',
                    (24, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.putText(canvas, 'Focus this window before pressing keys.',
                    (24, 145), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1)

        # Keep status readable without trying to render a long line beyond the UI.
        status = self.status[:92]
        cv2.putText(canvas, status, (24, 205), cv2.FONT_HERSHEY_SIMPLEX,
                    0.46, (200, 200, 200), 1)

        cv2.imshow(self.WINDOW, canvas)
        keycode = cv2.waitKeyEx(1)
        if keycode >= 0:
            self.handle_key(chr(keycode & 0xFF).lower())

    def on_timer(self):
        self.draw()

    def destroy_node(self):
        try:
            self.publish_arm(False)
            cv2.destroyAllWindows()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DriveArmNode()
    try:
        while rclpy.ok() and not node.should_quit:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
