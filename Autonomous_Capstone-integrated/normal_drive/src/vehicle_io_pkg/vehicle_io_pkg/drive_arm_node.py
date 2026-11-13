#!/usr/bin/env python3
"""Operator safety gate for autonomous track / mission driving.

A small OpenCV window is intentionally used instead of raw terminal stdin so
that key handling remains reliable when this node is started from ros2 launch.
Focus the window and press:
  W : ARM / start autonomous motion (only after calibration READY)
  S : DISARM / request all motor PWM off, leave ROS/camera/UI running
  X or SPACE : aliases for S
  Q : DISARM and close this UI node
"""

import cv2
import time
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
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
        # Only this gate converts operator keys into W authorization. Tuner
        # windows forward keys here so READY checking and logging are identical.
        # VOLATILE requests cannot replay a previous W to a restarted sender.
        event_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.request_pub = self.create_publisher(String, 'vehicle/arm_request', event_qos)
        self.heartbeat_pub = self.create_publisher(String, 'vehicle/ui_heartbeat', event_qos)
        self.create_subscription(String, 'vehicle/drive_key', self.on_forwarded_key, event_qos)
        self.create_subscription(String, 'vehicle/arm_challenge', self.on_challenge, qos)
        self.create_subscription(Bool, 'vehicle/drive_state', self.on_drive_state, qos)
        self.challenge = ''
        self._last_heartbeat = 0.0
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
        msg = Bool()
        msg.data = bool(value)
        self.arm_pub.publish(msg)
        if not value:
            self.armed = False

    def on_challenge(self, msg):
        if self.challenge != msg.data:
            self.armed = False
        self.challenge = msg.data

    def on_drive_state(self, msg):
        self.armed = bool(msg.data)

    def on_forwarded_key(self, msg):
        # A delayed W from the tuner must not adopt a newer challenge after S.
        if msg.data == f'w:{self.challenge}' and self.challenge:
            self.handle_key('w', source='tuning/debug window')
        elif len(msg.data) == 1 and msg.data.lower() in ('s', 'x', ' '):
            self.handle_key(msg.data.lower(), source='tuning/debug window')

    def on_ready(self, msg: Bool):
        self.ready = bool(msg.data)
        if not self.ready and self.armed:
            self.publish_arm(False)

    def on_status(self, msg: String):
        self.status = msg.data

    def handle_key(self, key: str, source='vehicle_start_gate'):
        key = key.lower()
        if key not in ('w', 's', 'x', ' ', 'q'):
            return
        self.get_logger().info(f'KEY received: {key!r} from {source}')
        if key == 'w':
            if not self.ready:
                self.get_logger().warn('W ignored: calibration is not READY yet')
                self.publish_arm(False)
            else:
                # Heartbeat must precede the W event even if the operator
                # presses immediately after a new challenge is received.
                self.heartbeat_pub.publish(String(data=self.challenge))
                self.publish_arm(True)
                self.request_pub.publish(String(data=self.challenge))
                self.get_logger().warn('W arm request sent; await sender ARM state log')
        elif key in ('s', 'x', ' '):
            self.publish_arm(False)
            self.get_logger().warn('S disarm request sent; motor PWM not confirmed')
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
        cv2.putText(canvas, 'W = start/arm    S = stop/disarm (X/SPACE also stop)',
                    (24, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.putText(canvas, 'Focus this window, Track Tuner or Track Debug View for W/S.',
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
        # Heartbeat follows the GUI event loop: a hung or closed UI cannot
        # keep a stale True authorization alive at the serial bridge.
        self.draw()
        if self.show_window and cv2.getWindowProperty(self.WINDOW, cv2.WND_PROP_VISIBLE) < 1:
            self.handle_key('q')
        now = time.monotonic()
        if not self.should_quit and self.challenge and now - self._last_heartbeat >= 0.1:
            self.heartbeat_pub.publish(String(data=self.challenge))
            self._last_heartbeat = now

    def destroy_node(self):
        try:
            if rclpy.ok(context=self.context):
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
