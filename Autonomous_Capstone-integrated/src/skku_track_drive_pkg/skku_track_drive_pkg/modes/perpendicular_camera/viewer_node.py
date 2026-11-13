"""Camera parking UI: six image panels, status, W/S/R/P and tuner lease."""
import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class CameraParkingViewer(Node):
    WINDOW = 'Camera perpendicular parking | W S R P'

    def __init__(self):
        super().__init__('perpendicular_camera_viewer')
        self.declare_parameter('preview', True)
        self.preview = self.get_parameter('preview').value
        self.bridge = CvBridge()
        self.images = {}
        self.status = {}
        self.challenge = ''
        self.closed = False
        self.last_heartbeat = 0.0
        self.preview_pub = self.create_publisher(Bool, '/parking_camera/preview_enabled', 1)
        self.key_pub = self.create_publisher(String, 'vehicle/drive_key', 1)
        self.heartbeat_pub = self.create_publisher(String, 'vehicle/tuner_heartbeat', 1)
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'vehicle/arm_challenge', self.on_challenge, state_qos)
        self.create_subscription(String, '/parking_camera/status', self.on_status, 1)
        for role in ('front', 'rear'):
            for view in ('oriented', 'segmentation', 'bev'):
                key = role + '/' + view
                self.create_subscription(Image, '/parking_camera/' + key,
                                         lambda msg, key=key: self.on_image(key, msg), qos_profile_sensor_data)
        self.reset_client = self.create_client(Trigger, '/perpendicular_camera_controller/reset')
        self.save_client = self.create_client(Trigger, '/perpendicular_camera_controller/save')
        cv2.namedWindow(self.WINDOW, cv2.WINDOW_NORMAL)
        self.timer = self.create_timer(.05, self.tick)

    def on_challenge(self, message):
        self.challenge = message.data

    def on_status(self, message):
        self.status = json.loads(message.data)

    def on_image(self, key, message):
        self.images[key] = (self.bridge.imgmsg_to_cv2(message, 'bgr8'), time.monotonic(),
                            message.header.stamp.sec*1000000000 + message.header.stamp.nanosec)

    def handle_key(self, key):
        if key in ('w', 's', 'x', ' ', 'q'):
            if self.preview:
                self.preview_pub.publish(Bool(data=key == 'w'))
            elif key == 'w':
                self.heartbeat_pub.publish(String(data=self.challenge))
                self.key_pub.publish(String(data='w:' + self.challenge))
            else:
                self.key_pub.publish(String(data='s'))
        if key in ('r', 'p'):
            client = self.reset_client if key == 'r' else self.save_client
            if client.service_is_ready():
                future = client.call_async(Trigger.Request())
                future.add_done_callback(lambda f: self.get_logger().info(str(f.result())))
        if key == 'q':
            self.closed = True

    def tick(self):
        canvas = np.zeros((760, 1200, 3), np.uint8)
        for row, role in enumerate(('front', 'rear')):
            for col, view in enumerate(('oriented', 'segmentation', 'bev')):
                key = role + '/' + view
                panel = np.zeros((250, 400, 3), np.uint8)
                label = key + ' WAITING'
                if key in self.images:
                    image, arrived, stamp = self.images[key]
                    panel = cv2.resize(image, (400, 250))
                    age = time.monotonic()-arrived
                    label = f'{key} age={age:.2f}s src={stamp}'
                    if age > .5:
                        label = 'STALE ' + label
                cv2.putText(panel, label, (5, 18), cv2.FONT_HERSHEY_SIMPLEX, .40, (0, 0, 255), 1)
                canvas[row*250:(row+1)*250, col*400:(col+1)*400] = panel
        s = self.status
        lines = [
            f"{'PREVIEW - NO MOTOR OUTPUT' if self.preview else 'LIVE'} {s.get('state', 'WAITING')} {s.get('reason', '')}",
            f"steer={s.get('steering')} PWM={s.get('pwm')} armed={s.get('armed')} calibrated={s.get('calibration_confirmed')}",
            f"errors={s.get('errors')} counts seen/missing/align={s.get('seen_count')}/{s.get('missing_count')}/{s.get('align_count')}",
            str(s.get('transition', '')), str(s.get('fault', '')),
            'W=start/resume  S=stop  R=reset while stopped  P=save while stopped  Q=close',
            'No rear obstacle sensor is implied. Camera/BEV values require commissioning.',
        ]
        for i, line in enumerate(lines):
            cv2.putText(canvas, line[:145], (12, 535+i*28), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1)
        cv2.imshow(self.WINDOW, canvas)
        key = cv2.waitKeyEx(1)
        if key >= 0:
            self.handle_key(chr(key & 255).lower())
        if cv2.getWindowProperty(self.WINDOW, cv2.WND_PROP_VISIBLE) < 1:
            self.handle_key('q')
        now = time.monotonic()
        if not self.closed and self.challenge and now-self.last_heartbeat >= .1:
            self.heartbeat_pub.publish(String(data=self.challenge))
            self.last_heartbeat = now


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraParkingViewer()
        while rclpy.ok() and not node.closed:
            rclpy.spin_once(node, timeout_sec=.05)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.handle_key('s')
            node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()
