"""One camera/model process, preserving the exact image stamp end to end."""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .configuration import load_config
from .model import SegmentModel
from .observations import Observation
from .perception import extract, orient_image


class ParkingPerceptionNode(Node):
    def __init__(self):
        super().__init__('camera_parking_perception')
        self.declare_parameter('config_file', '')
        self.declare_parameter('camera_role', 'front')
        self.declare_parameter('model_override', '')
        data = load_config(self.get_parameter('config_file').value)
        self.role = self.get_parameter('camera_role').value
        if self.role not in ('front', 'rear'):
            raise ValueError('camera_role must be front/rear')
        self.cfg = dict(data[self.role])
        if self.get_parameter('model_override').value:
            self.cfg['model_path'] = self.get_parameter('model_override').value
        self.timeout = data['control'].get('observation_timeout_s', .5)
        self.model = SegmentModel(self.cfg)
        self.bridge = CvBridge()
        self.last_stamp = 0
        self.last_clock_ns = self.get_clock().now().nanoseconds
        root = '/parking_camera/' + self.role
        self.obs_pub = self.create_publisher(String, root + '/observation', 1)
        self.status_pub = self.create_publisher(String, root + '/perception_status', 1)
        self.views = {name: self.create_publisher(Image, root + '/' + name, qos_profile_sensor_data)
                      for name in ('oriented', 'segmentation', 'bev')}
        self.create_subscription(Image, self.cfg['image_topic'], self.on_image, qos_profile_sensor_data)
        self.get_logger().info(f'{self.role}: model classes {self.model.names}; input {self.cfg["image_topic"]}')

    def on_image(self, message):
        stamp = message.header.stamp.sec*1000000000 + message.header.stamp.nanosec
        now = self.get_clock().now().nanoseconds
        if now < self.last_clock_ns:
            # Explicit replay may seek/loop. The controller independently enters
            # FAULT_STOP on clock reversal; R can then use the new timeline.
            self.last_stamp = 0
        self.last_clock_ns = now
        if stamp <= self.last_stamp or stamp <= 0 or not 0 <= (now-stamp)/1e9 <= self.timeout:
            self.status_pub.publish(String(data='REJECTED_SOURCE_STAMP_OR_AGE'))
            return
        self.last_stamp = stamp
        started = time.monotonic()
        try:
            frame = self.bridge.imgmsg_to_cv2(message, 'bgr8')
            frame = orient_image(frame, self.cfg['rotate_deg'], self.cfg['flip_horizontal'])
            # Inference and geometry use THIS same array: no approximate joining
            # of an old detection with a newer image, and no synthetic restamp.
            polygons = self.model.predict(frame)
            obs, segmentation, bev = extract(frame, polygons, self.cfg, self.role, stamp, message.header.frame_id)
            age = (self.get_clock().now().nanoseconds-stamp)/1e9
            if not 0 <= age <= self.timeout or time.monotonic()-started > self.timeout:
                raise RuntimeError('Inference result exceeded source-age budget')
            for name, image in (('oriented', frame), ('segmentation', segmentation), ('bev', bev)):
                output = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
                output.header = message.header
                self.views[name].publish(output)
            self.status_pub.publish(String(data=f'OK source_ns={stamp} age={age:.3f}s'))
        except Exception as exc:
            obs = Observation(stamp, self.role, message.header.frame_id, valid=False, error=str(exc))
            self.status_pub.publish(String(data='ERROR: ' + str(exc)))
            self.get_logger().error(f'{self.role} perception: {exc}')
        self.obs_pub.publish(String(data=obs.to_json()))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ParkingPerceptionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
