"""One camera/model process, preserving the exact image stamp end to end."""
from dataclasses import replace
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .configuration import MODEL_TOPIC, VISION_FIELDS, FRONT_VISION_FIELDS, REAR_VISION_FIELDS, TUNING_TOPIC, load_config
from .core import ControlConfig
from .model import SegmentModel
from .observations import Observation
from .parallel import PARALLEL_CLASSES, extract_parallel
from .perception import extract, orient_image


class ParkingPerceptionNode(Node):
    def __init__(self):
        super().__init__('camera_parking_perception')
        self.declare_parameter('config_file', '')
        self.declare_parameter('camera_role', 'front')
        self.declare_parameter('model_override', '')
        self.declare_parameter('parallel_model_override', '')
        data = load_config(self.get_parameter('config_file').value)
        self.role = self.get_parameter('camera_role').value
        if self.role not in ('front', 'rear'):
            raise ValueError('camera_role must be front/rear')
        self.cfg = dict(data[self.role])
        if self.get_parameter('model_override').value:
            self.cfg['model_path'] = self.get_parameter('model_override').value
        self.timeout = data['control'].get('observation_timeout_s', .5)
        self.tuning_revision = 0
        self.model_mode = 'perpendicular'
        self.model = SegmentModel(self.cfg)
        self.parallel_control = ControlConfig(**data['control'])
        self.parallel_bev = data['parallel'].get(self.role, {})
        self.parallel_cfg = dict(self.cfg, **self.parallel_bev)
        parallel_path = self.get_parameter('parallel_model_override').value or data['parallel']['model_path']
        self.parallel_model = None
        if parallel_path:
            self.parallel_cfg.update(model_path=parallel_path, classes={name: name for name in PARALLEL_CLASSES})
            self.parallel_model = SegmentModel(self.parallel_cfg)
            dummy = np.zeros((self.cfg['height'], self.cfg['width'], 3), np.uint8)
            self.parallel_model.predict(orient_image(dummy, self.cfg['rotate_deg'], self.cfg['flip_horizontal']))
        self.bridge = CvBridge()
        self.last_stamp = 0
        self.last_clock_ns = self.get_clock().now().nanoseconds
        root = '/parking_camera/' + self.role
        self.obs_pub = self.create_publisher(String, root + '/observation', 1)
        self.status_pub = self.create_publisher(String, root + '/perception_status', 1)
        self.views = {name: self.create_publisher(Image, root + '/' + name, qos_profile_sensor_data)
                      for name in ('oriented', 'segmentation', 'bev')}
        self.create_subscription(Image, self.cfg['image_topic'], self.on_image, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, TUNING_TOPIC, self.on_tuning, latched)
        self.create_subscription(String, MODEL_TOPIC, self.on_model, latched)
        self.get_logger().info(f'{self.role}: model classes {self.model.names}; input {self.cfg["image_topic"]}')

    def on_model(self, message):
        if message.data in ('perpendicular', 'parallel') and message.data != self.model_mode:
            self.model_mode = message.data
            self.get_logger().info(f'{self.role}: switched parking model to {self.model_mode}')

    def on_tuning(self, message):
        try:
            payload = json.loads(message.data)
            revision = payload['revision']
            if type(revision) is not int or revision <= self.tuning_revision:
                return
            config = payload['config']
            keys = VISION_FIELDS + (FRONT_VISION_FIELDS if self.role == 'front' else REAR_VISION_FIELDS)
            values = {key: config[self.role][key] for key in keys}
            self.timeout = config['control']['observation_timeout_s']
            self.parallel_control = ControlConfig(**config['control'])
            self.cfg.update(values)  # the model adapter shares this dictionary
            self.parallel_cfg.update(values)  # retain the parallel checkpoint and class map
            self.parallel_bev = config['parallel'].get(self.role, {})
            self.parallel_cfg.update(self.parallel_bev)
            self.tuning_revision = revision
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(f'Invalid camera tuning update: {exc}')

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
            if self.model_mode == 'parallel':
                if self.parallel_model is None:
                    raise RuntimeError('Parallel parking model is not configured')
                polygons = self.parallel_model.predict(frame)
                obs, segmentation, bev = extract_parallel(frame, polygons, self.parallel_cfg, self.parallel_control,
                                                          self.role, stamp, message.header.frame_id)
            else:
                polygons = self.model.predict(frame)
                obs, segmentation, bev = extract(frame, polygons, self.cfg, self.role, stamp, message.header.frame_id)
            age = (self.get_clock().now().nanoseconds-stamp)/1e9
            if not 0 <= age <= self.timeout or time.monotonic()-started > self.timeout:
                raise RuntimeError('Inference result exceeded source-age budget')
            for name, image in (('oriented', frame), ('segmentation', segmentation), ('bev', bev)):
                output = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
                output.header = message.header
                self.views[name].publish(output)
            self.status_pub.publish(String(data=f'OK model={self.model_mode} source_ns={stamp} age={age:.3f}s'))
        except Exception as exc:
            obs = Observation(stamp, self.role, message.header.frame_id, valid=False, error=str(exc),
                              model=self.model_mode)
            self.status_pub.publish(String(data='ERROR: ' + str(exc)))
            self.get_logger().error(f'{self.role} perception: {exc}')
        obs = replace(obs, tuning_revision=self.tuning_revision)
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
