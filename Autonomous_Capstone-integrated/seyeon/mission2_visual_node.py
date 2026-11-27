"""One publisher: run 1125 perpendicular parking, then use image feedback."""

from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from interfaces_pkg.msg import MotionCommand
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from skku_track_drive_pkg.modes.perpendicular_camera.controller_node import CameraParkingController
from skku_track_drive_pkg.modes.perpendicular_camera.perception import orient_image
from mission2_vision import image_blocked, RearBayTracker
from mission2_visual_core import Evidence, VisualMission


class ImageInputs:
    def __init__(self, views):
        from ultralytics import YOLO

        path = Path(get_package_share_directory('skku_track_drive_pkg'), 'models/mission/best_m_s_curve.pt')
        if not path.is_file():
            raise ValueError(f'1125 reference-vehicle model was not installed: {path}')
        self.detector = YOLO(str(path))
        names = self.detector.names
        listed = dict(enumerate(names)) if isinstance(names, list) else names
        self.ids = [int(i) for i, name in listed.items() if name in {'obstacle', 'obstacles', 'car'}]
        if not self.ids:
            raise ValueError('1125 model lacks an obstacle/car detection class')
        self.views = [views['front'], views['rear']]
        self.previous = None
        self.tracker = RearBayTracker()

    def observe(self, pair_stamps, images, park_hold):
        oriented, snapshots = [], []
        for image, info in zip(images, self.views):
            source = orient_image(image, info['rotate_deg'], info['flip_horizontal'])
            h, w = source.shape[:2]
            source = cv2.resize(source, (640, max(1, round(h * 640/w))))
            oriented.append(source)
            snapshots.append(cv2.cvtColor(cv2.resize(source, (160, 90)), cv2.COLOR_BGR2GRAY))
        # A nearly blank frame cannot establish standstill.
        still = self.previous is not None and all(
            float(now.std()) >= 6. and float(cv2.absdiff(now, old).mean()) < 2.8
            for now, old in zip(snapshots, self.previous))
        self.previous = snapshots
        results = self.detector.predict(oriented, classes=self.ids, conf=.50,
                                        device=self.views[1]['model_device'], verbose=False)
        if len(results) != 2:
            raise ValueError('expected a detection result from each camera')
        front, rear = [result.boxes.xyxy.cpu().tolist() for result in results]
        if not all(np.isfinite(box).all() for box in (*front, *rear)):
            raise ValueError('invalid detector output')
        return Evidence(min(pair_stamps), self.tracker.observe(oriented[1], rear, self.views[1],
                                                               pair_stamps[1], park_hold and still),
                        image_blocked(front, oriented[0].shape, wide=True),
                        image_blocked(rear, oriented[1].shape), still)


class Mission2VisualNode(CameraParkingController):
    def __init__(self):
        super().__init__()
        self.bridge = CvBridge()
        self.frames = [None, None]
        self.submitted = (0, 0)
        self.ttl = min(.45, self.core.p.observation_timeout_s)
        self.pending = None
        self.evidence = None
        self.mission = None
        self.sources = ImageInputs(self.data)
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='parking_camera_inputs')
        self.keys = self.create_publisher(String, 'vehicle/drive_key', 10)
        self.flow_status = self.create_publisher(String, '/seyeon/mission2/status', 1)
        self.last_status = ''
        for i, role in enumerate(('front', 'rear')):
            self.create_subscription(Image, self.data[role]['image_topic'],
                                     lambda msg, index=i: self.on_image(msg, index), qos_profile_sensor_data)

    def send(self, steering=0, speed=0):
        output = self.core.command(steering, speed)
        self.cmd_pub.publish(MotionCommand(steering=output.steering, left_speed=output.left_speed,
                                           right_speed=output.right_speed))

    def on_image(self, msg, index):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        previous = self.frames[index]
        if stamp <= 0 or (previous is not None and stamp <= previous[0]):
            return
        try:
            self.frames[index] = stamp, self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as err:
            self.frames[index] = None
            self.evidence = None
            self.get_logger().warn(f'Camera conversion failed: {err}')

    def fresh(self, stamp):
        return 0 <= (self.get_clock().now().nanoseconds - stamp) / 1e9 <= self.ttl

    def observe(self):
        if self.pending is not None and self.pending.done():
            try:
                self.evidence = self.pending.result()
            except Exception as err:
                self.evidence = None
                self.get_logger().error(f'Mission 2 visual input failed: {err}')
            self.pending = None
        if any(frame is None for frame in self.frames):
            return None
        stamps, images = zip(*self.frames)
        if not all(self.fresh(stamp) for stamp in stamps):
            return None
        if self.pending is None and abs(stamps[0] - stamps[1]) / 1e9 <= .12 and all(
                new > old for new, old in zip(stamps, self.submitted)):
            self.submitted = stamps
            self.pending = self.pool.submit(self.sources.observe, stamps, images, self.mission.state == 'PARALLEL_HOLD')
        return self.evidence if self.evidence is not None and self.fresh(self.evidence.sequence) else None

    def on_timer(self):
        if self.mission is None and self.core.state != 'STOPPED':
            return super().on_timer()
        now = time.monotonic()
        if self.mission is None:
            self.mission = VisualMission(self.core.p, now)
        if not self.armed or self.capture_restart_pending():
            self.send()
            if self.mission.state == 'PERP_HOLD':
                self.mission.step(now, None)
            elif self.mission.state not in ('PARALLEL_DONE', 'FAULT_STOP'):
                self.mission.stop(now, 'OPERATOR_STOP_OR_CAMERA_CONFIG_CHANGED')
            return
        evidence = self.observe()
        move = self.mission.step(now, evidence)
        if move.pwm and (evidence is None or not self.fresh(evidence.sequence)):
            self.send()
        else:
            self.send(move.steer, move.pwm)
        status = f'{self.mission.state} {self.mission.reason}'
        if status != self.last_status:
            self.get_logger().info(status)
            if self.mission.state in ('FAULT_STOP', 'PARALLEL_DONE') and not self.preview:
                self.keys.publish(String(data='s'))
            self.last_status = status
        self.flow_status.publish(String(data=status))

    def destroy_node(self):
        self.pool.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = Mission2VisualNode()
        # If an image callback blocks, the same executor stops servicing the
        # controller heartbeat; the existing sender lease then expires.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
