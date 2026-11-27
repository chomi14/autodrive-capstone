"""Use the installed 1125 camera parking node, then take over its one publisher."""

from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
import skku_track_drive_pkg
from cv_bridge import CvBridge
from interfaces_pkg.msg import MotionCommand
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Keep the 1125 installed package first. Expose only the new Seyeon helper
# modules from the sibling checkout, without overlaying its older ROS nodes.
HELPERS = Path(__file__).resolve().parents[1] / 'src/skku_track_drive_pkg/skku_track_drive_pkg'
skku_track_drive_pkg.__path__.append(str(HELPERS))

from skku_track_drive_pkg.modes.perpendicular_camera.controller_node import CameraParkingController
from skku_track_drive_pkg.parallel_camera_live import LiveConfig, ParkingFollower, rectangle, scalar, vector
from mission2_core import Frame, Mission2, parallel_bays, perpendicular_bays


class RearVehicleDetector:
    def __init__(self, settings, camera, bays):
        from ultralytics import YOLO

        path = Path(settings['model_path']).expanduser()
        if not path.is_file() or path.suffix.lower() != '.pt':
            raise ValueError('provide a verified local .pt checkpoint for the reference vehicles')
        self.network = YOLO(str(path))
        names = self.network.names
        listed = dict(enumerate(names)) if isinstance(names, list) else names
        self.ids = [int(key) for key, name in listed.items() if name in settings['classes']]
        if not self.ids:
            raise ValueError('reference vehicle class is missing from the selected checkpoint')
        self.device = settings['device']
        self.confidence = scalar(settings['confidence'])
        self.margin = scalar(settings['bay_edge_margin_m'])
        if not 0 < self.confidence <= 1 or not 0 < self.margin < .425:
            raise ValueError('invalid rear detector confidence or bay edge margin')
        self.matrix = np.linalg.inv(camera.image_from_ground)
        self.hull = camera.hull
        self.bays = bays

    def occupied(self, image, pose):
        result = self.network.predict(image, classes=self.ids, device=self.device,
                                      conf=self.confidence, verbose=False)[0]
        occupied = []
        for x0, _, x1, y1 in result.boxes.xyxy.cpu().tolist():
            pixel = ((x0 + x1) / 2, y1)
            if cv2.pointPolygonTest(self.hull, pixel, False) < 0:
                return ()
            ground = self.matrix @ np.array([*pixel, 1.])
            if not np.isfinite(ground).all() or abs(ground[2]) < 1e-9:
                return ()
            x, y = pose.point(*(ground[:2] / ground[2]))
            match = [number for number, bay in self.bays.items()
                     if bay.xmin + self.margin <= x <= bay.xmax - self.margin
                     and bay.ymin + self.margin <= y <= bay.ymax - self.margin]
            if len(match) != 1 or match[0] in occupied:
                return ()
            occupied.append(match[0])
        return tuple(occupied) if len(occupied) == 2 else ()


class Mission2Node(CameraParkingController):
    def __init__(self):
        super().__init__()
        self.declare_parameter('mission2_config', '')
        path = Path(self.get_parameter('mission2_config').value)
        raw = json.loads(path.read_text(encoding='utf-8-sig'))
        params = raw['mission2']
        self.bays = parallel_bays(vector(params['parallel_bay1_clear_corner_m'], 2), params['parallel_index_direction'])
        self.perpendicular = perpendicular_bays(vector(params['perpendicular_bay1_clear_corner_m'], 2),
                                                params['perpendicular_index_direction'])
        self.fixed = tuple(map(rectangle, params['fixed_obstacles']))
        raw['slot'] = list(vars(self.bays[2]).values())
        raw['obstacles'] = [list(vars(b).values()) for b in (*self.fixed, self.bays[1], self.bays[4])]
        self.cfg = LiveConfig(raw)
        if not self.preview and not self.cfg.commissioned:
            raise ValueError('the mission 2 measurements have not been commissioned')
        for bay in (*self.bays.values(), *self.perpendicular.values()):
            if not all(map(self.cfg.scene.bounds.contains, ((bay.xmin, bay.ymin), (bay.xmax, bay.ymax)))):
                raise ValueError('every parking bay must lie within the measured mission bounds')
        self.bridge = CvBridge()
        self.images = [None, None]
        self.last_pair = (-1, -1)
        self.pose = self.cfg.start
        self.pose_stamp = 0
        self.flow = None
        self.selection_limit = scalar(params['exit_right_limit_s'])
        self.stable_frames = params['target_confirm_frames']
        if self.selection_limit <= 0 or type(self.stable_frames) is not int or self.stable_frames < 2:
            raise ValueError('positive right-exit limit and at least two distinct detector frames required')
        for index, role in enumerate(('front', 'rear')):
            self.create_subscription(Image, self.data[role]['image_topic'],
                                     lambda msg, i=index: self.on_image(msg, i), qos_profile_sensor_data)
        self.keys = self.create_publisher(String, 'vehicle/drive_key', 10)
        self.flow_status = self.create_publisher(String, '/seyeon/mission2/status', 1)
        self.detector = RearVehicleDetector(params['rear_detector'], self.cfg.cameras[1], self.bays)
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='mission2_rear')
        self.pending = None
        self.submitted = 0
        self.detected = (0, ())
        self.last_status = ''

    def publish_command(self, command=None):
        self.cmd_pub.publish(MotionCommand() if command is None else MotionCommand(
            steering=command.steering, left_speed=command.left_speed, right_speed=command.right_speed))

    def on_image(self, msg, index):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if stamp <= 0 or (self.images[index] is not None and stamp <= self.images[index][0]):
            return
        try:
            self.images[index] = stamp, self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            self.images[index] = None
            self.pose_stamp = 0

    def fresh(self, stamp):
        return stamp > 0 and 0 <= (self.get_clock().now().nanoseconds - stamp) / 1e9 <= self.cfg.frame_age

    def localize(self):
        if any(item is None for item in self.images):
            self.pose_stamp = 0
            return
        stamps, images = zip(*self.images)
        if not all(self.fresh(stamp) for stamp in stamps):
            self.pose_stamp = 0
            return
        if abs(stamps[0] - stamps[1]) / 1e9 > self.cfg.frame_skew or not all(
                current > old for current, old in zip(stamps, self.last_pair)):
            return
        self.last_pair = stamps
        try:
            self.pose = self.cfg.locate(images, self.pose)
            self.pose_stamp = min(stamps)
        except (ValueError, cv2.error):
            self.pose_stamp = 0

    def get_frame(self, detect):
        if not self.fresh(self.pose_stamp):
            return None
        stamp, occupants = self.detected
        if detect:
            if self.pending is not None and self.pending.done():
                try:
                    occupants = self.pending.result()
                except Exception as exc:
                    occupants = ()
                    self.get_logger().error(f'Rear reference detection failed: {exc}')
                stamp = self.submitted
                self.detected = stamp, occupants
                self.pending = None
            image_stamp, image = self.images[1]
            if (self.pending is None and image_stamp > self.submitted and image_stamp == self.last_pair[1]
                    and self.fresh(image_stamp)):
                self.submitted = image_stamp
                self.pending = self.worker.submit(self.detector.occupied, image, self.pose)
        # Count a model result once. Repeated timer ticks between results do
        # not confirm a target, and an expired result cannot authorize entry.
        if detect and self.fresh(stamp):
            return Frame(stamp, self.pose, occupants)
        return Frame(0, self.pose)

    def on_timer(self):
        self.localize()
        if self.flow is None and self.core.state != 'STOPPED':
            return super().on_timer()
        if self.flow is None:
            self.flow = Mission2(self.cfg, ParkingFollower(self.cfg), self.bays, self.perpendicular, self.fixed,
                                 min(self.core.p.exit_pwm, self.cfg.forward_pwm), self.selection_limit,
                                 self.stable_frames, time.monotonic())
        if not self.armed or self.capture_restart_pending():
            self.publish_command()
            if self.flow.state not in ('PERP_HOLD', 'FAULT_STOP', 'PARALLEL_DONE'):
                self.flow.stop('OPERATOR_STOP_OR_CONFIG_CHANGED')
            elif self.flow.state == 'PERP_HOLD':
                self.flow.step(time.monotonic(), None)
            return
        try:
            detect = self.flow.state in ('EXIT_RIGHT', 'ENTRY_PAUSE')
            frame = self.get_frame(detect)
            command = self.flow.step(time.monotonic(), frame)
            if self.flow.state not in ('PERP_HOLD', 'PARALLEL_DONE') and not self.fresh(self.pose_stamp):
                command = self.flow.stop('POSE_EXPIRED_DURING_CONTROL')
        except (ValueError, cv2.error) as exc:
            command = self.flow.stop(str(exc))
        self.publish_command(command)
        status = (f'{self.flow.state} {self.flow.reason} '
                  f'perpendicular={self.flow.perp_target} parallel={self.flow.confirmed_target}')
        if status != self.last_status:
            self.get_logger().info(status)
            if self.flow.state in ('FAULT_STOP', 'PARALLEL_DONE') and not self.preview:
                self.keys.publish(String(data='s'))
            self.last_status = status
        self.flow_status.publish(String(data=status))

    def destroy_node(self):
        self.worker.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = Mission2Node()
        # One executor also services the controller heartbeat. A blocked image
        # callback cannot keep lease renewal running alongside stale movement.
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
