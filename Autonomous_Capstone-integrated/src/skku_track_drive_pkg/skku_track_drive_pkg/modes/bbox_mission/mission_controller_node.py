"""Reuse TrackController's YOLO -> lane -> path -> Stanley pipeline."""
import json
import time

import cv2
import numpy as np
from std_msgs.msg import String
from std_srvs.srv import Trigger

from skku_track_drive_pkg.track_controller_node import TrackControllerNode
from .mission_core import MissionCore
from skku_track_drive_pkg.mode_parameters import MISSION, limits
from skku_track_drive_pkg.messages import DetectionArray
from skku_track_drive_pkg.controller_liveness import run_controller
from skku_track_drive_pkg.traffic_light_display import add_traffic_light_panel, describe_traffic_boxes


class MissionControllerNode(TrackControllerNode):
    def __init__(self):
        super().__init__('mission_controller_node')
        self.create_timer(0.1, self.check_camera_freshness, callback_group=self.liveness.group)
        if isinstance(self.mission, MissionCore):
            self.create_service(Trigger, '~/toggle_target_lane', self.on_toggle_target_lane)

    def on_toggle_target_lane(self, _request, response):
        with self._parameter_lock:
            target = self.mission.toggle_target_lane(time.monotonic())
            self.status_pub.publish(String(data=json.dumps(self.mission.status)))
        response.success = True
        response.message = f'Target lane{target}; automatic avoidance cooldown restarted for 5s'
        self.get_logger().info(response.message)
        return response

    def declare_mode_parameters(self):
        for name, (default, *_rest) in MISSION.items():
            self.declare_parameter(name, default)

    def initialize_mode(self):
        required = {'lane1', 'lane2', 'traffic_light'}
        names = set(self.yolo.yolo.names.values())
        missing = required - names
        if missing or not names.intersection({'obstacle', 'obstacles'}) or self.yolo.yolo.task != 'segment':
            raise RuntimeError(f'Mission requires segmentation classes {sorted(required)} and obstacle/obstacles; missing={sorted(missing)}')
        self.mission = MissionCore({name: self.get_parameter(name).value for name in MISSION})
        self.status_pub = self.create_publisher(String, '/mission/status', 1)
        self._processed_lane = 2
        self._mission_lane_valid = False
        self.last_mission_frame = time.monotonic()
        self.get_logger().info(f'Mission model classes verified: {sorted(names)}')

    @staticmethod
    def _tuning_limits():
        return {**TrackControllerNode._tuning_limits(), **limits(MISSION)}

    def on_tuning_parameters(self, parameters):
        merged = dict(self.mission.p)
        merged.update({p.name: p.value for p in parameters if p.name in MISSION})
        if merged['green_h_min'] > merged['green_h_max'] or merged['yellow_h_min'] > merged['yellow_h_max']:
            from rcl_interfaces.msg import SetParametersResult
            return SetParametersResult(successful=False, reason='HSV minimum must not exceed maximum')
        return super().on_tuning_parameters(parameters)

    def _apply_tuning_values(self, updates):
        super()._apply_tuning_values(updates)
        self.mission.p.update({key: value for key, value in updates.items() if key in MISSION})
        if updates.get('wait_for_green') == 1:
            self.mission.red_latched = True

    def prepare_lane(self, detections, frame):
        target = self.mission.target_lane
        self.lane.lane_class_name = f'lane{target}'
        if self._processed_lane != target:
            # Never use lane2's held target points as lane1's current path.
            self.lane.smoothed_targets = self.lane.smoothed_target_ys = None
            self.lane.smoothed_fit = self.lane.smoothed_slope = None
            self.lane.missed_frames = 0
            self.motion.path_data = None
            self._processed_lane = target

    def adjust_command(self, command, detections, path, frame):
        self.last_mission_frame = time.monotonic()
        h, w = frame.shape[:2]
        source = np.float32(self.lane._src_mat())
        destination = np.float32([[round(w * .3), 0], [round(w * .7), 0],
                                  [round(w * .7), h], [round(w * .3), h]])
        inverse = cv2.getPerspectiveTransform(destination, source)
        points = np.asarray([(x, y + self.lane.roi_cut) for x, y in zip(path.x_points, path.y_points)], dtype=np.float32)
        camera_path = cv2.perspectiveTransform(points.reshape(1, -1, 2), inverse)[0].tolist() if len(points) >= 2 else []
        # Path validity is diagnostic. Obstacle switching uses the full-frame
        # selected lane mask and never pauses for an unavailable path.
        valid = len(points) >= 2 and self.lane.missed_frames == 0
        stop, limit = self.mission.decide(detections, frame, camera_path, self.last_mission_frame, valid)
        self.mission.status['traffic_boxes'] = describe_traffic_boxes(frame, self.mission.status, self.mission.p)
        self.mission.status['traffic_limits'] = {
            name: self.mission.p[name] for name in (
                'hsv_s_min', 'hsv_v_min', 'traffic_min_color_ratio',
                'traffic_min_width', 'traffic_max_bottom_y')
        }
        self.status_pub.publish(String(data=json.dumps(self.mission.status)))
        return self.mission.apply(command, stop, limit)

    def check_camera_freshness(self):
        if time.monotonic() - self.last_mission_frame > self.mission.p['image_timeout_s']:
            # Diagnostic image age, NOT controller process health. Preserve a
            # last mission STOP just as faithfully as a last driving command.
            self.status_pub.publish(String(data=json.dumps({**self.mission.status,
                'system_state': 'IMAGE_DELAY_HOLD',
                'image_age_s': round(time.monotonic() - self.last_mission_frame, 3)})))

    def publish_perception_fallback(self):
        # A mission exception must never bypass a latched red or obstruction.
        self.publish_stop()

    @staticmethod
    def _draw_detection(frame, detections):
        # Traffic boxes get their own matching TL# labels after the base image.
        other = DetectionArray([d for d in detections.detections if d.class_name != 'traffic_light'])
        TrackControllerNode._draw_detection(frame, other)

    def _make_debug(self, frame, detections, *args):
        marked = frame.copy()
        status = self.mission.status
        path = np.asarray(status.get('camera_path', []), np.int32)
        if len(path) >= 2:
            cv2.polylines(marked, [path], False, (255, 0, 255), 3)
        for evidence in status.get('obstacles', []):
            x1, y1, x2, y2 = map(int, evidence['box'])
            color = (0, 0, 255) if evidence['blocks_path'] else (0, 165, 255)
            cv2.rectangle(marked, (x1, y1), (x2, y2), color, 2)
            size_text = f" near={evidence.get('near')}"
            cv2.putText(marked, f"path={evidence['blocks_path']} alt={evidence['blocks_alternate']}" + size_text,
                        (x1, max(15, y1-5)), cv2.FONT_HERSHEY_SIMPLEX, .4, color, 1)
        debug = super()._make_debug(marked, detections, *args)
        # The parent adds a header above the untouched camera coordinates.
        camera_y = debug.shape[0] - frame.shape[0]
        for index, evidence in enumerate(status.get('traffic_boxes', [])):
            x1, y1, x2, y2 = map(int, evidence['box'])
            color = (255, 255, 0) if evidence.get('eligible') else (128, 128, 128)
            cv2.rectangle(debug, (x1, y1+camera_y), (x2, y2+camera_y), color, 2)
            cv2.putText(debug, f'TL#{index}', (max(0, x1), max(camera_y+15, y1+camera_y-5)),
                        cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1, cv2.LINE_AA)
            if 'sample_xy' in evidence:
                sx, sy = evidence['sample_xy']
                cv2.drawMarker(debug, (sx, sy+camera_y), color, cv2.MARKER_CROSS, 7, 1)
        return add_traffic_light_panel(debug, status, camera_frame=frame)


def main(args=None):
    run_controller(MissionControllerNode, args)
