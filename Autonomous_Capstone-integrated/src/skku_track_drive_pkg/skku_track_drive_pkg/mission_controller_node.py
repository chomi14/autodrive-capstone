"""Reuse TrackController's YOLO -> lane -> path -> Stanley pipeline."""
import json
import time

import cv2
import numpy as np
from std_msgs.msg import String

from .track_controller_node import TrackControllerNode
from .mission_core import MissionCore
from .mode_parameters import MISSION, limits
from .controller_liveness import run_controller


class MissionControllerNode(TrackControllerNode):
    def __init__(self):
        super().__init__('mission_controller_node')
        self.create_timer(0.1, self.check_camera_freshness, callback_group=self.liveness.group)

    def declare_mode_parameters(self):
        for name, (default, *_rest) in MISSION.items():
            self.declare_parameter(name, default)

    def initialize_mode(self):
        required = {'lane1', 'lane2', 'obstacle', 'traffic_light'}
        names = set(self.yolo.yolo.names.values())
        missing = required - names
        if missing or self.yolo.yolo.task != 'segment':
            raise RuntimeError(f'Mission requires segmentation classes {sorted(required)}; missing={sorted(missing)}')
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
        # Held paths remain useful for track driving, but cannot prove that an
        # avoidance lane is currently visible and clear.
        valid = len(points) >= 2 and self.lane.missed_frames == 0
        stop, limit = self.mission.decide(detections, frame, camera_path, self.last_mission_frame, valid)
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
            cv2.putText(marked, f"path={evidence['blocks_path']} alt={evidence['blocks_alternate']}",
                        (x1, max(15, y1-5)), cv2.FONT_HERSHEY_SIMPLEX, .4, color, 1)
        for evidence in status.get('traffic_boxes', []):
            x1, y1, x2, y2 = map(int, evidence['box'])
            cv2.rectangle(marked, (x1, y1), (x2, y2), (255, 255, 0), 2)
            label = evidence.get('color', evidence.get('rejection', 'Unknown'))
            ratios = evidence.get('ratios', {})
            text = f"{label} used={evidence.get('used_for_color')} R/G/Y=" + '/'.join(f"{ratios.get(k, 0):.2f}" for k in ('Red', 'Green', 'Yellow'))
            cv2.putText(marked, text, (max(0, x1), max(15, y1-5)), cv2.FONT_HERSHEY_SIMPLEX, .4, (255, 255, 0), 1)
        debug = super()._make_debug(marked, detections, *args)
        status = self.mission.status
        cv2.putText(debug, f"MISSION {status.get('state')} lane{self.mission.target_lane} light={status.get('traffic_color')}",
                    (12, 25), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 255, 255), 2)
        return debug


def main(args=None):
    run_controller(MissionControllerNode, args)
