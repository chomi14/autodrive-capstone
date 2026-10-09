"""Pure mission fusion logic, independently testable without ROS."""
from . import lidar_camera_fusion as cfg
from .messages import DetectionArray
from .mission_core import MissionCore, path_blocked, box_xyxy

class FusionCore(MissionCore):
    def __init__(self, base, owner):
        super().__init__(base.p)
        self.owner = owner

    def decide(self, detections, frame, camera_path, now, lane_valid=True):
        owner = self.owner
        p = owner.fusion_settings
        reason, xy = owner.scan_for_image()
        entries, retained = [], []
        forced_stop = bool(reason)
        geometry = {**self.p, 'obstacle_near_y': 0}
        for det in detections.detections:
            if det.class_name != 'obstacle':
                retained.append(det)
                continue
            blocks = path_blocked(det, camera_path, geometry, frame.shape)
            match = cfg.associate(det, xy, p, frame.shape[1]) if not reason else {'distance_m': None, 'points': 0, 'reason': reason}
            distance = match['distance_m']
            if blocks and distance is None:
                forced_stop = True
            if blocks and distance is not None and distance <= p['stop_m']:
                forced_stop = True
                match['reason'] = 'TOO_CLOSE'
            # Keep close obstacles on either lane for alternate-lane assessment.
            # Unknown alternate obstacles remain conservatively occupied.
            if distance is None or distance <= p['near_m']:
                retained.append(det)
            entries.append({'box': box_xyxy(det), 'camera_blocks_path': blocks, **match})
        original_y = self.p['obstacle_near_y']
        self.p['obstacle_near_y'] = 0
        if forced_stop:
            # Preserve traffic processing but do not initiate a blind lane switch.
            retained = [d for d in retained if d.class_name != 'obstacle']
            previous_lane, previous_state = self.target_lane, self.state
        try:
            stop, limit = super().decide(DetectionArray(retained), frame, camera_path, now, lane_valid)
        finally:
            self.p['obstacle_near_y'] = original_y
        if forced_stop:
            self.target_lane, self.state = previous_lane, previous_state
            self.block_count = self.clear_count = 0
            self.status.update(target_lane=self.target_lane, lane_switch_requested=False,
                               state='RED_STOP' if self.red_latched else 'FUSION_STOP',
                               path_blocked=any(e['camera_blocks_path'] for e in entries),
                               stop=True, reason=reason or 'LIDAR_UNKNOWN_OR_TOO_CLOSE')
        self.status['fusion'] = entries
        self.status['fusion_ready'] = not bool(reason)
        return stop or forced_stop, limit
