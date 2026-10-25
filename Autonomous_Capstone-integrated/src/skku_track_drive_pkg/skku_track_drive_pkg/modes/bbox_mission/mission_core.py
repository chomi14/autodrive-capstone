"""Immediate lane-mask obstacle switching on top of the Stanley command.

No ROS or actuator I/O here. A near obstacle on the selected lane requests the
opposite lane without an obstacle stop. Red is latched until confirmed green.
"""
import cv2
import numpy as np

from skku_track_drive_pkg.messages import MotionCommand
from skku_track_drive_pkg.mode_parameters import MISSION, defaults


# Fixed by the driving policy; legacy YAML cooldown values must not override it.
LANE_CHANGE_COOLDOWN_S = 5.0


def lane_mask(detections, name, shape):
    mask = np.zeros(shape[:2], dtype=np.uint8)
    for det in detections.detections:
        if det.class_name != name:
            continue
        if det.mask.bitmap is not None:
            bitmap = (np.asarray(det.mask.bitmap) > 0).astype(np.uint8)
            if bitmap.shape != mask.shape:
                bitmap = cv2.resize(bitmap, (mask.shape[1], mask.shape[0]), interpolation=cv2.INTER_NEAREST)
            mask |= bitmap
        elif len(det.mask.data) >= 3:
            polygon = np.asarray([(p.x, p.y) for p in det.mask.data], dtype=np.int32)
            cv2.fillPoly(mask, [polygon], 1)
    return mask


def box_xyxy(detection):
    box = detection.bbox
    x, y = box.center.position.x, box.center.position.y
    return [float(x-box.size.x/2), float(y-box.size.y/2),
            float(x+box.size.x/2), float(y+box.size.y/2)]


def light_evidence(frame, detection, settings):
    box = detection.bbox
    h, w = frame.shape[:2]
    x1 = max(0, int(box.center.position.x - box.size.x / 2))
    x2 = min(w, int(box.center.position.x + box.size.x / 2))
    y1 = max(0, int(box.center.position.y - box.size.y / 2))
    y2 = min(h, int(box.center.position.y + box.size.y / 2))
    if x2 <= x1 or y2 <= y1:
        return {'color': 'Unknown', 'strength': 0.0, 'ratios': {'Red': 0.0, 'Green': 0.0, 'Yellow': 0.0}}
    hsv = cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)
    valid = (saturation >= settings['hsv_s_min']) & (value >= settings['hsv_v_min'])
    ratios = {
        'Red': float(np.mean(valid & ((hue <= settings['red_h_low_max']) | (hue >= settings['red_h_high_min'])))),
        'Green': float(np.mean(valid & (hue >= settings['green_h_min']) & (hue <= settings['green_h_max']))),
        'Yellow': float(np.mean(valid & (hue >= settings['yellow_h_min']) & (hue <= settings['yellow_h_max']))),
    }
    best = max(ratios, key=ratios.get)
    strength = ratios[best]
    if strength <= 0.0 or strength < settings['traffic_min_color_ratio']:
        return {'color': 'Unknown', 'strength': strength, 'ratios': ratios}
    # Ambiguous colors do not release a previously latched red light.
    if sum(v == strength for v in ratios.values()) > 1:
        return {'color': 'Unknown', 'strength': strength, 'ratios': ratios}
    return {'color': best, 'strength': strength, 'ratios': ratios}


def color_of_light(frame, detection, settings):
    evidence = light_evidence(frame, detection, settings)
    return evidence['color'], evidence['strength']


def visible_box(detection, image_shape):
    """Finite, positive bbox clipped to current camera pixels."""
    box = detection.bbox
    values = (box.center.position.x, box.center.position.y, box.size.x, box.size.y)
    if not all(np.isfinite(value) for value in values) or box.size.x <= 0 or box.size.y <= 0:
        return None
    h, w = image_shape[:2]
    x1, y1, x2, y2 = box_xyxy(detection)
    x1, y1, x2, y2 = max(0., x1), max(0., y1), min(float(w), x2), min(float(h), y2)
    return (x1, y1, x2, y2) if x2 > x1 and y2 > y1 else None


def bbox_area_ratio(detection, image_shape):
    box = visible_box(detection, image_shape)
    if box is None:
        return 0.0
    x1, y1, x2, y2 = box
    return (x2 - x1) * (y2 - y1) / (image_shape[0] * image_shape[1])


def obstacle_is_near(detection, settings, image_shape):
    box = visible_box(detection, image_shape)
    return (box is not None
            and box[3] >= settings['obstacle_near_y'] * image_shape[0] / 480.0)


def bbox_overlaps_lane(detection, mask, settings, image_shape):
    """Match the bbox bottom footprint to the actual full-frame lane mask.

    Do not expand horizontally by path_margin_px: a nearby obstacle in the
    adjacent lane must not become an obstacle on our selected lane.
    """
    box = visible_box(detection, image_shape)
    if box is None:
        return False
    h, w = image_shape[:2]
    x1, _, x2, bottom = box
    y = min(h - 1, int(bottom))
    left, right = int(x1), min(w, int(np.ceil(x2)))
    return bool(np.any(mask[max(0, y-3):min(h, y+4), left:right]))


def path_blocked(detection, camera_path, settings, image_shape):
    """Horizontal footprint of a near obstacle crosses the actual path."""
    if len(camera_path) < 2:
        return False
    box = detection.bbox
    x, y = box.center.position.x, box.center.position.y + box.size.y / 2
    h = image_shape[0]
    if not obstacle_is_near(detection, settings, image_shape):
        return False
    points = sorted(camera_path, key=lambda p: p[1])
    if y < points[0][1] - 10 or y > points[-1][1] + 30:
        return False
    px = float(np.interp(y, [p[1] for p in points], [p[0] for p in points]))
    margin = settings['path_margin_px'] * image_shape[1] / 640.0
    return x - box.size.x / 2 - margin <= px <= x + box.size.x / 2 + margin


class MissionCore:
    def __init__(self, settings=None):
        self.p = defaults(MISSION)
        self.p.update(settings or {})
        self.target_lane = 2
        self.state = 'NORMAL'
        self.red_latched = bool(self.p['wait_for_green'])
        self.red_count = self.green_count = 0
        self.blocking_boxes = []
        self.status = {}
        self._lane_cooldown_until = None
        self.last_lane_switch_source = None

    def cooldown_remaining(self, now):
        if self._lane_cooldown_until is None:
            return 0.0
        return max(0.0, self._lane_cooldown_until - now)

    def _switch_target_lane(self, now, source):
        self.target_lane = 1 if self.target_lane == 2 else 2
        self._lane_cooldown_until = now + LANE_CHANGE_COOLDOWN_S
        self.last_lane_switch_source = source
        self.state = 'AVOIDING'

    def toggle_target_lane(self, now):
        """Manual override always works, restarting the automatic-switch hold."""
        self._switch_target_lane(now, 'manual')
        reasons = ['MANUAL_TARGET_LANE_SWITCH']
        if self.red_latched:
            reasons.insert(0, 'RED_LATCH_OR_WAIT_FOR_GREEN')
        self.status.update({
            'state': 'RED_STOP' if self.red_latched else self.state,
            'target_lane': self.target_lane, 'stop': self.red_latched,
            'red_latched': self.red_latched, 'lane_switch_requested': True,
            'last_lane_switch_source': self.last_lane_switch_source,
            'lane_change_cooldown_s': LANE_CHANGE_COOLDOWN_S,
            'lane_change_cooldown_remaining_s': self.cooldown_remaining(now),
            'reasons': reasons, 'reason': '|'.join(reasons),
        })
        return self.target_lane

    def decide(self, detections, frame, camera_path, now, lane_valid=True):
        p = self.p
        h, w = frame.shape[:2]
        lights = [d for d in detections.detections if d.class_name == 'traffic_light'
                  and d.bbox.size.x >= p['traffic_min_width'] * w / 640.0
                  and d.bbox.center.position.y + d.bbox.size.y / 2 >= p['traffic_min_bottom_y'] * h / 480.0]
        evidence = [light_evidence(frame, d, p) for d in lights]
        colors = [(e['color'], e['strength']) for e in evidence]
        traffic_boxes = [{'box': box_xyxy(d), 'eligible': True, **e} for d, e in zip(lights, evidence)]
        traffic_boxes.extend({'box': box_xyxy(d), 'eligible': False, 'rejection': 'width_or_bottom_threshold'}
                             for d in detections.detections if d.class_name == 'traffic_light'
                             and not any(d is selected for selected in lights))
        # A red observation has priority over yellow and green observations.
        color = next((c for c in ('Red', 'Yellow', 'Green') if any(k == c for k, _ in colors)), 'Unknown')
        for item in traffic_boxes:
            item['used_for_color'] = item.get('eligible', False) and item.get('color') == color
        self.red_count = self.red_count + 1 if color == 'Red' else 0
        self.green_count = self.green_count + 1 if color == 'Green' else 0
        if self.red_count >= p['red_confirm_frames']:
            self.red_latched = True
        if self.green_count >= p['green_confirm_frames']:
            self.red_latched = False

        obstacles = [d for d in detections.detections if d.class_name in ('obstacle', 'obstacles')]
        evaluated_lane = self.target_lane
        current_mask = lane_mask(detections, f'lane{evaluated_lane}', frame.shape)
        self.blocking_boxes = [d for d in obstacles
                               if obstacle_is_near(d, p, frame.shape)
                               and bbox_overlaps_lane(d, current_mask, p, frame.shape)]
        blocking = bool(self.blocking_boxes)
        alternative = 1 if self.target_lane == 2 else 2
        alternate_mask = lane_mask(detections, f'lane{alternative}', frame.shape)
        # Alternate-lane evidence is diagnostic only, never a switch/stop gate.
        alternate_visible = np.count_nonzero(alternate_mask) >= p['alternate_min_area_px']
        alternate_blockers = [d for d in obstacles if obstacle_is_near(d, p, frame.shape)
                              and bbox_overlaps_lane(d, alternate_mask, p, frame.shape)]
        alternate_blocked = bool(alternate_blockers)
        switched = blocking and not self.red_latched and self.cooldown_remaining(now) <= 0.0
        if switched:
            self._switch_target_lane(now, 'obstacle')
        cooldown_remaining = self.cooldown_remaining(now)
        self.state = 'AVOIDING' if cooldown_remaining > 0.0 else 'NORMAL'

        # Obstacles only select the lane. They never insert zero PWM, including
        # on a switch frame or while the destination path is missing.
        stop = self.red_latched
        reasons = []
        if self.red_latched:
            reasons.append('RED_LATCH_OR_WAIT_FOR_GREEN')
        if switched:
            reasons.append('CURRENT_LANE_OBSTACLE_SWITCH')
        elif cooldown_remaining > 0.0:
            reasons.append('TARGET_LANE_COOLDOWN')
        if not reasons:
            reasons.append('AVOIDING' if self.state == 'AVOIDING' else 'NORMAL')
        if color == 'Yellow' and not stop:
            reasons.append('YELLOW_SPEED_LIMIT')
        self.status = {
            'state': 'RED_STOP' if self.red_latched else self.state,
            'target_lane': self.target_lane, 'traffic_color': color,
            'color_ratio': round(max((r for _, r in colors), default=0.0), 3),
            'selected_color_ratio': max((r for k, r in colors if k == color), default=0.0),
            'red_count': self.red_count, 'green_count': self.green_count,
            'path_blocked': blocking, 'current_lane_blocked': blocking,
            'alternate_visible': alternate_visible,
            'alternate_blocked': alternate_blocked, 'stop': stop,
            'path_lane': evaluated_lane, 'path_valid': lane_valid,
            'camera_path': camera_path, 'alternate_lane': alternative,
            'alternate_mask_area_px': int(np.count_nonzero(alternate_mask)),
            'alternate_valid': alternate_visible and not alternate_blocked,
            'alternate_valid_basis': 'visible_mask_area_and_obstacle_overlap_not_planned_path',
            'obstacle_confirm_count': int(blocking), 'clear_count': 0,
            'obstacle_confirm_required': 1,
            'obstacle_trigger_mode': 'lane_mask_and_bbox_bottom',
            'obstacle_near_y': p['obstacle_near_y'],
            'lane_change_cooldown_s': LANE_CHANGE_COOLDOWN_S,
            'lane_change_cooldown_remaining_s': cooldown_remaining,
            'red_confirm_required': p['red_confirm_frames'], 'green_confirm_required': p['green_confirm_frames'],
            'red_latched': self.red_latched, 'lane_switch_requested': switched,
            'last_lane_switch_source': self.last_lane_switch_source,
            'reasons': reasons, 'reason': '|'.join(reasons),
            'obstacles': [{'box': box_xyxy(d),
                           'area_ratio': bbox_area_ratio(d, frame.shape),
                           'near': obstacle_is_near(d, p, frame.shape),
                           'blocks_path': any(d is b for b in self.blocking_boxes),
                           'blocks_alternate': any(d is b for b in alternate_blockers)} for d in obstacles],
            'traffic_boxes': traffic_boxes,

        }
        limit = 255
        if color == 'Yellow':
            limit = min(limit, p['yellow_speed'])
        return stop, limit

    def apply(self, baseline, stop, limit):
        if stop:
            return MotionCommand(0, 0, 0)
        if limit >= baseline.left_speed and limit >= baseline.right_speed:
            return baseline  # Preserve the exact track command without intervention.
        return MotionCommand(baseline.steering, min(baseline.left_speed, limit), min(baseline.right_speed, limit))
