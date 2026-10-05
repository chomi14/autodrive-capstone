"""Mission decisions on top of the existing camera path and Stanley command.

No ROS or actuator I/O here. Obstacle tests use the path projected into camera
coordinates, not a fixed image-half test. Red is latched until confirmed green.
"""
import cv2
import numpy as np

from .messages import MotionCommand
from .mode_parameters import MISSION, defaults


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


def path_blocked(detection, camera_path, settings, image_shape):
    """Horizontal footprint of a near obstacle crosses the actual path."""
    if len(camera_path) < 2:
        return False
    box = detection.bbox
    x, y = box.center.position.x, box.center.position.y + box.size.y / 2
    h = image_shape[0]
    if y < settings['obstacle_near_y'] * h / 480.0:
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
        self.red_count = self.green_count = self.block_count = self.clear_count = 0
        self.changed_at = None
        self.last_update = None
        self.blocking_boxes = []
        self.status = {}

    def decide(self, detections, frame, camera_path, now, lane_valid=True):
        p = self.p
        if self.red_latched and self.last_update is not None and self.changed_at is not None:
            self.changed_at += max(0.0, now - self.last_update)
        self.last_update = now
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

        obstacles = [d for d in detections.detections if d.class_name == 'obstacle']
        self.blocking_boxes = [d for d in obstacles if path_blocked(d, camera_path, p, frame.shape)]
        blocking = bool(self.blocking_boxes)
        evaluated_lane = self.target_lane
        alternative = 1 if self.target_lane == 2 else 2
        alternate_mask = lane_mask(detections, f'lane{alternative}', frame.shape)
        alternate_visible = np.count_nonzero(alternate_mask) >= p['alternate_min_area_px']
        alternate_blocked = False
        alternate_blockers = []
        for det in obstacles:
            box = det.bbox
            y = int(np.clip(box.center.position.y + box.size.y / 2, 0, h - 1))
            if y < p['obstacle_near_y'] * h / 480.0:
                continue
            margin = p['path_margin_px'] * w / 640.0
            x1 = max(0, int(box.center.position.x - box.size.x / 2 - margin))
            x2 = min(w, int(box.center.position.x + box.size.x / 2 + margin))
            if np.any(alternate_mask[max(0, y - 3):min(h, y + 4), x1:x2]):
                alternate_blocked = True
                alternate_blockers.append(det)
        self.block_count = self.block_count + 1 if blocking else 0
        self.clear_count = 0 if blocking else self.clear_count + 1
        confirmation_count = self.block_count
        switched = False
        if not self.red_latched:
            if self.block_count >= p['obstacle_confirm_frames'] and self.state != 'AVOIDING':
                if alternate_visible and not alternate_blocked:
                    self.target_lane = alternative
                    self.state = 'AVOIDING'
                    self.changed_at = now
                    self.clear_count = self.block_count = 0
                    switched = True
                else:
                    self.state = 'BLOCKED_STOP'
            elif self.state == 'AVOIDING':
                if blocking:
                    self.state = 'BLOCKED_STOP'
                elif lane_valid and now - self.changed_at >= p['avoid_hold_s'] and self.clear_count >= p['obstacle_clear_frames']:
                    self.state = 'NORMAL'
            elif self.state == 'BLOCKED_STOP' and self.clear_count >= p['obstacle_clear_frames']:
                self.state = 'NORMAL'

        # Stop immediately on an obstruction while accumulating confirmation;
        # do not drive into it during the confirmation window.
        stop = self.red_latched or self.state == 'BLOCKED_STOP' or switched or (blocking and self.state != 'AVOIDING') or (not lane_valid and self.state == 'AVOIDING')
        reasons = []
        if self.red_latched:
            reasons.append('RED_LATCH_OR_WAIT_FOR_GREEN')
        if switched:
            reasons.append('LANE_SWITCH_PAUSE')
        if self.state == 'BLOCKED_STOP':
            reasons.append('WAIT_OBSTACLE_CLEAR_CONFIRMATION' if not blocking else
                           'ALTERNATE_LANE_NOT_VISIBLE' if not alternate_visible else
                           'ALTERNATE_LANE_BLOCKED' if alternate_blocked else 'AVOIDANCE_PATH_BLOCKED')
        if blocking and self.state != 'AVOIDING':
            reasons.append('CURRENT_PATH_OBSTACLE')
        if not lane_valid and self.state == 'AVOIDING':
            reasons.append('AVOIDANCE_PATH_NOT_CURRENTLY_VALID')
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
            'path_blocked': blocking, 'alternate_visible': alternate_visible,
            'alternate_blocked': alternate_blocked, 'stop': stop,
            'path_lane': evaluated_lane, 'path_valid': lane_valid,
            'camera_path': camera_path, 'alternate_lane': alternative,
            'alternate_mask_area_px': int(np.count_nonzero(alternate_mask)),
            'alternate_valid': alternate_visible and not alternate_blocked,
            'alternate_valid_basis': 'visible_mask_area_and_obstacle_overlap_not_planned_path',
            'obstacle_confirm_count': confirmation_count, 'clear_count': self.clear_count,
            'obstacle_confirm_required': p['obstacle_confirm_frames'],
            'red_confirm_required': p['red_confirm_frames'], 'green_confirm_required': p['green_confirm_frames'],
            'red_latched': self.red_latched, 'lane_switch_requested': switched,
            'reasons': reasons, 'reason': '|'.join(reasons),
            'obstacles': [{'box': box_xyxy(d),
                           'blocks_path': any(d is b for b in self.blocking_boxes),
                           'blocks_alternate': any(d is b for b in alternate_blockers)} for d in obstacles],
            'traffic_boxes': traffic_boxes,

        }
        limit = p['avoid_speed'] if self.state == 'AVOIDING' else 255
        if color == 'Yellow':
            limit = min(limit, p['yellow_speed'])
        return stop, limit

    def apply(self, baseline, stop, limit):
        if stop:
            return MotionCommand(0, 0, 0)
        if limit >= baseline.left_speed and limit >= baseline.right_speed:
            return baseline  # Preserve the exact track command without intervention.
        return MotionCommand(baseline.steering, min(baseline.left_speed, limit), min(baseline.right_speed, limit))
