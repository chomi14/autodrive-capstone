"""Traffic-light-only decision overlay for the existing track command.

The lane, path, Stanley steering, speed, sender, and firmware remain owned by
the track pipeline.  This module only decides whether that already-computed
command must stop or obey the existing yellow-light speed limit.
"""

from .messages import MotionCommand
from .mission_core import box_xyxy, light_evidence
from .mode_parameters import TRAFFIC_LIGHT, defaults


class TrafficLightCore:
    """Classify eligible light boxes and keep red latched until confirmed green."""

    def __init__(self, settings=None):
        self.p = defaults(TRAFFIC_LIGHT)
        self.p.update(settings or {})
        self.red_latched = bool(self.p['wait_for_green'])
        self.red_count = 0
        self.green_count = 0
        self.status = {}

    def decide(self, detections, frame):
        p = self.p
        h, w = frame.shape[:2]
        candidates = [
            detection
            for detection in detections.detections
            if detection.class_name == 'traffic_light'
            and detection.bbox.size.x >= p['traffic_min_width'] * w / 640.0
            and detection.bbox.center.position.y + detection.bbox.size.y / 2
            >= p['traffic_min_bottom_y'] * h / 480.0
        ]
        evidence = [light_evidence(frame, detection, p) for detection in candidates]
        colors = [(item['color'], item['strength']) for item in evidence]

        traffic_boxes = [
            {'box': box_xyxy(detection), 'eligible': True, **item}
            for detection, item in zip(candidates, evidence)
        ]
        traffic_boxes.extend(
            {
                'box': box_xyxy(detection),
                'eligible': False,
                'rejection': 'width_or_bottom_threshold',
            }
            for detection in detections.detections
            if detection.class_name == 'traffic_light'
            and not any(detection is candidate for candidate in candidates)
        )

        # Preserve the mission policy: any eligible red wins over yellow/green.
        color = next(
            (name for name in ('Red', 'Yellow', 'Green')
             if any(candidate_color == name for candidate_color, _ in colors)),
            'Unknown',
        )
        for item in traffic_boxes:
            item['used_for_color'] = (
                item.get('eligible', False) and item.get('color') == color
            )

        self.red_count = self.red_count + 1 if color == 'Red' else 0
        self.green_count = self.green_count + 1 if color == 'Green' else 0
        if self.red_count >= p['red_confirm_frames']:
            self.red_latched = True
        if self.green_count >= p['green_confirm_frames']:
            self.red_latched = False

        stop = self.red_latched
        limit = p['yellow_speed'] if color == 'Yellow' and not stop else 255
        reason = (
            'RED_LATCH_OR_WAIT_FOR_GREEN'
            if stop
            else 'YELLOW_SPEED_LIMIT'
            if color == 'Yellow'
            else 'NORMAL'
        )
        self.status = {
            'state': 'RED_STOP' if stop else 'YELLOW_LIMIT' if color == 'Yellow' else 'NORMAL',
            'traffic_color': color,
            'color_ratio': round(max((ratio for _, ratio in colors), default=0.0), 3),
            'selected_color_ratio': max(
                (ratio for candidate_color, ratio in colors if candidate_color == color),
                default=0.0,
            ),
            'red_count': self.red_count,
            'green_count': self.green_count,
            'red_confirm_required': p['red_confirm_frames'],
            'green_confirm_required': p['green_confirm_frames'],
            'red_latched': self.red_latched,
            'stop': stop,
            'reason': reason,
            'reasons': [reason],
            'traffic_boxes': traffic_boxes,
        }
        return stop, limit

    @staticmethod
    def apply(baseline, stop, limit):
        if stop:
            return MotionCommand(0, 0, 0)
        if limit >= baseline.left_speed and limit >= baseline.right_speed:
            return baseline
        return MotionCommand(
            baseline.steering,
            min(baseline.left_speed, limit),
            min(baseline.right_speed, limit),
        )
