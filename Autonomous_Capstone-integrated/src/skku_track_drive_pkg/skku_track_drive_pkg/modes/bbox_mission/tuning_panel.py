"""Mission dashboard rendered in the tuner's existing OpenCV event loop."""

import cv2
import numpy as np

from ...traffic_light_display import BBOX_CUT_COLORS


MISSION_WINDOW_NAME = 'Mission Tuner'
BACKGROUND = (25, 21, 18)
SURFACE = (41, 34, 28)
INSET = (32, 27, 23)
EDGE = (68, 55, 43)
TEXT = (244, 240, 230)
MUTED = (179, 165, 150)
TEAL = (187, 220, 109)
GREEN = (131, 216, 146)
RED = (128, 127, 252)
AMBER = BBOX_CUT_COLORS['obstacle_near_y']

CONTROLS = (
    ('obstacle_near_y', 'Obstacle cutoff', 'BBox bottom at or below this line',
     '.0f', ' px', AMBER, 219),
    ('traffic_max_bottom_y', 'Traffic light cutoff', 'BBox bottom at or above this line',
     '.0f', ' px', BBOX_CUT_COLORS['traffic_max_bottom_y'], 303),
    ('lane_change_gain', 'Stanley gain limit', 'Applied during the lane change',
     '.3f', '', TEAL, 429),
    ('lane_change_gain_duration_s', 'Override duration', 'Zero disables the temporary gain',
     '.1f', ' s', TEAL, 513),
)
MISSION_PRIMARY_CONTROLS = tuple(row[0] for row in CONTROLS)

BUTTONS = (
    (24, 126, 'Start  W', 'w', GREEN),
    (136, 238, 'Stop  S', 's', RED),
    (248, 386, 'Switch lane  L', 'l', TEXT),
    (396, 518, 'Save YAML  P', 'p', TEAL),
    (528, 654, 'Detail view  V', 'v', TEXT),
)
SLIDER_LEFT, SLIDER_RIGHT = 767, 1014
SLIDER_OFFSET_Y = 45


def _text(canvas, value, x, y, size=.48, color=TEXT, weight=1):
    cv2.putText(canvas, value, (x, y), cv2.FONT_HERSHEY_SIMPLEX, size, color, weight, cv2.LINE_AA)


class MissionTuningPanel:
    def __init__(self, trackbars, on_change, on_key):
        self.trackbars = trackbars
        self.on_change = on_change
        self.on_key = on_key
        self.values = {}
        self.dragging = None

    def on_mouse(self, event, x, y, flags, _param):
        if event == cv2.EVENT_LBUTTONUP:
            self.dragging = None
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging is not None:
            if flags & cv2.EVENT_FLAG_LBUTTON:
                self._set_slider(self.dragging, x)
            else:
                self.dragging = None
        elif event == cv2.EVENT_LBUTTONDOWN:
            if 78 <= y <= 114:
                for left, right, _label, key, _color in BUTTONS:
                    if left <= x <= right:
                        self.on_key(ord(key))
                        return
            for name, *_labels, row in CONTROLS:
                if name not in self.values or abs(y - (row + SLIDER_OFFSET_Y)) > 13:
                    continue
                if SLIDER_LEFT - 8 <= x <= SLIDER_RIGHT + 8:
                    self.dragging = name
                    self._set_slider(name, x)
                elif 729 <= x <= 751 or 1030 <= x <= 1052:
                    _, maximum, _, encode = self.trackbars[name]
                    position = encode(self.values[name]) + (-1 if x <= 751 else 1)
                    self.on_change(name, max(0, min(maximum, position)))
                return

    def _set_slider(self, name, x):
        maximum = self.trackbars[name][1]
        position = round(maximum * (x - SLIDER_LEFT) / (SLIDER_RIGHT - SLIDER_LEFT))
        self.on_change(name, max(0, min(maximum, position)))

    def render(self, values, status, command, frame, preview_message, syncing):
        self.values = values
        canvas = np.full((610, 1080, 3), BACKGROUND, dtype=np.uint8)
        _text(canvas, 'Obstacle & traffic tuning', 24, 40, .89, TEXT, 2)
        _text(canvas, 'Set cutoffs in camera pixels and tune steering for lane changes.', 25, 63, .47, MUTED)
        for left, right, label, _key, color in BUTTONS:
            cv2.rectangle(canvas, (left, 78), (right, 114), SURFACE, -1)
            cv2.rectangle(canvas, (left, 78), (right, 114), EDGE, 1)
            _text(canvas, label, left + 10, 101, .48, color)
        armed = command.get('armed')
        arm_label = 'ARMED' if armed is True else 'STOPPED' if armed is False else 'ARM STATUS UNKNOWN'
        _text(canvas, arm_label, 857, 99, .48, GREEN if armed is True else MUTED, 2)

        cv2.rectangle(canvas, (24, 132), (696, 573), SURFACE, -1)
        _text(canvas, 'CAMERA / DETECTIONS', 40, 159, .54, TEXT)
        light = status.get('traffic_color', 'Unknown')
        light_color = {'Red': RED, 'Yellow': AMBER, 'Green': GREEN}.get(light, MUTED)
        _text(canvas, f'Light  {light}', 555, 159, .48, light_color)
        self._draw_preview(canvas, frame, preview_message)
        lane = status.get('target_lane', '-')
        cooldown = status.get('lane_change_cooldown_remaining_s', 0)
        _text(canvas, f'Target lane  {lane}     Auto cooldown  {cooldown:.1f} s', 40, 554, .53, TEXT)
        _text(canvas, status.get('state', 'WAITING'), 533, 554, .48, MUTED)

        cv2.rectangle(canvas, (712, 132), (1056, 573), SURFACE, -1)
        _text(canvas, 'Detection cutoffs', 729, 160, .64, TEXT)
        _text(canvas, 'Y coordinates on a 480 px reference image', 729, 182, .41, MUTED)
        _text(canvas, 'Lane change steering', 729, 378, .64, TEXT)
        gain = status.get('effective_stanley_gain')
        gain_text = 'Waiting for controller' if gain is None else (
            f'Applied {gain:.3f}   Override left {status.get("lane_change_gain_remaining_s", 0):.1f} s'
        )
        _text(canvas, gain_text, 729, 398, .43, TEAL if status.get('lane_change_gain_active') else MUTED)
        for name, title, description, fmt, unit, color, row in CONTROLS:
            _, maximum, _, encode = self.trackbars[name]
            value = values[name]
            value_text = format(value, fmt) + unit
            width = cv2.getTextSize(value_text, cv2.FONT_HERSHEY_SIMPLEX, .56, 2)[0][0]
            _text(canvas, title, 729, row - 11, .50, TEXT)
            _text(canvas, value_text, 1051 - width, row - 11, .56, color, 2)
            _text(canvas, description, 729, row + 9, .40, MUTED)
            y = row + SLIDER_OFFSET_Y
            thumb = SLIDER_LEFT + round((SLIDER_RIGHT - SLIDER_LEFT) * encode(value) / maximum)
            thumb = max(SLIDER_LEFT, min(SLIDER_RIGHT, thumb))
            cv2.line(canvas, (SLIDER_LEFT, y), (SLIDER_RIGHT, y), EDGE, 4, cv2.LINE_AA)
            cv2.line(canvas, (SLIDER_LEFT, y), (thumb, y), color, 4, cv2.LINE_AA)
            cv2.circle(canvas, (thumb, y), 6, color, -1, cv2.LINE_AA)
            for x, sign in ((729, '-'), (1030, '+')):
                cv2.rectangle(canvas, (x, y - 11), (x + 22, y + 11), INSET, -1)
                _text(canvas, sign, x + 5, y + 5, .43, TEXT)

        _text(canvas, 'D  Debug feed    B  BEV    Q  Close', 24, 596, .45, MUTED)
        _text(canvas, 'Applying changes...' if syncing else 'Edits apply live', 729, 596, .45, AMBER if syncing else TEAL)
        return canvas

    @staticmethod
    def _draw_preview(canvas, frame, message):
        preview = canvas[172:531, 40:680]
        preview[:] = INSET
        if frame is None:
            _text(preview, message, 20, 38, .55, MUTED)
            return
        height, width = frame.shape[:2]
        scale = min(preview.shape[1] / width, preview.shape[0] / height)
        width, height = max(1, round(width * scale)), max(1, round(height * scale))
        x, y = (preview.shape[1] - width) // 2, (preview.shape[0] - height) // 2
        preview[y:y + height, x:x + width] = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
