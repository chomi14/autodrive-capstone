"""Immediate confirmation AFTER the bbox bottom reaches the near threshold."""
import numpy as np
import pytest

from skku_track_drive_pkg.messages import Detection, DetectionArray, Mask
from skku_track_drive_pkg.mission_core import MissionCore
from test_mission_parking import FRAME, PATH, box, scene


def early_scene(*extra, y=380):
    current = np.zeros((480, 640), np.uint8)
    current[70:480, 380:460] = 1
    return scene(box('obstacle', 420, y=y),
                 Detection(class_name='lane2', mask=Mask(bitmap=current)), *extra)


def test_first_near_bbox_switches_without_waiting_three_frames():
    core = MissionCore({'immediate_obstacle_avoid': 1, 'obstacle_confirm_frames': 3})
    stop, _ = core.decide(early_scene(), FRAME, PATH, 1)
    assert core.target_lane == 1
    assert core.status['lane_switch_requested']
    assert core.status['obstacle_confirm_required'] == 1
    assert stop  # Never send lane2 steering as the first lane1 command.
    assert not core.decide(early_scene(), FRAME, [(130, 180), (130, 479)], 1.1)[0]


def test_alternate_is_observed_before_any_obstacle_and_not_cached_after_loss():
    core = MissionCore({'immediate_obstacle_avoid': 1, 'obstacle_confirm_frames': 1})
    core.decide(scene(), FRAME, PATH, 0)
    assert core.status['alternate_visible']
    core.decide(scene(box('obstacle', 420), alternative=False), FRAME, PATH, 1)
    assert not core.status['alternate_visible']
    assert core.target_lane == 2
    assert core.status['stop']


def test_near_alternate_obstacle_also_prevents_switch():
    core = MissionCore({'immediate_obstacle_avoid': 1, 'obstacle_confirm_frames': 1})
    # Full-frame alternate mask includes the distant obstacle's bottom row.
    alternate = np.zeros((480, 640), np.uint8)
    alternate[70:, 50:230] = 1
    detections = early_scene(box('obstacle', 130, y=380),
                            Detection(class_name='lane1', mask=Mask(bitmap=alternate)))
    assert core.decide(detections, FRAME, PATH, 1)[0]
    assert core.target_lane == 2
    assert core.status['alternate_blocked']


@pytest.mark.parametrize('bottom,expected', [(130, False), (299, False), (300, True), (301, True)])
@pytest.mark.parametrize('use_mask', [False, True])
def test_immediate_mode_never_bypasses_near_line(bottom, expected, use_mask):
    core = MissionCore({'immediate_obstacle_avoid': 1, 'obstacle_confirm_frames': 3})
    detections = early_scene(y=bottom - 20)
    # Empty path forces the full-frame mask branch; otherwise test path branch.
    if not use_mask:
        detections = scene(box('obstacle', 420, y=bottom - 20))
    core.decide(detections, FRAME, [] if use_mask else PATH, 1)
    assert core.status['path_blocked'] is expected
    assert core.status['lane_switch_requested'] is expected


def test_near_threshold_scales_with_frame_height():
    from skku_track_drive_pkg.mission_core import path_blocked, mask_blocked
    core = MissionCore({'immediate_obstacle_avoid': 1})
    shape = (960, 1280, 3)
    path = [(840, 360), (840, 959)]
    mask = np.ones(shape[:2], np.uint8)
    for bottom, expected in [(599, False), (600, True)]:
        det = box('obstacle', 840, y=bottom - 20)
        assert path_blocked(det, path, core.p, shape) is expected
        assert mask_blocked(det, mask, core.p, shape) is expected


@pytest.mark.parametrize('y,expected', [(460, True), (490, True), (510, False), (-30, False)])
def test_bottom_clipped_obstacle_remains_blocking_but_offscreen_box_does_not(y, expected):
    from skku_track_drive_pkg.mission_core import mask_blocked
    core = MissionCore({'immediate_obstacle_avoid': 1})
    mask = np.ones(FRAME.shape[:2], np.uint8)
    assert mask_blocked(box('obstacle', 420, y=y), mask, core.p, FRAME.shape) is expected


def test_early_mode_does_not_react_to_unrelated_roadside_box():
    core = MissionCore({'immediate_obstacle_avoid': 1, 'obstacle_confirm_frames': 1})
    assert not core.decide(scene(box('obstacle', 550)), FRAME, PATH, 1)[0]
    assert core.target_lane == 2


def test_red_and_missing_new_lane_path_still_stop():
    core = MissionCore({'immediate_obstacle_avoid': 1, 'obstacle_confirm_frames': 1,
                        'wait_for_green': 1})
    assert core.decide(early_scene(), FRAME, PATH, 1)[0]
    assert core.target_lane == 2
    core.red_latched = False
    core.decide(early_scene(), FRAME, PATH, 2)
    assert core.target_lane == 1
    assert core.decide(early_scene(), FRAME, [], 2.1, lane_valid=False)[0]
