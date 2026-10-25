"""Lane-mask/bbox-bottom switching for both legacy launch profiles; no I/O."""
import numpy as np
import pytest

from skku_track_drive_pkg.mission_core import MissionCore, bbox_area_ratio, visible_box
from skku_track_drive_pkg.messages import Detection, DetectionArray, Mask, MotionCommand, Point2D
from test_mission_parking import FRAME, PATH, box, scene


def core(**overrides):
    # Old saved tuning must not override the fixed 5s hold or insert zero PWM.
    return MissionCore({'obstacle_trigger_mode': 1, 'obstacle_min_area_ratio': .5,
                        'obstacle_confirm_frames': 30, 'obstacle_clear_frames': 60,
                        'lane_change_cooldown_s': 30.0, 'avoid_hold_s': 10.0,
                        'avoid_speed': 0, **overrides})


def obstacle(x=420, y=360, width=80, height=80, name='obstacle'):
    return box(name, x, y, width, height)


@pytest.mark.parametrize('trigger_mode', [0, 1])
@pytest.mark.parametrize('initial_lane,x', [(2, 420), (1, 130)])
def test_first_qualifying_frame_switches_both_directions_without_zero_pwm(trigger_mode, initial_lane, x):
    m = core(obstacle_trigger_mode=trigger_mode)
    m.target_lane = initial_lane
    baseline = MotionCommand(-3, 250, 250)
    detections = scene(obstacle(x=x))
    stop, limit = m.decide(detections, FRAME, [], 0, lane_valid=False)
    assert not stop
    assert m.target_lane == 3 - initial_lane
    assert m.status['lane_switch_requested']
    assert m.status['path_lane'] == initial_lane
    assert m.status['obstacle_confirm_required'] == 1
    assert m.apply(baseline, stop, limit) is baseline
    assert not m.decide(detections, FRAME, [], .01, lane_valid=False)[0]
    assert m.target_lane == 3 - initial_lane  # Old lane's obstacle cannot switch back.


@pytest.mark.parametrize('width,height', [(4, 4), (40, 40), (64, 96), (80, 80)])
def test_bbox_area_does_not_gate_switch(width, height):
    m = core()
    assert not m.decide(scene(obstacle(width=width, height=height)), FRAME, PATH, 0)[0]
    assert m.target_lane == 1


@pytest.mark.parametrize('bottom,expected', [(299, False), (300, True), (301, True)])
def test_bbox_bottom_threshold_is_inclusive(bottom, expected):
    m = core()
    m.decide(scene(obstacle(y=bottom-40)), FRAME, PATH, 0)
    assert m.status['lane_switch_requested'] is expected


def test_live_bottom_threshold_changes_the_next_decision():
    m = core(obstacle_near_y=401)
    detections = scene(obstacle())  # bottom=400
    assert not m.decide(detections, FRAME, PATH, 0)[0]
    assert m.target_lane == 2
    m.p['obstacle_near_y'] = 400
    assert not m.decide(detections, FRAME, PATH, .1)[0]
    assert m.target_lane == 1


def test_normal_command_and_obstacle_in_another_lane_do_not_change_lane():
    m = core()
    baseline = MotionCommand(-3, 250, 250)
    stop, limit = m.decide(scene(obstacle(x=130)), FRAME, [(130, 180), (130, 479)], 0)
    assert m.apply(baseline, stop, limit) is baseline
    assert m.target_lane == 2


def test_current_lane_mask_triggers_outside_planned_roi_without_valid_path():
    m = core(obstacle_near_y=0)
    mask = np.zeros((480, 640), np.uint8)
    mask[60:, 380:460] = 1
    detections = scene(obstacle(y=100), Detection(class_name='lane2', mask=Mask(bitmap=mask)))
    assert not m.decide(detections, FRAME, [], 0, lane_valid=False)[0]
    assert m.target_lane == 1


def test_camera_path_alone_does_not_replace_current_lane_observation():
    m = core()
    assert not m.decide(scene(obstacle(), current=False), FRAME, PATH, 0)[0]
    assert m.target_lane == 2


def test_destination_need_not_be_visible():
    m = core()
    assert not m.decide(scene(obstacle(), alternative=False), FRAME, PATH, 0)[0]
    assert m.target_lane == 1
    assert not m.status['alternate_visible']


def test_destination_obstacle_is_diagnostic_and_does_not_stop_switch():
    m = core()
    assert not m.decide(scene(obstacle(), obstacle(x=130)), FRAME, PATH, 0)[0]
    assert m.target_lane == 1 and m.status['alternate_blocked']


@pytest.mark.parametrize('saved_cooldown', [0.0, 2.0, 30.0])
def test_new_lane_obstacle_waits_exactly_five_seconds_without_stopping(saved_cooldown):
    m = core(lane_change_cooldown_s=saved_cooldown)
    assert not m.decide(scene(obstacle()), FRAME, PATH, 0)[0]
    assert m.target_lane == 1
    baseline = MotionCommand(3, 250, 250)
    for now in (.01, 1., 4.999):
        stop, limit = m.decide(scene(obstacle(x=130)), FRAME, [], now, lane_valid=False)
        assert m.target_lane == 1
        assert m.apply(baseline, stop, limit) is baseline
        assert not m.status['lane_switch_requested']
        assert m.status['lane_change_cooldown_remaining_s'] == pytest.approx(5. - now)
        assert m.status['reason'] == 'TARGET_LANE_COOLDOWN'
    assert not m.decide(scene(obstacle(x=130)), FRAME, [], 5., lane_valid=False)[0]
    assert m.target_lane == 2
    assert m.status['lane_switch_requested']
    assert m.status['lane_change_cooldown_remaining_s'] == 5.


def test_obstacle_disappearance_does_not_wait_for_clear_confirmation():
    m = core()
    m.decide(scene(obstacle()), FRAME, PATH, 0)
    assert not m.decide(scene(), FRAME, [], .01, lane_valid=False)[0]
    assert m.target_lane == 1 and m.state == 'AVOIDING'
    assert not m.decide(scene(), FRAME, [], 5., lane_valid=False)[0]
    assert m.target_lane == 1 and m.state == 'NORMAL'
    assert m.status['reason'] == 'NORMAL'


def test_missing_new_lane_path_continues_but_traffic_stop_remains():
    m = core()
    detections = scene(obstacle())
    m.decide(detections, FRAME, PATH, 0)
    baseline = MotionCommand(4, 250, 250)
    stop, limit = m.decide(detections, FRAME, [], .1, lane_valid=False)
    assert m.apply(baseline, stop, limit) is baseline
    red = core(wait_for_green=1)
    stop, limit = red.decide(detections, FRAME, PATH, 0)
    assert stop and red.target_lane == 2
    assert red.apply(baseline, stop, limit) == MotionCommand(0, 0, 0)


def test_resizing_frame_scales_bottom_threshold_and_lane_mask():
    frame = np.zeros((960, 1280, 3), np.uint8)
    mask = np.zeros((960, 1280), np.uint8)
    mask[:, 760:920] = 1
    m = core()
    det = obstacle(x=840, y=559, width=160, height=80)  # bottom=599 < 600
    detections = DetectionArray([det, Detection(class_name='lane2', mask=Mask(bitmap=mask))])
    m.decide(detections, frame, [], 0)
    assert m.target_lane == 2
    det.bbox.center.position.y = 560
    assert not m.decide(detections, frame, [], .1)[0]
    assert m.target_lane == 1
    assert bbox_area_ratio(obstacle(x=840, y=720, width=160, height=160), frame.shape) == bbox_area_ratio(obstacle(), FRAME.shape)


@pytest.mark.parametrize('width,height', [(0,80), (-1,80), (80,0), (float('nan'),80), (80,float('inf'))])
def test_invalid_boxes_do_not_trigger(width, height):
    m = core()
    det = obstacle(width=width, height=height)
    assert visible_box(det, FRAME.shape) is None
    assert not m.decide(scene(det), FRAME, PATH, 0)[0]
    assert m.target_lane == 2


def test_obstacles_plural_alias_is_supported():
    m = core()
    m.decide(scene(obstacle(name='obstacles')), FRAME, PATH, 0)
    assert m.target_lane == 1


def test_clipped_bottom_box_still_matches_current_lane():
    m = core()
    assert not m.decide(scene(obstacle(y=470, height=160)), FRAME, [], 0, lane_valid=False)[0]
    assert m.target_lane == 1


def test_lane_polygon_is_supported_without_bitmap():
    m = core()
    polygon = [Point2D(380, 180), Point2D(460, 180), Point2D(460, 479), Point2D(380, 479)]
    detections = DetectionArray([obstacle(), Detection(class_name='lane2', mask=Mask(data=polygon))])
    m.decide(detections, FRAME, [], 0)
    assert m.target_lane == 1


def test_path_margin_cannot_assign_adjacent_obstacle_to_current_lane():
    m = core(path_margin_px=200)
    m.decide(scene(obstacle(x=350, width=20)), FRAME, PATH, 0)
    assert m.target_lane == 2


@pytest.mark.parametrize('initial_lane', [1, 2])
def test_same_bbox_on_both_masks_cannot_flip_target_back_during_cooldown(initial_lane):
    m = core()
    m.target_lane = initial_lane
    mask = np.ones((480, 640), np.uint8)
    detections = DetectionArray([
        obstacle(), Detection(class_name='lane1', mask=Mask(bitmap=mask)),
        Detection(class_name='lane2', mask=Mask(bitmap=mask)),
    ])
    assert not m.decide(detections, FRAME, [], 100.)[0]
    for now in (100.01, 101., 104.999):
        assert not m.decide(detections, FRAME, [], now)[0]
        assert m.target_lane == 3 - initial_lane
        assert not m.status['lane_switch_requested']
    assert not m.decide(detections, FRAME, [], 105.)[0]
    assert m.target_lane == initial_lane


def test_manual_toggle_overrides_cooldown_and_restarts_five_second_hold():
    m = core()
    m.decide(scene(obstacle()), FRAME, [], 10.)
    assert m.target_lane == 1
    assert m.toggle_target_lane(11.) == 2
    assert m.status['last_lane_switch_source'] == 'manual'
    assert m.status['reason'] == 'MANUAL_TARGET_LANE_SWITCH'
    assert m.status['lane_change_cooldown_remaining_s'] == 5.
    # Repeated L is always allowed and resets the deadline to the latest press.
    assert m.toggle_target_lane(12.) == 1
    assert not m.decide(scene(obstacle(x=130)), FRAME, [], 16.999)[0]
    assert m.target_lane == 1
    assert not m.decide(scene(obstacle(x=130)), FRAME, [], 17.)[0]
    assert m.target_lane == 2


def test_manual_toggle_changes_target_without_releasing_traffic_stop():
    m = core(wait_for_green=1)
    assert m.toggle_target_lane(10.) == 1
    assert m.status['stop'] and m.status['red_latched']
    baseline = MotionCommand(2, 250, 250)
    stop, limit = m.decide(scene(obstacle(x=130)), FRAME, [], 11.)
    assert m.target_lane == 1
    assert m.apply(baseline, stop, limit) == MotionCommand(0, 0, 0)


def test_image_gap_does_not_extend_elapsed_cooldown():
    m = core()
    m.decide(scene(obstacle()), FRAME, [], 10.)
    assert m.target_lane == 1
    # No frames arrive for >5s; the next frame may trigger immediately.
    assert not m.decide(scene(obstacle(x=130)), FRAME, [], 16.)[0]
    assert m.target_lane == 2
