"""Camera-only bbox size triggering; no ROS, camera or actuator I/O."""
import numpy as np
import pytest

from skku_track_drive_pkg.mission_core import MissionCore, bbox_area_ratio, visible_box
from skku_track_drive_pkg.messages import Detection, DetectionArray, Mask, MotionCommand
from test_mission_parking import FRAME, PATH, box, scene


def core(**overrides):
    return MissionCore({'obstacle_trigger_mode': 1, 'obstacle_min_area_ratio': .02,
                        'obstacle_confirm_frames': 3, 'lane_change_cooldown_s': 2.0, **overrides})


def obstacle(x=420, y=360, width=80, height=80, name='obstacle'):
    return box(name, x, y, width, height)


def test_first_qualifying_frame_requests_opposite_lane_then_uses_new_path():
    m=core()
    detections=scene(obstacle())
    assert m.decide(detections, FRAME, PATH, 0)[0]
    assert m.target_lane == 1 and m.state == 'AVOIDING'
    assert m.status['lane_switch_requested']
    assert m.status['obstacle_confirm_required'] == 1
    assert not m.decide(detections, FRAME, [(130,180),(130,479)], .1)[0]
    assert m.target_lane == 1


@pytest.mark.parametrize('width,height,expected', [(40,40,False),(64,96,True),(80,80,True)])
def test_area_threshold(width,height,expected):
    m=core()
    m.decide(scene(obstacle(width=width,height=height)),FRAME,PATH,0)
    assert m.status['lane_switch_requested'] is expected


@pytest.mark.parametrize('bottom,expected', [(299,False),(300,True),(301,True)])
def test_existing_bottom_gate_is_retained(bottom,expected):
    m=core()
    m.decide(scene(obstacle(y=bottom-40)),FRAME,PATH,0)
    assert m.status['lane_switch_requested'] is expected


def test_optional_zero_bottom_gate_allows_size_only_when_explicitly_selected():
    m=core(obstacle_near_y=0)
    m.decide(scene(obstacle(y=200)),FRAME,PATH,0)
    assert m.target_lane == 1


def test_normal_command_identity_and_unrelated_obstacle_do_not_change_lane():
    m=core()
    baseline=MotionCommand(-3,80,80)
    stop,limit=m.decide(scene(obstacle(x=560)),FRAME,PATH,0)
    assert m.apply(baseline,stop,limit) is baseline
    assert m.target_lane == 2


def test_full_frame_current_lane_mask_can_trigger_outside_planned_roi():
    m=core(obstacle_near_y=0)
    mask=np.zeros((480,640),np.uint8);mask[60:,380:460]=1
    detections=scene(obstacle(y=100),Detection(class_name='lane2',mask=Mask(bitmap=mask)))
    m.decide(detections,FRAME,[],0,lane_valid=False)
    assert m.target_lane == 1
    assert m.status['stop']  # Request, never drive with the old/invalid path.


def test_held_path_is_not_enough_without_current_lane_observation():
    m=core()
    m.decide(scene(obstacle()),FRAME,PATH,0,lane_valid=False)
    assert m.target_lane == 2


def test_destination_not_observed_stops_and_is_not_remembered_from_old_frame():
    m=core()
    m.decide(scene(),FRAME,PATH,0)
    assert m.status['alternate_visible']
    assert m.decide(scene(obstacle(),alternative=False),FRAME,PATH,1)[0]
    assert m.target_lane == 2 and m.state == 'BLOCKED_STOP'


def test_even_small_far_obstacle_on_destination_prevents_switch():
    m=core()
    mask=np.zeros((480,640),np.uint8);mask[50:,50:230]=1
    detections=scene(obstacle(),box('obstacle',130,y=100),
                     Detection(class_name='lane1',mask=Mask(bitmap=mask)))
    assert m.decide(detections,FRAME,PATH,0)[0]
    assert m.target_lane == 2 and m.status['alternate_blocked']


def test_missing_new_lane_path_and_red_light_still_stop():
    m=core()
    detections=scene(obstacle())
    m.decide(detections,FRAME,PATH,0)
    assert m.decide(detections,FRAME,[],.1,lane_valid=False)[0]
    red=core(wait_for_green=1)
    assert red.decide(detections,FRAME,PATH,0)[0]
    assert red.target_lane == 2


def test_reverse_direction_lane1_to_lane2():
    m=core();m.target_lane=1
    other=np.zeros((480,640),np.uint8);other[180:,380:460]=1
    detections=DetectionArray([obstacle(x=130),Detection(class_name='lane2',mask=Mask(bitmap=other))])
    m.decide(detections,FRAME,[(130,180),(130,479)],0)
    assert m.target_lane == 2


def test_book_style_cooldown_blocks_quick_switch_back():
    m=core(avoid_hold_s=0,obstacle_clear_frames=1)
    m.decide(scene(obstacle()),FRAME,PATH,0)
    other=np.zeros((480,640),np.uint8);other[180:,380:460]=1
    empty=DetectionArray([Detection(class_name='lane2',mask=Mask(bitmap=other))])
    new_path=[(130,180),(130,479)]
    m.decide(empty,FRAME,new_path,.1)
    assert m.state == 'NORMAL'
    blocked=DetectionArray([obstacle(x=130),*empty.detections])
    assert m.decide(blocked,FRAME,new_path,.2)[0]
    assert m.target_lane == 1 and 'LANE_CHANGE_COOLDOWN' in m.status['reasons']
    m.decide(blocked,FRAME,new_path,2.1)
    assert m.target_lane == 2


def test_resizing_frame_does_not_change_area_ratio_or_decision():
    m=core()
    frame=np.zeros((960,1280,3),np.uint8)
    mask=np.zeros((960,1280),np.uint8);mask[360:,100:460]=1
    det=obstacle(x=840,y=720,width=160,height=160)
    assert bbox_area_ratio(det,frame.shape) == bbox_area_ratio(obstacle(),FRAME.shape)
    m.decide(DetectionArray([det,Detection(class_name='lane1',mask=Mask(bitmap=mask))]),
             frame,[(840,360),(840,958)],0)
    assert m.target_lane == 1


@pytest.mark.parametrize('width,height', [(0,80),(-1,80),(80,0),(float('nan'),80)])
def test_invalid_boxes_do_not_trigger(width,height):
    m=core()
    det=obstacle(width=width,height=height)
    assert visible_box(det,FRAME.shape) is None
    m.decide(scene(det),FRAME,PATH,0)
    assert m.target_lane == 2


def test_obstacles_plural_alias_is_supported():
    m=core()
    m.decide(scene(obstacle(name='obstacles')),FRAME,PATH,0)
    assert m.target_lane == 1


def test_clipped_bottom_box_still_blocks_from_current_mask():
    m=core()
    mask=np.zeros((480,640),np.uint8);mask[180:,380:460]=1
    det=obstacle(y=470,height=160)
    m.decide(scene(det,Detection(class_name='lane2',mask=Mask(bitmap=mask))),
             FRAME,[],0,lane_valid=False)
    assert m.target_lane == 1
