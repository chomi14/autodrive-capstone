"""Explicit opt-out removes obstacle stops, never red-light stops."""
import pytest
import numpy as np
from skku_track_drive_pkg.messages import Detection, DetectionArray, Mask, MotionCommand
from test_bbox_size_avoidance import core, obstacle
from test_mission_parking import FRAME, PATH, box, scene


def test_switch_frame_keeps_moving_and_preserves_speed_limit():
    m=core(obstacle_stop_enabled=0)
    stop,limit=m.decide(scene(obstacle()),FRAME,PATH,0)
    assert not stop and m.target_lane==1
    cmd=m.apply(MotionCommand(3,250,250),stop,limit)
    assert cmd.steering==3 and cmd.left_speed==cmd.right_speed==80
    assert m.status['reason']=='OBSTACLE_STOP_DISABLED'
    assert 'LANE_SWITCH_REQUESTED' in m.status['obstacle_warnings']


@pytest.mark.parametrize('extra,visible',[([],False),([obstacle(x=130)],True)])
def test_missing_or_blocked_alternate_does_not_stop(extra,visible):
    m=core(obstacle_stop_enabled=0)
    stop,limit=m.decide(scene(obstacle(),*extra,alternative=visible),FRAME,PATH,0)
    assert not stop and m.target_lane==2
    assert m.status['state']=='BLOCKED_NO_STOP'
    cmd=m.apply(MotionCommand(-2,80,80),stop,limit)
    assert cmd.left_speed==cmd.right_speed==80 and cmd.steering==-2


def test_missing_new_path_does_not_stop():
    m=core(obstacle_stop_enabled=0)
    detections=scene(obstacle())
    m.decide(detections,FRAME,PATH,0)
    stop,limit=m.decide(detections,FRAME,[],.1,lane_valid=False)
    assert not stop
    assert 'AVOIDANCE_PATH_NOT_CURRENTLY_VALID' in m.status['obstacle_warnings']
    assert m.apply(MotionCommand(4,80,80),stop,limit).left_speed==80


def test_cooldown_still_prevents_switch_but_not_motion():
    m=core(obstacle_stop_enabled=0,avoid_hold_s=0,obstacle_clear_frames=1)
    m.decide(scene(obstacle()),FRAME,PATH,0)
    other=np.zeros((480,640),np.uint8);other[180:,380:460]=1
    empty=DetectionArray([Detection(class_name='lane2',mask=Mask(bitmap=other))])
    path=[(130,180),(130,479)]
    m.decide(empty,FRAME,path,.1)
    stop,_=m.decide(DetectionArray([obstacle(x=130),*empty.detections]),FRAME,path,.2)
    assert not stop and m.target_lane==1
    assert 'LANE_CHANGE_COOLDOWN' in m.status['obstacle_warnings']


def test_red_still_stops_and_requires_green_to_release():
    m=core(obstacle_stop_enabled=0,red_confirm_frames=1,green_confirm_frames=1)
    light=box('traffic_light',80,y=120)
    red=FRAME.copy();red[100:140,60:100]=(0,0,255)
    green=FRAME.copy();green[100:140,60:100]=(0,255,0)
    stop,limit=m.decide(scene(obstacle(),light),red,PATH,0)
    assert stop and m.status['state']=='RED_STOP'
    cmd=m.apply(MotionCommand(2,80,80),stop,limit)
    assert cmd.left_speed==cmd.right_speed==0
    assert m.decide(scene(obstacle()),FRAME,PATH,.1)[0]
    assert not m.decide(scene(obstacle(),light),green,PATH,.2)[0]


@pytest.mark.parametrize('state',['NORMAL','AVOIDING','BLOCKED_STOP'])
@pytest.mark.parametrize('valid',[False,True])
def test_non_red_mission_never_requests_stop(state,valid):
    m=core(obstacle_stop_enabled=0)
    m.state=state;m.changed_at=0
    assert not m.decide(scene(obstacle(),alternative=False),FRAME,PATH,1,lane_valid=valid)[0]
