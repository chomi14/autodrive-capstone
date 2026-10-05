"""Synthetic mission/parking checks: no cameras, serial devices or motor I/O."""
from types import SimpleNamespace
import math
import numpy as np
import pytest

from skku_track_drive_pkg.messages import (
    BoundingBox2D, Detection, DetectionArray, Mask, MotionCommand, Point2D, Pose2D, Vector2,
)
from skku_track_drive_pkg.mission_core import MissionCore, color_of_light
from skku_track_drive_pkg.mode_parameters import MISSION, defaults
from skku_track_drive_pkg.parking_core import ParkingCore
from skku_track_drive_pkg.lidar_geometry import scan_points


FRAME = np.zeros((480, 640, 3), np.uint8)
PATH = [(420, 180), (420, 479)]

def box(name, x, y=380, width=40, height=40):
    return Detection(class_name=name, bbox=BoundingBox2D(Pose2D(Point2D(x, y)), Vector2(width, height)))

def scene(*objects, alternative=True):
    detections = list(objects)
    if alternative:
        mask = np.zeros((480, 640), np.uint8)
        mask[180:, 50:230] = 1
        detections.append(Detection(class_name='lane1', mask=Mask(bitmap=mask)))
    return DetectionArray(detections)


def test_uninterrupted_mission_preserves_exact_stanley_command():
    mission = MissionCore()
    baseline = MotionCommand(-3, 80, 80)
    stop, limit = mission.decide(scene(box('obstacle', 100)), FRAME, PATH, 1.0)
    assert mission.apply(baseline, stop, limit) is baseline
    assert mission.target_lane == 2


def test_no_mission_does_not_replace_track_dropout_command():
    mission = MissionCore()
    baseline = MotionCommand(4, 80, 80)
    stop, limit = mission.decide(scene(alternative=False), FRAME, [], 1, lane_valid=False)
    assert mission.apply(baseline, stop, limit) is baseline


def test_path_obstruction_switches_only_to_visible_clear_lane():
    mission = MissionCore({'obstacle_confirm_frames': 2})
    obstacles = scene(box('obstacle', 420))
    assert mission.decide(obstacles, FRAME, PATH, 1)[0]
    assert mission.target_lane == 2
    assert mission.decide(obstacles, FRAME, PATH, 2)[0]
    assert mission.target_lane == 1
    assert mission.state == 'AVOIDING'
    assert mission.decide(obstacles, FRAME, [(130, 180), (130, 479)], 2.1)[0] is False
    # Never resume using held points when the selected avoidance lane is lost.
    assert mission.decide(obstacles, FRAME, [], 9, lane_valid=False)[0]
    assert mission.state == 'AVOIDING'


@pytest.mark.parametrize('alternative,extra', [(False, []), (True, [box('obstacle', 130)])])
def test_no_clear_escape_lane_stops_without_lane_switch(alternative, extra):
    mission = MissionCore({'obstacle_confirm_frames': 1})
    stop, _ = mission.decide(scene(box('obstacle', 420), *extra, alternative=alternative), FRAME, PATH, 1)
    assert stop and mission.state == 'BLOCKED_STOP'
    assert mission.target_lane == 2


def test_small_far_obstacle_does_not_trigger_avoidance():
    mission = MissionCore({'obstacle_confirm_frames': 1})
    stop, _ = mission.decide(scene(box('obstacle', 420, y=100)), FRAME, PATH, 1)
    assert not stop


def test_black_light_is_unknown_and_red_requires_confirmed_green():
    light = box('traffic_light', 80, y=120)
    mission = MissionCore({'red_confirm_frames': 2, 'green_confirm_frames': 2})
    assert color_of_light(FRAME, light, mission.p)[0] == 'Unknown'
    red = FRAME.copy()
    red[100:140, 60:100] = (0, 0, 255)
    green = FRAME.copy()
    green[100:140, 60:100] = (0, 255, 0)
    assert not mission.decide(scene(light), red, PATH, 1)[0]
    assert mission.decide(scene(light), red, PATH, 2)[0]
    assert mission.decide(scene(), FRAME, PATH, 3)[0]
    assert mission.decide(scene(light), green, PATH, 4)[0]
    assert not mission.decide(scene(light), green, PATH, 5)[0]


def test_yellow_reduces_speed_without_changing_steering():
    light = box('traffic_light', 80, y=120)
    frame = FRAME.copy()
    frame[100:140, 60:100] = (0, 255, 255)
    mission = MissionCore({'yellow_speed': 40})
    stop, limit = mission.decide(scene(light), frame, PATH, 1)
    assert mission.apply(MotionCommand(3, 80, 80), stop, limit) == MotionCommand(3, 40, 40)


def test_outside_frame_light_crop_does_not_crash():
    assert color_of_light(FRAME, box('traffic_light', -200), defaults(MISSION))[0] == 'Unknown'


MEASURED = dict(geometry_confirmed=1, wheelbase_m=.26, vehicle_width_m=.20,
                front_overhang_m=.12, rear_overhang_m=.10, lidar_x_m=.30,
                forward_mps=.10, reverse_mps=.10, max_wheel_angle_deg=30.0,
                slot_depth_m=.60, entry_lateral_m=.25, confirm_scans=1)

def side_points(distance):
    return np.array([[.30, -distance, -90.]])


def test_scan_uses_angle_metadata_mount_offset_and_handedness():
    core = ParkingCore('perpendicular')
    p = {**core.p, 'lidar_x_m': .4, 'scan_angle_sign': -1}
    scan = SimpleNamespace(ranges=[1., 2., math.inf, math.nan], angle_min=-math.pi/2,
                           angle_increment=math.pi/2, range_min=.1, range_max=12.)
    points = scan_points(scan, p)
    assert len(points) == 2
    assert points[0, :2] == pytest.approx([.4, 1.])
    assert points[1, :2] == pytest.approx([2.4, 0.])


@pytest.mark.parametrize('mode', ['perpendicular', 'parallel'])
def test_unmeasured_geometry_never_generates_motion(mode):
    core = ParkingCore(mode)
    assert core.step(.1, side_points(.8), True).left_speed == 0
    assert 'MEASURE' in core.status['waiting']


def test_timer_ticks_cannot_count_as_new_lidar_observations():
    p = {key: value for key, value in MEASURED.items() if key not in ('slot_depth_m', 'entry_lateral_m')}
    core = ParkingCore('perpendicular', {**p, 'confirm_scans': 3})
    core.step(.1, side_points(2.25), True, new_scan=True)
    for _ in range(10):
        core.step(.1, side_points(2.25), True, new_scan=False)
    assert core.state == 'SEARCH'
    core.step(.1, side_points(2.25), True, new_scan=True)
    core.step(.1, side_points(2.25), True, new_scan=True)
    assert core.state == 'FWD_TURN' and core.case == 4


def test_disarm_and_stale_scan_pause_timed_parking():
    p = {key: value for key, value in MEASURED.items() if key not in ('slot_depth_m', 'entry_lateral_m')}
    core = ParkingCore('perpendicular', p)
    core.enter('REV_TURN')
    core.step(.1, side_points(.8), True)
    elapsed = core.elapsed
    assert core.last_command.left_speed < 0
    for armed, fresh in [(False, True), (True, False)]:
        assert core.step(.1, side_points(.8), armed, fresh).left_speed == 0
        assert core.elapsed == elapsed


def test_unknown_side_is_not_a_clear_parking_gap():
    core = ParkingCore('parallel', MEASURED)
    core.enter('SEARCH_GAP')
    command = core.step(.1, np.empty((0, 3)), True)
    assert command.left_speed == 0
    assert core.state == 'SEARCH_GAP' and core.gap_start is None


def test_parallel_rejects_space_shorter_than_measured_vehicle():
    core = ParkingCore('parallel', MEASURED)
    core.enter('MEASURE_GAP')
    core.gap_start = .2
    core.pose[0] = .3
    core.step(.1, side_points(.8), True)
    assert core.state == 'SEARCH_GAP'


def test_parallel_executes_two_opposite_arcs_and_stops_without_exit():
    core = ParkingCore('parallel', MEASURED)
    core.enter('MEASURE_GAP')
    core.gap_start = 0.
    core.pose[0] = .5
    core.step(.1, side_points(.8), True)
    assert core.state == 'APPROACH_ENTRY'
    assert core.entry_x > .5
    visited, reverse_steering = set(), set()
    for _ in range(400):
        points = side_points(.8)
        if core.state == 'ALIGN':
            # Synthetic straight boundary in the current vehicle frame.
            yaw = core.pose[2]
            xs = np.linspace(.4, 1.4, 10)
            ys = -.8 - np.tan(yaw) * xs
            points = np.column_stack((xs, ys, np.full(10, -90.)))
        command = core.step(.1, points, True)
        visited.add(core.state)
        if command.left_speed < 0:
            reverse_steering.add(command.steering)
        if core.state in ('DONE', 'FAULT_STOP'):
            break
    assert {'REVERSE_ARC_IN', 'REVERSE_COUNTER_STEER', 'ALIGN', 'DONE'} <= visited, core.status
    assert {7, -7} <= reverse_steering
    assert core.last_command.left_speed == 0
    assert core.status['exit_supported'] is False


def test_perpendicular_parks_then_exits_only_when_enabled():
    p = {key: value for key, value in MEASURED.items() if key not in ('slot_depth_m', 'entry_lateral_m')}
    core = ParkingCore('perpendicular', p)
    core.enter('REV_SIDE_CHECK')
    assert core.step(.1, side_points(.8), True).left_speed == 0
    assert core.state == 'PARKED'
    for _ in range(60):
        core.step(.1, side_points(.8), True)
    assert core.state == 'PARKED'
    core.p['exit_enabled'] = 1
    core.step(.1, side_points(.8), True)
    assert core.state == 'EXIT_STRAIGHT'
    # Legacy finish duration is 30s: the default watchdog must allow it to
    # complete, including the tick crossing that duration.
    visited = set()
    for _ in range(650):
        core.step(.1, side_points(.8), True)
        visited.add(core.state)
        if core.state in ('DONE', 'FAULT_STOP'):
            break
    assert {'EXIT_TURN', 'EXIT_FINISH', 'DONE'} <= visited, core.status
    assert core.last_command.left_speed == 0
