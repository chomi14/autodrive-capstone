"""Camera-free tests for the integrated parallel parking phase."""
from dataclasses import replace
import math

import numpy as np
import pytest

from skku_track_drive_pkg.modes.perpendicular_camera.core import CameraParkingCore, ControlConfig
from skku_track_drive_pkg.modes.perpendicular_camera.observations import Line, Observation, ParallelBay, ParallelView
from skku_track_drive_pkg.modes.perpendicular_camera.parallel import extract_parallel


def vertical(x):
    return Line(True, x, 1.0, math.pi/2)


class Run:
    def __init__(self):
        self.core = CameraParkingCore(ControlConfig(escape_s=.3, direction_pause_s=.2, exit_turn_s=1.0),
                                      require_calibration=False, parallel_enabled=True)
        self.now = 1_000_000_000
        self.core.enter('ESCAPE', self.now, 'parked')
        self.core.step(0, self.now, True)

    def tick(self, front=None, rear=None, *, previous_model=False, stale=(), armed=True):
        self.now += 100_000_000
        stamp = self.now-1
        if front is not None:
            front = Observation(stamp, 'front', model='parallel', parallel=front)
        elif previous_model:
            front = Observation(stamp, 'front', line=vertical(.35))
        if rear is not None:
            rear = Observation(stamp, 'rear', model='parallel', parallel=rear)
        return self.core.step(.1, self.now, armed, front, rear, stale_cameras=stale)

    def start_turn(self):
        for _ in range(3):
            command = self.tick(previous_model=True)
            assert (command.steering, command.left_speed) == (0, 200)
        command = self.tick(previous_model=True)
        assert (command.steering, command.left_speed) == (7, 110)
        assert self.core.state == 'TURN_RIGHT' and self.core.model == 'parallel'


def test_acquired_roadside_exits_right_turn_in_same_tick_and_steers_for_x035():
    run = Run()
    run.start_turn()
    assert run.tick(previous_model=True).steering == 7  # Old model's x=.35 cannot trigger.
    assert run.tick(ParallelView()).steering == 7
    command = run.tick(ParallelView(roadside=vertical(.32)))
    assert run.core.state == 'PARALLEL_APPROACH'
    assert abs(run.core.errors['center'] + .03) < 1e-9
    assert (command.steering, command.left_speed) == (-4, 70)
    assert '통로선 검출' in run.core.transition
    assert run.tick(ParallelView(roadside=vertical(.35))).steering == 0
    assert run.tick(ParallelView(roadside=vertical(.39))).steering > 0


def test_turn_limit_ends_right_steer_even_with_no_roadside():
    run = Run()
    run.start_turn()
    for _ in range(9):
        assert run.tick(ParallelView()).steering == 7
    command = run.tick(ParallelView())
    assert run.core.state == 'PARALLEL_APPROACH'
    assert (command.steering, command.left_speed) == (0, 70)
    assert run.core.reason == 'PARALLEL_ROADSIDE_MISSING'


def test_parallel_hands_off_from_left_and_straight_reverse_to_far_line_control():
    run = Run()
    run.start_turn()
    front = ParallelView(roadside=vertical(.35), route_target=3)
    rear = ParallelView(roadside=vertical(.65), depth=-.25, yaw_error=0.0,
                        bays=(ParallelBay(3, .84),))
    assert run.tick(front, rear, stale=('rear',)).left_speed == 70
    assert run.core.parallel_target == 3
    wrong_bay = replace(rear, bays=(ParallelBay(2, .84, target_mask=True),))
    assert run.tick(ParallelView(roadside=vertical(.35)), wrong_bay).left_speed == 70
    assert run.tick(front, rear).left_speed == 0
    assert run.core.state == 'PARALLEL_PAUSE'
    assert run.tick(rear=rear).left_speed == 0
    command = run.tick(rear=rear)
    assert run.core.state == 'PARALLEL_REVERSE_LEFT'
    assert (command.steering, command.left_speed) == (-7, -60)
    command = run.tick(rear=replace(rear, depth=.12, yaw_error=-.45, far_gap=.88, far_yaw_error=-.45))
    assert run.core.state == 'PARALLEL_REVERSE_STRAIGHT'
    assert (command.steering, command.left_speed) == (0, -60)
    assert run.tick(rear=replace(rear, depth=.26, yaw_error=-.45)).steering == 0
    command = run.tick(rear=replace(rear, depth=.26, yaw_error=-.45, far_gap=.74, far_yaw_error=-.45))
    assert run.core.state == 'PARALLEL_FAR_ALIGN'
    assert command.left_speed == -60 and command.steering > 0
    assert run.tick(rear=ParallelView()) == command
    assert run.core.reason == 'PARALLEL_REVERSE_COMMAND_HELD'
    assert run.tick(rear=replace(rear, depth=.62, yaw_error=-.1)).left_speed == -60
    assert run.core.reason == 'PARALLEL_FAR_LINE_LOST_USING_ROADSIDE'
    assert run.tick(rear=replace(rear, depth=.62, yaw_error=-.1, far_gap=.38, far_yaw_error=-.1)).left_speed == 0
    assert run.core.state == 'FINISHED'
    run.core.reset(run.now)
    assert run.core.model == 'perpendicular' and run.core.parallel_target == 0


def test_model_handoff_and_reverse_refuse_stale_frames():
    run = Run()
    run.start_turn()
    line = ParallelView(roadside=vertical(.35))
    assert run.tick(line, stale=('front',)).left_speed == 0
    assert run.core.state == 'TURN_RIGHT'
    assert run.tick(previous_model=True).left_speed == 0
    assert run.tick(line).left_speed == 70
    rear = ParallelView(roadside=vertical(.65), depth=-.15, yaw_error=0.0,
                        bays=(ParallelBay(0, .84, target_mask=True),))
    assert run.tick(line, rear).left_speed == 0
    assert run.core.state == 'PARALLEL_PAUSE'
    assert run.tick(rear=rear, stale=('rear',)).left_speed == 0
    assert run.tick(rear=rear).left_speed == 0
    assert run.tick(rear=ParallelView()).left_speed == -60
    assert run.tick(rear=replace(rear, far_gap=0.0, far_yaw_error=-.45)).left_speed == 0
    assert run.core.reason == 'PARALLEL_FAR_LINE_REACHED'


def rect(x0, y0, x1, y1):
    return np.float32([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])


def geometry(polygons, role='rear'):
    corners = [0., 1., 1., 1., 1., 0., 0., 0.]
    cfg = {'persp_src': corners, 'persp_dst': corners, 'min_longitudinal_span_ratio': .05}
    frame = np.zeros((480, 640, 3), np.uint8)
    obs, _, _ = extract_parallel(frame, polygons, cfg, ControlConfig(), role, 1)
    assert Observation.from_json(obs.to_json()) == obs
    return obs.parallel


@pytest.mark.parametrize('occupied_y, expected', [(180, 2), (350, 3)])
def test_rear_geometry_indexes_from_row_end_without_seeing_bay_one(occupied_y, expected):
    view = geometry({
        'parallel_roadside_line': [rect(414, 25, 418, 465)],
        'parallel_far_line': [rect(555, 25, 559, 465)],
        'parallel_divider': [rect(416, 93, 557, 97), rect(416, 266, 557, 270)],
        'parallel_row_end': [rect(416, 439, 557, 443)],
        'parallel_parking_space': [rect(422, 99, 551, 264), rect(40, 99, 180, 264)],
        'vehicle': [rect(460, occupied_y-26, 510, occupied_y), rect(110, 320, 160, 350)],
    })
    assert view.route_target == expected
    assert view.roadside.x == pytest.approx(.65, abs=.01)
    assert [b.number for b in view.bays] == [3, 4]
    assert [b.number for b in view.bays if b.occupied] == [3 if expected == 2 else 4]
    assert [b.number for b in view.bays if b.target_mask] == [3]


def test_space_mask_can_fill_one_edge_and_wrong_row_mask_cannot_create_a_bay():
    view = geometry({
        'parallel_roadside_line': [rect(414, 25, 418, 465)],
        'parallel_divider': [rect(416, 390, 557, 394)],
        'parallel_parking_space': [rect(421, 220, 552, 388), rect(40, 220, 180, 388)],
    })
    assert len(view.bays) == 1 and view.bays[0].target_mask
    assert view.bays[0].entry_y == pytest.approx(.82, abs=.02)
    rear = geometry({'parallel_far_line': [rect(381, 25, 385, 465)]})
    assert rear.depth == pytest.approx(.55, abs=.02)
    assert rear.far_gap == pytest.approx(.45, abs=.02)
