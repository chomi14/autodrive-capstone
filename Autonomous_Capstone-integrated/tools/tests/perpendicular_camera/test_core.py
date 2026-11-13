"""Offline behavior tests; no models, ROS, serial devices or invented images."""
from dataclasses import replace
import math

import pytest

from skku_track_drive_pkg.modes.perpendicular_camera.core import CameraParkingCore, ControlConfig
from skku_track_drive_pkg.modes.perpendicular_camera.observations import Line, Observation, axis_error


class Scenario:
    def __init__(self, **settings):
        self.ns = 1_000_000_000
        self.core = CameraParkingCore(ControlConfig(calibration_confirmed=True, **settings))
        self.core.step(.1, self.ns, True)  # explicit W/source-capture barrier

    def tick(self, role='front', repeat=None, armed=True, dt=.1, **values):
        self.ns += int(dt*1e9)
        stamp = self.ns-1 if repeat is None else repeat
        obs = Observation(stamp, role, **values)
        return self.core.step(dt, self.ns, armed, **{role: obs})

    def enter(self, state):
        self.core.enter(state, self.ns, 'test setup')


def front_line(x=.7):
    return Line(True, x, 1.0, math.pi/2)


def rear_line(x=.5, yaw=math.pi/2):
    return Line(True, x, 1.0, yaw)


def enter_reverse(s):
    for _ in range(3):
        s.tick(line=front_line(), space_found=True)
    for _ in range(3):
        s.tick(line=front_line(), space_found=False)
    assert s.core.state == 'ALIGN_TO_SPACE'
    for _ in range(3):
        s.tick('rear', space=rear_line())
    assert s.core.state == 'DIRECTION_PAUSE'
    for _ in range(8):
        cmd = s.tick('rear', space=rear_line())
        assert cmd.left_speed == 0
        if s.core.state == 'PARKING_IN':
            break
    assert s.core.state == 'PARKING_IN'


def test_full_parking_stops_on_same_tick_and_holds_without_exit():
    s = Scenario()
    enter_reverse(s)
    cmd = s.tick('rear', space=rear_line())
    assert cmd.left_speed == cmd.right_speed == -60
    cmd = s.tick('rear', end=Line(True, .5, .9, 0.0))
    assert s.core.state == 'STOPPED' and cmd.left_speed == 0
    for _ in range(50):
        assert s.tick('rear').left_speed == 0
    assert s.core.state == 'STOPPED'


def test_optional_exit_finishes_in_zero_command():
    s = Scenario(exit_enabled=True, stopped_hold_s=.8, escape_s=.3, final_straight_s=.3)
    enter_reverse(s)
    s.tick('rear', end=Line(True, .5, .9, 0.0))
    for _ in range(9):
        s.tick()
    assert s.core.state == 'ESCAPE'
    for _ in range(4):
        s.tick()
    assert s.core.state == 'TURN_RIGHT'
    assert s.tick(outline=Line(True, .5, .95, 0.0)).left_speed == 0
    assert s.core.state == 'FINAL_STRAIGHT'
    for _ in range(4):
        s.tick()
    assert s.core.state == 'FINISHED'
    assert s.tick().left_speed == 0


def test_initial_missing_never_selects_space():
    s = Scenario()
    for _ in range(40):
        s.tick(line=front_line(), space_found=False)
    assert s.core.state == 'SEARCHING_SPACE' and not s.core.seen_space


def test_same_frame_never_counts_more_than_once():
    s = Scenario(observation_timeout_s=2.0)
    s.tick(line=front_line(), space_found=True)
    same = s.ns-1
    for _ in range(5):
        s.tick(repeat=same, line=front_line(), space_found=True)
    assert s.core.seen_count == 1 and not s.core.seen_space


def test_noise_breaks_missing_confirmation_and_state_barrier_rejects_old_frame():
    s = Scenario()
    for _ in range(3):
        s.tick(line=front_line(), space_found=True)
    s.tick(line=front_line())
    s.tick(line=front_line())
    s.tick(line=front_line(), space_found=True)
    assert s.core.missing_count == 0
    old = s.ns-1
    s.enter('ALIGN_TO_SPACE')
    assert s.tick('rear', repeat=old, space=rear_line()).left_speed == 0
    assert s.core.align_count == 0


@pytest.mark.parametrize('yaw', [math.pi/2, -math.pi/2, math.pi/2+.2, -math.pi/2+.2])
def test_alignment_uses_pi_periodic_angle_error(yaw):
    s = Scenario()
    s.enter('ALIGN_TO_SPACE')
    for _ in range(3):
        s.tick('rear', space=rear_line(yaw=yaw))
    assert s.core.state == 'DIRECTION_PAUSE'


def test_alignment_missing_stops_and_never_changes_to_blind_forward():
    s = Scenario()
    s.enter('ALIGN_TO_SPACE')
    assert s.tick('rear').left_speed == 0
    assert s.core.elapsed == 0
    for _ in range(52):
        s.tick('rear')
    assert s.core.state == 'FAULT_STOP'


def test_maneuver_timeout_even_with_fresh_observations():
    s = Scenario(maneuver_timeout_s=.3)
    s.enter('ALIGN_TO_SPACE')
    for _ in range(4):
        cmd = s.tick('rear', space=rear_line(yaw=.1))
    assert s.core.state == 'FAULT_STOP' and cmd.left_speed == 0


def test_disarm_freezes_timers_and_resume_requires_new_capture():
    s = Scenario()
    s.enter('PARKING_IN')
    s.tick('rear', space=rear_line())
    elapsed = s.core.elapsed
    stamp = s.ns-1
    for _ in range(10):
        assert s.tick('rear', armed=False, space=rear_line()).left_speed == 0
    assert s.core.elapsed == elapsed
    assert s.tick('rear', repeat=stamp, space=rear_line()).left_speed == 0
    assert s.core.elapsed == elapsed
    assert s.tick('rear', space=rear_line()).left_speed < 0


@pytest.mark.parametrize('values', [{}, {'valid': False}, {'end': Line(True, .5, .4, 0.0)}])
def test_reverse_missing_or_unusable_geometry_stops(values):
    s = Scenario()
    s.enter('PARKING_IN')
    cmd = s.tick('rear', **values)
    assert cmd.left_speed == cmd.right_speed == 0 and s.core.elapsed == 0


def test_end_only_can_control_when_in_end_zone_without_uninitialized_space():
    s = Scenario()
    s.enter('PARKING_IN')
    cmd = s.tick('rear', end=Line(True, .5, .8, .2))
    assert cmd.left_speed < 0 and cmd.steering == -2


def test_stale_and_future_sources_freeze_timers():
    s = Scenario()
    s.enter('PARKING_IN')
    for stamp in (1, s.ns+1_000_000_000):
        assert s.tick('rear', repeat=stamp, space=rear_line()).left_speed == 0
    assert s.core.elapsed == 0


@pytest.mark.parametrize('x,expected', [(.7, -2), (.3, 2), (.5, 0)])
def test_reverse_center_sign(x, expected):
    s = Scenario()
    s.enter('PARKING_IN')
    assert s.tick('rear', space=rear_line(x=x)).steering == expected


def test_float_gain_rounding_deadband_and_saturation():
    core = CameraParkingCore(ControlConfig(calibration_confirmed=True))
    assert core.command(.29, 50).steering == 0
    assert core.command(.6, 50).steering == 1
    assert core.command(-.6, 50).steering == -1
    assert core.command(300, 50).steering == 7
    assert core.command(-300, 50).steering == -7
    core.p = replace(core.p, steering_sign=-1)
    assert core.command(.6, 50).steering == -1


def test_unconfirmed_live_core_never_moves():
    s = Scenario()
    s.core.p = replace(s.core.p, calibration_confirmed=False)
    assert s.tick(line=front_line()).left_speed == 0
    assert s.core.reason == 'CALIBRATION_NOT_CONFIRMED'


def test_reset_clears_fault_and_observation_history():
    s = Scenario()
    s.core.fail('test fault', s.ns)
    s.core.seen_space = True
    s.core.reset(s.ns)
    assert s.core.state == 'SEARCHING_SPACE' and not s.core.seen_space and not s.core.fault
    assert axis_error(-math.pi/2) == 0


@pytest.mark.parametrize('kwargs', [dict(max_steering=8), dict(reverse_pwm=256),
    dict(end_stop_y=.5, end_steering_y=.7), dict(steering_sign=0),
    dict(confirm_frames=0), dict(calibration_confirmed='false'),
    dict(rear_angle_gain=float('nan')), dict(direction_pause_s=0.0)])
def test_bad_control_settings_fail_before_runtime(kwargs):
    with pytest.raises(ValueError):
        ControlConfig(**kwargs)


def test_observation_serialization_preserves_source_and_rejects_invalid_geometry():
    obs = Observation(123, 'rear', 'camera', space=rear_line())
    assert Observation.from_json(obs.to_json()) == obs
    with pytest.raises(ValueError):
        Line(True, .5, float('nan'), 0.0)
    with pytest.raises(ValueError):
        Line(True, .5, 1250.0, 0.0)
