"""Offline behavior tests; no models, ROS, serial devices or invented images."""
from dataclasses import replace
import math

import pytest

from skku_track_drive_pkg.modes.perpendicular_camera.core import CameraParkingCore, ControlConfig
from skku_track_drive_pkg.modes.perpendicular_camera.observations import Line, Observation, axis_error
from skku_track_drive_pkg.modes.perpendicular_camera.status import reason_text


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


def limit_line(y=.5):
    return Line(True, .5, y, 0.0)


def enter_reverse(s):
    for _ in range(4):
        # Both the boundary and the parking space can disappear together.
        assert s.tick(front_limit=limit_line()).left_speed == 0
        assert s.core.state == 'SEARCHING_SPACE'

    cmd = s.tick(front_limit=limit_line())
    assert s.core.state == 'ALIGN_TO_SPACE'
    assert (cmd.steering, cmd.left_speed) == (-7, 70)
    cmd = s.tick(front_limit=limit_line())
    assert (cmd.steering, cmd.left_speed) == (-7, 70)  # no rear observation

    cmd = s.tick(front_limit=limit_line(s.core.p.front_limit_stop_y))
    assert s.core.state == 'DIRECTION_PAUSE'
    assert cmd.left_speed == cmd.right_speed == 0
    assert s.core.reason == 'FRONT_LIMIT_REACHED'
    assert '평행주차선 접근' in reason_text(s.core.reason)
    for _ in range(8):
        cmd = s.tick('rear')  # valid frames without a centerline
        if s.core.state == 'REVERSE_RIGHT':
            break
        assert cmd.left_speed == 0
        assert s.core.reason == 'DIRECTION_PAUSE'
    assert (cmd.steering, cmd.left_speed) == (7, -60)
    assert s.core.state == 'REVERSE_RIGHT'
    cmd = s.tick('rear', repeat=s.ns-1)
    assert (cmd.steering, cmd.left_speed) == (7, -60)
    cmd = s.tick('rear', space_found=True)  # mask alone has no center/direction
    assert (cmd.steering, cmd.left_speed) == (7, -60)
    for _ in range(round(s.core.p.reverse_right_s * 10) - 3):
        cmd = s.tick('rear', space=rear_line(x=.7))
        assert (cmd.steering, cmd.left_speed) == (7, -60)
        assert s.core.state == 'REVERSE_RIGHT'
    cmd = s.tick('rear', space=rear_line(x=.7))
    assert s.core.state == 'PARKING_IN'
    assert (cmd.steering, cmd.left_speed) == (-3, -60)


def test_full_parking_stops_on_same_tick_and_holds_without_exit():
    s = Scenario(exit_enabled=False)
    enter_reverse(s)
    cmd = s.tick('rear', space=rear_line())
    assert cmd.left_speed == cmd.right_speed == -60
    assert s.tick('rear', end=Line(True, .5, .97, 0.0)).left_speed == -60
    cmd = s.tick('rear', end=Line(True, .5, .98, 0.0))
    assert s.core.state == 'STOPPED' and cmd.left_speed == 0
    for _ in range(50):
        assert s.tick('rear').left_speed == 0
    assert s.core.state == 'STOPPED'


def test_timed_exit_keeps_initial_straight_before_ten_second_turn_and_twenty_second_straight():
    s = Scenario()
    enter_reverse(s)
    assert s.tick('rear', end=Line(True, .5, .98, 0.0)).left_speed == 0
    for _ in range(23):
        assert s.tick(dt=.125).left_speed == 0
        assert s.core.state == 'STOPPED'
    assert s.tick(dt=.125).left_speed == 0
    assert s.core.state == 'ESCAPE'

    initial = s.tick(dt=.125)
    assert (initial.steering, initial.left_speed, initial.right_speed) == (0, 200, 200)
    for _ in range(11):
        assert s.tick(dt=.125) == initial
        assert s.core.state == 'ESCAPE'
    # An outline at the old finishing threshold must not bypass the turn.
    cmd = s.tick(dt=.125, outline=Line(True, .5, .95, 0.0))
    assert s.core.state == 'TURN_RIGHT'
    assert (cmd.steering, cmd.left_speed, cmd.right_speed) == (7, 110, 110)
    for _ in range(79):
        assert s.tick(dt=.125) == cmd  # normal front frames with no outline
        assert s.core.state == 'TURN_RIGHT'
    cmd = s.tick(dt=.125)
    assert (cmd.steering, cmd.left_speed, cmd.right_speed) == (0, 110, 110)
    assert s.core.state == 'FINAL_STRAIGHT'
    for _ in range(159):
        assert s.tick(dt=.125) == cmd
        assert s.core.state == 'FINAL_STRAIGHT'
    assert s.tick(dt=.125).left_speed == 0
    assert s.core.state == 'FINISHED'
    assert s.tick().left_speed == 0


@pytest.mark.parametrize('state, steering, pwm', [
    ('ESCAPE', 0, 200), ('TURN_RIGHT', 7, 110), ('FINAL_STRAIGHT', 0, 110),
])
def test_timed_exit_stops_for_stale_video_and_excludes_paused_time(state, steering, pwm):
    s = Scenario()
    s.enter(state)
    assert s.tick(dt=.125).left_speed == pwm
    for _ in range(8):
        assert s.tick(dt=.125).left_speed == pwm
    assert s.core.elapsed == 1.0
    assert s.tick(dt=.125, repeat=1).left_speed == 0
    assert s.core.reason == 'front_INPUT_STALE'
    assert s.tick(dt=.125, armed=False).left_speed == 0
    assert s.tick(dt=.125).left_speed == 0  # first W tick waits for a new frame
    cmd = s.tick(dt=.125)
    assert (cmd.steering, cmd.left_speed) == (steering, pwm)
    assert s.core.elapsed == 1.0
    assert s.core.state == state


def test_initial_missing_transitions_on_fifth_frame():
    s = Scenario()
    log = []
    s.core.on_transition = log.append
    enter_reverse(s)
    assert [event.split(' / ')[0] for event in log] == [
        'SEARCHING_SPACE → ALIGN_TO_SPACE', 'ALIGN_TO_SPACE → DIRECTION_PAUSE',
        'DIRECTION_PAUSE → REVERSE_RIGHT', 'REVERSE_RIGHT → PARKING_IN',
    ]
    assert '전방 주차칸 연속 5회 미검출' in log[0]
    assert '고정 우조향 후진 3초 완료' in log[-1]


def test_same_frame_invalid_and_missing_input_never_count_as_new_misses():
    s = Scenario(observation_timeout_s=2.0)
    s.tick(line=front_line())
    same = s.ns-1
    for _ in range(5):
        s.tick(repeat=same, line=front_line())
    s.core.step(.1, s.ns, True)
    assert s.core.reason == 'front_INPUT_MISSING'
    s.tick(valid=False)
    assert s.core.reason == 'front_OBSERVATION_INVALID'
    assert s.core.missing_count == 1


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
    assert s.tick(repeat=old, front_limit=limit_line()).left_speed == 0
    assert s.core.reason == 'front_WAIT_FRESH'


def test_alignment_checks_front_limit_and_ignores_rear_yaw_and_exit_line():
    s = Scenario()
    s.enter('ALIGN_TO_SPACE')
    s.ns += 100_000_000
    front = Observation(s.ns - 1, 'front', front_limit=limit_line(.7), outline=limit_line(.99))
    rear = Observation(s.ns - 1, 'rear', space=rear_line())
    cmd = s.core.step(.1, s.ns, True, front=front, rear=rear)
    assert s.core.state == 'ALIGN_TO_SPACE'
    assert (cmd.steering, cmd.left_speed) == (-7, 70)


@pytest.mark.parametrize('duration', [5.0, 2.5])
def test_alignment_without_front_limit_stops_after_configured_time_then_reverses(duration):
    s = Scenario(align_fallback_s=duration)
    for _ in range(5):
        cmd = s.tick()
    assert s.core.state == 'ALIGN_TO_SPACE'
    assert (cmd.steering, cmd.left_speed) == (-7, 70)

    for _ in range(round(duration * 10) - 1):
        cmd = s.tick()
        assert (cmd.steering, cmd.left_speed) == (-7, 70)
    cmd = s.tick()
    assert s.core.state == 'DIRECTION_PAUSE'
    assert cmd.left_speed == cmd.right_speed == 0
    assert f'전방 평행주차선 미검출 — 좌회전 전진 {duration:g}초' in reason_text(s.core.reason)

    for _ in range(5):
        assert s.tick('rear', dt=.125).left_speed == 0
    cmd = s.tick('rear', dt=.125)
    assert s.core.state == 'REVERSE_RIGHT'
    assert (cmd.steering, cmd.left_speed) == (7, -60)


def test_alignment_fallback_counts_neither_stale_input_nor_operator_stop():
    s = Scenario()
    s.enter('ALIGN_TO_SPACE')
    for _ in range(32):
        assert s.tick(dt=.125).left_speed == 70

    for _ in range(12):
        assert s.tick(repeat=1, dt=.125).left_speed == 0
    assert s.core.reason == 'front_INPUT_STALE'
    for _ in range(12):
        assert s.tick(armed=False, dt=.125).left_speed == 0
    assert s.tick(dt=.125).left_speed == 0  # W requires a post-W capture
    assert s.core.elapsed == 4.0

    for _ in range(7):
        assert s.tick(dt=.125).left_speed == 70
    assert s.tick(dt=.125).left_speed == 0
    assert s.core.reason == 'FRONT_LIMIT_FALLBACK: 5'


def test_alignment_uses_the_detected_line_approach_after_five_seconds():
    s = Scenario()
    s.enter('ALIGN_TO_SPACE')
    for _ in range(41):
        cmd = s.tick(front_limit=limit_line(), dt=.125)
        assert cmd.left_speed == 70
    assert s.core.state == 'ALIGN_TO_SPACE'
    assert s.tick(front_limit=limit_line(.85), dt=.125).left_speed == 0
    assert s.core.reason == 'FRONT_LIMIT_REACHED'


def test_missing_front_input_in_alignment_preserves_observation_watchdog():
    s = Scenario()
    s.enter('ALIGN_TO_SPACE')
    assert s.tick('rear').left_speed == 0
    assert s.core.reason == 'front_INPUT_MISSING'
    assert s.core.elapsed == 0
    for _ in range(52):
        s.tick('rear')
    assert s.core.state == 'FAULT_STOP'
    assert '전방 영상 입력 없음' in reason_text(s.core.reason)


def test_fixed_reverse_has_no_added_timeout_and_distinguishes_stale_rear_input():
    s = Scenario(maneuver_timeout_s=.3)
    s.enter('REVERSE_RIGHT')
    for _ in range(10):
        assert s.tick('rear', end=limit_line(.80)).left_speed == -60
    elapsed = s.core.elapsed
    stale = s.ns - 700_000_000
    assert s.tick('rear', repeat=stale).left_speed == 0
    assert s.core.reason == 'rear_INPUT_STALE'
    assert s.core.state == 'REVERSE_RIGHT'
    assert s.tick('rear').left_speed == -60
    assert s.core.elapsed == elapsed


def test_fixed_reverse_uses_configured_duration_even_with_centerline_at_entry():
    s = Scenario(reverse_right_s=1.25)
    s.enter('DIRECTION_PAUSE')
    for _ in range(5):
        assert s.tick('rear', dt=.125, space=rear_line(x=.7)).left_speed == 0
    for _ in range(10):
        cmd = s.tick('rear', dt=.125, space=rear_line(x=.7))
        assert (cmd.steering, cmd.left_speed) == (7, -60)
        assert s.core.state == 'REVERSE_RIGHT'
    cmd = s.tick('rear', dt=.125, space=rear_line(x=.7))
    assert s.core.state == 'PARKING_IN'
    assert (cmd.steering, cmd.left_speed) == (-3, -60)


def test_fixed_reverse_hands_off_without_centerline_and_holds_its_last_command():
    s = Scenario(reverse_right_s=.25)
    s.enter('REVERSE_RIGHT')
    previous = s.tick('rear', dt=.125)
    assert (previous.steering, previous.left_speed) == (7, -60)
    assert s.tick('rear', dt=.125) == previous
    assert s.tick('rear', dt=.125) == previous
    assert s.core.state == 'PARKING_IN'
    assert s.core.reason == 'REAR_PARKING_COMMAND_HELD'
    assert s.tick('rear', space=rear_line(x=.7)).steering == -3


def test_maneuver_timeout_even_with_fresh_observations():
    s = Scenario(maneuver_timeout_s=.3)
    s.enter('ALIGN_TO_SPACE')
    for _ in range(4):
        cmd = s.tick(front_limit=limit_line())
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
    assert s.tick('rear').left_speed == 0  # no tracked command from this arm cycle
    assert s.tick('rear', space=rear_line()).left_speed < 0


def test_tracking_keeps_last_reverse_command_when_fresh_images_lose_geometry():
    s = Scenario()
    s.enter('PARKING_IN')
    previous = s.tick('rear', space=rear_line(x=.7))
    assert s.core.state == 'PARKING_IN'
    assert (previous.steering, previous.left_speed, previous.right_speed) == (-3, -60, -60)

    for _ in range(60):  # exceeds the existing observation-loss watchdog period
        assert s.tick('rear', space_found=True) == previous  # mask without usable direction
    assert s.core.state == 'PARKING_IN'
    assert s.core.loss_elapsed == 0
    assert '이전 조향·PWM 유지' in reason_text(s.core.reason)

    updated = s.tick('rear', space=rear_line(x=.3))
    assert (updated.steering, updated.left_speed) == (3, -60)
    assert s.tick('rear') == updated

    end_steer = s.tick('rear', end=Line(True, .5, .85, .2))
    assert end_steer.steering == -7
    assert s.tick('rear') == end_steer
    assert s.tick('rear', end=Line(True, .5, .99, 0.)).left_speed == 0
    assert s.core.state == 'STOPPED'


@pytest.mark.parametrize('values, reason', [
    ({'repeat': 1}, 'rear_INPUT_STALE'),
    ({'valid': False}, 'rear_OBSERVATION_INVALID'),
])
def test_unusable_rear_input_stops_instead_of_replaying_last_reverse_command(values, reason):
    s = Scenario()
    s.enter('PARKING_IN')
    assert s.tick('rear', space=rear_line(x=.7)).left_speed == -60
    cmd = s.tick('rear', **values)
    assert (cmd.left_speed, cmd.right_speed) == (0, 0)
    assert s.core.reason == reason
    assert s.tick('rear').left_speed == 0
    assert s.tick('rear', space=rear_line(x=.7)).left_speed == -60


@pytest.mark.parametrize('values', [{}, {'valid': False}, {'end': Line(True, .5, .4, 0.0)}])
def test_reverse_without_tracked_command_waits_for_usable_geometry(values):
    s = Scenario()
    s.enter('PARKING_IN')
    cmd = s.tick('rear', **values)
    assert cmd.left_speed == cmd.right_speed == 0 and s.core.elapsed == 0


def test_end_only_can_control_when_in_end_zone_without_uninitialized_space():
    s = Scenario()
    s.enter('PARKING_IN')
    cmd = s.tick('rear', end=Line(True, .5, .8, .2))
    assert cmd.left_speed < 0 and cmd.steering == -7


def test_stale_and_future_sources_freeze_timers():
    s = Scenario()
    s.enter('PARKING_IN')
    for stamp in (1, s.ns+1_000_000_000):
        assert s.tick('rear', repeat=stamp, space=rear_line()).left_speed == 0
    assert s.core.elapsed == 0


@pytest.mark.parametrize('x,expected', [(.7, -3), (.3, 3), (.5, 0)])
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
    log = []
    s.core.on_transition = log.append
    s.core.fail('test fault', s.ns)
    s.core.missing_count = 3
    s.core.reset(s.ns)
    assert s.core.state == 'SEARCHING_SPACE' and s.core.missing_count == 0 and not s.core.fault
    assert log[-1].startswith('FAULT_STOP → SEARCHING_SPACE / ')
    assert axis_error(-math.pi/2) == 0


@pytest.mark.parametrize('kwargs', [dict(max_steering=8), dict(reverse_pwm=256),
    dict(end_stop_y=.5, end_steering_y=.7), dict(steering_sign=0),
    dict(space_missing_frames=0), dict(calibration_confirmed='false'), dict(align_steering=7),
    dict(rear_angle_gain=float('nan')), dict(direction_pause_s=0.0), dict(align_fallback_s=0.0),
    dict(reverse_right_s=0.0)])
def test_bad_control_settings_fail_before_runtime(kwargs):
    with pytest.raises(ValueError):
        ControlConfig(**kwargs)


def test_observation_serialization_preserves_source_and_rejects_invalid_geometry():
    obs = Observation(123, 'front', 'camera', front_limit=limit_line())
    assert Observation.from_json(obs.to_json()) == obs
    with pytest.raises(ValueError):
        Line(True, .5, float('nan'), 0.0)
    with pytest.raises(ValueError):
        Line(True, .5, 1250.0, 0.0)
