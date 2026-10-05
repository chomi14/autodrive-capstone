"""Deterministic clock, real sender callbacks, fake serial: no hardware."""
import json
import math

import pytest
from std_msgs.msg import Bool, String
from interfaces_pkg.msg import MotionCommand

from vehicle_io_pkg.perception_delay_guard import PerceptionDelayGuard
from skku_track_drive_pkg.perception_progress import PerceptionProgress
from vehicle_bringup_pkg.perception_policy import measured_policy
import test_drive_safety as safety


def progress(sequence, age=0, camera_age=0, frame_age=None):
    return {'result_sequence': sequence, 'result_age_s': age,
            'result_frame_age_s': age if frame_age is None else frame_age,
            'camera_age_s': camera_age}


@pytest.mark.parametrize('invalid', [0, -1, math.inf, math.nan])
def test_guard_rejects_unbounded_or_zero_windows(invalid):
    with pytest.raises(ValueError):
        PerceptionDelayGuard(invalid)


def test_full_pwm_and_steering_hold_until_existing_six_second_stop():
    guard = PerceptionDelayGuard(6)
    guard.observe(progress(1), 10)
    for elapsed in (0, 2, 2.99, 3, 3.75, 4.5, 5.25, 5.99):
        assert guard.output(-5, 100, 100, 10 + elapsed) == (-5, 100, 100)
    assert guard.output(-5, 100, 100, 16) == (-5, 0, 0)
    assert guard.expired(16)


def test_repeated_health_and_camera_frames_do_not_renew_result_age():
    guard = PerceptionDelayGuard(6)
    guard.observe(progress(1), 10)
    for tick in range(1, 61):
        guard.observe(progress(1, tick / 10, camera_age=0), 10 + tick / 10)
    assert guard.expired(16)
    assert guard.reason(16) == 'inference worker/result stopped'


def test_camera_stop_is_distinguishable_from_worker_stop():
    guard = PerceptionDelayGuard(6)
    guard.observe(progress(1), 10)
    guard.observe(progress(1, 6, camera_age=6), 16)
    assert guard.expired(16) and guard.reason(16) == 'camera input stopped'


def test_just_completed_old_frame_does_not_renew_the_stop_deadline():
    guard = PerceptionDelayGuard(6)
    guard.observe(progress(1), 10)
    assert guard.output(3, 100, 100, 14) == (3, 100, 100)
    guard.observe(progress(2, age=0, frame_age=4), 14)
    assert guard.age(14) == 4
    assert guard.output(3, 100, 100, 15.99) == (3, 100, 100)
    assert guard.expired(16)
    guard.observe(progress(3, age=0, frame_age=.02), 16.1)
    assert guard.ready(16.1)


def test_mission_zero_is_held_without_reconstructing_prior_pwm():
    guard = PerceptionDelayGuard(6)
    guard.observe(progress(1), 10)
    assert guard.output(3, 100, 100, 13) == (3, 100, 100)
    for elapsed in (3.1, 4.5, 5.99, 6):
        assert guard.output(0, 0, 0, 10 + elapsed) == (0, 0, 0)


def test_signed_pwm_and_single_zero_wheel_are_held_unchanged():
    guard = PerceptionDelayGuard(6)
    guard.observe(progress(1), 10)
    for elapsed in (3, 4.5, 5.99):
        assert guard.output(-4, -100, 0, 10 + elapsed) == (-4, -100, 0)


def test_new_w_needs_a_post_stop_result_sequence():
    guard = PerceptionDelayGuard(6)
    guard.observe(progress(5), 10)
    guard.reset_run()
    guard.observe(progress(5, .1), 10.1)
    assert not guard.ready(10.1)
    guard.observe(progress(6), 10.2)
    assert guard.ready(10.2)


@pytest.mark.parametrize('mode', ['track', 'mission'])
def test_existing_six_second_deadline_is_preserved_without_reduction_parameters(mode):
    policy = measured_policy(mode)
    assert policy == {'perception_stop_s': 6.0}


def test_progress_camera_receipt_and_failures_do_not_count_as_success():
    state = PerceptionProgress()
    state.receive(10)
    state.start()
    state.finish(failed=True)
    snapshot = state.snapshot()
    assert snapshot['camera_sequence'] == 1 and snapshot['result_sequence'] == 0
    assert snapshot['result_age_s'] is None and not snapshot['worker_busy']
    state.complete(11, 10)
    assert state.snapshot()['result_sequence'] == 1


@pytest.fixture
def fixture():
    helper = safety.DriveSafetyTests(methodName='test_initial_disarm_and_legacy_true_cannot_start')
    helper.setUp()
    helper.node.require_controller_heartbeat = True
    helper.node.require_tuner_heartbeat = True
    helper.node.delay_guard = PerceptionDelayGuard(6)
    helper.sequence = 1
    helper.source_time = helper.clock
    def heartbeat(fresh=False, camera=True):
        if fresh:
            helper.sequence += 1
            helper.source_time = helper.clock
        helper.node.on_controller_heartbeat(String(data=json.dumps({
            'challenge': helper.node.challenge, 'instance': 'controller-a', 'active': True,
            'perception': progress(helper.sequence, helper.clock - helper.source_time,
                                   0 if camera else helper.clock - helper.source_time)})))
    helper.heartbeat = heartbeat
    heartbeat()
    helper.arm()
    helper.node.enforce_safe_state()
    try:
        yield helper
    finally:
        helper.doCleanups()


@pytest.mark.parametrize('camera', [True, False])
def test_sender_holds_unchanged_then_stops_and_recovery_needs_w(fixture, camera):
    h = fixture
    stale_token = h.node.challenge
    first_time = h.clock
    output = []
    for tick in range(1, 123):
        h.clock = first_time + tick * .05
        h.heartbeat(camera=camera)
        h.feed_ui()
        h.node.enforce_safe_state()
        output.append(h.port.events[-1])
    moving = [frame for frame in output if frame != b'X\n']
    assert len(moving) >= 118
    assert all(frame == b's3l80r80\n' for frame in moving)
    assert not any(frame in h.port.events for frame in
                   (b's3l64r64\n', b's3l48r48\n', b's3l32r32\n', b's3l16r16\n'))
    assert b's3l0r0\n' in h.port.events
    assert not h.node.armed and h.port.events[-1] == b'X\n'
    assert 'perception stale' in h.node.stop_reason
    h.heartbeat(fresh=True)
    h.feed_ui()
    h.node.on_cmd(h.command)
    h.node.on_arm_request(String(data=stale_token))
    h.node.enforce_safe_state()
    assert not h.node.armed
    h.node.on_arm_request(String(data=h.node.challenge))
    h.node.enforce_safe_state()
    assert h.node.armed


def test_s_is_immediate_during_command_hold_after_three_seconds(fixture):
    h = fixture
    start = h.clock
    for tick in range(1, 71):
        h.clock = start + tick * .05
        h.heartbeat()
        h.feed_ui()
        h.node.enforce_safe_state()
    assert h.port.events[-1] == b's3l80r80\n' and h.node.armed
    h.node.on_arm(Bool(data=False))
    assert h.port.events[-1] == b'X\n' and not h.node.armed


def test_short_result_gap_and_fresh_recovery_do_not_reduce_pwm(fixture):
    h = fixture
    start = h.clock
    for tick in range(1, 51):
        h.clock = start + tick * .05
        h.heartbeat()
        h.feed_ui()
        h.node.enforce_safe_state()
        assert h.node.armed and h.port.events[-1] == b's3l80r80\n'
    h.heartbeat(fresh=True)
    h.node.on_cmd(h.command)
    h.node.enforce_safe_state()
    assert h.port.events[-1] == b's3l80r80\n'


def test_late_success_cannot_erase_terminal_stop(fixture):
    h = fixture
    start = h.clock
    for tick in range(1, 120):
        h.clock = start + tick * .05
        h.heartbeat()
        h.feed_ui()
        h.node.enforce_safe_state()
    h.clock = start + 6.001
    token = h.node.challenge
    h.heartbeat(fresh=True)
    assert not h.node.armed and h.node.challenge != token
    assert h.port.events[-1] == b'X\n'


def test_sender_holds_mission_zero_until_terminal_disarm(fixture):
    h = fixture
    start = h.clock
    for tick in range(1, 119):
        h.clock = start + tick * .05
        h.heartbeat()
        h.feed_ui()
        if tick == 70:
            h.node.on_cmd(MotionCommand(steering=-2, left_speed=0, right_speed=0))
        h.node.enforce_safe_state()
        expected = b's3l80r80\n' if tick < 70 else b's-2l0r0\n'
        assert h.node.armed and h.port.events[-1] == expected
    h.clock = start + 6
    h.heartbeat()
    assert not h.node.armed and h.port.events[-1] == b'X\n'
