"""Pure camera parking FSM, inspired by 4th SKKU motion_parking.py.

No camera, ROS or serial effects. PWM is not velocity; steering is a signed
discrete actuator request, positive RIGHT before the installation sign.
"""
from dataclasses import dataclass, fields
import math

from skku_track_drive_pkg.messages import MotionCommand
from .observations import axis_error
from .status import reason_text


@dataclass(frozen=True)
class ControlConfig:
    calibration_confirmed: bool = False
    observation_timeout_s: float = 0.5
    observation_loss_fault_s: float = 5.0
    space_missing_frames: int = 5
    forward_pwm: int = 70
    reverse_pwm: int = 60
    exit_pwm: int = 70
    max_steering: int = 7
    steering_sign: int = 1
    front_target_x: float = 0.7
    rear_target_x: float = 0.5
    front_position_gain: float = 135.0
    front_angle_gain: float = 8.771929824561404
    rear_position_gain: float = 14.0
    rear_angle_gain: float = 16.0
    end_angle_gain: float = 50.0
    exit_position_gain: float = 0.5
    exit_angle_gain: float = 10.0
    steering_deadband: float = 0.3
    align_steering: int = -7
    align_fallback_s: float = 5.0
    front_limit_stop_y: float = 0.80
    direction_pause_s: float = 0.7
    reverse_right_s: float = 3.0
    end_steering_y: float = 1000 / 1280
    end_stop_y: float = 1250 / 1280
    stopped_hold_s: float = 3.0
    exit_enabled: bool = True
    escape_s: float = 1.5
    outline_finish_y: float = 0.90
    final_straight_s: float = 2.5
    search_timeout_s: float = 60.0
    maneuver_timeout_s: float = 25.0

    def __post_init__(self):
        # Exact types catch quoted booleans/integers and NaNs before arming.
        for item in fields(self):
            value = getattr(self, item.name)
            if type(value) is not type(item.default) or (type(value) in (int, float) and not math.isfinite(value)):
                raise ValueError(f'Invalid type/value for control.{item.name}')
        if not 1 <= self.space_missing_frames <= 100:
            raise ValueError('Space missing count must be in 1..100')
        if not (1 <= self.max_steering <= 7 and self.steering_sign in (-1, 1)):
            raise ValueError('Invalid confirmation or steering limits')
        if not all(1 <= pwm <= 255 for pwm in (self.forward_pwm, self.reverse_pwm, self.exit_pwm)):
            raise ValueError('PWM must be in 1..255')
        if not -self.max_steering <= self.align_steering < 0:
            raise ValueError('align_steering must request a left turn within the output range')
        for name in ('front_target_x', 'rear_target_x', 'outline_finish_y', 'front_limit_stop_y'):
            if not 0 <= getattr(self, name) <= 1:
                raise ValueError(f'{name} outside normalized image')
        if not 0 <= self.end_steering_y < self.end_stop_y <= 1:
            raise ValueError('Require 0 <= end_steering_y < end_stop_y <= 1')
        if self.steering_deadband < 0:
            raise ValueError('steering_deadband must be nonnegative')
        for item in fields(self):
            if item.name.endswith('_s') and getattr(self, item.name) <= 0:
                raise ValueError(f'{item.name} must be positive')
        if self.stopped_hold_s < self.direction_pause_s:
            raise ValueError('stopped_hold_s must cover reverse-to-forward stop')


class CameraParkingCore:
    def __init__(self, config=None, require_calibration=True, on_transition=None):
        self.p = config or ControlConfig()
        self.require_calibration = require_calibration
        self.on_transition = on_transition
        self.state = 'SEARCHING_SPACE'
        self.transition = ''
        self.reset()

    def reset(self, now_ns=0, reason='RESET'):
        self.enter('SEARCHING_SPACE', now_ns, reason)
        self.search_frame_stamp = 0
        self.was_armed = False
        self.fault = ''
        self.errors = {}

    def enter(self, state, now_ns, reason, *, observation_stamp_ns=None):
        previous = self.state
        self.state = state
        # Immediate transitions using the same camera may reuse the validated
        # source frame until a newer one arrives or the freshness budget expires.
        self.barrier_ns = now_ns if observation_stamp_ns is None else observation_stamp_ns - 1
        self.elapsed = self.loss_elapsed = 0.0
        self.missing_count = 0
        self.last_reverse_command = None
        self.reason = reason
        if previous != state:
            self.transition = f'{previous} → {state} / {reason_text(reason)}'
            if self.on_transition is not None:
                self.on_transition(self.transition)

    def fail(self, reason, now_ns):
        self.fault = reason
        self.enter('FAULT_STOP', now_ns, reason)
        return MotionCommand()

    def command(self, steering=0.0, speed=0):
        # Round once, after continuous gains/deadband and physical saturation.
        p = self.p
        value = steering * p.steering_sign
        if abs(value) <= p.steering_deadband:
            value = 0.0
        value = max(-p.max_steering, min(p.max_steering, value))
        step = int(math.copysign(math.floor(abs(value) + .5), value))
        return MotionCommand(step, speed, speed)

    def _wait(self, reason, dt, now_ns):
        self.reason = reason
        self.last_reverse_command = None
        # Maneuver elapsed freezes. A separate loss watchdog prevents a hidden
        # indefinite wait while the operator believes a maneuver is active.
        self.loss_elapsed += dt
        if self.loss_elapsed >= self.p.observation_loss_fault_s:
            return self.fail('OBSERVATION_LOSS_TIMEOUT: ' + reason, now_ns)
        return MotionCommand()

    def _observation_problem(self, camera, obs, now_ns, arrival_stale=False):
        if obs is None or obs.camera != camera:
            return camera + '_INPUT_MISSING'
        if arrival_stale or not 0 <= (now_ns - obs.stamp_ns) / 1e9 <= self.p.observation_timeout_s:
            return camera + '_INPUT_STALE'
        if not obs.valid:
            return camera + '_OBSERVATION_INVALID'
        if obs.stamp_ns <= self.barrier_ns:
            return camera + '_WAIT_FRESH'
        return ''

    def observe_front(self, obs, now_ns):
        # The adapter also calls this on reception so an intervening detection
        # cannot be overwritten by a newer frame before the control tick.
        if (self.state == 'SEARCHING_SPACE' and self.was_armed
                and self.missing_count < self.p.space_missing_frames
                and obs.stamp_ns > self.search_frame_stamp
                and not self._observation_problem('front', obs, now_ns)):
            self.search_frame_stamp = obs.stamp_ns
            self.missing_count = 0 if obs.space_found else self.missing_count + 1

    def step(self, dt, now_ns, armed, front=None, rear=None, *, stale_cameras=()):
        p = self.p
        self.errors = {}
        dt = max(0.0, min(float(dt), .2))
        if not armed:
            self.was_armed = False
            self.missing_count = 0
            self.reason = 'DISARMED'
            return MotionCommand()
        if not self.was_armed:
            # A W/resume must wait for captures made AFTER W, including after S.
            self.barrier_ns = max(self.barrier_ns, now_ns)
            self.missing_count = 0
            self.last_reverse_command = None
            self.was_armed = True
        if self.state in ('FAULT_STOP', 'FINISHED'):
            self.reason = self.fault or 'FINISHED'
            return MotionCommand()
        if self.require_calibration and not p.calibration_confirmed:
            self.reason = 'CALIBRATION_NOT_CONFIRMED'
            return MotionCommand()

        if self.state == 'STOPPED':
            self.reason = 'PARKED'
            self.elapsed += dt
            if p.exit_enabled and self.elapsed >= p.stopped_hold_s:
                self.enter('ESCAPE', now_ns, 'PARKED_HOLD_COMPLETE')
            return MotionCommand()

        camera = 'rear' if self.state in ('DIRECTION_PAUSE', 'REVERSE_RIGHT', 'PARKING_IN') else 'front'
        obs = rear if camera == 'rear' else front
        problem = self._observation_problem(camera, obs, now_ns, camera in stale_cameras)
        if problem:
            return self._wait(problem, dt, now_ns)

        if self.state == 'REVERSE_RIGHT':
            # After a stop, the first usable frame starts commanding reverse
            # again. Count only intervals that followed a reverse command.
            if self.last_reverse_command is None:
                dt = 0.0
            if self.elapsed + dt + 1e-9 >= p.reverse_right_s:
                last_command = self.last_reverse_command
                self.enter('PARKING_IN', now_ns, f'RIGHT_REVERSE_COMPLETE: {p.reverse_right_s:g}',
                           observation_stamp_ns=obs.stamp_ns)
                self.last_reverse_command = last_command
                dt = 0.0

        end_can_steer = obs.end.found and obs.end.y >= p.end_steering_y
        if self.state == 'SEARCHING_SPACE':
            self.observe_front(obs, now_ns)
            if self.missing_count >= p.space_missing_frames:
                self.enter('ALIGN_TO_SPACE', now_ns, f'SPACE_MISSING: {self.missing_count}',
                           observation_stamp_ns=obs.stamp_ns)
                dt = 0.0
            elif not obs.line.found:
                return self._wait('FRONT_BOUNDARY_MISSING', dt, now_ns)
        elif (self.state == 'PARKING_IN' and self.last_reverse_command is None
              and not (obs.space.found or end_can_steer)):
            return self._wait('REAR_PARKING_GEOMETRY_MISSING', dt, now_ns)
        elif self.state == 'TURN_RIGHT' and not obs.outline.found:
            return self._wait('EXIT_LINE_MISSING', dt, now_ns)

        self.loss_elapsed = 0.0
        self.reason = 'ACTIVE'
        self.elapsed += dt
        timeout = p.search_timeout_s if self.state == 'SEARCHING_SPACE' else p.maneuver_timeout_s
        if self.state != 'REVERSE_RIGHT' and self.elapsed >= timeout:
            return self.fail('STATE_TIMEOUT: ' + self.state, now_ns)

        if self.state == 'SEARCHING_SPACE':
            ex, ea = obs.line.x - p.front_target_x, axis_error(obs.line.yaw)
            self.errors = {'center': ex, 'yaw': ea}
            return self.command(p.front_position_gain * ex + p.front_angle_gain * ea, p.forward_pwm)

        if self.state == 'ALIGN_TO_SPACE':
            if obs.front_limit.found and obs.front_limit.y >= p.front_limit_stop_y:
                self.enter('DIRECTION_PAUSE', now_ns, 'FRONT_LIMIT_REACHED')
                return MotionCommand()
            if not obs.front_limit.found and self.elapsed + 1e-9 >= p.align_fallback_s:
                self.enter('DIRECTION_PAUSE', now_ns, f'FRONT_LIMIT_FALLBACK: {p.align_fallback_s:g}')
                return MotionCommand()
            return self.command(p.align_steering, p.forward_pwm)

        if self.state == 'DIRECTION_PAUSE':
            if self.elapsed < p.direction_pause_s:
                self.reason = 'DIRECTION_PAUSE'
                return MotionCommand()
            self.enter('REVERSE_RIGHT', now_ns, 'DIRECTION_PAUSE_COMPLETE', observation_stamp_ns=obs.stamp_ns)

        if self.state == 'REVERSE_RIGHT':
            self.last_reverse_command = self.command(p.max_steering, -p.reverse_pwm)
            return self.last_reverse_command

        if self.state == 'PARKING_IN':
            if obs.end.found and obs.end.y >= p.end_stop_y:
                self.enter('STOPPED', now_ns, 'REAR_END_REACHED')
                return MotionCommand()  # stop in THIS tick, never one more reverse
            if end_can_steer:
                ea = axis_error(obs.end.yaw, 0.0)
                self.errors = {'center': 0.0, 'yaw': ea}
                steer = -p.end_angle_gain * ea
            elif obs.space.found:
                ex, ea = obs.space.x - p.rear_target_x, axis_error(obs.space.yaw)
                self.errors = {'center': ex, 'yaw': ea}
                steer = -p.rear_position_gain * ex - p.rear_angle_gain * ea
            else:
                self.reason = 'REAR_PARKING_COMMAND_HELD'
                return self.last_reverse_command
            self.last_reverse_command = self.command(steer, -p.reverse_pwm)
            return self.last_reverse_command

        if self.state == 'ESCAPE':
            if self.elapsed >= p.escape_s:
                self.enter('TURN_RIGHT', now_ns, 'ESCAPE_COMPLETE')
                return MotionCommand()
            return self.command(speed=p.exit_pwm)

        if self.state == 'TURN_RIGHT':
            if obs.outline.y >= p.outline_finish_y:
                self.enter('FINAL_STRAIGHT', now_ns, 'EXIT_LINE_REACHED')
                return MotionCommand()
            ex, ea = obs.outline.x - .5, axis_error(obs.outline.yaw, 0.0)
            self.errors = {'center': ex, 'yaw': ea}
            return self.command(p.exit_position_gain * ex + p.exit_angle_gain * ea, p.exit_pwm)

        if self.state == 'FINAL_STRAIGHT':
            if self.elapsed >= p.final_straight_s:
                self.enter('FINISHED', now_ns, 'FINAL_STRAIGHT_COMPLETE')
                return MotionCommand()
            return self.command(speed=p.exit_pwm)
        return self.fail('UNKNOWN_STATE', now_ns)
