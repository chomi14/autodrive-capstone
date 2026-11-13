"""Pure camera parking FSM, inspired by 4th SKKU motion_parking.py.

No camera, ROS or serial effects. PWM is not velocity; steering is a signed
discrete actuator request, positive RIGHT before the installation sign.
"""
from dataclasses import dataclass, fields
import math

from skku_track_drive_pkg.messages import MotionCommand
from .observations import axis_error


@dataclass(frozen=True)
class ControlConfig:
    calibration_confirmed: bool = False
    observation_timeout_s: float = 0.5
    observation_loss_fault_s: float = 5.0
    confirm_frames: int = 3
    forward_pwm: int = 70
    reverse_pwm: int = 60
    max_steering: int = 7
    steering_sign: int = 1
    front_target_x: float = 0.7
    rear_target_x: float = 0.5
    front_position_gain: float = 20.0
    front_angle_gain: float = 5.0
    rear_position_gain: float = 10.0
    rear_angle_gain: float = 8.0
    end_angle_gain: float = 10.0
    exit_position_gain: float = 5.0
    exit_angle_gain: float = 5.0
    steering_deadband: float = 0.3
    align_steering: int = -7
    align_yaw_error_max_rad: float = 0.65
    direction_pause_s: float = 0.7
    end_steering_y: float = 0.70
    end_stop_y: float = 0.88
    stopped_hold_s: float = 3.0
    exit_enabled: bool = False
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
        if not (1 <= self.confirm_frames <= 100 and 1 <= self.max_steering <= 7 and self.steering_sign in (-1, 1)):
            raise ValueError('Invalid confirmation or steering limits')
        if not (1 <= self.forward_pwm <= 255 and 1 <= self.reverse_pwm <= 255):
            raise ValueError('PWM must be in 1..255')
        if not -self.max_steering <= self.align_steering <= self.max_steering:
            raise ValueError('align_steering outside output range')
        for name in ('front_target_x', 'rear_target_x', 'outline_finish_y'):
            if not 0 <= getattr(self, name) <= 1:
                raise ValueError(f'{name} outside normalized image')
        if not 0 <= self.end_steering_y < self.end_stop_y <= 1:
            raise ValueError('Require 0 <= end_steering_y < end_stop_y <= 1')
        if not 0 < self.align_yaw_error_max_rad < math.pi / 2:
            raise ValueError('Invalid alignment angular tolerance')
        if self.steering_deadband < 0:
            raise ValueError('steering_deadband must be nonnegative')
        for item in fields(self):
            if item.name.endswith('_s') and getattr(self, item.name) <= 0:
                raise ValueError(f'{item.name} must be positive')
        if self.stopped_hold_s < self.direction_pause_s:
            raise ValueError('stopped_hold_s must cover reverse-to-forward stop')


class CameraParkingCore:
    def __init__(self, config=None, require_calibration=True):
        self.p = config or ControlConfig()
        self.require_calibration = require_calibration
        self.reset()

    def reset(self, now_ns=0):
        self.state = 'SEARCHING_SPACE'
        self.barrier_ns = now_ns
        self.elapsed = self.loss_elapsed = 0.0
        self.seen_count = self.missing_count = self.align_count = 0
        self.seen_space = False
        self.consumed = {'front': 0, 'rear': 0}
        self.was_armed = False
        self.fault = ''
        self.reason = 'RESET'
        self.transition = 'RESET'
        self.errors = {}

    def enter(self, state, now_ns, reason):
        self.transition = f'{self.state}->{state}: {reason}'
        self.state, self.barrier_ns = state, now_ns
        self.elapsed = self.loss_elapsed = 0.0
        self.missing_count = self.align_count = 0
        self.reason = reason

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
        self.missing_count = self.align_count = self.seen_count = 0
        # Maneuver elapsed freezes. A separate loss watchdog prevents a hidden
        # indefinite wait while the operator believes a maneuver is active.
        self.loss_elapsed += dt
        if self.loss_elapsed >= self.p.observation_loss_fault_s:
            return self.fail('OBSERVATION_LOSS_TIMEOUT: ' + reason, now_ns)
        return MotionCommand()

    def step(self, dt, now_ns, armed, front=None, rear=None):
        p = self.p
        self.errors = {}
        dt = max(0.0, min(float(dt), .2))
        if not armed:
            self.was_armed = False
            self.missing_count = self.align_count = self.seen_count = 0
            self.reason = 'DISARMED'
            return MotionCommand()
        if not self.was_armed:
            # A W/resume must wait for captures made AFTER W, including after S.
            self.barrier_ns = max(self.barrier_ns, now_ns)
            self.missing_count = self.align_count = self.seen_count = 0
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
                self.enter('ESCAPE', now_ns, 'parked hold complete')
            return MotionCommand()

        camera = 'rear' if self.state in ('ALIGN_TO_SPACE', 'DIRECTION_PAUSE', 'PARKING_IN') else 'front'
        obs = rear if camera == 'rear' else front
        if obs is None or obs.camera != camera or not obs.valid:
            return self._wait(camera + '_OBSERVATION_INVALID', dt, now_ns)
        age_s = (now_ns - obs.stamp_ns) / 1e9
        if age_s < 0 or age_s > p.observation_timeout_s or obs.stamp_ns <= self.barrier_ns:
            return self._wait(camera + '_OBSERVATION_STALE_OR_BEFORE_STATE', dt, now_ns)
        new_frame = obs.stamp_ns > self.consumed[camera]
        if new_frame:
            self.consumed[camera] = obs.stamp_ns

        # Geometry needed to MOVE, rather than merely a live image stream.
        required = {
            'SEARCHING_SPACE': obs.line.found,
            'ALIGN_TO_SPACE': obs.space.found and 0 <= obs.space.x <= 1,
            'DIRECTION_PAUSE': obs.space.found,
            'PARKING_IN': obs.space.found or (obs.end.found and obs.end.y >= p.end_steering_y),
            'TURN_RIGHT': obs.outline.found,
        }.get(self.state, True)
        if not required:
            return self._wait(camera + '_GEOMETRY_MISSING', dt, now_ns)
        self.loss_elapsed = 0.0
        self.reason = 'ACTIVE'
        self.elapsed += dt
        timeout = p.search_timeout_s if self.state == 'SEARCHING_SPACE' else p.maneuver_timeout_s
        if self.elapsed >= timeout:
            return self.fail('STATE_TIMEOUT: ' + self.state, now_ns)

        if self.state == 'SEARCHING_SPACE':
            if new_frame:
                if obs.space_found:
                    self.seen_count += 1
                    self.missing_count = 0
                    if self.seen_count >= p.confirm_frames:
                        self.seen_space = True
                else:
                    self.seen_count = 0
                    self.missing_count = self.missing_count + 1 if self.seen_space else 0
                if self.missing_count >= p.confirm_frames:
                    self.enter('ALIGN_TO_SPACE', now_ns, 'seen space then consecutive missing frames')
                    return MotionCommand()
            ex, ea = obs.line.x - p.front_target_x, axis_error(obs.line.yaw)
            self.errors = {'center': ex, 'yaw': ea}
            return self.command(p.front_position_gain * ex + p.front_angle_gain * ea, p.forward_pwm)

        if self.state == 'ALIGN_TO_SPACE':
            error = axis_error(obs.space.yaw)
            self.errors = {'center': obs.space.x - p.rear_target_x, 'yaw': error}
            if new_frame:
                self.align_count = self.align_count + 1 if abs(error) <= p.align_yaw_error_max_rad else 0
                if self.align_count >= p.confirm_frames:
                    self.enter('DIRECTION_PAUSE', now_ns, 'rear line angle within tolerance')
                    return MotionCommand()
            return self.command(p.align_steering, p.forward_pwm)

        if self.state == 'DIRECTION_PAUSE':
            if self.elapsed >= p.direction_pause_s:
                self.enter('PARKING_IN', now_ns, 'forward-to-reverse stop complete')
            return MotionCommand()

        if self.state == 'PARKING_IN':
            if obs.end.found and obs.end.y >= p.end_stop_y:
                self.enter('STOPPED', now_ns, 'rear end line reached stop threshold')
                return MotionCommand()  # stop in THIS tick, never one more reverse
            if obs.end.found and obs.end.y >= p.end_steering_y:
                ea = axis_error(obs.end.yaw, 0.0)
                self.errors = {'center': 0.0, 'yaw': ea}
                steer = -p.end_angle_gain * ea
            else:
                ex, ea = obs.space.x - p.rear_target_x, axis_error(obs.space.yaw)
                self.errors = {'center': ex, 'yaw': ea}
                steer = -p.rear_position_gain * ex - p.rear_angle_gain * ea
            return self.command(steer, -p.reverse_pwm)

        if self.state == 'ESCAPE':
            if self.elapsed >= p.escape_s:
                self.enter('TURN_RIGHT', now_ns, 'bounded straight escape complete')
                return MotionCommand()
            return self.command(speed=p.forward_pwm)

        if self.state == 'TURN_RIGHT':
            if obs.outline.y >= p.outline_finish_y:
                self.enter('FINAL_STRAIGHT', now_ns, 'exit stripe reached threshold')
                return MotionCommand()
            ex, ea = obs.outline.x - .5, axis_error(obs.outline.yaw, 0.0)
            self.errors = {'center': ex, 'yaw': ea}
            return self.command(p.exit_position_gain * ex + p.exit_angle_gain * ea, p.forward_pwm)

        if self.state == 'FINAL_STRAIGHT':
            if self.elapsed >= p.final_straight_s:
                self.enter('FINISHED', now_ns, 'bounded final straight complete')
                return MotionCommand()
            return self.command(speed=p.forward_pwm)
        return self.fail('UNKNOWN_STATE', now_ns)
