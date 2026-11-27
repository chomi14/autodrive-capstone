"""Pure camera parking FSM, inspired by 4th SKKU motion_parking.py.

No camera, ROS or serial effects. PWM is not velocity; steering is a signed
discrete actuator request, positive RIGHT before the installation sign.
"""
from dataclasses import dataclass, fields
import math

from skku_track_drive_pkg.messages import MotionCommand
from .observations import axis_error
from .status import reason_text


PARALLEL_REVERSE_STATES = ('PARALLEL_PAUSE', 'PARALLEL_REVERSE_LEFT',
                           'PARALLEL_REVERSE_STRAIGHT', 'PARALLEL_FAR_ALIGN')


@dataclass(frozen=True)
class ControlConfig:
    calibration_confirmed: bool = False
    observation_timeout_s: float = 0.5
    observation_loss_fault_s: float = 5.0
    space_missing_frames: int = 5
    forward_pwm: int = 70
    reverse_pwm: int = 60
    escape_pwm: int = 200
    exit_pwm: int = 110
    max_steering: int = 7
    steering_sign: int = 1
    front_target_x: float = 0.7
    rear_target_x: float = 0.5
    front_position_gain: float = 135.0
    front_angle_gain: float = 8.771929824561404
    rear_position_gain: float = 14.0
    rear_angle_gain: float = 16.0
    end_angle_gain: float = 50.0
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
    exit_turn_s: float = 10.0
    exit_straight_s: float = 20.0
    search_timeout_s: float = 60.0
    maneuver_timeout_s: float = 25.0
    parallel_forward_pwm: int = 70
    parallel_reverse_pwm: int = 60
    parallel_target_x: float = 0.35
    parallel_position_gain: float = 135.0
    parallel_angle_gain: float = 8.771929824561404
    parallel_entry_y: float = 0.84
    parallel_entry_tolerance: float = 0.07
    parallel_approach_tolerance: float = 0.10
    parallel_row_width_ratio: float = 0.22
    parallel_slot_length_ratio: float = 0.36
    parallel_rear_reference_y: float = 1.0
    parallel_far_track_gap: float = 0.80
    parallel_stop_depth: float = 0.55
    parallel_insert_yaw_rad: float = 0.38
    parallel_align_yaw_rad: float = 0.18
    parallel_far_position_gain: float = 8.0
    parallel_far_angle_gain: float = 24.0

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
        if not all(1 <= pwm <= 255 for pwm in (self.forward_pwm, self.reverse_pwm,
                                               self.escape_pwm, self.exit_pwm, self.parallel_forward_pwm,
                                               self.parallel_reverse_pwm)):
            raise ValueError('PWM must be in 1..255')
        if not -self.max_steering <= self.align_steering < 0:
            raise ValueError('align_steering must request a left turn within the output range')
        for name in ('front_target_x', 'rear_target_x', 'front_limit_stop_y',
                     'parallel_target_x', 'parallel_rear_reference_y'):
            if not 0 <= getattr(self, name) <= 1:
                raise ValueError(f'{name} outside normalized image')
        for name in ('parallel_entry_tolerance', 'parallel_approach_tolerance', 'parallel_row_width_ratio',
                     'parallel_slot_length_ratio'):
            if not 0 < getattr(self, name) < .5:
                raise ValueError(f'{name} must be in (0, .5)')
        if not self.parallel_entry_tolerance < self.parallel_entry_y < 1 - self.parallel_entry_tolerance:
            raise ValueError('Parallel reverse start band must fit within the rear BEV')
        if not 0 < 1-self.parallel_far_track_gap < self.parallel_stop_depth < 1:
            raise ValueError('Parallel far-line tracking must start before the stop depth')
        if not 0 < self.parallel_align_yaw_rad < self.parallel_insert_yaw_rad < math.pi/2:
            raise ValueError('Parallel yaw tolerances must increase within (0, pi/2)')
        if min(self.parallel_position_gain, self.parallel_angle_gain,
               self.parallel_far_position_gain, self.parallel_far_angle_gain) < 0:
            raise ValueError('Parallel steering gains must be nonnegative')
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
    def __init__(self, config=None, require_calibration=True, on_transition=None, parallel_enabled=False):
        self.p = config or ControlConfig()
        self.require_calibration = require_calibration
        self.on_transition = on_transition
        self.parallel_enabled = parallel_enabled
        self.state = 'SEARCHING_SPACE'
        self.transition = ''
        self.reset()

    def reset(self, now_ns=0, reason='RESET'):
        self.model = 'perpendicular'
        self.parallel_switch_ns = 0
        self.parallel_target = 0
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
        self.exit_command_active = False
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
        self.exit_command_active = False
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
        if (self.state == 'SEARCHING_SPACE' and self.was_armed and obs.model == 'perpendicular'
                and self.missing_count < self.p.space_missing_frames
                and obs.stamp_ns > self.search_frame_stamp
                and not self._observation_problem('front', obs, now_ns)):
            self.search_frame_stamp = obs.stamp_ns
            self.missing_count = 0 if obs.space_found else self.missing_count + 1

    def _approach_parallel(self, front, rear, now_ns):
        p = self.p
        if self.elapsed >= p.exit_straight_s:
            return self.fail('PARALLEL_APPROACH_LIMIT', now_ns)
        road = front.roadside
        error, yaw = (road.x-p.parallel_target_x, axis_error(road.yaw)) if road.found else (0.0, 0.0)
        self.errors = {'center': error, 'yaw': yaw} if road.found else {}
        self.reason = 'ACTIVE' if road.found else 'PARALLEL_ROADSIDE_MISSING'
        self.exit_command_active = True
        command = self.command(p.parallel_position_gain*error + p.parallel_angle_gain*yaw, p.parallel_forward_pwm)
        votes = {view.route_target for view in (front, rear) if view is not None and view.route_target}
        if -1 in votes or len(votes) > 1 or (self.parallel_target and votes and self.parallel_target not in votes):
            self.reason = 'PARALLEL_TARGET_CONFLICT'
            return command
        if votes:
            self.parallel_target = next(iter(votes))
        elif not self.parallel_target:
            front_targets = {bay.number for bay in front.bays
                             if bay.number in (2, 3) and bay.target_mask and not bay.occupied}
            if len(front_targets) == 1:
                self.parallel_target = next(iter(front_targets))
        if rear is None:
            return command
        rear_aligned = (rear.roadside.found and abs(rear.roadside.x-(1-p.parallel_target_x)) <= p.parallel_approach_tolerance
                        and abs(axis_error(rear.roadside.yaw)) <= p.parallel_align_yaw_rad)
        front_aligned = (road.found and abs(error) <= p.parallel_approach_tolerance
                         and abs(yaw) <= p.parallel_align_yaw_rad)
        if not (rear_aligned or front_aligned):
            return command
        candidates = [bay for bay in rear.bays
                      if not bay.occupied and bay.number in (0, 2, 3)
                      and (not self.parallel_target or bay.number in (0, self.parallel_target))
                      and ((self.parallel_target and bay.number == self.parallel_target)
                           or bay.target_mask or bay.between_vehicles)
                      and abs(bay.entry_y-p.parallel_entry_y) <= p.parallel_entry_tolerance]
        if len(candidates) == 1:
            self.parallel_target = candidates[0].number or self.parallel_target
            self.enter('PARALLEL_PAUSE', now_ns, 'PARALLEL_REVERSE_POSITION_REACHED')
            return MotionCommand()
        return command

    def _parallel_rear(self, rear, now_ns, stale_cameras):
        if (not self._observation_problem('rear', rear, now_ns, 'rear' in stale_cameras)
                and rear.model == 'parallel'):
            return rear.parallel
        return None

    def _reverse_parallel(self, obs, dt, now_ns):
        p = self.p
        view = obs.parallel
        self.reason = 'ACTIVE'
        self.elapsed += dt
        depth = 1-view.far_gap if view.far_gap is not None else view.depth
        yaw = view.far_yaw_error if view.far_yaw_error is not None else view.yaw_error
        self.errors = {'depth': depth, 'yaw': yaw}
        if self.state == 'PARALLEL_PAUSE':
            if self.elapsed + 1e-9 < p.direction_pause_s:
                self.reason = 'DIRECTION_PAUSE'
                return MotionCommand()
            self.enter('PARALLEL_REVERSE_LEFT', now_ns, 'DIRECTION_PAUSE_COMPLETE',
                       observation_stamp_ns=obs.stamp_ns)
        if self.elapsed >= p.maneuver_timeout_s:
            return self.fail('STATE_TIMEOUT: ' + self.state, now_ns)
        if view.far_gap is not None and view.far_gap <= 0:
            return self.fail('PARALLEL_FAR_LINE_REACHED', now_ns)
        if (self.state != 'PARALLEL_FAR_ALIGN' and view.far_gap is not None
                and view.far_gap <= p.parallel_far_track_gap):
            self.enter('PARALLEL_FAR_ALIGN', now_ns, 'PARALLEL_FAR_LINE_ACQUIRED',
                       observation_stamp_ns=obs.stamp_ns)
        if self.state == 'PARALLEL_REVERSE_LEFT':
            self.loss_elapsed = 0.0
            if yaw is None or abs(yaw) < p.parallel_insert_yaw_rad:
                return self.command(-p.max_steering, -p.parallel_reverse_pwm)
            self.enter('PARALLEL_REVERSE_STRAIGHT', now_ns, 'PARALLEL_INSERT_ANGLE_REACHED',
                       observation_stamp_ns=obs.stamp_ns)
        if self.state == 'PARALLEL_REVERSE_STRAIGHT':
            self.loss_elapsed = 0.0
            return self.command(speed=-p.parallel_reverse_pwm)
        if depth is None or yaw is None:
            if self.last_reverse_command is not None:
                self.reason = 'PARALLEL_REVERSE_COMMAND_HELD'
                return self.last_reverse_command
            self.elapsed -= dt
            return self._wait('PARALLEL_REAR_GEOMETRY_MISSING', dt, now_ns)
        self.loss_elapsed = 0.0
        if view.far_gap is not None and depth >= p.parallel_stop_depth and abs(yaw) <= p.parallel_align_yaw_rad:
            self.enter('FINISHED', now_ns, 'PARALLEL_PARKED')
            return MotionCommand()
        if view.far_gap is None:
            self.reason = 'PARALLEL_FAR_LINE_LOST_USING_ROADSIDE'
        # A left turn while backing yaws the body away from the parking row.
        # The rear BEV then has negative angle error, so -gain*yaw unwinds right.
        steer = p.parallel_far_position_gain*(depth-p.parallel_stop_depth) - p.parallel_far_angle_gain*yaw
        self.last_reverse_command = self.command(steer, -p.parallel_reverse_pwm)
        return self.last_reverse_command

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
            self.exit_command_active = False
            self.was_armed = True
        if self.state in ('FAULT_STOP', 'FINISHED'):
            self.reason = self.fault or ('PARALLEL_PARKED' if self.model == 'parallel' else 'FINISHED')
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

        camera = ('rear' if self.state in ('DIRECTION_PAUSE', 'REVERSE_RIGHT', 'PARKING_IN',
                                           *PARALLEL_REVERSE_STATES) else 'front')
        obs = rear if camera == 'rear' else front
        problem = self._observation_problem(camera, obs, now_ns, camera in stale_cameras)
        if problem:
            return self._wait(problem, dt, now_ns)
        if obs.model != self.model:
            if (self.state == 'TURN_RIGHT' and self.model == 'parallel' and self.exit_command_active
                    and self.elapsed + dt + 1e-9 < p.exit_turn_s
                    and (now_ns - self.parallel_switch_ns)/1e9 <= p.observation_timeout_s):
                self.reason = 'PARALLEL_MODEL_SWITCH'
                self.elapsed += dt
                return self.command(p.max_steering, p.exit_pwm)
            return self._wait('PARKING_MODEL_WAIT_FRESH', dt, now_ns)

        if self.state in PARALLEL_REVERSE_STATES:
            return self._reverse_parallel(obs, dt, now_ns)
        if self.state == 'PARALLEL_APPROACH':
            secondary = self._parallel_rear(rear, now_ns, stale_cameras)
            self.loss_elapsed = 0.0
            if self.exit_command_active:
                self.elapsed += dt
            return self._approach_parallel(obs.parallel, secondary, now_ns)

        # Count intervals after an exit command, including the first command
        # after resuming from a stopped or stale-camera interval.
        if self.state in ('ESCAPE', 'TURN_RIGHT', 'FINAL_STRAIGHT') and not self.exit_command_active:
            dt = 0.0

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
            if self.elapsed + 1e-9 < p.escape_s:
                self.exit_command_active = True
                return self.command(speed=p.escape_pwm)
            self.enter('TURN_RIGHT', now_ns, 'ESCAPE_COMPLETE', observation_stamp_ns=obs.stamp_ns)
            if self.parallel_enabled:
                self.model = 'parallel'
                self.parallel_switch_ns = now_ns
                self.exit_command_active = True
                return self.command(p.max_steering, p.exit_pwm)

        if self.state == 'TURN_RIGHT':
            roadside_acquired = obs.parallel is not None and obs.parallel.roadside.found
            if not roadside_acquired and self.elapsed + 1e-9 < p.exit_turn_s:
                self.exit_command_active = True
                return self.command(p.max_steering, p.exit_pwm)
            if self.parallel_enabled:
                reason = ('PARALLEL_ROADSIDE_ACQUIRED' if roadside_acquired else
                          f'EXIT_RIGHT_COMPLETE: {p.exit_turn_s:g}')
                self.enter('PARALLEL_APPROACH', now_ns, reason,
                           observation_stamp_ns=obs.stamp_ns)
                return self._approach_parallel(obs.parallel, self._parallel_rear(rear, now_ns, stale_cameras),
                                               now_ns)
            self.enter('FINAL_STRAIGHT', now_ns, f'EXIT_RIGHT_COMPLETE: {p.exit_turn_s:g}',
                       observation_stamp_ns=obs.stamp_ns)

        if self.state == 'FINAL_STRAIGHT':
            if self.elapsed + 1e-9 >= p.exit_straight_s:
                self.enter('FINISHED', now_ns, 'FINAL_STRAIGHT_COMPLETE')
                return MotionCommand()
            self.exit_command_active = True
            return self.command(speed=p.exit_pwm)
        return self.fail('UNKNOWN_STATE', now_ns)
