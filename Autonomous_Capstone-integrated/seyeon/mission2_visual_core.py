"""Image-based handoff. No vehicle geometry, surveyed map or distance estimates."""

from dataclasses import dataclass


@dataclass(frozen=True)
class Command:
    steer: float = 0.
    pwm: int = 0


@dataclass(frozen=True)
class Evidence:
    sequence: int
    bay: object
    front_blocked: bool
    rear_blocked: bool
    still: bool


class VisualMission:
    def __init__(self, control, now):
        self.control = control
        self.state = 'PERP_HOLD'
        self.since = now
        self.hold_start = None
        self.last_seen = now
        self.sequence = -1
        self.previous_bay = None
        self.votes = 0
        self.adjustments = 0
        self.reason = 'WAIT_CAMERA_STILLNESS'

    def enter(self, state, now, reason):
        self.state, self.since, self.reason = state, now, reason
        self.hold_start = None

    def stop(self, now, reason):
        self.enter('FAULT_STOP', now, reason)
        return Command()

    def held(self, now, evidence, fresh, duration):
        # Verify standstill on distinct camera pairs; do not advance a hold
        # based on repeat timer callbacks serving the same image.
        if not evidence.still:
            self.hold_start = None
        elif fresh and self.hold_start is None:
            self.hold_start = now
        return fresh and self.hold_start is not None and now - self.hold_start >= duration

    @staticmethod
    def same_bay(a, b):
        return a is not None and b is not None and max(abs(x-y) for x, y in zip(a.center,b.center)) < .17

    def step(self, now, evidence):
        p = self.control
        if self.state in ('FAULT_STOP', 'PARALLEL_DONE'):
            return Command()
        if evidence is None:
            self.hold_start = None
            self.votes = 0
            if self.state != 'PERP_HOLD' and now - self.last_seen > 1.0:
                return self.stop(now, 'CAMERA_OR_DETECTOR_STALE')
            self.reason = 'WAIT_FRESH_FRONT_AND_REAR'
            return Command()
        fresh = evidence.sequence != self.sequence
        bay = evidence.bay
        if fresh:
            self.sequence, self.last_seen = evidence.sequence, now
            searching = self.state in ('PERP_HOLD', 'EXIT_RIGHT')
            if not searching and bay is not None and self.previous_bay is not None and not self.same_bay(bay, self.previous_bay):
                return self.stop(now, 'REAR_BAY_IDENTITY_CHANGED')
            if searching and bay is not None and not bay.anchored:
                self.votes = 0
            elif self.same_bay(bay, self.previous_bay):
                self.votes += 1
            else:
                self.votes = int(bay is not None)
            if bay is not None or self.state in ('PERP_HOLD', 'EXIT_RIGHT'):
                self.previous_bay = bay

        if self.state == 'PERP_HOLD':
            if self.held(now, evidence, fresh, 5.0):
                self.enter('EXIT_RIGHT', now, 'PERPENDICULAR_HOLD_COMPLETE')
            return Command()

        if self.state == 'EXIT_RIGHT':
            if now - self.since > min(p.maneuver_timeout_s, p.escape_s + p.reverse_right_s):
                return self.stop(now, 'NO_STABLE_REAR_BAY')
            if evidence.front_blocked:
                return self.stop(now, 'FRONT_NEAR_DURING_RIGHT_EXIT')
            if self.votes >= 3 and abs(bay.angle_error) < .74 and .15 < bay.center[0] < .85:
                self.enter('BEFORE_REVERSE', now, 'BAY_BETWEEN_REFERENCE_CARS')
                return Command()
            self.reason = 'RIGHT_FORWARD_SEARCH'
            # Match 1125's REVERSE_RIGHT steering convention; speed sign
            # determines direction, not the sign of the steering angle.
            return Command(p.max_steering, p.exit_pwm)

        if bay is None or self.votes < 2:
            self.hold_start = None
            if now - self.since > p.maneuver_timeout_s:
                return self.stop(now, 'SELECTED_BAY_NOT_VISIBLE')
            self.reason = 'WAIT_CONFIRMED_REAR_BAY'
            return Command()

        if self.state == 'BEFORE_REVERSE':
            if self.held(now, evidence, fresh, p.direction_pause_s):
                self.enter('REVERSE_IN', now, 'FOLLOW_REAR_PAINT_LINES')
            return Command()

        if self.state == 'REVERSE_IN':
            if now - self.since > p.maneuver_timeout_s:
                return self.stop(now, 'REVERSE_TIMEOUT')
            lateral = bay.aim_x - p.rear_target_x
            aligned = abs(lateral) <= .075 and abs(bay.angle_error) <= .18
            if bay.far_end_y >= p.end_stop_y:
                if aligned:
                    self.enter('PARALLEL_HOLD', now, 'IMAGE_ALIGNMENT_REACHED')
                elif self.adjustments < 2 and not evidence.front_blocked:
                    self.enter('BEFORE_ADJUST', now, 'STOP_BEFORE_FORWARD_ADJUST')
                else:
                    return self.stop(now, 'REAR_LIMIT_WITHOUT_ALIGNMENT')
                return Command()
            if evidence.rear_blocked:
                return self.stop(now, 'REAR_NEAR_BEFORE_ALIGNMENT')
            self.reason = 'STEER_BY_REAR_LINE_ANGLE_AND_OFFSET'
            return Command(-p.rear_position_gain*lateral - p.rear_angle_gain*bay.angle_error, -p.reverse_pwm)

        if self.state == 'BEFORE_ADJUST':
            if evidence.front_blocked:
                return self.stop(now, 'FRONT_NEAR_BEFORE_ADJUST')
            if self.held(now, evidence, fresh, p.direction_pause_s):
                self.adjustments += 1
                self.enter('FORWARD_ADJUST', now, 'SHORT_FORWARD_CORRECTION')
            return Command()

        if self.state == 'FORWARD_ADJUST':
            if evidence.front_blocked:
                return self.stop(now, 'FRONT_NEAR_DURING_ADJUST')
            if now - self.since >= min(.45, p.direction_pause_s):
                self.enter('BEFORE_REVERSE', now, 'STOP_BEFORE_RESUMING_REVERSE')
                return Command()
            steer = p.rear_angle_gain * bay.angle_error
            return Command(steer, min(p.forward_pwm, p.exit_pwm))

        if self.state == 'PARALLEL_HOLD':
            if abs(bay.aim_x - p.rear_target_x) > .075 or abs(bay.angle_error) > .18:
                return self.stop(now, 'ALIGNMENT_LOST_DURING_HOLD')
            if self.held(now, evidence, fresh, 5.0):
                self.enter('PARALLEL_DONE', now, 'IMAGE_HOLD_COMPLETE_OUT_NOT_IMPLEMENTED')
            return Command()
        return self.stop(now, 'UNKNOWN_STATE')
