"""Rulebook bay geometry and automatic perpendicular-to-parallel handoff."""

from dataclasses import dataclass
import math

from skku_track_drive_pkg.messages import MotionCommand
from skku_track_drive_pkg.parallel_camera_planner import Box, parked, sample_move


# PDF page 5 (printed page 3): 1500 mm clear length, 850 mm clear width.
# Total row is 6250 mm: four 1500 mm spaces and five 50 mm stripes.
PARALLEL_LENGTH_M = 1.50
PARALLEL_WIDTH_M = .85
PARALLEL_PITCH_M = 1.55
PERPENDICULAR_PITCH_M = 1.00
PERPENDICULAR_WIDTH_M = .95
PARK_HOLD_S = 5.0


def parallel_bays(first_clear_corner, index_direction):
    """Number 1 through 4 from the surveyed first bay, in the stated x order."""
    x, y = first_clear_corner
    if type(index_direction) is not int or index_direction not in (-1, 1):
        raise ValueError('parallel numbering direction must be +1 or -1')
    return {i: Box(x + index_direction * (i - 1) * PARALLEL_PITCH_M, y,
                   x + index_direction * (i - 1) * PARALLEL_PITCH_M + PARALLEL_LENGTH_M,
                   y + PARALLEL_WIDTH_M) for i in range(1, 5)}


def target_from_occupied(occupied):
    """Only the empty bay bracketed by the rulebook's two parked vehicles."""
    return {frozenset((1, 3)): 2, frozenset((2, 4)): 3}.get(frozenset(occupied))


def perpendicular_bays(first_clear_corner, index_direction):
    x, y = first_clear_corner
    if type(index_direction) is not int or index_direction not in (-1, 1):
        raise ValueError('perpendicular numbering direction must be +1 or -1')
    return {i: Box(x + index_direction * (i - 1) * PERPENDICULAR_PITCH_M, y,
                   x + index_direction * (i - 1) * PERPENDICULAR_PITCH_M + PERPENDICULAR_WIDTH_M,
                   y + PARALLEL_LENGTH_M) for i in range(1, 5)}


def left_rear(pose, slot):
    dx = (slot.xmin + slot.xmax) / 2 - pose.x
    dy = (slot.ymin + slot.ymax) / 2 - pose.y
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return dx * c + dy * s < 0 and -dx * s + dy * c > 0


@dataclass(frozen=True)
class Frame:
    stamp_ns: int
    pose: object
    occupied: tuple = ()


class Mission2:
    def __init__(self, cfg, follower, bays, perpendicular, fixed, exit_pwm, exit_timeout, stable_frames, now):
        self.cfg, self.follower, self.bays, self.fixed = cfg, follower, bays, fixed
        self.perpendicular = perpendicular
        self.perp_target = None
        self.base_scene = cfg.scene
        self.exit_pwm, self.exit_timeout, self.stable_frames = exit_pwm, exit_timeout, stable_frames
        self.state, self.reason = 'PERP_HOLD', 'WAIT_PERP_HOLD'
        self.phase_at = now
        self.last_stamp = 0
        self.stable = 0
        self.target = None
        self.confirmed_target = None
        self.hold_pose = None

    def enter(self, state, now):
        self.state, self.phase_at = state, now

    def stop(self, reason):
        self.state, self.reason = 'FAULT_STOP', reason
        return MotionCommand()

    def stable_position(self, pose):
        if self.hold_pose is None:
            self.hold_pose = pose
        old = self.hold_pose
        dyaw = (pose.yaw - old.yaw + math.pi) % math.tau - math.pi
        return (math.hypot(pose.x - old.x, pose.y - old.y) <= self.cfg.settle_drift
                and abs(dyaw) <= self.cfg.yaw_jump)

    def step(self, now, frame):
        from dataclasses import replace

        if self.state in ('FAULT_STOP', 'PARALLEL_DONE'):
            return MotionCommand()
        if frame is None:
            # A gap during a hold resets the continuous hold. During motion it
            # invalidates the maneuver and requires the operator to stop.
            if self.state != 'PERP_HOLD':
                return self.stop('CAMERA_OBSERVATION_LOST')
            self.phase_at = now
            self.hold_pose = None
            return MotionCommand()
        pose = frame.pose
        fresh = frame.stamp_ns > self.last_stamp
        if frame.stamp_ns == 0:
            self.target, self.stable = None, 0
        elif fresh:
            self.last_stamp = frame.stamp_ns
            detected = target_from_occupied(frame.occupied)
            self.stable = self.stable + 1 if detected is not None and detected == self.target else int(detected is not None)
            self.target = detected
        ready = self.target is not None and self.stable >= self.stable_frames

        if self.state == 'PERP_HOLD':
            eligible = [number for number in (2, 3)
                        if all(map(self.perpendicular[number].contains, self.cfg.vehicle.corners(pose)))]
            if len(eligible) != 1:
                self.reason = 'WAIT_IDENTIFIABLE_PERPENDICULAR_BAY_2_OR_3'
                self.phase_at, self.hold_pose = now, None
                return MotionCommand()
            if eligible[0] != self.perp_target:
                self.perp_target, self.phase_at, self.hold_pose = eligible[0], now, None
            self.reason = 'HOLDING_IN_PERPENDICULAR_BAY'
            if not self.stable_position(pose):
                self.phase_at, self.hold_pose = now, pose
            if now - self.phase_at >= PARK_HOLD_S:
                self.fixed += tuple(box for number, box in self.perpendicular.items() if number != self.perp_target)
                self.enter('EXIT_RIGHT', now)
            return MotionCommand()

        if self.state == 'EXIT_RIGHT':
            if now - self.phase_at > self.exit_timeout:
                return self.stop('NO_REAR_TARGET_BEFORE_EXIT_LIMIT')
            if ready and left_rear(pose, self.bays[self.target]):
                self.confirmed_target, self.hold_pose = self.target, pose
                self.enter('ENTRY_PAUSE', now)
                return MotionCommand()
            # Until the target is confirmed, reserve all four bays. Forward
            # right is only permitted while the swept car stays in the aisle.
            reserved = replace(self.base_scene, obstacles=tuple(self.bays.values()) + self.fixed,
                               segment_m=self.cfg.stop_distance)
            steer = -self.cfg.vehicle.steer_right_deg
            if not sample_move(pose, 1, steer, self.cfg.vehicle, reserved):
                return self.stop('RIGHT_EXIT_SWEEP_BLOCKED')
            step = min(range(-7, 8), key=lambda i: abs(self.cfg.steer_by_step[i + 7] - steer))
            return MotionCommand(step, self.exit_pwm, self.exit_pwm)

        if self.state == 'ENTRY_PAUSE':
            if now - self.phase_at > self.exit_timeout:
                return self.stop('COULD_NOT_RECONFIRM_PARALLEL_TARGET')
            if not self.stable_position(pose):
                self.hold_pose = pose
                self.phase_at = now
                return MotionCommand()
            if (fresh and ready and self.target == self.confirmed_target
                    and now - self.phase_at >= self.cfg.settle):
                slot = self.bays[self.target]
                # Reserve every other bay. The observed pair selects the goal;
                # an unoccupied non-target bay never becomes an alternate.
                self.cfg.scene = replace(self.base_scene, slot=slot,
                                         obstacles=self.fixed + tuple(box for number, box in self.bays.items()
                                                                      if number != self.target))
                self.follower.prepare(pose)
                if self.follower.route is None:
                    return self.stop('NO_CHECKED_REVERSE_PATH')
                self.enter('PARALLEL', now)
            return MotionCommand()

        if self.state == 'PARALLEL':
            command = self.follower.step(pose, now)
            if self.follower.status == 'PARKED':
                self.hold_pose = pose
                self.enter('PARALLEL_HOLD', now)
            elif self.follower.route is None:
                return self.stop(self.follower.status)
            return command

        if self.state == 'PARALLEL_HOLD':
            if not parked(pose, self.cfg.vehicle, self.cfg.scene):
                return self.stop('LEFT_TARGET_BAY_DURING_HOLD')
            if not self.stable_position(pose):
                self.phase_at, self.hold_pose = now, pose
            if now - self.phase_at >= PARK_HOLD_S:
                self.enter('PARALLEL_DONE', now)
                self.reason = 'HOLD_COMPLETE_OUT_NOT_IMPLEMENTED'
            return MotionCommand()
        return self.stop('UNKNOWN_STATE')
