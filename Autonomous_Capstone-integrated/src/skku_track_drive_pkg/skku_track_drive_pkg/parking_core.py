"""Gated parking state machines with no ROS/serial side effects.

Perpendicular preserves ver2's calibrated sequence and timing profiles.
Parallel searches a bounded gap and plans two reverse bicycle arcs. Its pose
is estimated from MEASURED PWM speed; it is not encoder/SLAM localization.
"""
import math

from .messages import MotionCommand
from .mode_parameters import PERPENDICULAR, PARALLEL, defaults
from .lidar_geometry import collision, sector_distance, side_line_angle


class ParkingCore:
    def __init__(self, mode, settings=None):
        if mode not in ('perpendicular', 'parallel'):
            raise ValueError('Unsupported parking mode')
        self.mode = mode
        self.p = defaults(PERPENDICULAR if mode == 'perpendicular' else PARALLEL)
        self.p.update(settings or {})
        self.reset()

    def reset(self):
        self.state = 'SEARCH' if self.mode == 'perpendicular' else 'SEARCH_FIRST_CAR'
        self.elapsed = 0.0
        self.pose = [0.0, 0.0, 0.0]
        self.last_command = MotionCommand()
        self.detect_count = self.clear_count = 0
        self.case = 1
        self.gap_start = self.gap_end = self.slot_center = None
        self.entry_x = self.arc_angle = None
        self.fault = ''
        self.side_hits = self.start_hits = 0
        self.status = {}

    def ready_reason(self):
        p = self.p
        if not p['geometry_confirmed']:
            return 'MEASURE_GEOMETRY_AND_SET_geometry_confirmed'
        if any(p[key] <= 0 for key in ('wheelbase_m', 'vehicle_width_m', 'front_overhang_m', 'rear_overhang_m')):
            return 'MISSING_VEHICLE_DIMENSIONS'
        if self.mode == 'parallel':
            if any(p[key] <= 0 for key in ('forward_mps', 'reverse_mps', 'max_wheel_angle_deg', 'slot_depth_m', 'entry_lateral_m')):
                return 'MISSING_SPEED_STEERING_SPACE_MEASUREMENTS'
            if p['slot_depth_m'] < p['vehicle_width_m'] + p['collision_margin_m'] * 2:
                return 'SLOT_TOO_NARROW'
        return ''

    def enter(self, state):
        self.state = state
        self.elapsed = 0.0
        self.detect_count = self.clear_count = 0

    def command(self, steering=0, reverse=False, moving=True):
        p = self.p
        speed = (-p['reverse_pwm'] if reverse else p['forward_pwm']) if moving else 0
        return MotionCommand(int(max(-7, min(7, steering * p['steering_sign']))), speed, speed)

    def fail(self, reason):
        self.fault = reason
        self.enter('FAULT_STOP')
        return MotionCommand()

    def _integrate(self, dt):
        p = self.p
        cmd = self.last_command
        if not cmd.left_speed or p['wheelbase_m'] <= 0:
            return
        speed = p['forward_mps'] if cmd.left_speed > 0 else -p['reverse_mps']
        physical_step = cmd.steering * p['steering_sign']
        # Positive command is RIGHT whereas bicycle yaw is positive LEFT.
        wheel_angle = -math.radians(p['max_wheel_angle_deg']) * physical_step / 7
        yaw_rate = speed / p['wheelbase_m'] * math.tan(wheel_angle)
        yaw_mid = self.pose[2] + yaw_rate * dt / 2
        self.pose[0] += speed * math.cos(yaw_mid) * dt
        self.pose[1] += speed * math.sin(yaw_mid) * dt
        self.pose[2] += yaw_rate * dt

    def step(self, dt, points, armed, fresh=True, new_scan=True):
        p = self.p
        dt = max(0.0, min(.2, dt))
        reason = self.ready_reason()
        side_center = p['side'] * p['side_center_deg']
        side_distance = sector_distance(points, side_center, p['side_half_width_deg'], p)
        opposite = sector_distance(points, -side_center, p['side_half_width_deg'], p)
        blocked = collision(points, p)
        unknown_search = (self.state.startswith('SEARCH') or self.state == 'MEASURE_GAP') and side_distance is None
        if reason or not armed or not fresh or blocked or unknown_search:
            self.last_command = MotionCommand()
            wait = reason or ('DISARMED_PAUSED' if not armed else 'SCAN_STALE_STOP' if not fresh else 'SIDE_OBSERVATION_UNKNOWN' if unknown_search else 'COLLISION_STOP')
            self._status(side_distance, opposite, wait)
            return self.last_command
        self._integrate(dt)
        self.elapsed += dt
        if new_scan:
            if side_distance is not None:
                near = side_distance <= p['side_detect_m']
                self.detect_count = self.detect_count + 1 if near else 0
                self.clear_count = 0 if near else self.clear_count + 1
            else:
                self.detect_count = self.clear_count = 0
        if self.state.startswith('SEARCH') and self.elapsed > p['search_timeout_s']:
            cmd = self.fail('SEARCH_TIMEOUT')
        elif self.state not in ('DONE', 'FAULT_STOP', 'PARKED') and self.elapsed > p['maneuver_timeout_s'] and not self.state.startswith('SEARCH'):
            cmd = self.fail('MANEUVER_TIMEOUT')
        elif self.mode == 'perpendicular':
            cmd = self._perpendicular(side_distance, opposite, new_scan)
        else:
            cmd = self._parallel(points)
        self.last_command = cmd
        self._status(side_distance, opposite, self.fault)
        return cmd

    def _perpendicular(self, distance, opposite, new_scan):
        p = self.p
        steer = p['steer_step'] * -p['side']  # +7 for a right-side bay.
        if self.state == 'SEARCH':
            if new_scan:
                self.start_hits = self.start_hits + 1 if distance is not None and distance <= p['start_detect_m'] else 0
            if self.start_hits >= p['confirm_scans']:
                self.case = 1 + sum(distance > p[f'case{i}_max_m'] for i in (1, 2, 3))
                self.enter('FWD_TURN')
                return MotionCommand()
            return self.command()
        if self.state == 'FWD_TURN':
            if self.elapsed >= p[f'case{self.case}_fwd_turn_s']:
                self.enter('SETTLE')
                return MotionCommand()
            return self.command(-steer)
        if self.state == 'SETTLE':
            if self.elapsed >= p['settle_s']:
                self.enter('REV_STRAIGHT')
            return MotionCommand()
        if self.state == 'REV_STRAIGHT':
            if self.elapsed >= p[f'case{self.case}_rev_straight_s']:
                self.enter('REV_TURN')
                return MotionCommand()
            return self.command(reverse=True)
        if self.state == 'REV_TURN':
            if self.elapsed >= p[f'case{self.case}_rev_turn_s']:
                self.enter('REV_SIDE_CHECK')
                return MotionCommand()
            return self.command(steer, reverse=True)
        if self.state == 'REV_SIDE_CHECK':
            # Fresh SCANS, not three timer ticks on one old scan.
            if new_scan:
                hit = any(d is not None and d <= p['side_detect_m'] for d in (distance, opposite))
                self.detect_count = getattr(self, 'side_hits', 0) + 1 if hit else 0
                self.side_hits = self.detect_count
                if self.detect_count >= p['confirm_scans']:
                    self.enter('PARKED')
                    return MotionCommand()
            return self.command(reverse=True)
        if self.state == 'PARKED':
            if p['exit_enabled'] and self.elapsed >= p['pause_s']:
                self.enter('EXIT_STRAIGHT')
            return MotionCommand()
        if self.state == 'EXIT_STRAIGHT':
            if self.elapsed >= p['fwd_straight_out_s']:
                self.enter('EXIT_TURN')
                return MotionCommand()
            return self.command()
        if self.state == 'EXIT_TURN':
            if self.elapsed >= p['fwd_turn_out_s']:
                self.enter('EXIT_FINISH')
                return MotionCommand()
            return self.command(steer)
        if self.state == 'EXIT_FINISH':
            if self.elapsed >= p['finish_s']:
                self.enter('DONE')
                return MotionCommand()
            return self.command()
        return MotionCommand()

    def _parallel(self, points):
        p = self.p
        steer = -p['side'] * p['steer_step']
        confirmed = self.detect_count >= p['confirm_scans']
        cleared = self.clear_count >= p['confirm_scans']
        if self.state == 'SEARCH_FIRST_CAR':
            if confirmed:
                self.enter('SEARCH_GAP')
            return self.command()
        if self.state == 'SEARCH_GAP':
            if cleared:
                self.gap_start = self.pose[0] + p['lidar_x_m']
                self.enter('MEASURE_GAP')
            return self.command()
        if self.state == 'MEASURE_GAP':
            if confirmed:
                self.gap_end = self.pose[0] + p['lidar_x_m']
                length = p['wheelbase_m'] + p['front_overhang_m'] + p['rear_overhang_m']
                if self.gap_end - self.gap_start < length + p['gap_margin_m']:
                    self.enter('SEARCH_GAP')
                    return MotionCommand()
                radius = p['wheelbase_m'] / math.tan(math.radians(p['max_wheel_angle_deg']) * p['steer_step']/7)
                cosine = 1 - p['entry_lateral_m'] / (2 * radius)
                if not 0 < cosine < 1:
                    return self.fail('LATERAL_OFFSET_UNREACHABLE')
                self.arc_angle = math.acos(cosine)
                self.slot_center = (self.gap_start + self.gap_end - p['wheelbase_m'] - p['front_overhang_m'] + p['rear_overhang_m']) / 2
                self.entry_x = self.slot_center + 2 * radius * math.sin(self.arc_angle) + p['entry_advance_m']
                if self.entry_x < self.pose[0]:
                    return self.fail('ENTRY_POSITION_ALREADY_PASSED')
                self.enter('APPROACH_ENTRY')
                return MotionCommand()
            return self.command()
        if self.state == 'APPROACH_ENTRY':
            if self.pose[0] >= self.entry_x:
                self.enter('ENTRY_SETTLE')
                return MotionCommand()
            return self.command()
        if self.state == 'ENTRY_SETTLE':
            if self.elapsed >= p['settle_s']:
                self.enter('REVERSE_ARC_IN')
            return MotionCommand()
        if self.state == 'REVERSE_ARC_IN':
            if abs(self.pose[2]) >= self.arc_angle:
                self.enter('REVERSE_COUNTER_STEER')
                return MotionCommand()
            return self.command(steer, reverse=True)
        if self.state == 'REVERSE_COUNTER_STEER':
            if abs(self.pose[2]) <= math.radians(p['align_tolerance_deg']):
                self.enter('ALIGN')
                return MotionCommand()
            return self.command(-steer, reverse=True)
        if self.state == 'ALIGN':
            angle = side_line_angle(points, p['side'], p)
            if angle is None:
                self.fault = 'WAIT_SIDE_BOUNDARY_FOR_ALIGNMENT'
                return MotionCommand()
            self.fault = ''
            error_x = self.slot_center - self.pose[0]
            if abs(angle) <= p['align_tolerance_deg'] and abs(error_x) <= p['center_tolerance_m']:
                # Both space boundaries were observed; do not claim exit support.
                self.enter('DONE')
                return MotionCommand()
            correction = 0 if abs(angle) <= p['align_tolerance_deg'] else (-2 if angle > 0 else 2)
            reverse = error_x < 0
            if reverse:
                correction = -correction
            return self.command(correction, reverse=reverse)
        return MotionCommand()

    def _status(self, side_distance, opposite, waiting):
        self.status = {
            'mode': self.mode, 'state': self.state, 'waiting': waiting,
            'side_m': None if side_distance is None else round(side_distance, 3),
            'opposite_m': None if opposite is None else round(opposite, 3),
            'elapsed_s': round(self.elapsed, 2), 'distance_case': self.case,
            'gap_m': None if self.gap_end is None else round(self.gap_end - self.gap_start, 3),
            'pose_estimate': [round(v, 3) for v in self.pose],
            'entry_x_estimate_m': self.entry_x,
            'exit_supported': self.mode == 'perpendicular',
            'pose_source': 'calibrated_PWM_estimate_not_odometry',
        }
