"""Gated parking state machines with no ROS/serial side effects.

Perpendicular preserves ver2's calibrated sequence and timing profiles.
Parallel searches a bounded gap and plans two reverse bicycle arcs. Its pose
is estimated from MEASURED PWM speed; it is not encoder/SLAM localization.
"""
import math

from skku_track_drive_pkg.messages import MotionCommand
from skku_track_drive_pkg.mode_parameters import PERPENDICULAR, PARALLEL, defaults
from .lidar_geometry import collision, sector_distance
from ..perpendicular_parking.state_machine import PerpendicularParkingStateMachine
from ..parallel_parking.state_machine import ParallelParkingStateMachine


class ParkingCore(PerpendicularParkingStateMachine, ParallelParkingStateMachine):
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
