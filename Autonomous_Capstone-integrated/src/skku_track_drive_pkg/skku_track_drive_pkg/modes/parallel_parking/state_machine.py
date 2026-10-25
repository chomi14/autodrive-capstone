"""Parallel parking state transitions; shared checks run in ParkingCore."""
import math
from skku_track_drive_pkg.messages import MotionCommand
from skku_track_drive_pkg.modes.parking_common.lidar_geometry import side_line_angle


class ParallelParkingStateMachine:
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

