"""Perpendicular parking state transitions; shared checks run in ParkingCore."""
from skku_track_drive_pkg.messages import MotionCommand


class PerpendicularParkingStateMachine:
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

