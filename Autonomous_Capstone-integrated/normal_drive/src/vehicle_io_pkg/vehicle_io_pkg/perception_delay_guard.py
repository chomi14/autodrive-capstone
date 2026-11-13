"""Bounded perception silence: unchanged command hold, then latched stop.

No timers, ROS, serial or hardware here. Numerical policy comes from the
measured launch configuration. A repeated process heartbeat is NOT a result.
"""
import math


class PerceptionDelayGuard:
    def __init__(self, stop_s):
        if not math.isfinite(stop_s) or stop_s <= 0:
            raise ValueError('perception stop age must be finite and positive')
        self.stop_s = stop_s
        self.sequence = -1
        self.fresh_at = None
        self.camera_at = None

    def reset_owner(self):
        self.sequence = -1
        self.reset_run()

    def reset_run(self):
        # Keep the sequence floor: a pre-stop result repeated in a new
        # challenge must not count as a post-stop successful observation.
        self.fresh_at = self.camera_at = None

    def observe(self, progress, now):
        if not isinstance(progress, dict):
            return False
        sequence = progress.get('result_sequence')
        if type(sequence) is not int or sequence < 0:
            return False
        camera_age = progress.get('camera_age_s')
        if type(camera_age) in (int, float) and math.isfinite(camera_age) and camera_age >= 0:
            self.camera_at = now - camera_age
        if sequence <= self.sequence:
            return False
        self.sequence = sequence
        ages = (progress.get('result_age_s'), progress.get('result_frame_age_s'))
        if not all(type(v) in (int, float) and math.isfinite(v) and v >= 0 for v in ages):
            return False
        # A result just completed from an old queued frame is still stale.
        self.fresh_at = now - max(ages)
        return True

    def age(self, now):
        return None if self.fresh_at is None else max(0.0, now - self.fresh_at)

    def ready(self, now):
        age = self.age(now)
        return age is not None and age < self.stop_s

    def expired(self, now):
        age = self.age(now)
        return age is not None and age >= self.stop_s

    def reason(self, now):
        return ('camera input stopped' if self.camera_at is None or now - self.camera_at >= self.stop_s
                else 'inference worker/result stopped')

    def output(self, steering, left, right, now):
        # No scaling or steering override during perception silence. The
        # sender caches the latest valid command, including intentional zero.
        if self.fresh_at is None or self.expired(now):
            return steering, 0, 0
        return steering, left, right

    def status(self, now):
        return {'result_age_s': self.age(now), 'perception_stop_s': self.stop_s}
