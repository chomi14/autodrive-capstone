"""Short-lock camera/worker progress, read by the independent health callback."""
import threading
import time


class PerceptionProgress:
    def __init__(self):
        self.lock = threading.Lock()
        self.camera_at = self.completed_at = self.result_frame_at = None
        self.camera_sequence = self.result_sequence = self.errors = 0
        self.busy = False

    def receive(self, now):
        with self.lock:
            self.camera_at = now
            self.camera_sequence += 1

    def start(self):
        with self.lock:
            self.busy = True

    def complete(self, completed, frame_received):
        with self.lock:
            self.completed_at, self.result_frame_at = completed, frame_received
            self.result_sequence += 1

    def finish(self, failed=False):
        with self.lock:
            self.busy = False
            self.errors += int(failed)

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            return {'camera_sequence': self.camera_sequence, 'result_sequence': self.result_sequence,
                'camera_age_s': None if self.camera_at is None else max(0.0, now - self.camera_at),
                'result_age_s': None if self.completed_at is None else max(0.0, now - self.completed_at),
                'result_frame_age_s': None if self.result_frame_at is None else max(0.0, now - self.result_frame_at),
                'worker_busy': self.busy, 'errors': self.errors}
