"""Retain transient stop observations after the live status recovers."""
from collections import deque
from dataclasses import dataclass
import time


@dataclass(frozen=True)
class StopEvent:
    source: str
    reason: str
    timestamp: str


class StopHistory:
    def __init__(self, capacity=3):
        self.events = deque(maxlen=capacity)
        self._active = {}

    def observe(self, source, reason):
        """Record a new reason once; recovery resets deduplication, not history."""
        if not reason:
            self._active.pop(source, None)
            return None
        if self._active.get(source) == reason:
            return None
        self._active[source] = reason
        event = StopEvent(source, reason, time.strftime('%H:%M:%S'))
        self.events.appendleft(event)
        return event
