"""Sequential execution with explicit outcomes, independently testable offline."""
from enum import Enum
import math
from threading import Event


class Outcome(str, Enum):
    COMPLETE = "complete"
    STOPPED = "stopped"
    FAILED = "failed"
    DISCONNECTED = "disconnected"
    INVALID = "invalid"


class TrajectoryExecutor:
    def __init__(self):
        self.cancelled = Event()

    def cancel(self):
        self.cancelled.set()

    def reset(self):
        self.cancelled.clear()

    def execute(self, points, move, connected):
        if not points or any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in points):
            return Outcome.INVALID
        if self.cancelled.is_set():
            return Outcome.STOPPED
        if not connected():
            return Outcome.DISCONNECTED
        for point in points:
            if self.cancelled.is_set():
                return Outcome.STOPPED
            if not connected():
                return Outcome.DISCONNECTED
            try:
                moved = move(*point)
            except Exception:
                return Outcome.STOPPED if self.cancelled.is_set() else Outcome.FAILED
            if self.cancelled.is_set():
                return Outcome.STOPPED
            if not moved:
                return Outcome.FAILED
        return Outcome.COMPLETE
