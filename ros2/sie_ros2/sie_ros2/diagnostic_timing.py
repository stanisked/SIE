"""Host-only stage timings; no dependency on ROS, cameras or model libraries."""
from __future__ import annotations

from functools import wraps
import time


class StageTimings:
    """Accumulate inclusive and exclusive wall time, even when a stage raises.

    Exclusive durations can be summed. Inclusive parents contain nested stages
    and must not be added again. Only synchronous work in this thread is timed.
    """
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.started = clock()
        self.stages = {}
        self.stack = []

    def call(self, name, function, *args, **kwargs):
        frame = [self.clock(), 0.0]
        self.stack.append(frame)
        try:
            return function(*args, **kwargs)
        finally:
            elapsed = self.clock() - frame[0]
            self.stack.pop()
            row = self.stages.setdefault(name, {"calls": 0, "inclusive_s": 0.0, "exclusive_s": 0.0})
            row["calls"] += 1
            row["inclusive_s"] += elapsed
            row["exclusive_s"] += max(0.0, elapsed - frame[1])
            if self.stack:
                self.stack[-1][1] += elapsed

    def snapshot(self):
        if self.stack:
            raise RuntimeError("cannot snapshot an unfinished timing stage")
        total = self.clock() - self.started
        stages = {name: dict(row) for name, row in sorted(self.stages.items())}
        accounted = sum(r["exclusive_s"] for r in stages.values())
        return {"host_capture_start_to_result_ready_s": total,
                "stages": stages,
                "unattributed_s": max(0.0, total - accounted),
                "endpoint": "BEFORE_FINAL_AUDIT_WRITE_AND_PUBLISH",
                "exposure_age_known": False,
                "definition": "Sum exclusive_s only; inclusive_s includes child stages. File stages include preparation and encoding."}


def timed_stage(name):
    def decorate(function):
        @wraps(function)
        def wrapped(self, *args, **kwargs):
            timer = getattr(self, "_stage_timings", None)
            if timer is None:
                return function(self, *args, **kwargs)
            return timer.call(name, function, self, *args, **kwargs)
        return wrapped
    return decorate
