"""Fail-closed temporal stabilization for live person-depth cycle records."""
from __future__ import annotations

import json
import math
from collections import deque
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from typing import Any, Callable

import numpy as np

WINDOW_SIZE = 5


@dataclass(frozen=True)
class PersonApproachDecision:
    schema_version: str
    decision_id: str
    timestamp: str
    status: str
    reference_frame: str | None
    units: str
    target_z_m: float
    median_x_m: float | None
    median_z_m: float | None
    median_range_m: float | None
    mad_x_m: float | None
    mad_z_m: float | None
    turn_angle_deg: float | None
    turn_angle_units: str
    forward_step_m: float | None
    source_measurement_ids: list[str]
    detail: str

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        json.dumps(value, allow_nan=False, sort_keys=True)
        return value


class PersonApproachDecisionEngine:
    """Consumes records only; it does not issue an Action or invoke hardware."""

    def __init__(self, *, target_z_m: float = 2.0, z_tolerance_m: float = .10,
                 x_tolerance_m: float = .10, maximum_forward_step_m: float = .10,
                 maximum_turn_angle_deg: float = 10.0,
                 now_utc: Callable[[], datetime] = lambda: datetime.now(timezone.utc)) -> None:
        values = (target_z_m, z_tolerance_m, x_tolerance_m, maximum_forward_step_m, maximum_turn_angle_deg)
        if not all(type(value) in (int, float) and math.isfinite(value) and value > 0 for value in values):
            raise ValueError("approach parameters must be finite positive int/float values")
        self.target_z_m, self.z_tolerance_m, self.x_tolerance_m = map(float, values[:3])
        self.maximum_forward_step_m, self.maximum_turn_angle_deg = map(float, values[3:])
        self.now_utc = now_utc
        self._records: deque[dict[str, Any]] = deque(maxlen=WINDOW_SIZE)
        self._decision_sequence = 0

    def ingest(self, record: dict[str, Any]) -> PersonApproachDecision:
        if type(record) is not dict:
            return self._blocked("BLOCKED_DEPTH_UNAVAILABLE", "cycle record must be an object", None, [])
        try:
            copied = json.loads(json.dumps(record, allow_nan=False, sort_keys=True))
        except (TypeError, ValueError):
            self._records.append({"status": "INVALID_RECORD"})
            return self._blocked("BLOCKED_UNSTABLE", "cycle record is not JSON-safe finite data", None, [])
        self._records.append(copied)
        return self._decide()

    def _blocked(self, status: str, detail: str, reference_frame: str | None, sources: list[str], *, medians: tuple[float, float, float, float, float] | None = None) -> PersonApproachDecision:
        values = (None, None, None, None, None) if medians is None else medians
        return PersonApproachDecision("sie.person_approach_decision.v1", self._next_id(), self._timestamp(), status, reference_frame, "m", self.target_z_m, *values, None, "deg", None, sources, detail)

    def _next_id(self) -> str:
        self._decision_sequence += 1
        return f"decision.person_approach.{self._decision_sequence:06d}"

    def _timestamp(self) -> str:
        value = self.now_utc()
        if not isinstance(value, datetime) or value.tzinfo is None:
            raise ValueError("now_utc must return timezone-aware datetime")
        return value.astimezone(timezone.utc).isoformat()

    @staticmethod
    def _parse_timestamp(value: object) -> datetime | None:
        if type(value) is not str:
            return None
        try:
            parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
        except ValueError:
            return None
        return parsed.astimezone(timezone.utc) if parsed.tzinfo is not None else None

    def _decide(self) -> PersonApproachDecision:
        records = list(self._records)
        if len(records) < WINDOW_SIZE:
            return self._blocked("BLOCKED_WARMUP", f"need {WINDOW_SIZE} cycles, have {len(records)}", None, [])
        if any(row.get("status") == "MULTIPLE_PERSONS" for row in records):
            return self._blocked("BLOCKED_MULTIPLE_PERSONS", "multiple persons occurred in stabilization window", None, [])
        latest = records[-1]
        if latest.get("status") == "PERSON_LOST":
            return self._blocked("BLOCKED_PERSON_LOST", "latest cycle lost person", None, [])
        if latest.get("status") != "SUCCESS":
            return self._blocked("BLOCKED_DEPTH_UNAVAILABLE", f"latest cycle status is {latest.get('status')!r}", None, [])
        successes = [row for row in records if row.get("status") == "SUCCESS"]
        if len(successes) < 4:
            return self._blocked("BLOCKED_DEPTH_UNAVAILABLE", "fewer than four successful measurements in window", None, [])
        parsed = [self._measurement(row) for row in successes]
        if any(value is None for value in parsed):
            return self._blocked("BLOCKED_UNSTABLE", "successful cycle has malformed or non-finite measurement", None, [])
        measurements = [value for value in parsed if value is not None]
        frames = {value[0] for value in measurements}
        timestamps = [value[1] for value in measurements]
        if len(frames) != 1 or any(later < earlier for earlier, later in zip(timestamps, timestamps[1:])):
            return self._blocked("BLOCKED_UNSTABLE", "measurement reference frame or timestamp ordering is invalid", None, [])
        age = (self.now_utc().astimezone(timezone.utc) - timestamps[-1]).total_seconds()
        if not math.isfinite(age) or age < 0 or age > 1.0:
            return self._blocked("BLOCKED_STALE", "latest measurement is stale or from the future", next(iter(frames)), [])
        xs = np.array([value[2] for value in measurements], dtype=np.float64)
        zs = np.array([value[3] for value in measurements], dtype=np.float64)
        ranges = np.array([value[4] for value in measurements], dtype=np.float64)
        median_x, median_z, median_range = float(np.median(xs)), float(np.median(zs)), float(np.median(ranges))
        mad_x, mad_z = float(np.median(abs(xs-median_x))), float(np.median(abs(zs-median_z)))
        sources = [value[5] for value in measurements]
        medians = (median_x, median_z, median_range, mad_x, mad_z)
        frame = next(iter(frames))
        if mad_x > .08 or mad_z > .08:
            return self._blocked("BLOCKED_UNSTABLE", "median absolute deviation exceeds MVP stability gate", frame, sources, medians=medians)
        if abs(median_x) <= self.x_tolerance_m and abs(median_z-self.target_z_m) <= self.z_tolerance_m:
            return self._blocked("HOLD_TARGET_REACHED", "within lateral and axial tolerance", frame, sources, medians=medians)
        if median_z < self.target_z_m-self.z_tolerance_m:
            return self._blocked("HOLD_TARGET_REACHED", "TOO_CLOSE_NO_REVERSE", frame, sources, medians=medians)
        if abs(median_x) > self.x_tolerance_m:
            angle = math.degrees(math.atan2(median_x, median_z))
            angle = max(-self.maximum_turn_angle_deg, min(self.maximum_turn_angle_deg, angle))
            return PersonApproachDecision("sie.person_approach_decision.v1", self._next_id(), self._timestamp(), "TURN_RIGHT" if angle > 0 else "TURN_LEFT", frame, "m", self.target_z_m, *medians, angle, "deg", None, sources, "lateral error exceeds tolerance")
        step = min(median_z-self.target_z_m, self.maximum_forward_step_m)
        return PersonApproachDecision("sie.person_approach_decision.v1", self._next_id(), self._timestamp(), "ADVANCE", frame, "m", self.target_z_m, *medians, None, "deg", step, sources, "aligned and beyond target")

    def _measurement(self, record: dict[str, Any]) -> tuple[str, datetime, float, float, float, str] | None:
        value = record.get("measurement")
        if type(value) is not dict or value.get("status") != "SUCCESS":
            return None
        frame, timestamp, identifier = value.get("reference_frame"), self._parse_timestamp(value.get("timestamp")), value.get("measurement_id")
        numbers = (value.get("x_m"), value.get("z_m"), value.get("range_m"))
        if type(frame) is not str or not frame or timestamp is None or type(identifier) is not str or not identifier:
            return None
        if any(type(number) not in (int, float) or not math.isfinite(number) for number in numbers):
            return None
        return frame, timestamp, float(numbers[0]), float(numbers[1]), float(numbers[2]), identifier
