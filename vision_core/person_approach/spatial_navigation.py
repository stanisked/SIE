"""Metric bearing and distance decisions for supervised person approach."""
from __future__ import annotations

import json
import math
from datetime import datetime, timezone
from typing import Any, Callable

import numpy as np

from vision_core.person_approach.bounded_bridge import WINDOW_SIZE


RECTIFIED_LEFT_FRAME = "rectified_left_optical_frame"


def _safe(value: object) -> Any:
    return json.loads(json.dumps(value, allow_nan=False, sort_keys=True))


def _stamp(value: object) -> datetime | None:
    if type(value) is not str:
        return None
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None
    return parsed.astimezone(timezone.utc) if parsed.tzinfo is not None else None


def _finite(value: object) -> float | None:
    if type(value) not in (int, float) or not math.isfinite(value):
        return None
    return float(value)


def evaluate_person_navigation_window(
    cycles: object,
    *,
    safe_distance_m: float,
    bearing_deadband_deg: float,
    now_utc: Callable[[], datetime] = lambda: datetime.now(timezone.utc),
) -> dict[str, Any]:
    """Fail closed on invalid shared evidence; never use image-centre as a gate."""
    if (
        _finite(safe_distance_m) is None
        or float(safe_distance_m) <= 0.0
        or _finite(bearing_deadband_deg) is None
        or float(bearing_deadband_deg) < 0.0
    ):
        raise ValueError("safe distance and bearing deadband must be finite; distance positive")
    try:
        window = _safe(cycles)
    except (TypeError, ValueError):
        return {"result": "BLOCKED", "reason": "WINDOW_NOT_JSON_SAFE"}
    if type(window) is not list or len(window) != WINDOW_SIZE or any(type(item) is not dict for item in window):
        return {"result": "BLOCKED", "reason": "WINDOW_MUST_CONTAIN_FIVE_CYCLES"}

    cycle_ids: list[str] = []
    measurements: list[tuple[str, datetime, float, float, float]] = []
    yolo_statuses: list[str] = []
    for cycle in window:
        cycle_id = cycle.get("cycle_id")
        primary = cycle.get("primary_person_observation")
        if type(cycle_id) is not str or type(primary) is not dict:
            return {"result": "BLOCKED", "reason": "MISSING_CYCLE_OR_YOLO_PROVENANCE"}
        cycle_ids.append(cycle_id)
        target_status = primary.get("target_status")
        if type(target_status) is not str:
            return {"result": "BLOCKED", "reason": "INVALID_YOLO_TARGET_STATUS", "source_window_cycle_ids": cycle_ids}
        yolo_statuses.append(target_status)
        if target_status == "MULTIPLE_TARGETS":
            return {"result": "BLOCKED", "reason": "YOLO_MULTIPLE_TARGETS", "source_window_cycle_ids": cycle_ids}
        if cycle.get("status") != "SUCCESS":
            continue
        measurement = cycle.get("measurement")
        if type(measurement) is not dict or measurement.get("status") != "SUCCESS":
            return {"result": "BLOCKED", "reason": "SUCCESS_CYCLE_METRIC_MISSING", "source_window_cycle_ids": cycle_ids}
        identifier = measurement.get("measurement_id")
        timestamp = _stamp(measurement.get("timestamp"))
        x_m, z_m, range_m = (_finite(measurement.get(name)) for name in ("x_m", "z_m", "range_m"))
        if (
            type(identifier) is not str
            or measurement.get("reference_frame") != RECTIFIED_LEFT_FRAME
            or measurement.get("units") != "m"
            or timestamp is None or x_m is None or z_m is None or range_m is None
            or z_m <= 0.0
        ):
            return {"result": "BLOCKED", "reason": "INVALID_METRIC_MEASUREMENT", "source_window_cycle_ids": cycle_ids}
        measurements.append((identifier, timestamp, x_m, z_m, range_m))

    if yolo_statuses[-1] != "SINGLE_TARGET" or yolo_statuses.count("SINGLE_TARGET") < 4:
        return {"result": "BLOCKED", "reason": "YOLO_SINGLE_TARGET_NOT_STABLE", "source_window_cycle_ids": cycle_ids}
    if window[-1].get("status") != "SUCCESS" or len(measurements) < 4:
        return {"result": "BLOCKED", "reason": "CURRENT_METRIC_DEPTH_UNAVAILABLE", "source_window_cycle_ids": cycle_ids}
    if any(later[1] < earlier[1] for earlier, later in zip(measurements, measurements[1:])):
        return {"result": "BLOCKED", "reason": "METRIC_TIMESTAMPS_NOT_MONOTONIC", "source_window_cycle_ids": cycle_ids}
    current = now_utc()
    if not isinstance(current, datetime) or current.tzinfo is None:
        return {"result": "BLOCKED", "reason": "INVALID_RUNNER_CLOCK", "source_window_cycle_ids": cycle_ids}
    age_s = (current.astimezone(timezone.utc) - measurements[-1][1]).total_seconds()
    if not math.isfinite(age_s) or age_s < 0.0 or age_s > 1.0:
        return {"result": "BLOCKED", "reason": "METRIC_EVIDENCE_STALE_OR_FUTURE", "source_window_cycle_ids": cycle_ids, "metric_age_s": age_s}

    median_x = float(np.median([item[2] for item in measurements]))
    median_z = float(np.median([item[3] for item in measurements]))
    median_range = float(np.median([item[4] for item in measurements]))
    bearing = math.degrees(math.atan2(median_x, median_z))
    base: dict[str, Any] = {
        "schema_version": "sie.person_navigation_metric_window.v1",
        "source_window_cycle_ids": cycle_ids,
        "source_measurement_ids": [item[0] for item in measurements],
        "reference_frame": RECTIFIED_LEFT_FRAME,
        "units": "m",
        "median_x_m": median_x,
        "median_z_m": median_z,
        "median_range_m": median_range,
        "bearing_deg": bearing,
        "safe_distance_m": float(safe_distance_m),
        "bearing_deadband_deg": float(bearing_deadband_deg),
        "metric_age_s": age_s,
        "image_center_used_as_gate": False,
    }
    if median_z <= float(safe_distance_m):
        return {**base, "result": "ARRIVED", "reason": "SAFE_DISTANCE_REACHED", "action": None}
    if abs(bearing) > float(bearing_deadband_deg):
        angle = min(10.0, max(1.0, abs(bearing)))
        return {
            **base,
            "result": "TURN_REQUIRED",
            "reason": "BEARING_OUTSIDE_DEADBAND",
            "action": {"status": "TURN_RIGHT" if bearing > 0.0 else "TURN_LEFT", "angle_deg": angle},
        }
    return {
        **base,
        "result": "FORWARD_REQUIRED",
        "reason": "BEARING_ALIGNED_AND_BEYOND_SAFE_DISTANCE",
        "action": {"status": "ADVANCE", "distance_m": 0.10},
    }


def navigation_decision(window: dict[str, Any], *, sequence: int, timestamp: str) -> dict[str, Any]:
    """Adapt the metric navigation result to the existing bounded bridge contract."""
    action = window.get("action")
    if type(action) is not dict or type(window.get("source_measurement_ids")) is not list:
        raise ValueError("navigation result has no bounded action")
    status = action.get("status")
    if status not in {"ADVANCE", "TURN_LEFT", "TURN_RIGHT"}:
        raise ValueError("navigation action is not bounded")
    decision = {
        "schema_version": "sie.person_approach_decision.v1",
        "decision_id": f"decision.person_navigation.{sequence:06d}",
        "timestamp": timestamp,
        "status": status,
        "reference_frame": window.get("reference_frame"),
        "units": "m",
        "target_z_m": window.get("safe_distance_m"),
        "median_x_m": window.get("median_x_m"),
        "median_z_m": window.get("median_z_m"),
        "median_range_m": window.get("median_range_m"),
        "mad_x_m": 0.0,
        "mad_z_m": 0.0,
        "turn_angle_deg": None,
        "turn_angle_units": "deg",
        "forward_step_m": None,
        "source_measurement_ids": window["source_measurement_ids"],
        "detail": window.get("reason"),
    }
    if status == "ADVANCE":
        decision["forward_step_m"] = action["distance_m"]
    else:
        decision["turn_angle_deg"] = action["angle_deg"] if status == "TURN_RIGHT" else -action["angle_deg"]
    return _safe(decision)
