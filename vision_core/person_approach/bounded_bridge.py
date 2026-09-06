"""Offline-only planner from person-approach decisions to bounded API commands."""
from __future__ import annotations

import hashlib
import json
import math
import re
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from typing import Any, Callable


WINDOW_SIZE = 5
MAX_EVIDENCE_AGE_S = 1.0
BOOT_SESSION_PATTERN = re.compile(r"[0-9A-F]{16}")
COMMAND_ID_PATTERN = re.compile(r"[A-Za-z0-9._-]{1,64}")
MOTION_SPECS = {
    "ADVANCE": ("/move-forward", "distance_m", "forward_step_m", "m", 0.02, 0.10),
    "TURN_LEFT": ("/turn-left", "angle_deg", "turn_angle_deg", "deg", 1.0, 10.0),
    "TURN_RIGHT": ("/turn-right", "angle_deg", "turn_angle_deg", "deg", 1.0, 10.0),
}


@dataclass(frozen=True)
class PlannedBoundedCommand:
    schema_version: str
    result: str
    decision_id: str
    method: str
    endpoint: str
    query: dict[str, str]
    command_id: str
    evidence_cycle_ids: list[str]
    evidence_measurement_ids: list[str]
    reference_frame: str
    units: str
    network_performed: bool = False
    reobserve_required: bool = False
    block_reason: None = None

    def to_dict(self) -> dict[str, Any]:
        return _json_safe(asdict(self))


@dataclass(frozen=True)
class BoundedCommandBlock:
    schema_version: str
    result: str
    decision_id: str | None
    method: None
    endpoint: None
    query: None
    command_id: None
    evidence_cycle_ids: list[str]
    evidence_measurement_ids: list[str]
    reference_frame: str | None
    units: str | None
    network_performed: bool
    reobserve_required: bool
    block_reason: str

    def to_dict(self) -> dict[str, Any]:
        return _json_safe(asdict(self))


BridgeResult = PlannedBoundedCommand | BoundedCommandBlock


def _json_safe(value: Any) -> Any:
    return json.loads(json.dumps(value, allow_nan=False, sort_keys=True))


def _timestamp(value: object) -> datetime | None:
    if type(value) is not str:
        return None
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None
    return parsed.astimezone(timezone.utc) if parsed.tzinfo is not None else None


def _text(value: object) -> str | None:
    return value if type(value) is str and value else None


def _finite_number(value: object) -> float | None:
    if type(value) not in (int, float) or not math.isfinite(value):
        return None
    return float(value)


def _canonical_number(value: float) -> str:
    return format(value, ".15g")


def _block(
    reason: str,
    *,
    decision: object = None,
    cycle_ids: list[str] | None = None,
    measurement_ids: list[str] | None = None,
    reobserve_required: bool = False,
) -> BoundedCommandBlock:
    decision_object = decision if type(decision) is dict else {}
    return BoundedCommandBlock(
        schema_version="sie.person_approach_bounded_bridge.v1",
        result="BLOCKED_NO_COMMAND",
        decision_id=_text(decision_object.get("decision_id")),
        method=None,
        endpoint=None,
        query=None,
        command_id=None,
        evidence_cycle_ids=cycle_ids or [],
        evidence_measurement_ids=measurement_ids or [],
        reference_frame=_text(decision_object.get("reference_frame")),
        units=_text(decision_object.get("units")),
        network_performed=False,
        reobserve_required=reobserve_required,
        block_reason=reason,
    )


def plan_bounded_command(
    envelope: object,
    *,
    now_utc: Callable[[], datetime] = lambda: datetime.now(timezone.utc),
) -> BridgeResult:
    """Validate one JSON-safe envelope and return a plan or explicit refusal."""
    try:
        value = _json_safe(envelope)
    except (TypeError, ValueError):
        return _block("ENVELOPE_NOT_JSON_SAFE")
    if type(value) is not dict:
        return _block("ENVELOPE_NOT_OBJECT")

    decision = value.get("decision")
    window = value.get("evidence_window")
    session = value.get("boot_session_id")
    previous = value.get("previous_terminal_motion_outcome")
    if type(decision) is not dict:
        return _block("INVALID_DECISION")
    if type(window) is not list or len(window) != WINDOW_SIZE:
        return _block("EVIDENCE_WINDOW_MUST_CONTAIN_FIVE_CYCLES", decision=decision)
    if type(session) is not str or BOOT_SESSION_PATTERN.fullmatch(session) is None:
        return _block("INVALID_BOOT_SESSION_ID", decision=decision)
    if previous is not None and type(previous) is not dict:
        return _block("INVALID_PREVIOUS_TERMINAL_OUTCOME", decision=decision)
    if previous is not None:
        terminal = previous.get("command_state", previous.get("last_command_state", previous.get("status")))
        if terminal == "PARTIAL_PROGRESS" and previous.get("reobserve_required") is True:
            return _block("REOBSERVE_REQUIRED_AFTER_PARTIAL_PROGRESS", decision=decision, reobserve_required=True)

    cycle_ids: list[str] = []
    measurements: list[tuple[str, str, str, datetime]] = []
    cycle_times: list[datetime] = []
    success_count = 0
    for cycle in window:
        if type(cycle) is not dict:
            return _block("INVALID_EVIDENCE_CYCLE", decision=decision, cycle_ids=cycle_ids)
        cycle_id = _text(cycle.get("cycle_id"))
        captured = _timestamp(cycle.get("captured_at_utc"))
        if cycle_id is None or captured is None:
            return _block("INVALID_EVIDENCE_CYCLE", decision=decision, cycle_ids=cycle_ids)
        cycle_ids.append(cycle_id)
        cycle_times.append(captured)
        if cycle.get("status") == "MULTIPLE_PERSONS" or (
            type(cycle.get("person")) is dict and cycle["person"].get("status") == "MULTIPLE_PERSONS"
        ):
            return _block("MULTIPLE_PERSONS_IN_EVIDENCE_WINDOW", decision=decision, cycle_ids=cycle_ids)
        if cycle.get("status") != "SUCCESS":
            continue
        success_count += 1
        measurement = cycle.get("measurement")
        if type(measurement) is not dict or measurement.get("status") != "SUCCESS":
            return _block("SUCCESS_CYCLE_HAS_INVALID_MEASUREMENT", decision=decision, cycle_ids=cycle_ids)
        identifier = _text(measurement.get("measurement_id"))
        frame = _text(measurement.get("reference_frame"))
        units = _text(measurement.get("units"))
        stamp = _timestamp(measurement.get("timestamp"))
        numeric_names = ("x_m", "y_m", "z_m", "range_m", "confidence")
        if identifier is None or frame is None or units is None or stamp is None:
            return _block("SUCCESS_CYCLE_HAS_INVALID_MEASUREMENT", decision=decision, cycle_ids=cycle_ids)
        if any(_finite_number(measurement.get(name)) is None for name in numeric_names):
            return _block("NON_FINITE_OR_INVALID_MEASUREMENT", decision=decision, cycle_ids=cycle_ids)
        measurements.append((identifier, frame, units, stamp))

    measurement_ids = [item[0] for item in measurements]
    if any(later < earlier for earlier, later in zip(cycle_times, cycle_times[1:])):
        return _block("EVIDENCE_TIMESTAMPS_NOT_MONOTONIC", decision=decision, cycle_ids=cycle_ids, measurement_ids=measurement_ids)
    if window[-1].get("status") != "SUCCESS" or not measurements or measurements[-1][3] != _timestamp(window[-1].get("measurement", {}).get("timestamp")):
        return _block("LATEST_CYCLE_NOT_SUCCESS_WITH_VALID_MEASUREMENT", decision=decision, cycle_ids=cycle_ids, measurement_ids=measurement_ids)
    if success_count < 4:
        return _block("FEWER_THAN_FOUR_SUCCESS_CYCLES", decision=decision, cycle_ids=cycle_ids, measurement_ids=measurement_ids)
    measurement_times = [item[3] for item in measurements]
    if any(later < earlier for earlier, later in zip(measurement_times, measurement_times[1:])):
        return _block("MEASUREMENT_TIMESTAMPS_NOT_MONOTONIC", decision=decision, cycle_ids=cycle_ids, measurement_ids=measurement_ids)
    now = now_utc()
    if not isinstance(now, datetime) or now.tzinfo is None:
        return _block("INVALID_BRIDGE_CLOCK", decision=decision, cycle_ids=cycle_ids, measurement_ids=measurement_ids)
    age = (now.astimezone(timezone.utc) - measurements[-1][3]).total_seconds()
    if not math.isfinite(age) or age < 0 or age > MAX_EVIDENCE_AGE_S:
        return _block("LATEST_CYCLE_STALE_OR_FROM_FUTURE", decision=decision, cycle_ids=cycle_ids, measurement_ids=measurement_ids)

    frames = {item[1] for item in measurements}
    measurement_units = {item[2] for item in measurements}
    decision_id = _text(decision.get("decision_id"))
    decision_frame = _text(decision.get("reference_frame"))
    decision_units = _text(decision.get("units"))
    source_ids = decision.get("source_measurement_ids")
    if decision_id is None or decision_frame is None or decision_units is None or type(source_ids) is not list:
        return _block("INVALID_DECISION_CONTRACT", decision=decision, cycle_ids=cycle_ids, measurement_ids=measurement_ids)
    if frames != {decision_frame} or measurement_units != {decision_units}:
        return _block("REFERENCE_FRAME_OR_UNITS_MISMATCH", decision=decision, cycle_ids=cycle_ids, measurement_ids=measurement_ids)
    if source_ids != measurement_ids or len(set(measurement_ids)) != len(measurement_ids):
        return _block("SOURCE_EVIDENCE_MISMATCH", decision=decision, cycle_ids=cycle_ids, measurement_ids=measurement_ids)

    status = decision.get("status")
    if status not in MOTION_SPECS:
        return _block(f"DECISION_STATUS_{status}_HAS_NO_COMMAND", decision=decision, cycle_ids=cycle_ids, measurement_ids=measurement_ids)
    endpoint, query_name, field_name, command_units, lower, upper = MOTION_SPECS[status]
    parameter = _finite_number(decision.get(field_name))
    if parameter is None:
        return _block("INVALID_COMMAND_PARAMETER", decision=decision, cycle_ids=cycle_ids, measurement_ids=measurement_ids)
    if status.startswith("TURN_"):
        angle_units = _text(decision.get("turn_angle_units"))
        if angle_units != command_units:
            return _block("REFERENCE_FRAME_OR_UNITS_MISMATCH", decision=decision, cycle_ids=cycle_ids, measurement_ids=measurement_ids)
        if (status == "TURN_LEFT" and parameter >= 0) or (status == "TURN_RIGHT" and parameter <= 0):
            return _block("TURN_DIRECTION_ANGLE_SIGN_MISMATCH", decision=decision, cycle_ids=cycle_ids, measurement_ids=measurement_ids)
        parameter = abs(parameter)
    if parameter < lower or parameter > upper:
        return _block("COMMAND_PARAMETER_OUT_OF_RANGE", decision=decision, cycle_ids=cycle_ids, measurement_ids=measurement_ids)

    canonical_parameter = _canonical_number(parameter)
    identity = "\n".join((decision_id, endpoint, query_name, canonical_parameter, session))
    command_id = "pa-" + hashlib.sha256(identity.encode("ascii")).hexdigest()[:61]
    if COMMAND_ID_PATTERN.fullmatch(command_id) is None:
        return _block("INTERNAL_COMMAND_ID_INVALID", decision=decision, cycle_ids=cycle_ids, measurement_ids=measurement_ids)
    query = {"boot_session_id": session, "command_id": command_id, query_name: canonical_parameter}
    return PlannedBoundedCommand(
        schema_version="sie.person_approach_bounded_bridge.v1",
        result="PLANNED_BOUNDED_COMMAND",
        decision_id=decision_id,
        method="POST",
        endpoint=endpoint,
        query=query,
        command_id=command_id,
        evidence_cycle_ids=cycle_ids,
        evidence_measurement_ids=measurement_ids,
        reference_frame=decision_frame,
        units=command_units,
    )
