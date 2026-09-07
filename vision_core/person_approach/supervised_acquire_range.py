"""Offline-only coordinator for supervised yaw alignment and range acquisition."""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from typing import Any, Callable


TEMPORAL_SCHEMA = "sie.temporal_yaw_alignment.v1"
DECISION_SCHEMA = "sie.person_approach_decision.v1"
IMAGE_FRAME = "ar0234_image_frame"
MAX_RECORD_AGE_S = 1.0


@dataclass(frozen=True)
class SupervisedAcquireRangeResult:
    schema_version: str
    stage: str
    result: str
    reason: str | None
    reobserve_required: bool
    source_evidence_ids: list[str]
    source_measurement_ids: list[str]
    temporal_reference_frame: str | None
    temporal_units: str | None
    depth_reference_frame: str | None
    depth_units: str | None
    planned_turn: dict[str, Any] | None
    network_performed: bool = False
    motor_command_performed: bool = False

    def to_dict(self) -> dict[str, Any]:
        return _json_safe(asdict(self))


def _json_safe(value: Any) -> Any:
    return json.loads(json.dumps(value, allow_nan=False, sort_keys=True))


def _text(value: object) -> str | None:
    return value if type(value) is str and value else None


def _finite(value: object) -> float | None:
    if type(value) not in (int, float) or not math.isfinite(value):
        return None
    return float(value)


def _timestamp(value: object) -> datetime | None:
    if type(value) is not str:
        return None
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None
    return parsed.astimezone(timezone.utc) if parsed.tzinfo is not None else None


def _fresh(value: object, now_utc: Callable[[], datetime]) -> bool:
    stamp = _timestamp(value)
    now = now_utc()
    if stamp is None or not isinstance(now, datetime) or now.tzinfo is None:
        return False
    age = (now.astimezone(timezone.utc) - stamp).total_seconds()
    return math.isfinite(age) and 0.0 <= age <= MAX_RECORD_AGE_S


def _result(
    *,
    stage: str,
    result: str,
    reason: str | None,
    reobserve_required: bool,
    evidence_ids: list[str] | None = None,
    measurement_ids: list[str] | None = None,
    temporal_frame: str | None = None,
    temporal_units: str | None = None,
    depth_frame: str | None = None,
    depth_units: str | None = None,
    planned_turn: dict[str, Any] | None = None,
) -> dict[str, Any]:
    return SupervisedAcquireRangeResult(
        schema_version="sie.supervised_acquire_range.v1",
        stage=stage,
        result=result,
        reason=reason,
        reobserve_required=reobserve_required,
        source_evidence_ids=evidence_ids or [],
        source_measurement_ids=measurement_ids or [],
        temporal_reference_frame=temporal_frame,
        temporal_units=temporal_units,
        depth_reference_frame=depth_frame,
        depth_units=depth_units,
        planned_turn=planned_turn,
    ).to_dict()


def _block(reason: str, *, temporal: object = None) -> dict[str, Any]:
    value = temporal if type(temporal) is dict else {}
    evidence_ids = value.get("used_evidence_ids")
    return _result(
        stage="BLOCKED_NO_ACTION", result="BLOCKED_NO_ACTION", reason=reason,
        reobserve_required=True,
        evidence_ids=evidence_ids if type(evidence_ids) is list and all(_text(item) is not None for item in evidence_ids) else [],
        temporal_frame=_text(value.get("reference_frame")), temporal_units=_text(value.get("units")),
    )


def _temporal_contract(value: object, now_utc: Callable[[], datetime]) -> tuple[dict[str, Any], list[str]] | None:
    if type(value) is not dict or value.get("schema_version") != TEMPORAL_SCHEMA:
        return None
    if value.get("reference_frame") != IMAGE_FRAME or value.get("units") != "px":
        return None
    if not _fresh(value.get("timestamp"), now_utc):
        return None
    evidence_ids = value.get("used_evidence_ids")
    if type(evidence_ids) is not list or not evidence_ids or any(_text(item) is None for item in evidence_ids):
        return None
    if type(value.get("valid_single_person_count")) is not int or not 4 <= value["valid_single_person_count"] <= 5:
        return None
    if type(value.get("person_lost_count")) is not int or not 0 <= value["person_lost_count"] <= 1:
        return None
    if value.get("latest_person_status") != "SINGLE_PERSON":
        return None
    if len(evidence_ids) != value["valid_single_person_count"] or len(set(evidence_ids)) != len(evidence_ids):
        return None
    if any(_finite(value.get(name)) is None for name in ("robust_median_image_offset_px", "mad_image_offset_px", "center_tolerance_px")):
        return None
    if value["center_tolerance_px"] < 0:
        return None
    return value, list(evidence_ids)


def _valid_measurement(value: object, now_utc: Callable[[], datetime]) -> tuple[str, str, str] | None:
    if type(value) is not dict or value.get("status") != "SUCCESS":
        return None
    identifier = _text(value.get("measurement_id"))
    frame = _text(value.get("reference_frame"))
    units = _text(value.get("units"))
    if identifier is None or frame is None or units != "m" or not _fresh(value.get("timestamp"), now_utc):
        return None
    if any(_finite(value.get(name)) is None for name in ("x_m", "y_m", "z_m", "range_m")):
        return None
    return identifier, frame, units


def _depth_contract(value: object, now_utc: Callable[[], datetime]) -> tuple[list[str], str, str] | None:
    if type(value) is not dict:
        return None
    schema = _text(value.get("schema_version"))
    if schema == "sie.person_depth_live_cycle.v1":
        person = value.get("person")
        if value.get("status") != "SUCCESS" or type(person) is not dict or person.get("status") != "SINGLE_PERSON":
            return None
        if not _fresh(value.get("captured_at_utc"), now_utc):
            return None
    elif schema != "sie.person_depth_offline_report.v1":
        return None
    measurement = _valid_measurement(value.get("measurement"), now_utc)
    if measurement is None:
        return None
    identifier, frame, units = measurement
    return [identifier], frame, units


def _decision_contract(value: object, now_utc: Callable[[], datetime]) -> tuple[list[str], str, str] | None:
    if type(value) is not dict or value.get("schema_version") != DECISION_SCHEMA:
        return None
    status = _text(value.get("status"))
    frame = _text(value.get("reference_frame"))
    units = _text(value.get("units"))
    sources = value.get("source_measurement_ids")
    if status is None or status.startswith("BLOCKED_") or frame is None or units != "m" or not _fresh(value.get("timestamp"), now_utc):
        return None
    if type(sources) is not list or not sources or any(_text(item) is None for item in sources):
        return None
    return list(sources), frame, units


def coordinate_supervised_acquire_range(
    envelope: object,
    *,
    now_utc: Callable[[], datetime] = lambda: datetime.now(timezone.utc),
) -> dict[str, Any]:
    """Route existing offline records between supervised stages without execution."""
    try:
        value = _json_safe(envelope)
    except (TypeError, ValueError):
        return _block("INPUT_NOT_JSON_SAFE")
    if type(value) is not dict:
        return _block("INPUT_MUST_BE_OBJECT")
    temporal = value.get("temporal_alignment")
    temporal_contract = _temporal_contract(temporal, now_utc)
    if temporal_contract is None:
        return _block("INVALID_STALE_OR_MULTIPLE_TEMPORAL_ALIGNMENT", temporal=temporal)
    temporal_value, evidence_ids = temporal_contract
    if any(key in value for key in ("person_depth", "person_approach_decision")) and (
        value.get("person_depth") is not None and value.get("person_approach_decision") is not None
    ):
        return _block("DEPTH_AND_DECISION_INPUTS_ARE_MUTUALLY_EXCLUSIVE", temporal=temporal_value)

    temporal_result = temporal_value.get("result")
    if temporal_result == "PLANNED_TURN":
        planned_turn = temporal_value.get("planned_command")
        if (
            temporal_value.get("stage") != "AWAIT_REOBSERVATION"
            or temporal_value.get("reobserve_required") is not True
            or type(planned_turn) is not dict
            or planned_turn.get("method") != "POST"
            or planned_turn.get("endpoint") not in {"/turn-left", "/turn-right"}
            or _finite(planned_turn.get("angle_deg")) != 4.0
        ):
            return _block("INVALID_TEMPORAL_PLANNED_TURN", temporal=temporal_value)
        return _result(
            stage="AWAIT_OPERATOR_TURN_AND_REOBSERVATION", result="AWAIT_OPERATOR_TURN_AND_REOBSERVATION",
            reason=None, reobserve_required=True, evidence_ids=evidence_ids,
            temporal_frame=IMAGE_FRAME, temporal_units="px", planned_turn=planned_turn,
        )
    if (
        temporal_result != "NO_TURN_CENTERED"
        or temporal_value.get("stage") != "NO_TURN_CENTERED"
        or temporal_value.get("planned_command") is not None
        or temporal_value.get("reobserve_required") is not False
    ):
        return _block("NONCENTERED_ALIGNMENT_HAS_NO_VALID_PLANNED_TURN", temporal=temporal_value)

    depth_value = value.get("person_depth")
    decision_value = value.get("person_approach_decision")
    if depth_value is None and decision_value is None:
        return _result(
            stage="RANGE_ACQUISITION_REQUIRED", result="RANGE_ACQUISITION_REQUIRED", reason="NO_CURRENT_VALID_DEPTH_RECORD",
            reobserve_required=True, evidence_ids=evidence_ids, temporal_frame=IMAGE_FRAME, temporal_units="px",
        )
    if depth_value is not None:
        depth = _depth_contract(depth_value, now_utc)
        if depth is None:
            return _block("INVALID_STALE_MULTIPLE_OR_MISMATCHED_DEPTH_RECORD", temporal=temporal_value)
        measurement_ids, frame, units = depth
    else:
        decision = _decision_contract(decision_value, now_utc)
        if decision is None:
            return _block("INVALID_STALE_MULTIPLE_OR_MISMATCHED_DECISION_RECORD", temporal=temporal_value)
        measurement_ids, frame, units = decision
    return _result(
        stage="DEPTH_APPROACH_DECISION_REQUIRED", result="DEPTH_APPROACH_DECISION_REQUIRED",
        reason="VALID_DEPTH_PROVENANCE_AVAILABLE", reobserve_required=False,
        evidence_ids=evidence_ids, measurement_ids=measurement_ids,
        temporal_frame=IMAGE_FRAME, temporal_units="px", depth_frame=frame, depth_units=units,
    )
