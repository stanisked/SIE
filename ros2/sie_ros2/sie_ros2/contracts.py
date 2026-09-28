""""Strict JSON contracts for the first safe SIE ROS 2 pipeline."""

from __future__ import annotations

import json
import math
from datetime import datetime
from typing import Any

PERCEPTION_SCHEMA = "sie.perception.measurement.v1"
AR0234_OBSERVATION_SCHEMA = "sie.ar0234.yolo11_person_upper_body_observation.v1"
NAVIGATION_SCHEMA = "sie.navigation.decision.v1"
SUPERVISOR_SCHEMA = "sie.supervisor.state.v1"


class ContractError(ValueError):
    """Raised when a message cannot enter the SIE ROS graph."""


def _object(value: object, name: str) -> dict[str, Any]:
    if type(value) is not dict:
        raise ContractError(f"{name} must be an object")
    return value


def _text(value: object, name: str) -> str:
    if type(value) is not str or not value.strip():
        raise ContractError(f"{name} must be a non-empty string")
    return value


def _finite(value: object, name: str, *, minimum: float | None = None) -> float:
    if type(value) not in (int, float) or isinstance(value, bool) or not math.isfinite(float(value)):
        raise ContractError(f"{name} must be finite")
    result = float(value)
    if minimum is not None and result < minimum:
        raise ContractError(f"{name} must be >= {minimum}")
    return result


def _integer(value: object, name: str, *, minimum: int = 0) -> int:
    if type(value) is not int or value < minimum:
        raise ContractError(f"{name} must be an integer >= {minimum}")
    return value


def _timestamp(value: object, field_name: str = "timestamp") -> str:
    text = _text(value, field_name)
    try:
        parsed = datetime.fromisoformat(text.replace("Z", "+00:00"))
    except ValueError as error:
        raise ContractError(f"{field_name} must be ISO-8601") from error
    if parsed.tzinfo is None:
        raise ContractError(f"{field_name} must be timezone-aware")
    return text


def _sha256(value: object, field_name: str) -> str:
    sha = _text(value, field_name)
    if len(sha) != 64 or any(char not in "0123456789abcdef" for char in sha.lower()):
        raise ContractError(f"{field_name} must be a SHA-256 hex digest")
    return sha


def decode_json(text: str) -> dict[str, Any]:
    try:
        return _object(json.loads(text), "message")
    except json.JSONDecodeError as error:
        raise ContractError(f"invalid JSON: {error.msg}") from error


def encode(value: dict[str, Any]) -> str:
    return json.dumps(value, allow_nan=False, sort_keys=True, separators=(",", ":"))


def validate_perception(value: object) -> dict[str, Any]:
    item = _object(value, "perception")
    if item.get("schema_version") != PERCEPTION_SCHEMA:
        raise ContractError("unexpected perception schema_version")
    status = _text(item.get("status"), "status")
    if status not in {"SUCCESS", "NO_TARGET", "MULTIPLE_TARGETS", "INVALID", "CALIBRATION_INVALID", "DEPTH_UNAVAILABLE"}:
        raise ContractError("unsupported perception status")
    _text(item.get("measurement_id"), "measurement_id")
    _timestamp(item.get("timestamp"))
    _text(item.get("reference_frame"), "reference_frame")
    if item.get("units") != "m":
        raise ContractError("perception units must be m")
    _finite(item.get("confidence"), "confidence", minimum=0.0)
    if float(item["confidence"]) > 1.0:
        raise ContractError("confidence must be <= 1")
    calibration = _object(item.get("calibration"), "calibration")
    _text(calibration.get("calibration_id"), "calibration.calibration_id")
    _sha256(calibration.get("sha256"), "calibration.sha256")
    if status == "SUCCESS":
        _finite(item.get("range_m"), "range_m", minimum=0.0)
        _finite(item.get("bearing_deg"), "bearing_deg")
    return item


def _validate_frame_size(value: object, name: str, *, width: int, height: int) -> None:
    size = _object(value, name)
    if size.get("width") != width or size.get("height") != height:
        raise ContractError(f"{name} must be {width}x{height}")


def _null_target_fields(item: dict[str, Any]) -> None:
    for field in ("bbox_xyxy_px", "center_x_px", "confidence"):
        if item.get(field) is not None:
            raise ContractError(f"{field} must be null when there is no selected target")
    for field in ("truncated_left", "truncated_right", "truncated_top", "truncated_bottom"):
        if item.get(field) is not None and type(item[field]) is not bool:
            raise ContractError(f"{field} must be bool or null")


def validate_ar0234_observation(value: object) -> dict[str, Any]:
    """Validate an AR0234 YOLO Observation; it is deliberately not a metric Measurement."""
    item = _object(value, "ar0234 observation")
    if item.get("schema_version") != AR0234_OBSERVATION_SCHEMA:
        raise ContractError("unexpected AR0234 observation schema_version")
    _text(item.get("observation_id"), "observation_id")
    _text(item.get("evidence_id"), "evidence_id")
    _text(item.get("source_cycle_id"), "source_cycle_id")
    _timestamp(item.get("captured_at_utc"), "captured_at_utc")
    if item.get("reference_frame") != "ar0234_image_frame":
        raise ContractError("AR0234 observation reference_frame must be ar0234_image_frame")
    if item.get("units") != "px":
        raise ContractError("AR0234 observation units must be px")
    if item.get("entity_type") != "person" or item.get("class_name") != "person_upper_body":
        raise ContractError("unexpected AR0234 observation entity or class")
    _sha256(item.get("model_sha256"), "model_sha256")
    threshold = _finite(item.get("confidence_threshold"), "confidence_threshold", minimum=0.0)
    if threshold > 1.0:
        raise ContractError("confidence_threshold must be <= 1")
    _validate_frame_size(item.get("frame_size_px"), "frame_size_px", width=1920, height=1200)
    _validate_frame_size(item.get("model_input_size_px"), "model_input_size_px", width=640, height=640)
    _integer(item.get("raw_detection_count"), "raw_detection_count")
    _integer(item.get("detection_count"), "detection_count")
    eligible = _integer(item.get("eligible_detection_count"), "eligible_detection_count")
    if type(item.get("detections")) is not list:
        raise ContractError("detections must be a list")
    status = item.get("target_status")
    if status not in {"NO_TARGET", "SINGLE_TARGET", "MULTIPLE_TARGETS"}:
        raise ContractError("unsupported AR0234 target_status")
    if status == "NO_TARGET":
        if eligible != 0:
            raise ContractError("NO_TARGET requires zero eligible detections")
        _null_target_fields(item)
        return item
    if status == "MULTIPLE_TARGETS":
        if eligible < 2:
            raise ContractError("MULTIPLE_TARGETS requires at least two eligible detections")
        _null_target_fields(item)
        return item
    if eligible != 1:
        raise ContractError("SINGLE_TARGET requires exactly one eligible detection")
    box = item.get("bbox_xyxy_px")
    if type(box) is not list or len(box) != 4:
        raise ContractError("bbox_xyxy_px must contain four coordinates")
    x1, y1, x2, y2 = (_finite(v, "bbox_xyxy_px") for v in box)
    if not 0 <= x1 < x2 <= 1920 or not 0 <= y1 < y2 <= 1200:
        raise ContractError("bbox_xyxy_px must lie inside the AR0234 frame")
    center_x = _finite(item.get("center_x_px"), "center_x_px")
    if not 0 <= center_x <= 1920:
        raise ContractError("center_x_px must lie inside the AR0234 frame")
    confidence = _finite(item.get("confidence"), "confidence", minimum=threshold)
    if confidence > 1.0:
        raise ContractError("confidence must be <= 1")
    for field in ("truncated_left", "truncated_right", "truncated_top", "truncated_bottom"):
        if type(item.get(field)) is not bool:
            raise ContractError(f"{field} must be bool for SINGLE_TARGET")
    return item


def navigation_decision(
    measurement: dict[str, Any], *, safe_distance_m: float, bearing_deadband_deg: float,
    min_confidence: float,
) -> dict[str, Any]:
    validate_perception(measurement)
    if safe_distance_m <= 0 or bearing_deadband_deg < 0 or not 0 <= min_confidence <= 1:
        raise ContractError("invalid navigation policy")
    status = measurement["status"]
    action, result, reason = "STOP", "BLOCKED", status
    if status == "SUCCESS" and float(measurement["confidence"]) >= min_confidence:
        bearing = float(measurement["bearing_deg"])
        distance = float(measurement["range_m"])
        if abs(bearing) > bearing_deadband_deg:
            action, result, reason = "TURN_REOBSERVE", "RECOMMENDED", "BEARING_OUTSIDE_DEADBAND"
        elif distance > safe_distance_m:
            action, result, reason = "FORWARD_REOBSERVE", "RECOMMENDED", "OUTSIDE_SAFE_DISTANCE"
        else:
            action, result, reason = "STOP", "ARRIVED", "WITHIN_SAFE_DISTANCE"
    elif status == "SUCCESS":
        reason = "LOW_CONFIDENCE"
    return {
        "schema_version": NAVIGATION_SCHEMA,
        "decision_id": f"navigation:{measurement['measurement_id']}",
        "timestamp": measurement["timestamp"],
        "measurement_id": measurement["measurement_id"],
        "result": result,
        "reason": reason,
        "recommended_action": action,
        "execution_authorized": False,
        "policy": {
            "safe_distance_m": safe_distance_m,
            "bearing_deadband_deg": bearing_deadband_deg,
            "min_confidence": min_confidence,
        },
    }


def validate_navigation(value: object) -> dict[str, Any]:
    item = _object(value, "navigation")
    if item.get("schema_version") != NAVIGATION_SCHEMA:
        raise ContractError("unexpected navigation schema_version")
    _text(item.get("decision_id"), "decision_id")
    _timestamp(item.get("timestamp"))
    _text(item.get("measurement_id"), "measurement_id")
    if item.get("result") not in {"BLOCKED", "RECOMMENDED", "ARRIVED"}:
        raise ContractError("unsupported navigation result")
    if item.get("recommended_action") not in {"STOP", "TURN_REOBSERVE", "FORWARD_REOBSERVE"}:
        raise ContractError("unsupported recommended_action")
    if item.get("execution_authorized") is not False:
        raise ContractError("execution_authorized must remain false in phase 1")
    return item


def supervisor_state(decision: dict[str, Any]) -> dict[str, Any]:
    validate_navigation(decision)
    return {
        "schema_version": SUPERVISOR_SCHEMA,
        "state_id": f"supervisor:{decision['decision_id']}",
        "timestamp": decision["timestamp"],
        "decision_id": decision["decision_id"],
        "result": decision["result"],
        "reason": decision["reason"],
        "recommended_action": decision["recommended_action"],
        "actuator_bridge": "DISABLED_PHASE_1",
        "motor_command_performed": False,
    }
