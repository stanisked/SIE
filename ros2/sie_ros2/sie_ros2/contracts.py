"""Strict JSON contracts for the first safe SIE ROS 2 pipeline."""

from __future__ import annotations

import json
import math
from datetime import datetime, timezone
from typing import Any

PERCEPTION_SCHEMA = "sie.perception.measurement.v1"
AR0234_OBSERVATION_SCHEMA = "sie.ar0234.yolo11_person_upper_body_observation.v1"
AR0234_TARGET_SUITABILITY_SCHEMA = "sie.ar0234.target_suitability.v1"
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


def ar0234_target_suitability(observation: object) -> dict[str, Any]:
    """Interpret target geometry without promoting it to a metric Measurement."""
    item = validate_ar0234_observation(observation)
    status = item["target_status"]
    disposition = "REJECTED"
    reason = status
    geometry_eligible = False
    if status == "SINGLE_TARGET":
        truncated = [
            edge
            for edge in ("left", "right", "top", "bottom")
            if item[f"truncated_{edge}"]
        ]
        if truncated:
            reason = "EDGE_TRUNCATED:" + ",".join(truncated)
        else:
            disposition = "ACCEPTED_FOR_FURTHER_INTERPRETATION"
            reason = "COMPLETE_SINGLE_TARGET"
            geometry_eligible = True
    return {
        "schema_version": AR0234_TARGET_SUITABILITY_SCHEMA,
        "interpretation_id": f"ar0234-target-suitability:{item['observation_id']}",
        "timestamp": item["captured_at_utc"],
        "observation_id": item["observation_id"],
        "evidence_id": item["evidence_id"],
        "disposition": disposition,
        "reason": reason,
        "geometry_eligible": geometry_eligible,
        "metric_measurement_authorized": False,
    }


def _ar0234_non_edge_candidates(
    item: dict[str, Any],
) -> tuple[float, list[tuple[int, dict[str, Any]]]]:
    """Return current-frame candidates that can support a dynamic track.

    This deliberately does not choose a target.  The caller must make the
    ambiguity decision explicitly; a past detection is never returned here.
    """
    threshold = float(item["confidence_threshold"])
    frame = item["frame_size_px"]
    model_input = item["model_input_size_px"]
    model_to_source_scale = min(
        float(model_input["width"]) / float(frame["width"]),
        float(model_input["height"]) / float(frame["height"]),
    )
    if model_to_source_scale <= 0.0:
        raise ContractError("invalid model-to-source scale")
    # Decoder coordinates can finish fractionally inside an image edge even
    # when the source candidate reached the border. One model-grid pixel,
    # mapped back through the documented letterbox scale, is the tolerance.
    edge_margin_px = 1.0 / model_to_source_scale
    candidates: list[tuple[int, dict[str, Any]]] = []
    for index, candidate in enumerate(item["detections"]):
        if type(candidate) is not dict:
            continue
        box = candidate.get("bbox_xyxy_px")
        if type(box) is not list or len(box) != 4:
            continue
        try:
            x1, y1, x2, y2 = (float(value) for value in box)
            center_x = float(candidate["center_x_px"])
            confidence = float(candidate["confidence"])
        except (KeyError, TypeError, ValueError):
            continue
        if (
            not all(math.isfinite(value) for value in (x1, y1, x2, y2, center_x, confidence))
            or not 0.0 <= x1 < x2 <= 1920.0
            or not 0.0 <= y1 < y2 <= 1200.0
            or not math.isclose(center_x, (x1 + x2) / 2.0, abs_tol=1e-6)
            or not threshold <= confidence <= 1.0
            or any(type(candidate.get(f"truncated_{edge}")) is not bool for edge in ("top", "left", "right", "bottom"))
            or candidate["truncated_top"]
            or candidate["truncated_left"]
            or candidate["truncated_right"]
            or x1 <= edge_margin_px
            or x2 >= float(frame["width"]) - edge_margin_px
            or y1 <= edge_margin_px
        ):
            continue
        candidates.append((index, candidate))
    return edge_margin_px, candidates


def ar0234_unique_non_edge_candidate_selection(observation: object) -> dict[str, Any]:
    """Select exactly one non-edge candidate from raw ambiguous AR evidence.

    This is an interpretation of an existing visual Observation, not a
    Measurement and never an execution authorization. A candidate may reach
    the bottom edge so the separately activated static upper-body policy can
    decide that case later. Top, left, and right truncation are always
    rejected here because they do not provide a stable approach target.
    """
    item = validate_ar0234_observation(observation)
    status = item["target_status"]
    base = {
        "schema_version": "sie.ar0234.unique_non_edge_target_selection.v1",
        "selection_id": f"ar0234-target-selection:{item['observation_id']}",
        "timestamp": item["captured_at_utc"],
        "observation_id": item["observation_id"],
        "evidence_id": item["evidence_id"],
        "source_target_status": status,
        "original_eligible_detection_count": item["eligible_detection_count"],
        "metric_measurement_authorized": False,
    }
    if status != "MULTIPLE_TARGETS":
        return {
            **base,
            "disposition": "NOT_APPLICABLE",
            "reason": "SOURCE_NOT_MULTIPLE_TARGETS",
            "selected_candidate_index": None,
            "selected_detection": None,
        }

    edge_margin_px, candidates = _ar0234_non_edge_candidates(item)

    if len(candidates) != 1:
        return {
            **base,
            "disposition": "REJECTED",
            "reason": (
                "NO_NON_EDGE_CANDIDATE"
                if not candidates
                else "MULTIPLE_NON_EDGE_CANDIDATES"
            ),
            "edge_margin_px": edge_margin_px,
            "non_edge_candidate_count": len(candidates),
            "selected_candidate_index": None,
            "selected_detection": None,
        }

    index, candidate = candidates[0]
    return {
        **base,
        "disposition": "SELECTED_FOR_FURTHER_INTERPRETATION",
        "reason": "UNIQUE_NON_EDGE_CANDIDATE",
        "edge_margin_px": edge_margin_px,
        "non_edge_candidate_count": 1,
        "selected_candidate_index": index,
        "selected_detection": candidate,
    }


def ar0234_dynamic_target_candidate(observation: object) -> dict[str, Any]:
    """Publish current-frame dynamic target evidence without a static-scene gate.

    The result is a 2D target interpretation only. It never creates a metric
    Measurement or execution authorization. Bottom truncation is accepted for
    upper-body tracking; top, left, and right edges remain ineligible because
    the current image cannot locate that target completely in those axes.
    """
    item = validate_ar0234_observation(observation)
    status = item["target_status"]
    selection: dict[str, Any] | None = None
    candidate: dict[str, Any] | None = None
    reason = status
    if status == "SINGLE_TARGET":
        candidate = {
            "bbox_xyxy_px": list(item["bbox_xyxy_px"]),
            "center_x_px": item["center_x_px"],
            "confidence": item["confidence"],
            "truncated_top": item["truncated_top"],
            "truncated_left": item["truncated_left"],
            "truncated_right": item["truncated_right"],
            "truncated_bottom": item["truncated_bottom"],
        }
        reason = "DIRECT_SINGLE_TARGET"
    elif status == "MULTIPLE_TARGETS":
        selection = ar0234_unique_non_edge_candidate_selection(item)
        if selection["disposition"] == "SELECTED_FOR_FURTHER_INTERPRETATION":
            candidate = selection["selected_detection"]
            reason = "UNIQUE_NON_EDGE_CANDIDATE"

    base = {
        "schema_version": "sie.ar0234.dynamic_target_candidate.v1",
        "interpretation_id": f"ar0234-dynamic-target:{item['observation_id']}",
        "timestamp": item["captured_at_utc"],
        "observation_id": item["observation_id"],
        "evidence_id": item["evidence_id"],
        "metric_measurement_authorized": False,
        "source_target_status": status,
        "selection": selection,
    }
    if type(candidate) is not dict:
        return {
            **base,
            "disposition": "UNAVAILABLE",
            "reason": (
                str(selection["reason"])
                if selection is not None
                else reason
            ),
            "candidate": None,
        }
    if (
        candidate.get("truncated_top") is True
        or candidate.get("truncated_left") is True
        or candidate.get("truncated_right") is True
    ):
        return {
            **base,
            "disposition": "UNAVAILABLE",
            "reason": "EDGE_TRUNCATED_FOR_DYNAMIC_TRACK",
            "candidate": candidate,
        }
    return {
        **base,
        "disposition": "OBSERVED",
        "reason": reason,
        "candidate": candidate,
    }


def _bbox_iou(first: list[float], second: list[float]) -> float:
    left = max(first[0], second[0])
    top = max(first[1], second[1])
    right = min(first[2], second[2])
    bottom = min(first[3], second[3])
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    first_area = (first[2] - first[0]) * (first[3] - first[1])
    second_area = (second[2] - second[0]) * (second[3] - second[1])
    union = first_area + second_area - intersection
    return intersection / union if union > 0.0 else 0.0


def _bbox_center(box: list[float]) -> tuple[float, float]:
    return ((box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0)


def _timestamp_datetime(value: object, field_name: str) -> datetime:
    text = _timestamp(value, field_name)
    return datetime.fromisoformat(text.replace("Z", "+00:00"))


def ar0234_temporal_target_candidate(
    observation: object,
    *,
    prior_track: dict[str, Any] | None,
    max_age_s: float,
    minimum_iou: float,
    maximum_center_displacement_relative: float,
    minimum_match_margin: float,
) -> dict[str, Any]:
    """Choose a *current-frame* AR target, using a confirmed track only for ties.

    A track is evidence about identity, not a cache of image content.  Therefore
    this function only returns a detection present in ``observation``.  It is
    intentionally diagnostic-level evidence and never authorizes metric
    measurement, navigation, or an actuator.
    """
    if (
        max_age_s <= 0.0
        or not 0.0 <= minimum_iou <= 1.0
        or maximum_center_displacement_relative <= 0.0
        or minimum_match_margin < 0.0
    ):
        raise ContractError("invalid temporal AR target association policy")
    item = validate_ar0234_observation(observation)
    legacy = ar0234_dynamic_target_candidate(item)
    base = {
        "schema_version": "sie.ar0234.temporal_target_candidate.v1",
        "interpretation_id": f"ar0234-temporal-target:{item['observation_id']}",
        "timestamp": item["captured_at_utc"],
        "observation_id": item["observation_id"],
        "evidence_id": item["evidence_id"],
        "metric_measurement_authorized": False,
        "source_target_status": item["target_status"],
        "selection": legacy["selection"],
    }
    if legacy["disposition"] == "OBSERVED":
        return {
            **base,
            "disposition": "OBSERVED",
            "reason": (
                "DIRECT_CURRENT_FRAME_SINGLE_TARGET"
                if legacy["reason"] == "DIRECT_SINGLE_TARGET"
                else "DIRECT_CURRENT_FRAME_UNIQUE_NON_EDGE"
            ),
            "candidate": legacy["candidate"],
            "temporal_track": {
                "mode": "DIRECT_CURRENT_FRAME",
                "prior_track_available": prior_track is not None,
            },
        }
    if item["target_status"] != "MULTIPLE_TARGETS":
        return {
            **base,
            "disposition": "UNAVAILABLE",
            "reason": legacy["reason"],
            "candidate": None,
            "temporal_track": {"mode": "NOT_APPLICABLE", "prior_track_available": prior_track is not None},
        }

    edge_margin_px, candidates = _ar0234_non_edge_candidates(item)
    if len(candidates) < 2:
        # Preserve the existing fail-closed reason if parsing made this case
        # unavailable after the ordinary dynamic-candidate pass.
        return {
            **base,
            "disposition": "UNAVAILABLE",
            "reason": legacy["reason"],
            "candidate": None,
            "temporal_track": {
                "mode": "NOT_APPLICABLE",
                "prior_track_available": prior_track is not None,
                "edge_margin_px": edge_margin_px,
                "current_non_edge_candidate_count": len(candidates),
            },
        }
    if type(prior_track) is not dict:
        return {
            **base,
            "disposition": "UNAVAILABLE",
            "reason": "TEMPORAL_TRACK_UNAVAILABLE",
            "candidate": None,
            "temporal_track": {
                "mode": "CURRENT_FRAME_MATCH_REQUIRED",
                "prior_track_available": False,
                "edge_margin_px": edge_margin_px,
                "current_non_edge_candidate_count": len(candidates),
            },
        }

    try:
        track_id = _text(prior_track["track_id"], "prior_track.track_id")
        confirmed_at = _timestamp_datetime(
            prior_track["confirmed_at_utc"], "prior_track.confirmed_at_utc"
        )
        previous_box = [
            _finite(value, "prior_track.bbox_xyxy_px")
            for value in prior_track["bbox_xyxy_px"]
        ]
        if len(previous_box) != 4 or previous_box[2] <= previous_box[0] or previous_box[3] <= previous_box[1]:
            raise ContractError("prior_track.bbox_xyxy_px is invalid")
        current_at = _timestamp_datetime(item["captured_at_utc"], "captured_at_utc")
    except (KeyError, TypeError, ContractError):
        return {
            **base,
            "disposition": "UNAVAILABLE",
            "reason": "TEMPORAL_TRACK_INVALID",
            "candidate": None,
            "temporal_track": {
                "mode": "CURRENT_FRAME_MATCH_REQUIRED",
                "prior_track_available": True,
                "edge_margin_px": edge_margin_px,
                "current_non_edge_candidate_count": len(candidates),
            },
        }
    age_s = (current_at - confirmed_at).total_seconds()
    if age_s < 0.0 or age_s > max_age_s:
        return {
            **base,
            "disposition": "UNAVAILABLE",
            "reason": "TEMPORAL_TRACK_STALE",
            "candidate": None,
            "temporal_track": {
                "mode": "CURRENT_FRAME_MATCH_REQUIRED",
                "prior_track_available": True,
                "track_id": track_id,
                "age_s": age_s,
                "max_age_s": max_age_s,
                "edge_margin_px": edge_margin_px,
                "current_non_edge_candidate_count": len(candidates),
            },
        }

    predicted_box = list(previous_box)
    prediction_mode = "LAST_CONFIRMED_BBOX"
    try:
        older_box = [
            _finite(value, "prior_track.previous_bbox_xyxy_px")
            for value in prior_track["previous_bbox_xyxy_px"]
        ]
        older_at = _timestamp_datetime(
            prior_track["previous_confirmed_at_utc"],
            "prior_track.previous_confirmed_at_utc",
        )
        interval_s = (confirmed_at - older_at).total_seconds()
        if len(older_box) == 4 and interval_s > 0.0 and all(math.isfinite(value) for value in older_box):
            predicted_box = [
                value + (value - older) * age_s / interval_s
                for value, older in zip(previous_box, older_box)
            ]
            prediction_mode = "CONSTANT_VELOCITY_BBOX"
    except (KeyError, TypeError, ContractError):
        pass
    if predicted_box[2] <= predicted_box[0] or predicted_box[3] <= predicted_box[1]:
        return {
            **base,
            "disposition": "UNAVAILABLE",
            "reason": "TEMPORAL_TRACK_INVALID_PREDICTION",
            "candidate": None,
            "temporal_track": {
                "mode": "CURRENT_FRAME_MATCH_REQUIRED",
                "prior_track_available": True,
                "track_id": track_id,
                "age_s": age_s,
            },
        }
    predicted_center = _bbox_center(predicted_box)
    predicted_diagonal = math.hypot(
        predicted_box[2] - predicted_box[0], predicted_box[3] - predicted_box[1]
    )
    matches: list[tuple[float, int, dict[str, Any], dict[str, float]]] = []
    diagnostic_matches: list[dict[str, Any]] = []
    for index, candidate in candidates:
        box = [float(value) for value in candidate["bbox_xyxy_px"]]
        center = _bbox_center(box)
        center_delta = math.hypot(center[0] - predicted_center[0], center[1] - predicted_center[1])
        normalized_center_delta = center_delta / predicted_diagonal
        iou = _bbox_iou(predicted_box, box)
        eligible = (
            iou >= minimum_iou
            and normalized_center_delta <= maximum_center_displacement_relative
        )
        score = iou + 0.25 * max(
            0.0, 1.0 - normalized_center_delta / maximum_center_displacement_relative
        )
        diagnostic = {
            "candidate_index": index,
            "iou_to_predicted_bbox": iou,
            "center_delta_px": center_delta,
            "normalized_center_delta": normalized_center_delta,
            "match_score": score,
            "eligible": eligible,
        }
        diagnostic_matches.append(diagnostic)
        if eligible:
            matches.append((score, index, candidate, diagnostic))
    matches.sort(key=lambda value: value[0], reverse=True)
    margin = (
        matches[0][0] - matches[1][0]
        if len(matches) >= 2
        else None
    )
    selected = (
        matches[0]
        if len(matches) == 1 or (margin is not None and margin >= minimum_match_margin)
        else None
    )
    temporal_track = {
        "mode": "CURRENT_FRAME_MATCH_REQUIRED",
        "prior_track_available": True,
        "track_id": track_id,
        "age_s": age_s,
        "max_age_s": max_age_s,
        "prediction_mode": prediction_mode,
        "predicted_bbox_xyxy_px": predicted_box,
        "edge_margin_px": edge_margin_px,
        "current_non_edge_candidate_count": len(candidates),
        "minimum_iou": minimum_iou,
        "maximum_center_displacement_relative": maximum_center_displacement_relative,
        "minimum_match_margin": minimum_match_margin,
        "eligible_current_match_count": len(matches),
        "current_candidate_matches": diagnostic_matches,
        "best_to_second_match_margin": margin,
    }
    if selected is None:
        return {
            **base,
            "disposition": "UNAVAILABLE",
            "reason": "TEMPORAL_CURRENT_FRAME_MATCH_AMBIGUOUS_OR_ABSENT",
            "candidate": None,
            "temporal_track": temporal_track,
        }
    score, index, candidate, _ = selected
    return {
        **base,
        "disposition": "OBSERVED",
        "reason": "TEMPORAL_CURRENT_FRAME_MATCH",
        "candidate": candidate,
        "temporal_track": {
            **temporal_track,
            "mode": "TEMPORAL_CURRENT_FRAME_MATCH",
            "selected_candidate_index": index,
            "selected_match_score": score,
        },
    }


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
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "measurement_timestamp": measurement["timestamp"],
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
    _timestamp(item.get("measurement_timestamp"), "measurement_timestamp")
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
