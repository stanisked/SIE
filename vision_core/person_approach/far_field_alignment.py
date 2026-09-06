"""Offline 2D far-field alignment policy in the AR0234 image frame."""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
from typing import Any, Callable


IMAGE_FRAME = "ar0234_image_frame"
IMAGE_UNITS = "px"


@dataclass(frozen=True)
class FarFieldAlignmentResult:
    schema_version: str
    result: str
    semantic_decision: str
    timestamp: str
    evidence_id: str | None
    image_width_px: float | None
    optical_axis_cx_px: float | None
    center_tolerance_px: float | None
    person_center_x_px: float | None
    image_offset_px: float | None
    reference_frame: str | None
    units: str | None
    block_reason: str | None

    def to_dict(self) -> dict[str, Any]:
        return _json_safe(asdict(self))


def _json_safe(value: Any) -> Any:
    return json.loads(json.dumps(value, allow_nan=False, sort_keys=True))


def _finite(value: object) -> float | None:
    if type(value) not in (int, float) or not math.isfinite(value):
        return None
    return float(value)


def _text(value: object) -> str | None:
    return value if type(value) is str and value else None


def _timestamp(now_utc: Callable[[], datetime]) -> str:
    value = now_utc()
    if not isinstance(value, datetime) or value.tzinfo is None:
        raise ValueError("now_utc must return a timezone-aware datetime")
    return value.astimezone(timezone.utc).isoformat()


def _result(
    *,
    now_utc: Callable[[], datetime],
    semantic_decision: str,
    block_reason: str | None,
    evidence_id: str | None,
    image_width_px: float | None,
    optical_axis_cx_px: float | None,
    center_tolerance_px: float | None,
    person_center_x_px: float | None,
    image_offset_px: float | None,
    reference_frame: str | None,
    units: str | None,
) -> dict[str, Any]:
    return FarFieldAlignmentResult(
        schema_version="sie.far_field_alignment.v1",
        result="READY_ALIGNMENT_DECISION" if block_reason is None else "BLOCKED_NO_ALIGNMENT",
        semantic_decision=semantic_decision,
        timestamp=_timestamp(now_utc),
        evidence_id=evidence_id,
        image_width_px=image_width_px,
        optical_axis_cx_px=optical_axis_cx_px,
        center_tolerance_px=center_tolerance_px,
        person_center_x_px=person_center_x_px,
        image_offset_px=image_offset_px,
        reference_frame=reference_frame,
        units=units,
        block_reason=block_reason,
    ).to_dict()


def evaluate_far_field_alignment(
    envelope: object,
    *,
    now_utc: Callable[[], datetime] = lambda: datetime.now(timezone.utc),
) -> dict[str, Any]:
    """Evaluate one JSON-safe 2D person envelope without actuation or transport."""
    try:
        value = _json_safe(envelope)
    except (TypeError, ValueError):
        return _result(
            now_utc=now_utc, semantic_decision="BLOCKED_INVALID_INPUT", block_reason="INPUT_NOT_JSON_SAFE",
            evidence_id=None, image_width_px=None, optical_axis_cx_px=None, center_tolerance_px=None,
            person_center_x_px=None, image_offset_px=None, reference_frame=None, units=None,
        )
    if type(value) is not dict:
        return _result(
            now_utc=now_utc, semantic_decision="BLOCKED_INVALID_INPUT", block_reason="INPUT_MUST_BE_OBJECT",
            evidence_id=None, image_width_px=None, optical_axis_cx_px=None, center_tolerance_px=None,
            person_center_x_px=None, image_offset_px=None, reference_frame=None, units=None,
        )

    width = _finite(value.get("image_width_px"))
    axis = _finite(value.get("optical_axis_cx_px"))
    tolerance = _finite(value.get("center_tolerance_px"))
    common = {
        "image_width_px": width, "optical_axis_cx_px": axis,
        "center_tolerance_px": tolerance,
    }
    if width is None or width <= 0 or axis is None or not 0 <= axis <= width:
        return _result(now_utc=now_utc, semantic_decision="BLOCKED_INVALID_INPUT", block_reason="INVALID_IMAGE_AXIS_GEOMETRY", evidence_id=None, person_center_x_px=None, image_offset_px=None, reference_frame=None, units=None, **common)
    if tolerance is None or tolerance < 0:
        return _result(now_utc=now_utc, semantic_decision="BLOCKED_INVALID_INPUT", block_reason="CENTER_TOLERANCE_MUST_BE_EXPLICIT_FINITE_NON_NEGATIVE", evidence_id=None, person_center_x_px=None, image_offset_px=None, reference_frame=None, units=None, **common)
    evidence = value.get("person_evidence")
    if type(evidence) is not list:
        return _result(now_utc=now_utc, semantic_decision="BLOCKED_INVALID_INPUT", block_reason="PERSON_EVIDENCE_MUST_BE_LIST", evidence_id=None, person_center_x_px=None, image_offset_px=None, reference_frame=None, units=None, **common)
    if len(evidence) == 0:
        return _result(now_utc=now_utc, semantic_decision="BLOCKED_NO_PERSON", block_reason="NO_PERSON_EVIDENCE", evidence_id=None, person_center_x_px=None, image_offset_px=None, reference_frame=IMAGE_FRAME, units=IMAGE_UNITS, **common)
    if len(evidence) != 1:
        return _result(now_utc=now_utc, semantic_decision="BLOCKED_MULTIPLE_PERSONS", block_reason="EXPECTED_EXACTLY_ONE_PERSON", evidence_id=None, person_center_x_px=None, image_offset_px=None, reference_frame=IMAGE_FRAME, units=IMAGE_UNITS, **common)

    person = evidence[0]
    if type(person) is not dict:
        return _result(now_utc=now_utc, semantic_decision="BLOCKED_INVALID_PERSON_EVIDENCE", block_reason="PERSON_EVIDENCE_MUST_BE_OBJECT", evidence_id=None, person_center_x_px=None, image_offset_px=None, reference_frame=None, units=None, **common)
    evidence_id = _text(person.get("evidence_id"))
    frame = _text(person.get("reference_frame"))
    units = _text(person.get("units"))
    if frame != IMAGE_FRAME or units != IMAGE_UNITS:
        return _result(now_utc=now_utc, semantic_decision="BLOCKED_INVALID_PERSON_EVIDENCE", block_reason="REFERENCE_FRAME_OR_UNITS_MISMATCH", evidence_id=evidence_id, person_center_x_px=None, image_offset_px=None, reference_frame=frame, units=units, **common)
    if person.get("status") not in ("SINGLE_PERSON", "SUCCESS"):
        return _result(now_utc=now_utc, semantic_decision="BLOCKED_INVALID_PERSON_EVIDENCE", block_reason="PERSON_EVIDENCE_STATUS_NOT_VALID", evidence_id=evidence_id, person_center_x_px=None, image_offset_px=None, reference_frame=frame, units=units, **common)
    if evidence_id is None:
        return _result(now_utc=now_utc, semantic_decision="BLOCKED_INVALID_PERSON_EVIDENCE", block_reason="MISSING_EVIDENCE_ID", evidence_id=None, person_center_x_px=None, image_offset_px=None, reference_frame=frame, units=units, **common)

    bbox = person.get("bbox_xyxy_px")
    if type(bbox) is not list or len(bbox) != 4:
        return _result(now_utc=now_utc, semantic_decision="BLOCKED_INVALID_PERSON_EVIDENCE", block_reason="INVALID_BBOX", evidence_id=evidence_id, person_center_x_px=None, image_offset_px=None, reference_frame=frame, units=units, **common)
    coords = [_finite(item) for item in bbox]
    if any(item is None for item in coords):
        return _result(now_utc=now_utc, semantic_decision="BLOCKED_INVALID_PERSON_EVIDENCE", block_reason="NON_FINITE_BBOX", evidence_id=evidence_id, person_center_x_px=None, image_offset_px=None, reference_frame=frame, units=units, **common)
    x1, y1, x2, y2 = coords
    if x1 < 0 or x2 > width or x2 <= x1 or y2 <= y1:
        return _result(now_utc=now_utc, semantic_decision="BLOCKED_INVALID_PERSON_EVIDENCE", block_reason="INVALID_BBOX", evidence_id=evidence_id, person_center_x_px=None, image_offset_px=None, reference_frame=frame, units=units, **common)
    center_x = (x1 + x2) / 2.0
    offset = center_x - axis
    if abs(offset) <= tolerance:
        decision = "READY_FOR_RANGE_ACQUISITION"
    elif offset < 0:
        decision = "ALIGN_TOWARD_IMAGE_LEFT"
    else:
        decision = "ALIGN_TOWARD_IMAGE_RIGHT"
    return _result(now_utc=now_utc, semantic_decision=decision, block_reason=None, evidence_id=evidence_id, person_center_x_px=center_x, image_offset_px=offset, reference_frame=frame, units=units, **common)
