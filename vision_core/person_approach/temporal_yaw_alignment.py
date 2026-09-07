"""Offline temporal 2D yaw-alignment planning in the AR0234 image frame."""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from statistics import median
from typing import Any, Callable


IMAGE_FRAME = "ar0234_image_frame"
IMAGE_UNITS = "px"
TURN_ANGLE_DEG = 4


@dataclass(frozen=True)
class TemporalYawAlignmentResult:
    schema_version: str
    result: str
    stage: str
    timestamp: str
    reference_frame: str | None
    units: str | None
    evidence_ids: list[str]
    valid_single_person_count: int
    person_lost_count: int
    latest_person_status: str | None
    used_evidence_ids: list[str]
    window_evidence_ids: list[str]
    robust_median_image_offset_px: float | None
    mad_image_offset_px: float | None
    center_tolerance_px: float | None
    planned_command: dict[str, Any] | None
    block_reason: str | None
    reobserve_required: bool

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


def _parse_timestamp(value: object) -> str | None:
    if not isinstance(value, str) or not value:
        return None
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None
    return value if parsed.tzinfo is not None else None


class TemporalYawAlignmentPlanner:
    """Plan at most one dry-run turn for a stable evidence window."""

    def __init__(self, *, now_utc: Callable[[], datetime] = lambda: datetime.now(timezone.utc)) -> None:
        self._now_utc = now_utc
        self._planned_window_signature: tuple[str, ...] | None = None

    def _result(
        self,
        *,
        result: str,
        stage: str,
        evidence_ids: list[str],
        offset_median: float | None,
        offset_mad: float | None,
        tolerance: float | None,
        planned_command: dict[str, Any] | None,
        block_reason: str | None,
        reobserve_required: bool,
        reference_frame: str | None = IMAGE_FRAME,
        units: str | None = IMAGE_UNITS,
        valid_single_person_count: int = 0,
        person_lost_count: int = 0,
        latest_person_status: str | None = None,
        used_evidence_ids: list[str] | None = None,
        window_evidence_ids: list[str] | None = None,
    ) -> dict[str, Any]:
        return TemporalYawAlignmentResult(
            schema_version="sie.temporal_yaw_alignment.v1",
            result=result,
            stage=stage,
            timestamp=_timestamp(self._now_utc),
            reference_frame=reference_frame,
            units=units,
            evidence_ids=evidence_ids,
            valid_single_person_count=valid_single_person_count,
            person_lost_count=person_lost_count,
            latest_person_status=latest_person_status,
            used_evidence_ids=evidence_ids if used_evidence_ids is None else used_evidence_ids,
            window_evidence_ids=evidence_ids if window_evidence_ids is None else window_evidence_ids,
            robust_median_image_offset_px=offset_median,
            mad_image_offset_px=offset_mad,
            center_tolerance_px=tolerance,
            planned_command=planned_command,
            block_reason=block_reason,
            reobserve_required=reobserve_required,
        ).to_dict()

    def evaluate_window(self, observations: object) -> dict[str, Any]:
        """Evaluate up to five JSON-safe observations without transport or actuation."""
        try:
            window = _json_safe(observations)
        except (TypeError, ValueError):
            return self._result(
                result="BLOCKED_NO_TURN", stage="BLOCKED_NO_TURN", evidence_ids=[],
                offset_median=None, offset_mad=None, tolerance=None, planned_command=None,
                block_reason="INPUT_NOT_JSON_SAFE", reobserve_required=True,
                reference_frame=None, units=None,
            )
        if type(window) is not list or not window or len(window) > 5:
            return self._result(
                result="BLOCKED_NO_TURN", stage="BLOCKED_NO_TURN", evidence_ids=[],
                offset_median=None, offset_mad=None, tolerance=None, planned_command=None,
                block_reason="WINDOW_MUST_CONTAIN_1_TO_5_OBSERVATIONS", reobserve_required=True,
                reference_frame=None, units=None,
            )

        window_evidence_ids: list[str] = []
        used_evidence_ids: list[str] = []
        offsets: list[float] = []
        tolerance: float | None = None
        previous_timestamp: datetime | None = None
        person_lost_count = 0
        latest_person_status: str | None = None

        def blocked(
            reason: str,
            *,
            reference_frame: str | None = IMAGE_FRAME,
            units: str | None = IMAGE_UNITS,
            offset_median: float | None = None,
            offset_mad: float | None = None,
        ) -> dict[str, Any]:
            return self._result(
                result="BLOCKED_NO_TURN", stage="BLOCKED_NO_TURN", evidence_ids=used_evidence_ids,
                offset_median=offset_median, offset_mad=offset_mad, tolerance=tolerance, planned_command=None,
                block_reason=reason, reobserve_required=True, reference_frame=reference_frame, units=units,
                valid_single_person_count=len(used_evidence_ids), person_lost_count=person_lost_count,
                latest_person_status=latest_person_status, used_evidence_ids=used_evidence_ids,
                window_evidence_ids=window_evidence_ids,
            )

        for observation in window:
            if type(observation) is not dict:
                return blocked("OBSERVATION_MUST_BE_OBJECT", reference_frame=None, units=None)
            evidence_id = _text(observation.get("evidence_id"))
            timestamp = _parse_timestamp(observation.get("timestamp"))
            if evidence_id is None or timestamp is None:
                return blocked("MISSING_EVIDENCE_ID_OR_TIMESTAMP", reference_frame=None, units=None)
            window_evidence_ids.append(evidence_id)
            latest_person_status = _text(observation.get("person_status", observation.get("status")))
            parsed_timestamp = datetime.fromisoformat(timestamp.replace("Z", "+00:00"))
            if previous_timestamp is not None and parsed_timestamp <= previous_timestamp:
                return blocked("TIMESTAMPS_MUST_BE_STRICTLY_INCREASING", reference_frame=None, units=None)
            previous_timestamp = parsed_timestamp
            if observation.get("reference_frame") != IMAGE_FRAME or observation.get("units") != IMAGE_UNITS:
                return blocked(
                    "REFERENCE_FRAME_OR_UNITS_MISMATCH",
                    reference_frame=_text(observation.get("reference_frame")), units=_text(observation.get("units")),
                )
            if latest_person_status == "MULTIPLE_PERSONS":
                return blocked("MULTIPLE_PERSONS_IN_EVIDENCE_WINDOW")
            if latest_person_status == "PERSON_LOST":
                person_lost_count += 1
                if person_lost_count > 1:
                    return blocked("MORE_THAN_ONE_PERSON_LOST_IN_EVIDENCE_WINDOW")
                continue
            if latest_person_status != "SINGLE_PERSON":
                return blocked("PERSON_STATUS_NOT_SINGLE_PERSON")
            observation_tolerance = _finite(observation.get("center_tolerance_px"))
            if observation_tolerance is None or observation_tolerance < 0:
                return blocked("CENTER_TOLERANCE_MUST_BE_EXPLICIT_FINITE_NON_NEGATIVE")
            if tolerance is not None and observation_tolerance != tolerance:
                return blocked("CENTER_TOLERANCE_MISMATCH")
            tolerance = observation_tolerance
            offset = _finite(observation.get("image_offset_px"))
            if offset is None:
                center_x = _finite(observation.get("bbox_center_x_px"))
                optical_axis = _finite(observation.get("optical_axis_cx_px"))
                bbox = observation.get("bbox_xyxy_px")
                if center_x is None and type(bbox) is list and len(bbox) == 4:
                    bbox_values = [_finite(value) for value in bbox]
                    if all(value is not None for value in bbox_values):
                        x1, _y1, x2, _y2 = bbox_values
                        center_x = (x1 + x2) / 2.0
                if center_x is None or optical_axis is None:
                    return blocked("MISSING_FINITE_OFFSET_OR_CENTER_AXIS")
                offset = center_x - optical_axis
            used_evidence_ids.append(evidence_id)
            offsets.append(offset)

        if latest_person_status != "SINGLE_PERSON":
            return blocked("LATEST_PERSON_STATUS_NOT_SINGLE_PERSON")
        if len(offsets) < 4:
            return blocked("FEWER_THAN_FOUR_VALID_SINGLE_PERSON_OBSERVATIONS")

        offset_median = float(median(offsets))
        offset_mad = float(median([abs(value - offset_median) for value in offsets]))
        assert tolerance is not None
        classifications = [
            "CENTER" if abs(value) <= tolerance else "IMAGE_LEFT" if value < 0 else "IMAGE_RIGHT"
            for value in offsets
        ]
        if offset_mad > tolerance or len(set(classifications)) != 1:
            return blocked("UNSTABLE_EVIDENCE_WINDOW", offset_median=offset_median, offset_mad=offset_mad)

        signature = tuple(window_evidence_ids)
        if self._planned_window_signature == signature:
            return self._result(
                result="BLOCKED_NO_TURN", stage="AWAIT_REOBSERVATION", evidence_ids=used_evidence_ids,
                offset_median=offset_median, offset_mad=offset_mad, tolerance=tolerance, planned_command=None,
                block_reason="EVIDENCE_WINDOW_ALREADY_PLANNED", reobserve_required=True,
                valid_single_person_count=len(used_evidence_ids), person_lost_count=person_lost_count,
                latest_person_status=latest_person_status, used_evidence_ids=used_evidence_ids,
                window_evidence_ids=window_evidence_ids,
            )
        if classifications[0] == "CENTER":
            return self._result(
                result="NO_TURN_CENTERED", stage="NO_TURN_CENTERED", evidence_ids=used_evidence_ids,
                offset_median=offset_median, offset_mad=offset_mad, tolerance=tolerance, planned_command=None,
                block_reason=None, reobserve_required=False,
                valid_single_person_count=len(used_evidence_ids), person_lost_count=person_lost_count,
                latest_person_status=latest_person_status, used_evidence_ids=used_evidence_ids,
                window_evidence_ids=window_evidence_ids,
            )

        endpoint = "/turn-right" if classifications[0] == "IMAGE_RIGHT" else "/turn-left"
        planned_command = {"method": "POST", "endpoint": endpoint, "angle_deg": TURN_ANGLE_DEG}
        self._planned_window_signature = signature
        return self._result(
            result="PLANNED_TURN", stage="AWAIT_REOBSERVATION", evidence_ids=used_evidence_ids,
            offset_median=offset_median, offset_mad=offset_mad, tolerance=tolerance,
            planned_command=planned_command, block_reason=None, reobserve_required=True,
            valid_single_person_count=len(used_evidence_ids), person_lost_count=person_lost_count,
            latest_person_status=latest_person_status, used_evidence_ids=used_evidence_ids,
            window_evidence_ids=window_evidence_ids,
        )


def evaluate_temporal_yaw_alignment(
    observations: object,
    *,
    now_utc: Callable[[], datetime] = lambda: datetime.now(timezone.utc),
) -> dict[str, Any]:
    """Evaluate one offline evidence window with a fresh planner instance."""
    return TemporalYawAlignmentPlanner(now_utc=now_utc).evaluate_window(observations)
