"""Metric-first, offline-only supervision over one shared live target window."""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from typing import Any, Callable

from vision_core.person_approach.bounded_bridge import MOTION_SPECS
from vision_core.person_approach.decision import PersonApproachDecisionEngine
from vision_core.person_approach.temporal_yaw_alignment import evaluate_temporal_yaw_alignment
from vision_core.tools.prepare_ar0234_yaw_observations import prepare_person_depth_live_cycle_observation


WINDOW_SIZE = 5
PERSON_ENTITY_TYPE = "person"
METRIC_STATUSES = {"ADVANCE", "HOLD_TARGET_REACHED", "TURN_LEFT", "TURN_RIGHT"}


@dataclass(frozen=True)
class MetricFirstTargetResult:
    schema_version: str
    stage: str
    result: str
    reason: str | None
    entity_type: str
    winning_evidence_path: str | None
    source_window_cycle_ids: list[str]
    source_evidence_ids: list[str]
    source_measurement_ids: list[str]
    alignment_reference_frame: str | None
    alignment_units: str | None
    measurement_reference_frame: str | None
    measurement_units: str | None
    alignment_summary: dict[str, Any]
    metric_decision: dict[str, Any] | None
    metric_measurement_provenance: dict[str, Any] | None
    planned_dry_run: dict[str, Any] | None
    reobserve_required: bool
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


def _empty_alignment_summary(cycle_ids: list[str]) -> dict[str, Any]:
    return {
        "temporal_result": None, "temporal_block_reason": None,
        "valid_single_person_count": 0, "person_lost_count": 0,
        "latest_person_status": None, "robust_median_image_offset_px": None,
        "mad_image_offset_px": None, "center_tolerance_px": None,
        "planned_turn_endpoint": None, "source_window_cycle_ids": list(cycle_ids),
    }


def _result(
    *, stage: str, reason: str | None, cycle_ids: list[str], winning_path: str | None = None,
    evidence_ids: list[str] | None = None, measurement_ids: list[str] | None = None,
    alignment_frame: str | None = None, alignment_units: str | None = None,
    measurement_frame: str | None = None, measurement_units: str | None = None,
    alignment_summary: dict[str, Any] | None = None, metric_decision: dict[str, Any] | None = None,
    metric_provenance: dict[str, Any] | None = None, planned_dry_run: dict[str, Any] | None = None,
    reobserve_required: bool = True,
) -> dict[str, Any]:
    return MetricFirstTargetResult(
        schema_version="sie.metric_first_target_supervisor.v1", stage=stage, result=stage,
        reason=reason, entity_type=PERSON_ENTITY_TYPE, winning_evidence_path=winning_path,
        source_window_cycle_ids=cycle_ids, source_evidence_ids=evidence_ids or [],
        source_measurement_ids=measurement_ids or [], alignment_reference_frame=alignment_frame,
        alignment_units=alignment_units, measurement_reference_frame=measurement_frame,
        measurement_units=measurement_units,
        alignment_summary=alignment_summary or _empty_alignment_summary(cycle_ids),
        metric_decision=metric_decision, metric_measurement_provenance=metric_provenance,
        planned_dry_run=planned_dry_run, reobserve_required=reobserve_required,
    ).to_dict()


def _cycle_ids(window: list[dict[str, Any]]) -> list[str] | None:
    values = [_text(cycle.get("cycle_id")) for cycle in window]
    return None if any(value is None for value in values) else [value for value in values if value is not None]


def _alignment_summary(temporal: dict[str, Any], cycle_ids: list[str]) -> dict[str, Any]:
    planned = temporal.get("planned_command")
    return {
        "temporal_result": _text(temporal.get("result")),
        "temporal_block_reason": _text(temporal.get("block_reason")),
        "valid_single_person_count": temporal.get("valid_single_person_count"),
        "person_lost_count": temporal.get("person_lost_count"),
        "latest_person_status": _text(temporal.get("latest_person_status")),
        "robust_median_image_offset_px": temporal.get("robust_median_image_offset_px"),
        "mad_image_offset_px": temporal.get("mad_image_offset_px"),
        "center_tolerance_px": temporal.get("center_tolerance_px"),
        "planned_turn_endpoint": _text(planned.get("endpoint")) if type(planned) is dict else None,
        "source_window_cycle_ids": list(cycle_ids),
    }


def _metric_decision_from_window(
    window: list[dict[str, Any]], *, now_utc: Callable[[], datetime],
    factory: Callable[[], PersonApproachDecisionEngine],
) -> tuple[dict[str, Any], dict[str, Any]] | None:
    """Accept only a fresh, explicit-metric decision proven by this exact window."""
    engine = factory()
    decision: dict[str, Any] | None = None
    for cycle in window:
        decision = engine.ingest(cycle).to_dict()
    assert decision is not None
    if decision.get("status") not in METRIC_STATUSES or decision.get("units") != "m":
        return None

    successes = [cycle for cycle in window if cycle.get("status") == "SUCCESS"]
    if len(successes) < 4 or window[-1].get("status") != "SUCCESS":
        return None
    identifiers: list[str] = []
    frames: set[str] = set()
    timestamps: list[datetime] = []
    for cycle in successes:
        measurement = cycle.get("measurement")
        if type(measurement) is not dict or measurement.get("status") != "SUCCESS":
            return None
        identifier = _text(measurement.get("measurement_id"))
        frame = _text(measurement.get("reference_frame"))
        stamp = _timestamp(measurement.get("timestamp"))
        if identifier is None or frame is None or measurement.get("units") != "m" or stamp is None:
            return None
        if any(_finite(measurement.get(name)) is None for name in ("x_m", "y_m", "z_m", "range_m")):
            return None
        identifiers.append(identifier)
        frames.add(frame)
        timestamps.append(stamp)
    now = now_utc()
    if not isinstance(now, datetime) or now.tzinfo is None or len(frames) != 1:
        return None
    if any(later < earlier for earlier, later in zip(timestamps, timestamps[1:])):
        return None
    age = (now.astimezone(timezone.utc) - timestamps[-1]).total_seconds()
    if not math.isfinite(age) or age < 0 or age > 1.0:
        return None
    frame = next(iter(frames))
    source_ids = decision.get("source_measurement_ids")
    if decision.get("reference_frame") != frame or source_ids != identifiers or len(set(identifiers)) != len(identifiers):
        return None
    return decision, {"reference_frame": frame, "units": "m", "source_measurement_ids": identifiers}


def _metric_plan(decision: dict[str, Any]) -> tuple[str, dict[str, Any] | None, bool] | None:
    status = decision.get("status")
    if status == "HOLD_TARGET_REACHED":
        return "HOLD_TARGET_REACHED", None, False
    if status not in {"ADVANCE", "TURN_LEFT", "TURN_RIGHT"}:
        return None
    endpoint, query_name, field_name, expected_units, lower, upper = MOTION_SPECS[status]
    value = _finite(decision.get(field_name))
    if value is None:
        return None
    if status == "ADVANCE":
        if value < lower or value > upper:
            return None
        return "AWAIT_OPERATOR_ADVANCE_AND_REOBSERVATION", {"method": "POST", "endpoint": endpoint, query_name: value}, True
    if decision.get("turn_angle_units") != expected_units:
        return None
    if (status == "TURN_LEFT" and value >= 0) or (status == "TURN_RIGHT" and value <= 0):
        return None
    angle = abs(value)
    if angle < lower or angle > upper:
        return None
    return "AWAIT_OPERATOR_TURN_AND_REOBSERVATION", {"method": "POST", "endpoint": endpoint, query_name: angle}, True


class MetricFirstTargetSupervisor:
    """Coordinates a person adapter through generic metric-first target stages."""

    def __init__(
        self, *, live_runtime: Any, optical_axis_cx_px: float, center_tolerance_px: float,
        now_utc: Callable[[], datetime] = lambda: datetime.now(timezone.utc),
        decision_engine_factory: Callable[[], PersonApproachDecisionEngine] | None = None,
    ) -> None:
        if _finite(optical_axis_cx_px) is None or _finite(center_tolerance_px) is None or center_tolerance_px < 0:
            raise ValueError("optical axis and center tolerance must be finite; tolerance non-negative")
        self.live_runtime = live_runtime
        self.optical_axis_cx_px = float(optical_axis_cx_px)
        self.center_tolerance_px = float(center_tolerance_px)
        self.now_utc = now_utc
        self.decision_engine_factory = decision_engine_factory or (lambda: PersonApproachDecisionEngine(now_utc=self.now_utc))

    def run_live_window(self) -> dict[str, Any]:
        window = [self.live_runtime.cycle(f"metric-first-target-{index:06d}") for index in range(1, WINDOW_SIZE + 1)]
        return self.process_shared_window(window)

    def process_shared_window(self, cycles: object) -> dict[str, Any]:
        try:
            window = _json_safe(cycles)
        except (TypeError, ValueError):
            return _result(stage="BLOCKED_NO_ACTION", reason="WINDOW_NOT_JSON_SAFE", cycle_ids=[])
        if type(window) is not list or len(window) != WINDOW_SIZE or any(type(cycle) is not dict for cycle in window):
            return _result(stage="BLOCKED_NO_ACTION", reason="WINDOW_MUST_CONTAIN_FIVE_CYCLE_OBJECTS", cycle_ids=[])
        cycle_ids = _cycle_ids(window)
        if cycle_ids is None:
            return _result(stage="BLOCKED_NO_ACTION", reason="MISSING_SOURCE_WINDOW_CYCLE_ID", cycle_ids=[])

        observations = [
            prepare_person_depth_live_cycle_observation(
                cycle, line_number=index, optical_axis_cx_px=self.optical_axis_cx_px,
                center_tolerance_px=self.center_tolerance_px,
            ) for index, cycle in enumerate(window, start=1)
        ]
        temporal = evaluate_temporal_yaw_alignment(observations, now_utc=self.now_utc)
        alignment = _alignment_summary(temporal, cycle_ids)
        evidence_ids = temporal.get("window_evidence_ids")
        if type(evidence_ids) is not list or any(_text(item) is None for item in evidence_ids):
            evidence_ids = []

        metric = _metric_decision_from_window(window, now_utc=self.now_utc, factory=self.decision_engine_factory)
        if metric is not None:
            decision, provenance = metric
            metric_plan = _metric_plan(decision)
            if metric_plan is not None:
                stage, planned, reobserve = metric_plan
                return _result(
                    stage=stage, reason=decision.get("detail"), winning_path="metric_depth",
                    cycle_ids=cycle_ids, evidence_ids=evidence_ids,
                    measurement_ids=provenance["source_measurement_ids"],
                    alignment_frame=_text(temporal.get("reference_frame")), alignment_units=_text(temporal.get("units")),
                    measurement_frame=provenance["reference_frame"], measurement_units=provenance["units"],
                    alignment_summary=alignment, metric_decision=decision, metric_provenance=provenance,
                    planned_dry_run=planned, reobserve_required=reobserve,
                )

        temporal_result = temporal.get("result")
        if temporal_result == "PLANNED_TURN":
            planned = temporal.get("planned_command")
            if type(planned) is dict and planned.get("method") == "POST" and planned.get("endpoint") in {"/turn-left", "/turn-right"} and _finite(planned.get("angle_deg")) == 4.0:
                return _result(
                    stage="AWAIT_OPERATOR_TURN_AND_REOBSERVATION", reason=None,
                    winning_path="image_alignment_fallback", cycle_ids=cycle_ids, evidence_ids=evidence_ids,
                    alignment_frame=_text(temporal.get("reference_frame")), alignment_units=_text(temporal.get("units")),
                    alignment_summary=alignment, planned_dry_run=planned, reobserve_required=True,
                )
        if temporal_result == "NO_TURN_CENTERED":
            return _result(
                stage="RANGE_ACQUISITION_REQUIRED", reason="NO_CURRENT_VALID_METRIC_DEPTH_DECISION",
                winning_path="image_alignment_fallback", cycle_ids=cycle_ids, evidence_ids=evidence_ids,
                alignment_frame=_text(temporal.get("reference_frame")), alignment_units=_text(temporal.get("units")),
                alignment_summary=alignment, reobserve_required=True,
            )
        return _result(
            stage="BLOCKED_NO_ACTION", reason=f"TEMPORAL_{temporal.get('block_reason', 'INVALID')}",
            cycle_ids=cycle_ids, evidence_ids=evidence_ids,
            alignment_frame=_text(temporal.get("reference_frame")), alignment_units=_text(temporal.get("units")),
            alignment_summary=alignment,
        )
