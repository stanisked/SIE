"""Generic supervised target-stage orchestration over one shared live window."""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from typing import Any, Callable

from vision_core.person_approach.decision import PersonApproachDecisionEngine
from vision_core.person_approach.supervised_acquire_range import coordinate_supervised_acquire_range
from vision_core.person_approach.temporal_yaw_alignment import evaluate_temporal_yaw_alignment
from vision_core.tools.prepare_ar0234_yaw_observations import prepare_person_depth_live_cycle_observation


WINDOW_SIZE = 5
PERSON_ENTITY_TYPE = "person"


@dataclass(frozen=True)
class SupervisedTargetResult:
    schema_version: str
    stage: str
    result: str
    reason: str | None
    entity_type: str
    source_window_cycle_ids: list[str]
    source_evidence_ids: list[str]
    source_measurement_ids: list[str]
    alignment_reference_frame: str | None
    alignment_units: str | None
    measurement_reference_frame: str | None
    measurement_units: str | None
    temporal_result: str | None
    decision_status: str | None
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


def _result(
    *,
    stage: str,
    reason: str | None,
    cycle_ids: list[str],
    evidence_ids: list[str] | None = None,
    measurement_ids: list[str] | None = None,
    alignment_frame: str | None = None,
    alignment_units: str | None = None,
    measurement_frame: str | None = None,
    measurement_units: str | None = None,
    temporal_result: str | None = None,
    decision_status: str | None = None,
    planned_dry_run: dict[str, Any] | None = None,
    reobserve_required: bool = True,
) -> dict[str, Any]:
    return SupervisedTargetResult(
        schema_version="sie.supervised_target_dry_run.v1",
        stage=stage,
        result=stage,
        reason=reason,
        entity_type=PERSON_ENTITY_TYPE,
        source_window_cycle_ids=cycle_ids,
        source_evidence_ids=evidence_ids or [],
        source_measurement_ids=measurement_ids or [],
        alignment_reference_frame=alignment_frame,
        alignment_units=alignment_units,
        measurement_reference_frame=measurement_frame,
        measurement_units=measurement_units,
        temporal_result=temporal_result,
        decision_status=decision_status,
        planned_dry_run=planned_dry_run,
        reobserve_required=reobserve_required,
    ).to_dict()


def _cycle_ids(window: list[dict[str, Any]]) -> list[str] | None:
    identifiers = [_text(cycle.get("cycle_id")) for cycle in window]
    return None if any(identifier is None for identifier in identifiers) else [identifier for identifier in identifiers if identifier is not None]


def _latest_depth_is_present(cycle: dict[str, Any]) -> bool:
    measurement = cycle.get("measurement")
    if type(measurement) is not dict:
        return False
    return measurement.get("status") == "SUCCESS" and _text(measurement.get("measurement_id")) is not None


class SupervisedTargetDryRun:
    """Coordinates a person source adapter through generic target stages only."""

    def __init__(
        self,
        *,
        live_runtime: Any,
        optical_axis_cx_px: float,
        center_tolerance_px: float,
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
        """Read exactly one five-cycle shared window from an already-started runtime."""
        window = [self.live_runtime.cycle(f"supervised-target-{index:06d}") for index in range(1, WINDOW_SIZE + 1)]
        return self.process_shared_window(window)

    def process_shared_window(self, cycles: object) -> dict[str, Any]:
        """Process one supplied shared window; no sensor or action is invoked here."""
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
                cycle,
                line_number=index,
                optical_axis_cx_px=self.optical_axis_cx_px,
                center_tolerance_px=self.center_tolerance_px,
            )
            for index, cycle in enumerate(window, start=1)
        ]
        temporal = evaluate_temporal_yaw_alignment(observations, now_utc=self.now_utc)
        temporal_result = _text(temporal.get("result"))
        if temporal_result not in {"PLANNED_TURN", "NO_TURN_CENTERED"}:
            return _result(
                stage="BLOCKED_NO_ACTION", reason=f"TEMPORAL_{temporal.get('block_reason', 'INVALID')}", cycle_ids=cycle_ids,
                evidence_ids=temporal.get("used_evidence_ids") if type(temporal.get("used_evidence_ids")) is list else [],
                alignment_frame=_text(temporal.get("reference_frame")), alignment_units=_text(temporal.get("units")),
                temporal_result=temporal_result,
            )

        coordinator_envelope: dict[str, Any] = {"temporal_alignment": temporal}
        if _latest_depth_is_present(window[-1]):
            coordinator_envelope["person_depth"] = window[-1]
        coordinated = coordinate_supervised_acquire_range(coordinator_envelope, now_utc=self.now_utc)
        evidence_ids = coordinated["source_evidence_ids"]
        if coordinated["stage"] == "AWAIT_OPERATOR_TURN_AND_REOBSERVATION":
            return _result(
                stage="AWAIT_OPERATOR_TURN_AND_REOBSERVATION", reason=None, cycle_ids=cycle_ids,
                evidence_ids=evidence_ids, alignment_frame=coordinated["temporal_reference_frame"],
                alignment_units=coordinated["temporal_units"], temporal_result=temporal_result,
                planned_dry_run=coordinated["planned_turn"], reobserve_required=True,
            )
        if coordinated["stage"] == "RANGE_ACQUISITION_REQUIRED":
            return _result(
                stage="RANGE_ACQUISITION_REQUIRED", reason=coordinated["reason"], cycle_ids=cycle_ids,
                evidence_ids=evidence_ids, alignment_frame=coordinated["temporal_reference_frame"],
                alignment_units=coordinated["temporal_units"], temporal_result=temporal_result,
            )
        if coordinated["stage"] != "DEPTH_APPROACH_DECISION_REQUIRED":
            return _result(
                stage="BLOCKED_NO_ACTION", reason=f"COORDINATOR_{coordinated['reason']}", cycle_ids=cycle_ids,
                evidence_ids=evidence_ids, alignment_frame=coordinated["temporal_reference_frame"],
                alignment_units=coordinated["temporal_units"], temporal_result=temporal_result,
            )

        engine = self.decision_engine_factory()
        decision = None
        for cycle in window:
            decision = engine.ingest(cycle).to_dict()
        assert decision is not None
        measurement_ids = decision.get("source_measurement_ids")
        if type(measurement_ids) is not list or any(_text(item) is None for item in measurement_ids):
            measurement_ids = []
        common = {
            "cycle_ids": cycle_ids, "evidence_ids": evidence_ids, "measurement_ids": measurement_ids,
            "alignment_frame": coordinated["temporal_reference_frame"], "alignment_units": coordinated["temporal_units"],
            "measurement_frame": decision.get("reference_frame"), "measurement_units": decision.get("units"),
            "temporal_result": temporal_result, "decision_status": decision.get("status"),
        }
        if decision.get("status") == "HOLD_TARGET_REACHED":
            return _result(stage="HOLD_TARGET_REACHED", reason=decision.get("detail"), reobserve_required=False, **common)
        if decision.get("status") != "ADVANCE":
            return _result(stage="BLOCKED_NO_ACTION", reason=f"DECISION_STATUS_{decision.get('status')}", **common)
        step = _finite(decision.get("forward_step_m"))
        if decision.get("units") != "m" or step is None or not 0.0 < step <= 0.10:
            return _result(stage="BLOCKED_NO_ACTION", reason="INVALID_EXISTING_ADVANCE_DECISION", **common)
        return _result(
            stage="AWAIT_OPERATOR_ADVANCE_AND_REOBSERVATION", reason=None, reobserve_required=True,
            planned_dry_run={"method": "POST", "endpoint": "/move-forward", "distance_m": step}, **common,
        )
