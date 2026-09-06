"""Supervised, non-executing composition of the person-approach MVP layers."""
from __future__ import annotations

import json
from collections import deque
from datetime import datetime, timezone
from typing import Any, Callable

from .bounded_bridge import plan_bounded_command
from .decision import PersonApproachDecisionEngine


WINDOW_SIZE = 5
AWAITING_OPERATOR_CONFIRMATION = "AWAITING_OPERATOR_CONFIRMATION"


def _json_safe(value: Any) -> dict[str, Any]:
    return json.loads(json.dumps(value, allow_nan=False, sort_keys=True))


def _timestamp(now_utc: Callable[[], datetime]) -> str:
    value = now_utc()
    if not isinstance(value, datetime) or value.tzinfo is None:
        raise ValueError("now_utc must return a timezone-aware datetime")
    return value.astimezone(timezone.utc).isoformat()


def _person_depth_summary(cycle: object) -> dict[str, Any]:
    if type(cycle) is not dict:
        return {"cycle_status": "INVALID_CYCLE", "person_status": None, "measurement": None}
    person = cycle.get("person") if type(cycle.get("person")) is dict else {}
    measurement = cycle.get("measurement") if type(cycle.get("measurement")) is dict else None
    compact_measurement = None if measurement is None else {
        key: measurement.get(key)
        for key in ("measurement_id", "status", "x_m", "y_m", "z_m", "range_m", "confidence")
    }
    return {
        "cycle_status": cycle.get("status"),
        "person_status": person.get("status"),
        "measurement": compact_measurement,
    }


def _evidence_window_summary(window: deque[dict[str, Any]]) -> list[dict[str, Any]]:
    summary = []
    for cycle in window:
        person = cycle.get("person") if type(cycle.get("person")) is dict else {}
        measurement = cycle.get("measurement") if type(cycle.get("measurement")) is dict else {}
        summary.append({
            "cycle_id": cycle.get("cycle_id"),
            "cycle_status": cycle.get("status"),
            "person_status": person.get("status"),
            "measurement_status": measurement.get("status"),
        })
    return summary


class SupervisedPersonApproachDemo:
    """Runs perception records through decision and planning, never through execution."""

    def __init__(
        self,
        *,
        boot_session_id: str,
        decision_engine: PersonApproachDecisionEngine | None = None,
        previous_terminal_motion_outcome: dict[str, Any] | None = None,
        now_utc: Callable[[], datetime] = lambda: datetime.now(timezone.utc),
    ) -> None:
        self.boot_session_id = boot_session_id
        self.decision_engine = decision_engine or PersonApproachDecisionEngine(now_utc=now_utc)
        self.previous_terminal_motion_outcome = previous_terminal_motion_outcome
        self.now_utc = now_utc
        self._window: deque[dict[str, Any]] = deque(maxlen=WINDOW_SIZE)
        self._stage = "OBSERVING"

    @property
    def stage(self) -> str:
        return self._stage

    def process_cycle(self, cycle: dict[str, Any]) -> dict[str, Any]:
        """Accept one live cycle record and return one JSON-safe supervised demo record."""
        if self._stage == AWAITING_OPERATOR_CONFIRMATION:
            return self._record(
                stage=AWAITING_OPERATOR_CONFIRMATION,
                cycle=cycle,
                decision=None,
                planned=None,
                block_reason="OPERATOR_CONFIRMATION_REQUIRED",
                reobservation_required=False,
            )

        decision = self.decision_engine.ingest(cycle).to_dict()
        if type(cycle) is dict:
            self._window.append(cycle)
        if decision["status"] in {"TURN_LEFT", "TURN_RIGHT"}:
            return self._record(
                stage="NO_ACTION",
                cycle=cycle,
                decision=decision,
                planned=None,
                block_reason="TURN_NOT_ENABLED_IN_SUPERVISED_DEMO_V0",
                reobservation_required=True,
            )
        if decision["status"] != "ADVANCE":
            return self._record(
                stage="NO_ACTION",
                cycle=cycle,
                decision=decision,
                planned=None,
                block_reason=f"DECISION_STATUS_{decision['status']}_HAS_NO_ACTION",
                reobservation_required=decision["status"].startswith("BLOCKED_"),
            )

        planned = plan_bounded_command(
            {
                "decision": decision,
                "evidence_window": list(self._window),
                "boot_session_id": self.boot_session_id,
                "previous_terminal_motion_outcome": self.previous_terminal_motion_outcome,
            },
            now_utc=self.now_utc,
        ).to_dict()
        if planned["result"] != "PLANNED_BOUNDED_COMMAND":
            return self._record(
                stage="NO_ACTION",
                cycle=cycle,
                decision=decision,
                planned=planned,
                block_reason=planned["block_reason"],
                reobservation_required=planned["reobserve_required"] or "STALE" in planned["block_reason"],
            )
        self._stage = AWAITING_OPERATOR_CONFIRMATION
        return self._record(
            stage=AWAITING_OPERATOR_CONFIRMATION,
            cycle=cycle,
            decision=decision,
            planned=planned,
            block_reason=None,
            reobservation_required=False,
        )

    def _record(
        self,
        *,
        stage: str,
        cycle: object,
        decision: dict[str, Any] | None,
        planned: dict[str, Any] | None,
        block_reason: str | None,
        reobservation_required: bool,
    ) -> dict[str, Any]:
        cycle_object = cycle if type(cycle) is dict else {}
        measurement = cycle_object.get("measurement") if type(cycle_object.get("measurement")) is dict else {}
        return _json_safe({
            "schema_version": "sie.supervised_person_approach_demo.v0",
            "timestamp": _timestamp(self.now_utc),
            "stage": stage,
            "cycle_id": cycle_object.get("cycle_id"),
            "decision_id": None if decision is None else decision.get("decision_id"),
            "person_depth_summary": _person_depth_summary(cycle),
            "evidence_window_summary": _evidence_window_summary(self._window),
            "decision_status": None if decision is None else decision.get("status"),
            "planned_command": planned,
            "block_reason": block_reason,
            "reobservation_required": reobservation_required,
            "boot_session_id": self.boot_session_id,
            "boot_session_freshness_verified": False,
            "network_performed": False,
            "motor_command_performed": False,
            "reference_frame": None if decision is None else decision.get("reference_frame", measurement.get("reference_frame")),
            "units": None if decision is None else decision.get("units", measurement.get("units")),
        })
