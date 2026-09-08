from __future__ import annotations

import json

from sie_core.actuator_capability_gate import (
    RESULT_ALLOWED,
    RESULT_NOT_QUALIFIED,
    RESULT_UNKNOWN,
    evaluate_actuator_capability_gate,
)


REQUIREMENT = {
    "adapter_id": "example_adapter",
    "capability_id": "short_forward",
}
PLANNED_ACTION = {"method": "POST", "endpoint": "/move-forward", "distance_m": 0.1}
SOURCE_DECISION = {"decision_id": "decision-001", "status": "ADVANCE"}


def capability(status: str) -> dict:
    return {
        "schema_version": "sie.actuator_capability.v1",
        **REQUIREMENT,
        "qualification_status": status,
        "reason": f"{status.lower()} evidence",
        "evidence_ids": ["evidence-001"],
        "timestamp": "2026-09-08T00:00:00+00:00",
    }


def evaluate(status: str) -> dict:
    return evaluate_actuator_capability_gate(
        planned_action=PLANNED_ACTION,
        source_decision=SOURCE_DECISION,
        source_evidence_ids=["decision-evidence-001"],
        required_adapter_id=REQUIREMENT["adapter_id"],
        required_capability_id=REQUIREMENT["capability_id"],
        capability_record=capability(status),
    )


def test_not_qualified_blocks_and_preserves_action_decision_and_evidence():
    result = evaluate("NOT_QUALIFIED")

    assert result["result"] == RESULT_NOT_QUALIFIED
    assert result["planned_action"] == PLANNED_ACTION
    assert result["source_decision"] == SOURCE_DECISION
    assert result["source_evidence_ids"] == ["decision-evidence-001"]
    assert result["capability_record"]["evidence_ids"] == ["evidence-001"]
    assert result["network_performed"] is False
    assert result["motor_command_performed"] is False
    json.dumps(result, allow_nan=False, sort_keys=True)


def test_qualified_allows_only_dry_run_handoff():
    result = evaluate("QUALIFIED")

    assert result["result"] == RESULT_ALLOWED
    assert result["planned_action"] == PLANNED_ACTION
    assert result["network_performed"] is False
    assert result["motor_command_performed"] is False


def test_unknown_blocks_fail_closed():
    result = evaluate("UNKNOWN")

    assert result["result"] == RESULT_UNKNOWN
    assert result["network_performed"] is False
    assert result["motor_command_performed"] is False
