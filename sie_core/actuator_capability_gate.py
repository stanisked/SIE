"""Deterministic dry-run gate between planning and a future execution layer."""
from __future__ import annotations

import json
from typing import Any

from sie_core.contracts.actuator_capability import (
    ActuatorCapability,
    ActuatorCapabilityContractError,
)


SCHEMA_VERSION = "sie.actuator_capability_gate_result.v1"
RESULT_ALLOWED = "DRY_RUN_ACTION_ALLOWED"
RESULT_NOT_QUALIFIED = "BLOCKED_ACTUATOR_CAPABILITY_NOT_QUALIFIED"
RESULT_UNKNOWN = "BLOCKED_ACTUATOR_CAPABILITY_UNKNOWN"


def _json_safe(value: object, name: str) -> Any:
    try:
        return json.loads(json.dumps(value, allow_nan=False, sort_keys=True))
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be JSON-safe") from error


def _identifier(value: object, name: str) -> str:
    if type(value) is not str or not value:
        raise ValueError(f"{name} must be a non-empty string")
    return value


def _source_evidence_ids(value: object) -> list[str]:
    copied = _json_safe(value, "source_evidence_ids")
    if type(copied) is not list or any(type(item) is not str or not item for item in copied):
        raise ValueError("source_evidence_ids must be a list of non-empty strings")
    return copied


def _result(
    *,
    result: str,
    reason: str,
    required_adapter_id: str,
    required_capability_id: str,
    planned_action: Any,
    source_decision: Any,
    source_evidence_ids: list[str],
    capability: dict[str, Any] | None,
) -> dict[str, Any]:
    return _json_safe(
        {
            "schema_version": SCHEMA_VERSION,
            "result": result,
            "reason": reason,
            "required_capability": {
                "adapter_id": required_adapter_id,
                "capability_id": required_capability_id,
            },
            "capability_record": capability,
            "planned_action": planned_action,
            "source_decision": source_decision,
            "source_evidence_ids": source_evidence_ids,
            "network_performed": False,
            "motor_command_performed": False,
        },
        "gate result",
    )


def evaluate_actuator_capability_gate(
    *,
    planned_action: object,
    source_decision: object,
    source_evidence_ids: object,
    required_adapter_id: object,
    required_capability_id: object,
    capability_record: object,
) -> dict[str, Any]:
    """Allow a qualified action only as dry-run; unknown capability fails closed."""
    action = _json_safe(planned_action, "planned_action")
    decision = _json_safe(source_decision, "source_decision")
    adapter_id = _identifier(required_adapter_id, "required_adapter_id")
    capability_id = _identifier(required_capability_id, "required_capability_id")
    evidence_ids = _source_evidence_ids(source_evidence_ids)
    try:
        capability = ActuatorCapability.from_dict(capability_record)
    except ActuatorCapabilityContractError as error:
        return _result(
            result=RESULT_UNKNOWN,
            reason=f"INVALID_CAPABILITY_RECORD: {error}",
            required_adapter_id=adapter_id,
            required_capability_id=capability_id,
            planned_action=action,
            source_decision=decision,
            source_evidence_ids=evidence_ids,
            capability=None,
        )

    serialized_capability = capability.to_dict()
    if (capability.adapter_id, capability.capability_id) != (adapter_id, capability_id):
        return _result(
            result=RESULT_UNKNOWN,
            reason="CAPABILITY_RECORD_DOES_NOT_MATCH_ACTION_REQUIREMENT",
            required_adapter_id=adapter_id,
            required_capability_id=capability_id,
            planned_action=action,
            source_decision=decision,
            source_evidence_ids=evidence_ids,
            capability=serialized_capability,
        )
    if capability.qualification_status == "QUALIFIED":
        return _result(
            result=RESULT_ALLOWED,
            reason=capability.reason,
            required_adapter_id=adapter_id,
            required_capability_id=capability_id,
            planned_action=action,
            source_decision=decision,
            source_evidence_ids=evidence_ids,
            capability=serialized_capability,
        )
    return _result(
        result=(RESULT_NOT_QUALIFIED if capability.qualification_status == "NOT_QUALIFIED" else RESULT_UNKNOWN),
        reason=capability.reason,
        required_adapter_id=adapter_id,
        required_capability_id=capability_id,
        planned_action=action,
        source_decision=decision,
        source_evidence_ids=evidence_ids,
        capability=serialized_capability,
    )
