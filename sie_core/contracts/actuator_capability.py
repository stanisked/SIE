"""JSON-safe generic actuator capability records for SIE decisions."""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from typing import Any


QUALIFICATION_STATUSES = frozenset({"QUALIFIED", "NOT_QUALIFIED", "UNKNOWN"})
SCHEMA_VERSION = "sie.actuator_capability.v1"


class ActuatorCapabilityContractError(ValueError):
    """Raised when a capability record cannot be used by a safety gate."""


def _text(value: object, field: str) -> str:
    if type(value) is not str or not value:
        raise ActuatorCapabilityContractError(f"{field} must be a non-empty string")
    return value


def _timestamp(value: object) -> str:
    text = _text(value, "timestamp")
    try:
        parsed = datetime.fromisoformat(text.replace("Z", "+00:00"))
    except ValueError as error:
        raise ActuatorCapabilityContractError("timestamp must be ISO-8601") from error
    if parsed.tzinfo is None:
        raise ActuatorCapabilityContractError("timestamp must be timezone-aware")
    return parsed.astimezone(timezone.utc).isoformat()


def _evidence_ids(value: object) -> list[str]:
    if type(value) is not list or not value:
        raise ActuatorCapabilityContractError("evidence_ids must be a non-empty list")
    identifiers = [_text(item, "evidence_ids item") for item in value]
    if len(set(identifiers)) != len(identifiers):
        raise ActuatorCapabilityContractError("evidence_ids must be unique")
    return identifiers


@dataclass(frozen=True)
class ActuatorCapability:
    """Qualification of one adapter capability, independent of any target."""

    schema_version: str
    adapter_id: str
    capability_id: str
    qualification_status: str
    reason: str
    evidence_ids: list[str]
    timestamp: str

    @classmethod
    def from_dict(cls, value: object) -> "ActuatorCapability":
        if type(value) is not dict:
            raise ActuatorCapabilityContractError("capability record must be an object")
        try:
            copied = json.loads(json.dumps(value, allow_nan=False, sort_keys=True))
        except (TypeError, ValueError) as error:
            raise ActuatorCapabilityContractError(
                "capability record must be JSON-safe"
            ) from error
        schema_version = _text(copied.get("schema_version"), "schema_version")
        if schema_version != SCHEMA_VERSION:
            raise ActuatorCapabilityContractError("unsupported capability schema_version")
        qualification_status = _text(
            copied.get("qualification_status"), "qualification_status"
        )
        if qualification_status not in QUALIFICATION_STATUSES:
            raise ActuatorCapabilityContractError("unsupported qualification_status")
        return cls(
            schema_version=schema_version,
            adapter_id=_text(copied.get("adapter_id"), "adapter_id"),
            capability_id=_text(copied.get("capability_id"), "capability_id"),
            qualification_status=qualification_status,
            reason=_text(copied.get("reason"), "reason"),
            evidence_ids=_evidence_ids(copied.get("evidence_ids")),
            timestamp=_timestamp(copied.get("timestamp")),
        )

    def to_dict(self) -> dict[str, Any]:
        return json.loads(json.dumps(asdict(self), allow_nan=False, sort_keys=True))
