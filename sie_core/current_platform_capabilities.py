"""Versioned JSON-safe capability configuration for the current SIE platform."""
from __future__ import annotations

from typing import Any


CURRENT_PLATFORM_CAPABILITY_PROFILE_V1: dict[str, Any] = {
    "schema_version": "sie.actuator_capability.v1",
    "adapter_id": "esp32_zk5ad_sgm37_520",
    "capability_id": "bounded_forward_0.10_m",
    "qualification_status": "NOT_QUALIFIED",
    "reason": "repeatable underreach/overshoot trade-off and asymmetric low-speed response",
    "evidence_ids": ["git_commit:77dd01bfe2e0256b6f03029f0cec01576b42f9da"],
    "timestamp": "2026-09-08T00:00:00+00:00",
}
