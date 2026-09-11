from __future__ import annotations

import json

from sie_core.supervised_bounded_executor import (
    authorize_operator_trial,
    execute_one_supervised_command,
    validate_planned_bounded_command,
)
from vision_core.tools.run_sie_static_target_mvp import bridge_envelope


def plan() -> dict:
    return {
        "schema_version": "sie.person_approach_bounded_bridge.v1",
        "result": "PLANNED_BOUNDED_COMMAND",
        "method": "POST",
        "endpoint": "/move-forward",
        "query": {
            "boot_session_id": "0123456789ABCDEF",
            "command_id": "pa-supervised-mvp-001",
            "distance_m": "0.1",
        },
        "network_performed": False,
    }


def ready_status(**extra: object) -> dict:
    return {
        "motion_api_version": "v2_bounded_motion_api",
        "boot_session_id": "0123456789ABCDEF",
        "state": "READY",
        "bounded_fault_latched": False,
        "active_command_id": None,
        "last_command_id": None,
        "last_command_state": None,
        **extra,
    }


def authorization(mode: str = "QUALIFIED") -> dict:
    return authorize_operator_trial(
        planned_command=plan(),
        confirmation_command_id="pa-supervised-mvp-001",
        authorization_mode=mode,
        experimental_reason="supervised static-target MVP" if mode != "QUALIFIED" else None,
    )


def test_plan_contract_and_experimental_authorization_are_json_safe() -> None:
    command = validate_planned_bounded_command(plan())
    auth = authorization("SUPERVISED_EXPERIMENTAL_TRIAL")

    assert command["query"]["distance_m"] == "0.1"
    assert auth["automatic_capability_qualification_changed"] is False
    assert auth["experimental_reason"] == "supervised static-target MVP"
    json.dumps({"command": command, "authorization": auth}, allow_nan=False)


def test_preflight_blocks_before_post_when_session_is_not_fresh() -> None:
    calls: list[tuple[str, str]] = []

    def request(method: str, url: str, timeout_s: float) -> tuple[int, dict]:
        calls.append((method, url))
        return 200, ready_status(boot_session_id="FEDCBA9876543210")

    result = execute_one_supervised_command(
        planned_command=plan(), authorization=authorization(), base_url="http://127.0.0.1",
        request=request, sleep=lambda _: None,
    )

    assert result["result"] == "BLOCKED_PREFLIGHT"
    assert result["reason"] == "BOOT_SESSION_CHANGED_OR_PLAN_NOT_FRESH"
    assert [method for method, _ in calls] == ["GET"]
    assert result["motor_command_performed"] is False


def test_executes_one_post_then_requires_reobservation_without_retry() -> None:
    calls: list[tuple[str, str]] = []
    responses = iter(
        [
            (200, ready_status()),
            (202, {"accepted": True, "command_id": "pa-supervised-mvp-001", "command_state": "ACCEPTED"}),
            (200, ready_status(last_command_id="pa-supervised-mvp-001", last_command_state="PARTIAL_PROGRESS")),
        ]
    )

    def request(method: str, url: str, timeout_s: float) -> tuple[int, dict]:
        calls.append((method, url))
        return next(responses)

    result = execute_one_supervised_command(
        planned_command=plan(), authorization=authorization("SUPERVISED_EXPERIMENTAL_TRIAL"),
        base_url="http://127.0.0.1", request=request, sleep=lambda _: None,
        monotonic=lambda: 0.0,
    )

    assert result["result"] == "AWAIT_REOBSERVATION"
    assert [method for method, _ in calls] == ["GET", "POST", "GET"]
    assert result["network_performed"] is True
    assert result["motor_command_performed"] is True
    assert result["reobserve_required"] is True


def test_static_target_adapter_reuses_only_metric_advance_and_shared_window() -> None:
    cycles = [{"cycle_id": f"cycle-{index}"} for index in range(5)]
    decision = {"decision_id": "decision-1", "status": "ADVANCE"}
    envelope = bridge_envelope(
        supervision={"metric_decision": decision}, cycles=cycles,
        boot_session_id="0123456789ABCDEF",
    )

    assert envelope is not None
    assert envelope["decision"] is decision
    assert envelope["evidence_window"] is cycles
    assert bridge_envelope(supervision={"metric_decision": {"status": "HOLD_TARGET_REACHED"}}, cycles=cycles, boot_session_id="0123456789ABCDEF") is None
