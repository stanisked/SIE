from __future__ import annotations

from datetime import datetime, timedelta, timezone
from inspect import getsource
import json

import pytest

from sie_core.supervised_bounded_executor import (
    ExecutionContractError,
    SUPERVISED_PERSON_APPROACH_SESSION,
    authorize_person_approach_session_action,
    execute_one_supervised_command,
)
from vision_core.person_approach.bounded_bridge import plan_bounded_command
from vision_core.person_approach.spatial_navigation import (
    evaluate_person_navigation_window,
    navigation_decision,
)
from vision_core.tools.run_sie_static_target_mvp import fresh_supervised_execution_plan
from vision_core.tools.run_sie_static_target_mvp import parse_args as parse_static_target_args
from vision_core.tools.run_sie_person_approach_demo_mvp import (
    SESSION_CONFIRMATION,
    _bridge_envelope,
    _next_action_ready_status,
    _record,
    main as person_approach_main,
    parse_args as parse_person_approach_args,
)


NOW = datetime(2026, 9, 17, 12, 0, tzinfo=timezone.utc)


def _window(*, x_m: float, z_m: float, yolo: str = "SINGLE_TARGET") -> list[dict]:
    rows = []
    for index in range(5):
        timestamp = (NOW - timedelta(milliseconds=40 - index)).isoformat()
        rows.append({
            "cycle_id": f"cycle-{index}", "captured_at_utc": timestamp,
            "status": "SUCCESS",
            "primary_person_observation": {"target_status": yolo},
            "measurement": {
                "status": "SUCCESS", "measurement_id": f"measurement-{index}",
                "timestamp": timestamp, "reference_frame": "rectified_left_optical_frame",
                "units": "m", "x_m": x_m, "y_m": 0.0, "z_m": z_m,
                "range_m": (x_m * x_m + z_m * z_m) ** 0.5,
                "confidence": 0.9,
            },
        })
    return rows


def _evaluate(**values: object) -> dict:
    return evaluate_person_navigation_window(
        now_utc=lambda: NOW, safe_distance_m=1.0, bearing_deadband_deg=2.0,
        **values,
    )


def test_left_bearing_plans_left_turn_then_requires_reobservation() -> None:
    result = _evaluate(cycles=_window(x_m=-0.5, z_m=2.0))
    assert result["result"] == "TURN_REQUIRED"
    assert result["action"] == {"status": "TURN_LEFT", "angle_deg": 10.0}
    assert result["image_center_used_as_gate"] is False


def test_right_bearing_plans_right_turn() -> None:
    result = _evaluate(cycles=_window(x_m=0.5, z_m=2.0))
    assert result["result"] == "TURN_REQUIRED"
    assert result["action"]["status"] == "TURN_RIGHT"


def test_centered_person_plans_fixed_bounded_forward_step() -> None:
    result = _evaluate(cycles=_window(x_m=0.0, z_m=2.0))
    assert result["result"] == "FORWARD_REQUIRED"
    assert result["action"] == {"status": "ADVANCE", "distance_m": 0.10}
    decision = navigation_decision(result, sequence=1, timestamp=NOW.isoformat())
    assert decision["forward_step_m"] == 0.10


def test_far_windows_continue_and_arrive_only_at_safe_distance() -> None:
    far = _evaluate(cycles=_window(x_m=0.0, z_m=3.5))
    nearer = _evaluate(cycles=_window(x_m=0.0, z_m=1.5))
    arrived = _evaluate(cycles=_window(x_m=0.0, z_m=1.0))
    assert far["result"] == nearer["result"] == "FORWARD_REQUIRED"
    assert arrived["result"] == "ARRIVED" and arrived["action"] is None


def test_invalid_post_action_window_blocks_without_action() -> None:
    invalid = _window(x_m=0.0, z_m=2.0)
    invalid[-1]["status"] = "BLOCKED_QUALITY"
    result = _evaluate(cycles=invalid)
    assert result["result"] == "BLOCKED"
    assert result.get("action") is None


def test_each_navigation_action_receives_a_new_execution_command_id() -> None:
    plan = {
        "result": "PLANNED_BOUNDED_COMMAND", "method": "POST", "endpoint": "/turn-left",
        "command_id": "pa-deterministic", "network_performed": False,
        "query": {"boot_session_id": "0123456789ABCDEF", "command_id": "pa-deterministic", "angle_deg": "4"},
    }
    first, second = fresh_supervised_execution_plan(plan), fresh_supervised_execution_plan(plan)
    assert first["command_id"] != second["command_id"]
    assert first["query"]["command_id"] == first["command_id"]
    assert second["query"]["command_id"] == second["command_id"]


def test_no_fixed_start_range_or_image_center_gate() -> None:
    result = evaluate_person_navigation_window(
        _window(x_m=0.01, z_m=8.0), safe_distance_m=1.0,
        bearing_deadband_deg=2.0, now_utc=lambda: NOW,
    )
    assert result["result"] == "FORWARD_REQUIRED"
    assert result["image_center_used_as_gate"] is False


def test_bridge_uses_window_evaluation_freshness_not_later_processing_time() -> None:
    cycles = _window(x_m=0.0, z_m=2.0)
    navigation = evaluate_person_navigation_window(
        cycles, safe_distance_m=1.0, bearing_deadband_deg=2.0, now_utc=lambda: NOW,
    )
    decision = navigation_decision(navigation, sequence=1, timestamp=NOW.isoformat())
    evaluation_reference = NOW + timedelta(milliseconds=900)
    envelope = _bridge_envelope(
        decision=decision,
        cycles=cycles,
        boot_session_id="0123456789ABCDEF",
        freshness_reference_utc=evaluation_reference.isoformat(),
    )

    result = plan_bounded_command(
        envelope, now_utc=lambda: NOW + timedelta(seconds=2),
    ).to_dict()

    assert result["result"] == "PLANNED_BOUNDED_COMMAND"
    assert result["freshness_diagnostics"]["measured_age_s"] < 1.0
    assert result["freshness_diagnostics"]["bridge_checked_at_utc"] == (NOW + timedelta(seconds=2)).isoformat()


def test_stale_window_is_blocked_before_bridge_planning() -> None:
    cycles = _window(x_m=0.0, z_m=2.0)
    for cycle in cycles:
        timestamp = (NOW - timedelta(seconds=2)).isoformat()
        cycle["captured_at_utc"] = timestamp
        cycle["measurement"]["timestamp"] = timestamp

    result = evaluate_person_navigation_window(
        cycles, safe_distance_m=1.0, bearing_deadband_deg=2.0, now_utc=lambda: NOW,
    )

    assert result["result"] == "BLOCKED"
    assert result["reason"] == "METRIC_EVIDENCE_STALE_OR_FUTURE"
    assert result.get("action") is None


def _ready_status(*, command_id: str | None = None, terminal: str | None = None) -> dict:
    return {
        "motion_api_version": "v2_bounded_motion_api",
        "boot_session_id": "0123456789ABCDEF",
        "state": "READY",
        "bounded_fault_latched": False,
        "active_command_id": None,
        "last_command_id": command_id,
        "last_command_state": terminal,
    }


def _session_plan(endpoint: str, parameter_name: str, parameter: str) -> dict:
    return fresh_supervised_execution_plan({
        "result": "PLANNED_BOUNDED_COMMAND", "method": "POST", "endpoint": endpoint,
        "command_id": "pa-deterministic", "network_performed": False,
        "query": {
            "boot_session_id": "0123456789ABCDEF", "command_id": "pa-deterministic",
            parameter_name: parameter,
        },
    })


def test_second_action_retries_read_only_status_then_posts_once_in_same_session() -> None:
    first = _session_plan("/move-forward", "distance_m", "0.1")
    second = _session_plan("/move-forward", "distance_m", "0.1")
    first_auth = authorize_person_approach_session_action(
        planned_command=first, session_id="session-test", operator_session_confirmation=SESSION_CONFIRMATION,
    )
    second_auth = authorize_person_approach_session_action(
        planned_command=second, session_id="session-test", operator_session_confirmation=SESSION_CONFIRMATION,
    )
    first_calls: list[str] = []

    def first_request(method: str, _url: str, _timeout_s: float) -> tuple[int, dict]:
        first_calls.append(method)
        if method == "POST":
            return 202, {"accepted": True, "command_id": first["command_id"]}
        return 200, _ready_status(command_id=first["command_id"], terminal="PARTIAL_PROGRESS")

    first_result = execute_one_supervised_command(
        planned_command=first, authorization=first_auth, base_url="http://127.0.0.1",
        request=first_request, sleep=lambda _: None, monotonic=lambda: 0.0,
    )
    read_attempts: list[str] = []
    responses: list[object] = [OSError("transient timeout"), OSError("transient timeout"), (200, _ready_status())]

    def second_status(**_kwargs: object) -> tuple[int, dict]:
        read_attempts.append("GET")
        response = responses.pop(0)
        if isinstance(response, Exception):
            raise response
        return response  # type: ignore[return-value]

    second_status_record = _next_action_ready_status(
        base_url="http://127.0.0.1", timeout_s=1.0, poll_interval_s=0.1,
        fetch_status=second_status, monotonic=lambda: 0.0, sleep=lambda _: None,
    )
    second_calls: list[str] = []

    def second_request(method: str, _url: str, _timeout_s: float) -> tuple[int, dict]:
        second_calls.append(method)
        if method == "POST":
            return 202, {"accepted": True, "command_id": second["command_id"]}
        return 200, _ready_status(command_id=second["command_id"], terminal="PARTIAL_PROGRESS")

    second_result = execute_one_supervised_command(
        planned_command=second, authorization=second_auth, base_url="http://127.0.0.1",
        request=second_request, sleep=lambda _: None, monotonic=lambda: 0.0,
    )

    assert first_result["result"] == second_result["result"] == "AWAIT_REOBSERVATION"
    assert first["command_id"] != second["command_id"]
    assert read_attempts == ["GET", "GET", "GET"]
    assert second_status_record["state"] == "READY"
    assert first_calls.count("POST") == second_calls.count("POST") == 1
    assert getsource(person_approach_main).count("input(") == 1


def test_status_read_deadline_expires_without_post() -> None:
    clock = [0.0]

    def unavailable_status(**_kwargs: object) -> tuple[int, dict]:
        raise OSError("timeout")

    def sleep(seconds: float) -> None:
        clock[0] += seconds

    with pytest.raises(ExecutionContractError, match="STATUS_READ_ERROR"):
        _next_action_ready_status(
            base_url="http://127.0.0.1", timeout_s=0.5, poll_interval_s=0.2,
            fetch_status=unavailable_status, monotonic=lambda: clock[0], sleep=sleep,
        )
    assert "POST" not in getsource(_next_action_ready_status)


def test_terminal_status_timeouts_after_post_never_repeat_post() -> None:
    plan = _session_plan("/move-forward", "distance_m", "0.1")
    authorization = authorize_person_approach_session_action(
        planned_command=plan, session_id="session-test", operator_session_confirmation=SESSION_CONFIRMATION,
    )
    clock = [0.0]
    calls: list[str] = []

    def request(method: str, _url: str, _timeout_s: float) -> tuple[int, dict]:
        calls.append(method)
        if method == "POST":
            return 202, {"accepted": True, "command_id": plan["command_id"]}
        if calls.count("GET") == 1:
            return 200, _ready_status()
        raise OSError("terminal timeout")

    def sleep(seconds: float) -> None:
        clock[0] += seconds

    result = execute_one_supervised_command(
        planned_command=plan, authorization=authorization, base_url="http://127.0.0.1",
        request=request, sleep=sleep, monotonic=lambda: clock[0],
        poll_interval_s=0.2, terminal_timeout_s=0.5,
    )

    assert result["result"] == "TERMINAL_STATUS_UNKNOWN"
    assert calls.count("POST") == 1


def _person_approach_cli_args(*extra: str) -> list[str]:
    return [
        "--model", "model.onnx", "--reference", "reference.txt",
        "--project-root", ".", "--ar-intrinsic", "ar.json",
        "--safe-distance-m", "1.0", "--bearing-deadband-deg", "2.0",
        "--jsonl-output", "session.jsonl", *extra,
    ]


def test_session_mode_is_approach_only_and_static_target_rejects_it() -> None:
    args = parse_person_approach_args(_person_approach_cli_args())
    assert args.authorization_mode == SUPERVISED_PERSON_APPROACH_SESSION
    with pytest.raises(SystemExit):
        parse_static_target_args([
            "--model", "model.onnx", "--reference", "reference.txt",
            "--project-root", ".", "--ar-intrinsic", "ar.json",
            "--center-tolerance-px", "40",
            "--authorization-mode", SUPERVISED_PERSON_APPROACH_SESSION,
        ])


def test_one_session_confirmation_binds_multiple_new_action_ids_without_qualification_change() -> None:
    left_turn = fresh_supervised_execution_plan({
        "result": "PLANNED_BOUNDED_COMMAND", "method": "POST", "endpoint": "/turn-left",
        "command_id": "pa-deterministic", "network_performed": False,
        "query": {"boot_session_id": "0123456789ABCDEF", "command_id": "pa-deterministic", "angle_deg": "4"},
    })
    forward = fresh_supervised_execution_plan({
        "result": "PLANNED_BOUNDED_COMMAND", "method": "POST", "endpoint": "/move-forward",
        "command_id": "pa-deterministic", "network_performed": False,
        "query": {"boot_session_id": "0123456789ABCDEF", "command_id": "pa-deterministic", "distance_m": "0.1"},
    })
    left_auth = authorize_person_approach_session_action(
        planned_command=left_turn, session_id="session-test", operator_session_confirmation=SESSION_CONFIRMATION,
    )
    forward_auth = authorize_person_approach_session_action(
        planned_command=forward, session_id="session-test", operator_session_confirmation=SESSION_CONFIRMATION,
    )

    assert left_turn["command_id"] != forward["command_id"]
    assert getsource(person_approach_main).count("input(") == 1
    assert left_auth["mode"] == forward_auth["mode"] == SUPERVISED_PERSON_APPROACH_SESSION
    assert left_auth["operator_session_confirmation"] == forward_auth["operator_session_confirmation"] == SESSION_CONFIRMATION
    assert left_auth["automatic_capability_qualification_changed"] is False
    record = _record(
        session_id="session-test", operator_session_confirmation=SESSION_CONFIRMATION,
        action_command_ids=[left_turn["command_id"], forward["command_id"]],
        state="REOBSERVE_AFTER_FORWARD", index=2, navigation={"result": "FORWARD_REQUIRED", "action": {"status": "ADVANCE"}},
    )
    assert record["execution_scope"] == SUPERVISED_PERSON_APPROACH_SESSION
    assert record["automatic_capability_qualification_changed"] is False
    assert record["action_command_ids"] == [left_turn["command_id"], forward["command_id"]]
    json.dumps(record, allow_nan=False, sort_keys=True)
