from __future__ import annotations

import inspect
import json

from vision_core.tools import run_sie_static_target_mvp as runner


def _cycle(index: int, *, metric: bool) -> dict:
    measurement = {
        "status": "SUCCESS" if metric else "BLOCKED_QUALITY",
        "measurement_id": f"m-{index}",
        "timestamp": "2026-09-16T12:00:00+00:00",
        "reference_frame": "rectified_left_optical_frame",
        "units": "m",
        "x_m": 0.0,
        "y_m": 0.0,
        "z_m": 2.2,
        "range_m": 2.2,
    }
    return {
        "cycle_id": f"cycle-{index}",
        "status": "SUCCESS",
        "measurement": measurement,
    }


def _candidate(*, metric: bool) -> dict:
    return {
        "cycles": [_cycle(index, metric=metric) for index in range(1, 6)],
        "supervision": {
            "result": "AWAIT_OPERATOR_ADVANCE_AND_REOBSERVATION" if metric else "RANGE_ACQUISITION_REQUIRED",
            "reason": None if metric else "NO_CURRENT_VALID_METRIC_DEPTH_DECISION",
            "metric_decision": {"status": "ADVANCE"} if metric else None,
        },
    }


def test_invalid_first_window_reacquires_independent_window_until_metric_decision() -> None:
    calls: list[int] = []

    def acquire(attempt: int) -> dict:
        calls.append(attempt)
        return _candidate(metric=attempt == 2)

    selected, diagnostic = runner.acquire_metric_depth_with_one_retry(acquire)

    assert calls == [1, 2]
    assert selected["supervision"]["metric_decision"]["status"] == "ADVANCE"
    assert diagnostic["attempt_count"] == 2
    first = diagnostic["attempts"][0]
    assert first["valid_metric_measurement_count"] == 0
    assert len(first["invalid_metric_cycles"]) == 5
    assert all(item["measurement_status"] == "BLOCKED_QUALITY" for item in first["invalid_metric_cycles"])
    assert all(item["reason"] == "MEASUREMENT_STATUS_NOT_SUCCESS" for item in first["invalid_metric_cycles"])
    assert json.loads(json.dumps(diagnostic, allow_nan=False)) == diagnostic


def test_two_invalid_windows_return_diagnostics_without_command() -> None:
    selected, diagnostic = runner.acquire_metric_depth_with_one_retry(
        lambda _: _candidate(metric=False)
    )

    assert diagnostic["attempt_count"] == 2
    assert diagnostic["final_metric_decision_available"] is False
    assert selected["supervision"]["result"] == "RANGE_ACQUISITION_REQUIRED"
    combined = runner._combined(
        "RANGE_ACQUISITION_REQUIRED",
        "NO_CURRENT_VALID_METRIC_DEPTH_DECISION",
        supervision=selected["supervision"], bridge=None, executor=None,
        yolo_primary_evidence=None, network=False, metric_acquisition=diagnostic,
    )
    assert combined["bridge_plan"] is None
    assert combined["command_id"] is None
    assert combined["network_performed"] is False
    assert combined["motor_command_performed"] is False


def test_static_runner_reaches_status_and_post_only_after_metric_retry_selection() -> None:
    main_source = inspect.getsource(runner.main)
    retry = main_source.index("acquire_metric_depth_with_one_retry(")
    decision_block = main_source.index("blocked = execution_block_result(")
    status_read = main_source.index("fetch_bounded_status(")
    post = main_source.index("execute_one_supervised_command(")
    assert retry < decision_block < status_read < post
    assert main_source.index("if blocked is not None:", decision_block) < status_read
