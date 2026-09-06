from __future__ import annotations

import json
import math
import subprocess
import sys
from copy import deepcopy
from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest

from vision_core.person_approach.bounded_bridge import plan_bounded_command


NOW = datetime(2026, 9, 6, 12, 0, tzinfo=timezone.utc)
FRAME = "rectified_left_optical_frame"
SESSION = "0123456789ABCDEF"


def cycle(index: int, *, status: str = "SUCCESS") -> dict:
    stamp = (NOW - timedelta(milliseconds=(4 - index) * 100)).isoformat()
    measurement = None
    if status == "SUCCESS":
        measurement = {
            "status": "SUCCESS",
            "measurement_id": f"measurement-{index}",
            "timestamp": stamp,
            "reference_frame": FRAME,
            "units": "m",
            "x_m": 0.02,
            "y_m": 0.0,
            "z_m": 2.4,
            "range_m": math.hypot(0.02, 2.4),
            "confidence": 0.9,
        }
    return {
        "schema_version": "sie.person_depth_live_cycle.v1",
        "cycle_id": f"cycle-{index}",
        "captured_at_utc": stamp,
        "status": status,
        "person": {"status": "SINGLE_PERSON" if status == "SUCCESS" else status},
        "measurement": measurement,
    }


def envelope(status: str = "ADVANCE", parameter: float = 0.1) -> dict:
    records = [cycle(index) for index in range(5)]
    decision = {
        "schema_version": "sie.person_approach_decision.v1",
        "decision_id": "decision.person_approach.000001",
        "timestamp": NOW.isoformat(),
        "status": status,
        "reference_frame": FRAME,
        "units": "m",
        "turn_angle_units": "deg",
        "forward_step_m": parameter if status == "ADVANCE" else None,
        "turn_angle_deg": parameter if status.startswith("TURN_") else None,
        "source_measurement_ids": [f"measurement-{index}" for index in range(5)],
    }
    return {"decision": decision, "evidence_window": records, "boot_session_id": SESSION}


def plan(value: dict):
    return plan_bounded_command(value, now_utc=lambda: NOW).to_dict()


def test_valid_advance_and_cli_emit_one_json_record(tmp_path: Path):
    expected = plan(envelope())
    assert expected["result"] == "PLANNED_BOUNDED_COMMAND"
    assert expected["method"] == "POST" and expected["endpoint"] == "/move-forward"
    assert expected["query"]["distance_m"] == "0.1"
    assert expected["network_performed"] is False and expected["units"] == "m"
    cli_envelope = envelope()
    cli_now = datetime.now(timezone.utc)
    for index, item in enumerate(cli_envelope["evidence_window"]):
        stamp = (cli_now - timedelta(milliseconds=(4-index)*10)).isoformat()
        item["captured_at_utc"] = stamp
        item["measurement"]["timestamp"] = stamp
    input_path = tmp_path / "input.json"
    input_path.write_text(json.dumps(cli_envelope))
    completed = subprocess.run(
        [sys.executable, "vision_core/tools/run_person_approach_bounded_bridge_dry_run.py", str(input_path)],
        check=True, capture_output=True, text=True,
    )
    assert completed.stderr == "" and len(completed.stdout.splitlines()) == 1
    assert json.loads(completed.stdout)["command_id"] == expected["command_id"]


@pytest.mark.parametrize(
    ("status", "endpoint"),
    [("TURN_LEFT", "/turn-left"), ("TURN_RIGHT", "/turn-right")],
)
def test_valid_turns(status: str, endpoint: str):
    result = plan(envelope(status, -4.0 if status == "TURN_LEFT" else 4.0))
    assert result["result"] == "PLANNED_BOUNDED_COMMAND"
    assert result["endpoint"] == endpoint and result["query"]["angle_deg"] == "4"
    assert result["units"] == "deg"


@pytest.mark.parametrize(("status", "parameter"), [("TURN_LEFT", 4.0), ("TURN_RIGHT", -4.0)])
def test_turn_direction_must_match_signed_decision_angle(status: str, parameter: float):
    assert plan(envelope(status, parameter))["block_reason"] == "TURN_DIRECTION_ANGLE_SIGN_MISMATCH"


def test_command_id_is_stable_for_retry_and_changes_with_identity_inputs():
    original = envelope()
    first = plan(original)["command_id"]
    assert plan(deepcopy(original))["command_id"] == first
    session = deepcopy(original)
    session["boot_session_id"] = "FEDCBA9876543210"
    decision = deepcopy(original)
    decision["decision"]["decision_id"] = "decision.person_approach.000002"
    payload = deepcopy(original)
    payload["decision"]["forward_step_m"] = 0.09
    assert len({first, plan(session)["command_id"], plan(decision)["command_id"], plan(payload)["command_id"]}) == 4


def test_stale_latest_cycle_blocks():
    value = envelope()
    for index, item in enumerate(value["evidence_window"]):
        stamp = (NOW - timedelta(seconds=2, milliseconds=4-index)).isoformat()
        item["captured_at_utc"] = stamp
        item["measurement"]["timestamp"] = stamp
    assert plan(value)["block_reason"] == "LATEST_CYCLE_STALE_OR_FROM_FUTURE"


def test_fewer_than_four_successes_blocks():
    value = envelope()
    for index in (0, 1):
        value["evidence_window"][index] = cycle(index, status="DEPTH_UNAVAILABLE")
    value["decision"]["source_measurement_ids"] = [f"measurement-{index}" for index in (2, 3, 4)]
    assert plan(value)["block_reason"] == "FEWER_THAN_FOUR_SUCCESS_CYCLES"


def test_multiple_persons_anywhere_blocks():
    value = envelope()
    value["evidence_window"][1] = cycle(1, status="MULTIPLE_PERSONS")
    assert plan(value)["block_reason"] == "MULTIPLE_PERSONS_IN_EVIDENCE_WINDOW"


def test_non_finite_value_is_never_json_output():
    value = envelope()
    value["evidence_window"][4]["measurement"]["z_m"] = float("nan")
    result = plan(value)
    assert result["block_reason"] == "ENVELOPE_NOT_JSON_SAFE"
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize("field", ["reference_frame", "units"])
def test_frame_or_units_mismatch_blocks(field: str):
    value = envelope()
    value["evidence_window"][2]["measurement"][field] = "wrong"
    assert plan(value)["block_reason"] == "REFERENCE_FRAME_OR_UNITS_MISMATCH"


@pytest.mark.parametrize(
    ("status", "parameter"),
    [("ADVANCE", 0.019), ("ADVANCE", 0.101), ("TURN_LEFT", -0.9), ("TURN_RIGHT", 10.1)],
)
def test_out_of_range_parameters_block_without_rounding(status: str, parameter: float):
    assert plan(envelope(status, parameter))["block_reason"] == "COMMAND_PARAMETER_OUT_OF_RANGE"


def test_partial_progress_reobserve_blocks_new_command():
    value = envelope()
    value["previous_terminal_motion_outcome"] = {
        "command_state": "PARTIAL_PROGRESS", "reobserve_required": True,
    }
    result = plan(value)
    assert result["block_reason"] == "REOBSERVE_REQUIRED_AFTER_PARTIAL_PROGRESS"
    assert result["reobserve_required"] is True


def test_source_evidence_mismatch_blocks():
    value = envelope()
    value["decision"]["source_measurement_ids"][-1] = "other"
    assert plan(value)["block_reason"] == "SOURCE_EVIDENCE_MISMATCH"


@pytest.mark.parametrize("status", ["HOLD_TARGET_REACHED", "BLOCKED_STALE", "TOO_CLOSE_NO_REVERSE", "PERSON_LOST", "MULTIPLE_PERSONS"])
def test_non_motion_decisions_are_explicit_no_command(status: str):
    result = plan(envelope(status))
    assert result["result"] == "BLOCKED_NO_COMMAND" and result["endpoint"] is None
    assert result["block_reason"].startswith("DECISION_STATUS_")


def test_source_scan_has_no_network_clients_or_execution_calls():
    paths = [
        Path("vision_core/person_approach/bounded_bridge.py"),
        Path("vision_core/tools/run_person_approach_bounded_bridge_dry_run.py"),
    ]
    source = "\n".join(path.read_text() for path in paths).lower()
    forbidden = ("requests", "urllib", "http.client", "socket", "wifi", "serial", "server.send", "motor")
    assert all(token not in source for token in forbidden)
