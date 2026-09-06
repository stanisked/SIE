from __future__ import annotations

import json
import math
from datetime import datetime, timezone
from pathlib import Path

import pytest

from vision_core.person_approach.far_field_alignment import evaluate_far_field_alignment


NOW = datetime(2026, 9, 7, 10, 0, tzinfo=timezone.utc)
FRAME = "ar0234_image_frame"


def envelope(*, center_x: float = 960., tolerance: float = 40., evidence=None) -> dict:
    return {
        "image_width_px": 1920,
        "optical_axis_cx_px": 960,
        "center_tolerance_px": tolerance,
        "person_evidence": [{
            "evidence_id": "person-evidence-001",
            "status": "SINGLE_PERSON",
            "reference_frame": FRAME,
            "units": "px",
            "bbox_xyxy_px": [center_x - 50., 300., center_x + 50., 700.],
        }] if evidence is None else evidence,
    }


def evaluate(value: dict):
    return evaluate_far_field_alignment(value, now_utc=lambda: NOW)


@pytest.mark.parametrize(
    ("center_x", "decision"),
    [(800., "ALIGN_TOWARD_IMAGE_LEFT"), (1120., "ALIGN_TOWARD_IMAGE_RIGHT"), (960., "READY_FOR_RANGE_ACQUISITION")],
)
def test_left_right_and_center(center_x: float, decision: str):
    result = evaluate(envelope(center_x=center_x))
    assert result["result"] == "READY_ALIGNMENT_DECISION"
    assert result["semantic_decision"] == decision
    assert result["evidence_id"] == "person-evidence-001"
    assert result["reference_frame"] == FRAME and result["units"] == "px"


@pytest.mark.parametrize("center_x", [920., 1000.])
def test_tolerance_boundary_is_ready(center_x: float):
    result = evaluate(envelope(center_x=center_x, tolerance=40.))
    assert result["semantic_decision"] == "READY_FOR_RANGE_ACQUISITION"
    assert abs(result["image_offset_px"]) == 40.


@pytest.mark.parametrize(
    ("evidence", "reason"),
    [([], "NO_PERSON_EVIDENCE"), ([{"evidence_id": "a"}, {"evidence_id": "b"}], "EXPECTED_EXACTLY_ONE_PERSON")],
)
def test_zero_and_multiple_persons_block(evidence: list[dict], reason: str):
    result = evaluate(envelope(evidence=evidence))
    assert result["result"] == "BLOCKED_NO_ALIGNMENT"
    assert result["block_reason"] == reason


@pytest.mark.parametrize(
    ("mutator", "reason"),
    [
        (lambda value: value["person_evidence"][0].update({"bbox_xyxy_px": [900., 300., 900., 700.]}), "INVALID_BBOX"),
        (lambda value: value["person_evidence"][0].update({"bbox_xyxy_px": [float("nan"), 300., 1000., 700.]}), "INPUT_NOT_JSON_SAFE"),
        (lambda value: value["person_evidence"][0].update({"reference_frame": "rectified_left_optical_frame"}), "REFERENCE_FRAME_OR_UNITS_MISMATCH"),
        (lambda value: value["person_evidence"][0].update({"units": "m"}), "REFERENCE_FRAME_OR_UNITS_MISMATCH"),
    ],
)
def test_invalid_nonfinite_and_frame_units_block(mutator, reason: str):
    value = envelope()
    mutator(value)
    result = evaluate(value)
    assert result["result"] == "BLOCKED_NO_ALIGNMENT"
    assert result["block_reason"] == reason


def test_tolerance_is_explicit_and_no_default_exists():
    value = envelope()
    del value["center_tolerance_px"]
    result = evaluate(value)
    assert result["block_reason"] == "CENTER_TOLERANCE_MUST_BE_EXPLICIT_FINITE_NON_NEGATIVE"


def test_output_is_json_safe_and_has_no_depth_or_transport_fields():
    result = evaluate(envelope(center_x=800.))
    json.dumps(result, allow_nan=False)
    assert not any(name in result for name in ("z_m", "range_m", "x_m", "y_m", "http", "endpoint", "motor"))
    assert result["image_offset_px"] == -160.


def test_cli_source_has_no_network_or_hardware_clients():
    source = Path("vision_core/tools/run_far_field_alignment_dry_run.py").read_text().lower()
    forbidden = ("requests", "urllib", "http.client", "socket", "wifi", "serial", "esp32", "motor", "subprocess")
    assert all(token not in source for token in forbidden)
