from __future__ import annotations

import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from vision_core.ar0234_intrinsic_independent_validation import (
    AR_FPS,
    AR_HEIGHT,
    AR_REFERENCE_FRAME,
    AR_SHAPE,
    AR_WIDTH,
    CAPTURE_BACKEND,
    CHECKERBOARD_CORNER_COUNT,
    CHECKERBOARD_SIZE,
    CLOCK_DOMAIN,
    SQUARE_SIZE_MM,
    ArIntrinsics,
    IntrinsicValidationError,
    KernelFrame,
    _append_session,
    _coverage_summary,
    _frame_diagnostic,
    _frame_record,
    _load_dataset,
    _new_dataset,
    _persist_session,
    build_independent_validation_report,
)


def _configuration() -> dict[str, object]:
    return {
        "capture_backend": CAPTURE_BACKEND,
        "timestamp_clock_domain": CLOCK_DOMAIN,
        "device": "/dev/video4",
        "capture_configuration": {"pixel_format": "MJPG", "width": AR_WIDTH, "height": AR_HEIGHT, "fps": AR_FPS},
        "intrinsic": {"path": "/evidence/calibration_fullres.json", "sha256": "a" * 64, "status": "CANDIDATE_NOT_ACTIVATED"},
        "checkerboard": {"inner_corners": list(CHECKERBOARD_SIZE), "square_size_mm": SQUARE_SIZE_MM},
        "reference_frame": AR_REFERENCE_FRAME,
    }


def _persist_fake_session(root: Path, session_id: str, existing: dict[str, object] | None) -> dict[str, object]:
    (root / "sessions" / session_id).mkdir(parents=True)
    frame = KernelFrame(np.zeros(AR_SHAPE, dtype=np.uint8), timestamp_ns=1_000, sequence=3)
    record = _frame_record(session_id=session_id, frame_id=f"{session_id}__frame_000", frame=frame, output_root=root)
    return _persist_session(
        output_root=root,
        session_id=session_id,
        pose_label=f"{session_id}_center",
        configuration=_configuration(),
        frames=[record],
        existing=existing,
    )


def test_append_two_sessions_preserves_first_session_manifest_and_raw_hashes(tmp_path: Path):
    root = tmp_path / "evidence"
    first = _persist_fake_session(root, "session_01", None)
    first_manifest = root / "sessions" / "session_01" / "session_manifest.json"
    before_manifest = first_manifest.read_bytes()
    before_raw = json.loads(before_manifest)["frames"][0]["image"]["file"]["sha256"]

    second = _persist_fake_session(root, "session_02", first)

    assert len(second["sessions"]) == 2
    assert len(second["frames"]) == 2
    assert first_manifest.read_bytes() == before_manifest
    assert _load_dataset(root)["frames"][0]["image"]["file"]["sha256"] == before_raw
    assert (root / "sessions" / "session_01" / "frames" / "session_01__frame_000.png").is_file()


def test_duplicate_session_id_is_fail_closed():
    config = _configuration()
    dataset = _new_dataset(config)
    session = {
        "schema_version": "sie.ar0234_intrinsic_independent_validation_session.v1",
        "status": "INDEPENDENT_INTRINSIC_SESSION_CAPTURED",
        "session_id": "same",
        "operator_pose_label": "center",
        "configuration": config,
        "frame_ids": ["same__frame_000"],
        "frames": [{"frame_id": "same__frame_000", "session_id": "same"}],
        "created_utc": "2026-09-10T00:00:00+00:00",
    }
    reference = {"filename": "sessions/same/session_manifest.json", "sha256": "b" * 64, "bytes": 1}
    dataset = _append_session(dataset, configuration=config, session=session, session_manifest_file=reference)

    with pytest.raises(IntrinsicValidationError, match="duplicate session_id"):
        _append_session(dataset, configuration=config, session=session, session_manifest_file=reference)


def test_offline_report_is_json_safe(tmp_path: Path):
    manifest = tmp_path / "capture_manifest.json"
    intrinsic = tmp_path / "intrinsic.json"
    manifest.write_text(json.dumps(_new_dataset(_configuration())), encoding="utf-8")
    intrinsic.write_text("{}", encoding="utf-8")
    records = [{
        "frame_id": "synthetic", "session_id": "s1", "status": "VALIDATED", "reason": None,
        "pnp_reprojection_px": {"rms_px": 0.25},
        "coverage": {"center_px": [300.0, 200.0], "span_px": [100.0, 80.0], "grid_3x3_bin": {"row": 0, "column": 0}},
    }]
    report = build_independent_validation_report(dataset=_new_dataset(_configuration()), dataset_manifest=manifest, intrinsic=intrinsic, records=records)
    json.dumps(report, allow_nan=False)
    assert report["status"] == "INDEPENDENT_INTRINSIC_VALIDATION_DIAGNOSTIC_COMPLETE"


def test_missing_wrong_dimensions_and_failed_checkerboard_have_explicit_diagnostics(tmp_path: Path):
    intrinsics = ArIntrinsics(matrix=np.eye(3), distortion=np.zeros((1, 5)), sha256="c" * 64)
    missing = _frame_diagnostic(root=tmp_path, frame={"frame_id": "missing", "session_id": "s1", "image": {"file": {"filename": "missing.png", "sha256": "d" * 64}}}, intrinsics=intrinsics)
    assert missing["status"] == "MISSING_RAW_FILE"

    wrong_path = tmp_path / "wrong.png"
    cv2.imwrite(str(wrong_path), np.zeros((10, 10, 3), dtype=np.uint8))
    wrong = _frame_diagnostic(root=tmp_path, frame={"frame_id": "wrong", "session_id": "s1", "image": {"file": {"filename": "wrong.png", "sha256": hashlib.sha256(wrong_path.read_bytes()).hexdigest()}}}, intrinsics=intrinsics)
    assert wrong["status"] == "WRONG_IMAGE_DIMENSIONS"

    blank_path = tmp_path / "blank.png"
    cv2.imwrite(str(blank_path), np.zeros(AR_SHAPE, dtype=np.uint8))
    blank = _frame_diagnostic(root=tmp_path, frame={"frame_id": "blank", "session_id": "s1", "image": {"file": {"filename": "blank.png", "sha256": hashlib.sha256(blank_path.read_bytes()).hexdigest()}}}, intrinsics=intrinsics)
    assert blank["status"] == "CHECKERBOARD_NOT_FOUND"


def test_coverage_bin_aggregation():
    records = []
    for row, column in ((0, 0), (0, 0), (2, 1)):
        records.append({
            "status": "VALIDATED",
            "coverage": {"grid_3x3_bin": {"row": row, "column": column}},
            "pnp_reprojection_px": {"rms_px": 0.2 + row + column},
        })
    summary = _coverage_summary(records)
    by_bin = {(row["row"], row["column"]): row for row in summary["bins"]}
    assert by_bin[(0, 0)]["validated_frame_count"] == 2
    assert by_bin[(2, 1)]["validated_frame_count"] == 1
    assert by_bin[(1, 1)]["validated_frame_count"] == 0
