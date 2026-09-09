from __future__ import annotations

import json

import cv2
import numpy as np
import pytest

import vision_core.ar0234_ov9281_static_extrinsic as static_extrinsic
from vision_core.ar0234_ov9281_static_extrinsic import (
    CameraIntrinsics,
    OV_COMBINED_SHAPE,
    AR_SHAPE,
    KernelFrame,
    StaticExtrinsicError,
    StaticPair,
    _append_session_to_dataset,
    _capture_counters,
    _classify_static_candidates,
    _new_dataset_manifest,
    _raise_insufficient_static_pairs,
    build_static_transform_record,
    object_points_mm,
    require_acceptable_skew,
    split_physical_left,
)


def _capture_configuration(*, ar_device: str = "/dev/video4") -> dict[str, object]:
    return {
        "capture_backend": "DIRECT_V4L2_MMAP_DQBUF",
        "timestamp_clock_domain": "CLOCK_MONOTONIC",
        "maximum_pair_skew_ns": 5_000_000,
        "devices": {"ar0234": ar_device, "ov9281_combined": "/dev/video2"},
        "capture_configuration": {
            "ar0234": {"width": 1920, "height": 1200, "fps": 30.0, "fourcc": "MJPG"},
            "ov9281_combined": {"width": 2560, "height": 800, "fps": 60.0, "fourcc": "MJPG"},
        },
        "calibrations": {
            "ar0234_intrinsic_candidate": {"sha256": "a" * 64},
            "ov9281_stereo_v6": {"sha256": "b" * 64},
        },
    }


def _session_with_pair(session_id: str) -> tuple[dict[str, object], list[dict[str, object]]]:
    pair_id = f"{session_id}__pair_000"
    pairs = [{"pair_id": pair_id, "session_id": session_id, "skew_ns": 1000}]
    return (
        {"schema_version": "sie.ar0234_ov9281_static_capture_session.v1", "session_id": session_id, "pair_ids": [pair_id]},
        pairs,
    )


def _intrinsics() -> CameraIntrinsics:
    return CameraIntrinsics(
        matrix=np.array([[800.0, 0.0, 640.0], [0.0, 800.0, 400.0], [0.0, 0.0, 1.0]]),
        distortion=np.zeros((1, 5)),
        frame="test_optical_frame",
    )


def _corners(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    pixels, _ = cv2.projectPoints(
        object_points_mm(), rotation, translation.reshape(3, 1),
        _intrinsics().matrix, _intrinsics().distortion,
    )
    return pixels.astype(np.float64)


def _pair(index: int) -> StaticPair:
    ar_rvec = np.array([0.0, 0.0, 0.01 * index])
    left_rvec = np.array([0.0, 0.0, 0.03 + 0.01 * index])
    ar_translation = np.array([10.0 * index, 0.0, 1100.0])
    left_rotation, _ = cv2.Rodrigues(left_rvec)
    ar_rotation, _ = cv2.Rodrigues(ar_rvec)
    relative_translation = np.array([65.0, -4.0, 3.0])
    left_translation = left_rotation @ ar_rotation.T @ ar_translation + relative_translation
    return StaticPair(
        pair_id=f"pair_{index:03d}",
        ar_corners=_corners(ar_rvec, ar_translation),
        ov_left_corners=_corners(left_rvec, left_translation),
        ar_timestamp_ns=1_000_000_000 + index * 30_000_000,
        ov_timestamp_ns=1_000_200_000 + index * 30_000_000,
    )


def test_split_preserves_confirmed_physical_left_semantics():
    combined = np.zeros(OV_COMBINED_SHAPE, dtype=np.uint8)
    combined[:, :1280] = 17
    combined[:, 1280:] = 93
    assert np.all(split_physical_left(combined) == 93)


def test_pair_rejects_skew_over_five_ms():
    with pytest.raises(StaticExtrinsicError, match="exceeds 5 ms"):
        require_acceptable_skew(1_000_000_000, 1_005_000_001)


def test_static_transform_record_is_json_safe_and_provisional():
    record = build_static_transform_record(
        pairs=[_pair(index) for index in range(3)],
        ar_intrinsics=_intrinsics(),
        ov_intrinsics=_intrinsics(),
        provenance={"source": "synthetic"},
    )
    assert record["status"] == "PROVISIONAL_DIAGNOSTIC_ONLY"
    assert record["transform"]["name"] == "T_stereo_left_rgb"
    assert record["quality_metrics"]["inlier_pair_count"] == 3
    json.dumps(record, allow_nan=False, sort_keys=True)


def test_solver_rejects_insufficient_pairs_before_pose_solve():
    with pytest.raises(StaticExtrinsicError, match="at least three"):
        build_static_transform_record(
            pairs=[_pair(index) for index in range(2)],
            ar_intrinsics=_intrinsics(),
            ov_intrinsics=_intrinsics(),
            provenance={"source": "synthetic"},
        )


def test_candidate_counters_distinguish_ar_and_ov_checkerboard_rejections(monkeypatch):
    ar_frames = [
        KernelFrame(np.zeros(AR_SHAPE, dtype=np.uint8), 1_000_000_000 + index, index)
        for index in range(2)
    ]
    ov_frames = [
        KernelFrame(np.zeros(OV_COMBINED_SHAPE, dtype=np.uint8), 1_000_000_000 + index, index)
        for index in range(2)
    ]
    valid_corners = np.zeros((54, 1, 2), dtype=np.float64)
    results = iter((None, valid_corners, None))
    monkeypatch.setattr(static_extrinsic, "_checkerboard", lambda _frame: next(results))

    accepted, counters = _classify_static_candidates(ar_frames, ov_frames)

    assert accepted == []
    assert counters == {
        "candidate_pairs_seen": 2,
        "rejected_skew": 0,
        "rejected_ar_checkerboard_not_found": 1,
        "rejected_ov_left_checkerboard_not_found": 1,
        "rejected_decode_or_frame_error": 0,
        "accepted_pairs": 0,
    }


def test_insufficient_capture_writes_summary_before_raising(tmp_path):
    root = tmp_path / "capture"
    root.mkdir()
    with pytest.raises(StaticExtrinsicError, match="insufficient static valid pairs: 0/3"):
        _raise_insufficient_static_pairs(
            root=root,
            pair_count=3,
            accepted_count=0,
            configuration={"static_target_affirmed": True},
            counters=_capture_counters(),
            preview_frames={"ar0234": {"saved": False}, "ov9281_physical_left": {"saved": False}},
        )

    summary = json.loads((root / "capture_rejection_summary.json").read_text(encoding="utf-8"))
    assert summary["result"] == "INSUFFICIENT_STATIC_VALID_PAIRS"
    assert summary["counters"]["accepted_pairs"] == 0
    assert summary["diagnostic_preview_limitation"].endswith("not calibration pairs.")
    json.dumps(summary, allow_nan=False, sort_keys=True)


def test_append_second_session_preserves_first_session_and_pairs():
    configuration = _capture_configuration()
    first_session, first_pairs = _session_with_pair("pose-a")
    second_session, second_pairs = _session_with_pair("pose-b")
    dataset = _append_session_to_dataset(
        _new_dataset_manifest(configuration),
        configuration=configuration,
        session=first_session,
        pairs=first_pairs,
    )
    dataset = _append_session_to_dataset(
        dataset,
        configuration=configuration,
        session=second_session,
        pairs=second_pairs,
    )

    assert [session["session_id"] for session in dataset["sessions"]] == ["pose-a", "pose-b"]
    assert [pair["pair_id"] for pair in dataset["pairs"]] == ["pose-a__pair_000", "pose-b__pair_000"]
    json.dumps(dataset, allow_nan=False, sort_keys=True)


def test_duplicate_session_id_is_blocked():
    configuration = _capture_configuration()
    session, pairs = _session_with_pair("pose-a")
    dataset = _append_session_to_dataset(
        _new_dataset_manifest(configuration), configuration=configuration, session=session, pairs=pairs
    )

    with pytest.raises(StaticExtrinsicError, match="duplicate session_id"):
        _append_session_to_dataset(dataset, configuration=configuration, session=session, pairs=pairs)


def test_mismatched_dataset_provenance_is_blocked():
    configuration = _capture_configuration()
    session, pairs = _session_with_pair("pose-a")
    dataset = _append_session_to_dataset(
        _new_dataset_manifest(configuration), configuration=configuration, session=session, pairs=pairs
    )
    new_session, new_pairs = _session_with_pair("pose-b")

    with pytest.raises(StaticExtrinsicError, match="capture provenance differs"):
        _append_session_to_dataset(
            dataset,
            configuration=_capture_configuration(ar_device="/dev/video9"),
            session=new_session,
            pairs=new_pairs,
        )
