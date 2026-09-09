from __future__ import annotations

import json

import cv2
import numpy as np
import pytest

from vision_core.ar0234_ov9281_static_extrinsic import (
    CameraIntrinsics,
    OV_COMBINED_SHAPE,
    StaticExtrinsicError,
    StaticPair,
    build_static_transform_record,
    object_points_mm,
    require_acceptable_skew,
    split_physical_left,
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
