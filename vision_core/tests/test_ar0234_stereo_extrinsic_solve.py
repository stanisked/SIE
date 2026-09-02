from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from vision_core.rgb_stereo_extrinsic.solve import (
    ExtrinsicSolveError,
    Intrinsics,
    object_points_mm,
    pnp_relative_transform,
    rectified_from_raw,
    reverse_corners_180,
    rigid_inverse,
    rigid_matrix,
    stereo_calibrate_fixed,
    write_candidate_artifact,
    _project_direction,
    _choose_validation_corner_order,
)


def intrinsics() -> Intrinsics:
    return Intrinsics(np.array([[400.0, 0.0, 320.0], [0.0, 400.0, 240.0], [0.0, 0.0, 1.0]]), np.zeros((1, 5)), "opencv_standard_5", (640, 480))


def image_points(rotation: np.ndarray, translation: np.ndarray, camera: Intrinsics) -> np.ndarray:
    rvec, _ = cv2.Rodrigues(rotation)
    pixels, _ = cv2.projectPoints(object_points_mm(), rvec, translation.reshape(3, 1), camera.camera_matrix, camera.distortion)
    return pixels.astype(np.float64)


def test_object_points_are_row_major_mm() -> None:
    points = object_points_mm()
    assert points.shape == (54, 3)
    assert points[0].tolist() == [0.0, 0.0, 0.0]
    assert points[8].tolist() == [196.0, 0.0, 0.0]
    assert points[9].tolist() == [0.0, 24.5, 0.0]
    assert points[-1].tolist() == [196.0, 122.5, 0.0]


def test_rigid_inverse_and_rectification_chain() -> None:
    raw = rigid_matrix(np.eye(3), np.array([10.0, -3.0, 2.0]))
    inverse = rigid_inverse(raw)
    assert np.allclose(raw @ inverse, np.eye(4))
    r1, _ = cv2.Rodrigues(np.array([0.0, 0.0, np.deg2rad(90.0)]))
    rectified = rectified_from_raw(raw, r1)
    assert np.allclose(rectified[:3, :3], r1)
    assert np.allclose(rectified[:3, 3], r1 @ raw[:3, 3])


def test_reverse_corner_order_is_exact_180_permutation() -> None:
    corners = np.arange(108, dtype=np.float64).reshape(54, 1, 2)
    assert np.array_equal(reverse_corners_180(corners), corners[::-1])


def test_synthetic_relative_transform_direction_and_stereo_calibrate() -> None:
    camera = intrinsics()
    r_ar = np.eye(3)
    t_ar = np.array([0.0, 0.0, 1000.0])
    r_raw, _ = cv2.Rodrigues(np.array([0.0, 0.0, 0.03]))
    t_raw = np.array([80.0, -10.0, 5.0])
    r_left = r_raw @ r_ar
    t_left = r_raw @ t_ar + t_raw
    ar = image_points(r_ar, t_ar, camera)
    left = image_points(r_left, t_left, camera)
    relative = pnp_relative_transform(ar, left, camera, camera)
    assert np.allclose(relative[:3, :3], r_raw, atol=1e-6)
    assert np.allclose(relative[:3, 3], t_raw, atol=1e-5)
    from vision_core.rgb_stereo_extrinsic.solve import PairCorners
    rms, r_fit, t_fit = stereo_calibrate_fixed([PairCorners("pair_000", ar, left)], {"pair_000": left}, camera, camera)
    assert rms < 1e-4
    assert np.allclose(r_fit, r_raw, atol=1e-4)
    assert np.allclose(t_fit, t_raw, atol=1e-3)


def test_holdout_projection_is_exact_for_synthetic_pose() -> None:
    camera = intrinsics()
    raw = rigid_matrix(np.eye(3), np.array([50.0, 0.0, 0.0]))
    ar = image_points(np.eye(3), np.array([0.0, 0.0, 1000.0]), camera)
    left = image_points(np.eye(3), np.array([50.0, 0.0, 1000.0]), camera)
    from vision_core.rgb_stereo_extrinsic.solve import PairCorners, reprojection_metrics
    row = reprojection_metrics(raw, PairCorners("pair_000", ar, left), left, camera, camera)
    assert row["max_px"] < 1e-8


def test_bad_input_sha_blocks_before_json_parse(tmp_path: Path) -> None:
    from vision_core.rgb_stereo_extrinsic.solve import load_ar_intrinsics
    source = tmp_path / "intrinsic.json"
    source.write_text("{}")
    with pytest.raises(ExtrinsicSolveError, match="SHA-256 mismatch"):
        load_ar_intrinsics(source, "0" * 64)


def test_artifact_json_refuses_non_finite_values_and_no_overwrite(tmp_path: Path) -> None:
    target = tmp_path / "candidate.json"
    with pytest.raises(ExtrinsicSolveError, match="non-finite"):
        write_candidate_artifact({"value": float("nan")}, target)
    write_candidate_artifact({"status": "CANDIDATE_OFFLINE_SOLVED", "value": 1.0}, target)
    assert json.loads(target.read_text())["value"] == 1.0
    with pytest.raises(ExtrinsicSolveError, match="refusing to overwrite"):
        write_candidate_artifact({"status": "CANDIDATE_OFFLINE_SOLVED"}, target)


def test_directional_validation_projection_is_exact_and_positive_depth() -> None:
    camera = intrinsics()
    transform = rigid_matrix(np.eye(3), np.array([50.0, 0.0, 0.0]))
    ar = image_points(np.eye(3), np.array([0.0, 0.0, 1000.0]), camera)
    left = image_points(np.eye(3), np.array([50.0, 0.0, 1000.0]), camera)
    result = _project_direction(np.eye(3), np.array([0.0, 0.0, 1000.0]), transform, camera, left)
    assert result["max_px"] < 1e-8
    assert result["source_positive_depth_count"] == 54
    assert result["target_positive_depth_count"] == 54


def test_validation_corner_order_compares_with_candidate() -> None:
    camera = intrinsics()
    raw = rigid_matrix(np.eye(3), np.array([50.0, 0.0, 0.0]))
    ar = image_points(np.eye(3), np.array([0.0, 0.0, 1000.0]), camera)
    left = image_points(np.eye(3), np.array([50.0, 0.0, 1000.0]), camera)
    from vision_core.rgb_stereo_extrinsic.solve import PairCorners
    selected, order, _, translation_delta, rotation_delta = _choose_validation_corner_order(PairCorners("pair_000", ar, left), raw, camera, camera)
    assert order == "as_detected"
    assert np.array_equal(selected, left)
    assert translation_delta < 1e-5
    assert rotation_delta < 1e-5
