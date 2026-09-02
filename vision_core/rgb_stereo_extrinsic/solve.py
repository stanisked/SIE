"""Offline AR0234-to-raw-physical-left OV9281 extrinsic candidate solve.

This module consumes immutable paired capture evidence.  It does not open a
camera, rectify images, emit an SIE Measurement, or activate a calibration.
"""
from __future__ import annotations

import hashlib
import itertools
import json
import math
import os
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import cv2
import numpy as np

from vision_core.rgb_stereo_extrinsic.capture import (
    AR_INTRINSIC,
    AR_INTRINSIC_SHA256,
    CHECKERBOARD_CORNERS,
    CHECKERBOARD_SIZE,
    DATASET_ROOT,
    LEFT_SHAPE,
    SQUARE_SIZE_MM,
    STEREO_CALIBRATION,
    STEREO_CALIBRATION_SHA256,
    checkerboard_corners,
)

ARTIFACT_PATH = Path(
    "/home/stanislav/sie_rgb_stereo_fusion/p1_extrinsic/solution_v1/"
    "ar0234_to_stereo_v6_extrinsic_candidate_v1.json"
)
VALIDATION_DATASET_ROOT = Path("/home/stanislav/dev_ws/datasets/ar0234_stereo_extrinsic_validation_v1")
VALIDATION_ARTIFACT_PATH = Path(
    "/home/stanislav/sie_rgb_stereo_fusion/p1_extrinsic/solution_v1/"
    "ar0234_to_stereo_v6_extrinsic_physical_validation_v1.json"
)
TRAINING_PAIR_IDS = tuple(f"pair_{index:03d}" for index in range(9))
HOLDOUT_PAIR_IDS = tuple(f"pair_{index:03d}" for index in range(9, 12))
AR_SHAPE = (1200, 1920, 3)
COMBINED_SHAPE = (800, 2560, 3)


class ExtrinsicSolveError(RuntimeError):
    """Raised when immutable evidence cannot support an offline solve."""


@dataclass(frozen=True)
class PairCorners:
    pair_id: str
    ar: np.ndarray
    stereo_left_raw: np.ndarray


@dataclass(frozen=True)
class Intrinsics:
    camera_matrix: np.ndarray
    distortion: np.ndarray
    model: str
    image_size: tuple[int, int]


def sha256_file(path: Path) -> str:
    if not path.is_file() or path.is_symlink():
        raise ExtrinsicSolveError(f"input is not a regular file: {path}")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def object_points_mm() -> np.ndarray:
    """Return row-major checkerboard points, with x right and y down, in mm."""
    columns, rows = CHECKERBOARD_SIZE
    points = np.zeros((columns * rows, 3), dtype=np.float64)
    points[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
    points[:, :2] *= SQUARE_SIZE_MM
    return points


def reverse_corners_180(corners: np.ndarray) -> np.ndarray:
    if type(corners) is not np.ndarray or corners.shape != (CHECKERBOARD_CORNERS, 1, 2):
        raise ExtrinsicSolveError("corners must have the canonical 54x1x2 shape")
    return corners[::-1].copy()


def rigid_matrix(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    rotation = np.asarray(rotation, dtype=np.float64)
    translation = np.asarray(translation, dtype=np.float64).reshape(3)
    if rotation.shape != (3, 3) or not np.isfinite(rotation).all() or not np.isfinite(translation).all():
        raise ExtrinsicSolveError("invalid rigid transform")
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3], matrix[:3, 3] = rotation, translation
    return matrix


def rigid_inverse(matrix: np.ndarray) -> np.ndarray:
    matrix = np.asarray(matrix, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all() or not np.allclose(matrix[3], (0.0, 0.0, 0.0, 1.0)):
        raise ExtrinsicSolveError("invalid homogeneous rigid matrix")
    rotation, translation = matrix[:3, :3], matrix[:3, 3]
    return rigid_matrix(rotation.T, -rotation.T @ translation)


def rectified_from_raw(raw_from_ar: np.ndarray, r1: np.ndarray) -> np.ndarray:
    raw_from_ar = np.asarray(raw_from_ar, dtype=np.float64)
    r1 = np.asarray(r1, dtype=np.float64)
    if raw_from_ar.shape != (4, 4) or r1.shape != (3, 3):
        raise ExtrinsicSolveError("invalid raw transform or R1")
    return rigid_matrix(r1 @ raw_from_ar[:3, :3], r1 @ raw_from_ar[:3, 3])


def rotation_angle_rad(first: np.ndarray, second: np.ndarray) -> float:
    relative = np.asarray(first, dtype=np.float64) @ np.asarray(second, dtype=np.float64).T
    cosine = float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
    return float(math.acos(cosine))


def _json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as error:
        raise ExtrinsicSolveError(f"invalid JSON: {path}") from error
    if type(value) is not dict:
        raise ExtrinsicSolveError(f"JSON root must be an object: {path}")
    return value


def load_ar_intrinsics(path: Path = AR_INTRINSIC, expected_sha256: str = AR_INTRINSIC_SHA256) -> Intrinsics:
    if sha256_file(path) != expected_sha256:
        raise ExtrinsicSolveError("AR0234 intrinsic SHA-256 mismatch")
    document = _json(path)
    if document.get("model") != "opencv_pinhole_radial_tangential":
        raise ExtrinsicSolveError("AR0234 intrinsic model is not opencv_standard_5-compatible")
    image = document.get("image")
    if type(image) is not dict or (image.get("width"), image.get("height")) != (1920, 1200):
        raise ExtrinsicSolveError("AR0234 intrinsic image size mismatch")
    matrix = np.asarray(document.get("camera_matrix"), dtype=np.float64)
    distortion = np.asarray(document.get("distortion_coefficients"), dtype=np.float64).reshape(1, -1)
    if matrix.shape != (3, 3) or distortion.shape != (1, 5) or not np.isfinite(matrix).all() or not np.isfinite(distortion).all():
        raise ExtrinsicSolveError("invalid AR0234 opencv_standard_5 intrinsics")
    return Intrinsics(matrix, distortion, "opencv_standard_5", (1920, 1200))


def load_stereo_left_intrinsics(path: Path = STEREO_CALIBRATION, expected_sha256: str = STEREO_CALIBRATION_SHA256) -> tuple[Intrinsics, np.ndarray, str]:
    if sha256_file(path) != expected_sha256:
        raise ExtrinsicSolveError("Stereo V6 calibration SHA-256 mismatch")
    try:
        archive = np.load(path, allow_pickle=False)
        k1, d1, r1 = archive["K1"].astype(np.float64), archive["D1"].astype(np.float64), archive["R1"].astype(np.float64)
        size = tuple(int(value) for value in archive["size"].reshape(-1))
        calibration_id = str(archive["calibration_id"].item())
    except (KeyError, OSError, ValueError) as error:
        raise ExtrinsicSolveError("invalid Stereo V6 calibration archive") from error
    if k1.shape != (3, 3) or d1.shape not in ((1, 5), (5, 1)) or r1.shape != (3, 3) or size != (1280, 800):
        raise ExtrinsicSolveError("Stereo V6 left intrinsics or image size mismatch")
    if not np.isfinite(k1).all() or not np.isfinite(d1).all() or not np.isfinite(r1).all():
        raise ExtrinsicSolveError("non-finite Stereo V6 calibration")
    return Intrinsics(k1, d1.reshape(1, 5), "opencv_standard_5", size), r1, calibration_id


def _read_records(dataset_root: Path, expected_pair_ids: Sequence[str]) -> tuple[list[dict[str, Any]], str, str, dict[str, Any]]:
    records_path, manifest_path = dataset_root / "pair_records.jsonl", dataset_root / "capture_manifest.json"
    records_sha, manifest_sha = sha256_file(records_path), sha256_file(manifest_path)
    rows: list[dict[str, Any]] = []
    for line in records_path.read_text().splitlines():
        if not line:
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError as error:
            raise ExtrinsicSolveError("invalid pair records JSONL") from error
        if type(row) is not dict:
            raise ExtrinsicSolveError("pair record must be an object")
        rows.append(row)
    if [row.get("pair_id") for row in rows] != list(expected_pair_ids):
        raise ExtrinsicSolveError(f"dataset pair IDs must be exactly {list(expected_pair_ids)}")
    return rows, records_sha, manifest_sha, _json(manifest_path)


def _load_verified_png(dataset_root: Path, section: str, record: dict[str, Any], shape: tuple[int, int, int]) -> np.ndarray:
    files = record.get("files")
    item = files.get(section) if type(files) is dict else None
    if type(item) is not dict or type(item.get("filename")) is not str or type(item.get("sha256")) is not str or type(item.get("bytes")) is not int:
        raise ExtrinsicSolveError(f"invalid {section} file metadata for {record.get('pair_id')}")
    pair_id = record["pair_id"]
    if item["filename"] != f"{pair_id}.png":
        raise ExtrinsicSolveError(f"filename mismatch for {pair_id}/{section}")
    path = dataset_root / section / item["filename"]
    if not path.is_file() or path.is_symlink() or path.stat().st_size != item["bytes"] or sha256_file(path) != item["sha256"]:
        raise ExtrinsicSolveError(f"digest or byte size mismatch for {pair_id}/{section}")
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if type(image) is not np.ndarray or image.dtype != np.uint8 or image.shape != shape:
        raise ExtrinsicSolveError(f"invalid decoded PNG for {pair_id}/{section}")
    return image


def _load_verified_pairs(dataset_root: Path, expected_pair_ids: Sequence[str]) -> tuple[list[PairCorners], dict[str, Any]]:
    rows, records_sha, manifest_sha, manifest = _read_records(dataset_root, expected_pair_ids)
    pairs: list[PairCorners] = []
    for row in rows:
        if row.get("ar0234_intrinsic", {}).get("sha256") != AR_INTRINSIC_SHA256 or row.get("stereo_v6_calibration", {}).get("sha256") != STEREO_CALIBRATION_SHA256:
            raise ExtrinsicSolveError(f"calibration provenance mismatch in {row['pair_id']}")
        mapping = row.get("combined_frame_mapping")
        expected_mapping = {"combined_left_half": "physical_right", "combined_right_half": "physical_left", "saved_stereo": "physical_left_from_combined_right_half"}
        if mapping != expected_mapping:
            raise ExtrinsicSolveError(f"combined-frame mapping mismatch in {row['pair_id']}")
        ar = _load_verified_png(dataset_root, "ar0234", row, AR_SHAPE)
        left = _load_verified_png(dataset_root, "stereo_left_raw", row, LEFT_SHAPE)
        combined = _load_verified_png(dataset_root, "stereo_combined", row, COMBINED_SHAPE)
        if not np.array_equal(left, combined[:, 1280:]):
            raise ExtrinsicSolveError(f"physical-left split mismatch in {row['pair_id']}")
        ar_corners, left_corners = checkerboard_corners(ar), checkerboard_corners(left)
        if ar_corners is None or left_corners is None:
            raise ExtrinsicSolveError(f"54-corner check failed in {row['pair_id']}")
        pairs.append(PairCorners(row["pair_id"], ar_corners, left_corners))
    manifest_note = None
    if manifest.get("status") != "COMPLETE" or manifest.get("accepted_pair_count") != len(pairs):
        manifest_note = "capture_manifest status/count do not describe the independently verified pair-record JSONL"
    return pairs, {"dataset_root": str(dataset_root), "pair_records_sha256": records_sha, "capture_manifest_sha256": manifest_sha, "capture_manifest_status": manifest.get("status"), "capture_manifest_accepted_pair_count": manifest.get("accepted_pair_count"), "manifest_provenance_note": manifest_note}


def load_verified_pairs(dataset_root: Path = DATASET_ROOT) -> tuple[list[PairCorners], dict[str, Any]]:
    return _load_verified_pairs(dataset_root, tuple(f"pair_{index:03d}" for index in range(12)))


def load_verified_validation_pairs(dataset_root: Path = VALIDATION_DATASET_ROOT) -> tuple[list[PairCorners], dict[str, Any]]:
    return _load_verified_pairs(dataset_root, tuple(f"pair_{index:03d}" for index in range(3)))


def _pnp_object_to_camera(corners: np.ndarray, intrinsics: Intrinsics) -> tuple[np.ndarray, np.ndarray]:
    ok, rvec, translation = cv2.solvePnP(object_points_mm(), corners, intrinsics.camera_matrix, intrinsics.distortion, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise ExtrinsicSolveError("solvePnP failed")
    rotation, _ = cv2.Rodrigues(rvec)
    return rotation.astype(np.float64), translation.reshape(3).astype(np.float64)


def pnp_relative_transform(ar_corners: np.ndarray, left_corners: np.ndarray, ar_intrinsics: Intrinsics, left_intrinsics: Intrinsics) -> np.ndarray:
    """Return raw physical-left from AR0234, expressed in checkerboard mm units."""
    ar_rotation, ar_translation = _pnp_object_to_camera(ar_corners, ar_intrinsics)
    left_rotation, left_translation = _pnp_object_to_camera(left_corners, left_intrinsics)
    rotation = left_rotation @ ar_rotation.T
    translation = left_translation - rotation @ ar_translation
    return rigid_matrix(rotation, translation)


def _pose_medoid(transforms: Sequence[np.ndarray]) -> int:
    if not transforms:
        raise ExtrinsicSolveError("no pose transforms")
    costs: list[float] = []
    for first in transforms:
        costs.append(sum(float(np.linalg.norm(first[:3, 3] - second[:3, 3])) + 100.0 * rotation_angle_rad(first[:3, :3], second[:3, :3]) for second in transforms))
    return int(np.argmin(costs))


def choose_corner_order(pairs: Sequence[PairCorners], ar_intrinsics: Intrinsics, left_intrinsics: Intrinsics) -> tuple[dict[str, np.ndarray], list[dict[str, Any]], list[np.ndarray]]:
    """Choose as-detected/reversed-180 left ordering by global PnP pose consistency.

    The objective is the sum of distances to the transform medoid, with one
    radian weighted as 100 mm.  It is only an ordering decision, not the final
    stereo calibration fit.
    """
    candidates: list[tuple[np.ndarray, np.ndarray]] = []
    for pair in pairs:
        candidates.append((pnp_relative_transform(pair.ar, pair.stereo_left_raw, ar_intrinsics, left_intrinsics), pnp_relative_transform(pair.ar, reverse_corners_180(pair.stereo_left_raw), ar_intrinsics, left_intrinsics)))
    best_bits: tuple[int, ...] | None = None
    best_objective = math.inf
    for bits in itertools.product((0, 1), repeat=len(pairs)):
        transforms = [candidates[index][bit] for index, bit in enumerate(bits)]
        medoid = _pose_medoid(transforms)
        reference = transforms[medoid]
        objective = sum(float(np.linalg.norm(item[:3, 3] - reference[:3, 3])) + 100.0 * rotation_angle_rad(item[:3, :3], reference[:3, :3]) for item in transforms)
        if objective < best_objective:
            best_bits, best_objective = bits, objective
    assert best_bits is not None
    selected = {pair.pair_id: (pair.stereo_left_raw if bit == 0 else reverse_corners_180(pair.stereo_left_raw)) for pair, bit in zip(pairs, best_bits)}
    transforms = [candidates[index][bit] for index, bit in enumerate(best_bits)]
    medoid = transforms[_pose_medoid(transforms)]
    decisions = []
    for pair, bit, transform in zip(pairs, best_bits, transforms):
        decisions.append({"pair_id": pair.pair_id, "stereo_left_corner_order": "as_detected" if bit == 0 else "reversed_180", "selection_method": "global_minimum_pnp_relative_pose_inconsistency", "relative_to_medoid_translation_mm": float(np.linalg.norm(transform[:3, 3] - medoid[:3, 3])), "relative_to_medoid_rotation_deg": float(math.degrees(rotation_angle_rad(transform[:3, :3], medoid[:3, :3]))), "global_objective_mm_equivalent": float(best_objective)})
    return selected, decisions, transforms


def stereo_calibrate_fixed(pairs: Sequence[PairCorners], selected_left: dict[str, np.ndarray], ar_intrinsics: Intrinsics, left_intrinsics: Intrinsics) -> tuple[float, np.ndarray, np.ndarray]:
    if not pairs:
        raise ExtrinsicSolveError("stereoCalibrate requires at least one pair")
    points = object_points_mm().astype(np.float32)
    object_views = [points for _ in pairs]
    ar_views = [pair.ar.astype(np.float32) for pair in pairs]
    left_views = [selected_left[pair.pair_id].astype(np.float32) for pair in pairs]
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-12)
    result = cv2.stereoCalibrate(object_views, ar_views, left_views, ar_intrinsics.camera_matrix.copy(), ar_intrinsics.distortion.copy(), left_intrinsics.camera_matrix.copy(), left_intrinsics.distortion.copy(), ar_intrinsics.image_size, criteria=criteria, flags=cv2.CALIB_FIX_INTRINSIC)
    rms, rotation, translation = float(result[0]), np.asarray(result[5], dtype=np.float64), np.asarray(result[6], dtype=np.float64).reshape(3)
    if not math.isfinite(rms) or rotation.shape != (3, 3) or translation.shape != (3,) or not np.isfinite(rotation).all() or not np.isfinite(translation).all():
        raise ExtrinsicSolveError("stereoCalibrate returned invalid transform")
    return rms, rotation, translation


def reprojection_metrics(transform_raw_from_ar: np.ndarray, pair: PairCorners, selected_left: np.ndarray, ar_intrinsics: Intrinsics, left_intrinsics: Intrinsics) -> dict[str, Any]:
    ar_rotation, ar_translation = _pnp_object_to_camera(pair.ar, ar_intrinsics)
    object_mm = object_points_mm()
    ar_points = (ar_rotation @ object_mm.T + ar_translation.reshape(3, 1)).T
    raw_points = (transform_raw_from_ar[:3, :3] @ ar_points.T + transform_raw_from_ar[:3, 3:4]).T
    projected, _ = cv2.projectPoints(raw_points, np.zeros((3, 1), dtype=np.float64), np.zeros((3, 1), dtype=np.float64), left_intrinsics.camera_matrix, left_intrinsics.distortion)
    errors = np.linalg.norm(projected.reshape(-1, 2) - selected_left.reshape(-1, 2), axis=1)
    return {"pair_id": pair.pair_id, "corner_count": int(errors.size), "mean_px": float(np.mean(errors)), "median_px": float(np.median(errors)), "rms_px": float(np.sqrt(np.mean(errors * errors))), "p95_px": float(np.percentile(errors, 95)), "max_px": float(np.max(errors)), "ar_positive_depth_count": int(np.count_nonzero(ar_points[:, 2] > 0.0)), "stereo_left_raw_positive_depth_count": int(np.count_nonzero(raw_points[:, 2] > 0.0))}


def summarize_metrics(rows: Sequence[dict[str, Any]]) -> dict[str, float]:
    errors: list[float] = []
    for row in rows:
        # The per-view fields are summaries; holdout aggregate is intentionally
        # recomputed from all corners by the caller where exact values matter.
        errors.append(float(row["mean_px"]))
    values = np.asarray(errors, dtype=np.float64)
    return {"view_mean_mean_px": float(np.mean(values)), "view_mean_median_px": float(np.median(values)), "view_mean_p95_px": float(np.percentile(values, 95)), "view_mean_max_px": float(np.max(values))}


def transform_dispersion(transforms: Sequence[np.ndarray]) -> dict[str, float]:
    medoid = transforms[_pose_medoid(transforms)]
    translation = np.asarray([np.linalg.norm(item[:3, 3] - medoid[:3, 3]) for item in transforms], dtype=np.float64)
    rotation = np.asarray([math.degrees(rotation_angle_rad(item[:3, :3], medoid[:3, :3])) for item in transforms], dtype=np.float64)
    return {"reference": "PnP relative-transform medoid", "translation_mm_median": float(np.median(translation)), "translation_mm_p95": float(np.percentile(translation, 95)), "translation_mm_max": float(np.max(translation)), "rotation_deg_median": float(np.median(rotation)), "rotation_deg_p95": float(np.percentile(rotation, 95)), "rotation_deg_max": float(np.max(rotation))}


def _matrix(value: np.ndarray) -> list[list[float]]:
    return np.asarray(value, dtype=np.float64).tolist()


def _assert_json_finite(value: Any) -> None:
    if type(value) is float and not math.isfinite(value):
        raise ExtrinsicSolveError("artifact contains non-finite number")
    if type(value) is dict:
        for item in value.values():
            _assert_json_finite(item)
    elif type(value) is list:
        for item in value:
            _assert_json_finite(item)


def _error_statistics(errors: np.ndarray) -> dict[str, Any]:
    errors = np.asarray(errors, dtype=np.float64).reshape(-1)
    if errors.size == 0 or not np.isfinite(errors).all():
        raise ExtrinsicSolveError("invalid reprojection errors")
    return {
        "corner_count": int(errors.size),
        "mean_px": float(np.mean(errors)),
        "median_px": float(np.median(errors)),
        "rms_px": float(np.sqrt(np.mean(errors * errors))),
        "p95_px": float(np.percentile(errors, 95)),
        "max_px": float(np.max(errors)),
    }


def _project_direction(
    object_to_source_rotation: np.ndarray,
    object_to_source_translation_mm: np.ndarray,
    transform_target_from_source_mm: np.ndarray,
    target_intrinsics: Intrinsics,
    observed_target_corners: np.ndarray,
) -> dict[str, Any]:
    source_points = (object_to_source_rotation @ object_points_mm().T + object_to_source_translation_mm.reshape(3, 1)).T
    target_points = (transform_target_from_source_mm[:3, :3] @ source_points.T + transform_target_from_source_mm[:3, 3:4]).T
    projected, _ = cv2.projectPoints(
        target_points,
        np.zeros((3, 1), dtype=np.float64),
        np.zeros((3, 1), dtype=np.float64),
        target_intrinsics.camera_matrix,
        target_intrinsics.distortion,
    )
    statistics = _error_statistics(np.linalg.norm(projected.reshape(-1, 2) - observed_target_corners.reshape(-1, 2), axis=1))
    statistics["source_positive_depth_count"] = int(np.count_nonzero(source_points[:, 2] > 0.0))
    statistics["target_positive_depth_count"] = int(np.count_nonzero(target_points[:, 2] > 0.0))
    return statistics


def _candidate_matrix_mm(candidate: dict[str, Any], key: str, expected_name: str) -> np.ndarray:
    node = candidate.get(key)
    if type(node) is not dict or node.get("name") != expected_name:
        raise ExtrinsicSolveError(f"candidate lacks {expected_name}")
    matrix = np.asarray(node.get("matrix_4x4"), dtype=np.float64)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ExtrinsicSolveError(f"candidate {expected_name} matrix is invalid")
    if not np.allclose(matrix[3], (0.0, 0.0, 0.0, 1.0)):
        raise ExtrinsicSolveError(f"candidate {expected_name} is not homogeneous")
    return rigid_matrix(matrix[:3, :3], matrix[:3, 3] * 1000.0)


def load_candidate(candidate_path: Path = ARTIFACT_PATH, expected_sha256: str | None = None) -> tuple[dict[str, Any], np.ndarray, np.ndarray]:
    expected_sha256 = expected_sha256 or "0d9b4a2f4110eecbc4994eddc0f7be267868520920988cfb426fdf034041689e"
    if sha256_file(candidate_path) != expected_sha256:
        raise ExtrinsicSolveError("candidate artifact SHA-256 mismatch")
    candidate = _json(candidate_path)
    if candidate.get("status") != "CANDIDATE_OFFLINE_SOLVED" or candidate.get("source_frame") != "ar0234_optical_frame" or candidate.get("target_frame") != "stereo_left_raw_optical_frame" or candidate.get("translation_units") != "m":
        raise ExtrinsicSolveError("candidate artifact contract mismatch")
    raw = _candidate_matrix_mm(candidate, "raw_transform", "T_stereo_left_raw_from_ar0234")
    inverse = _candidate_matrix_mm(candidate, "raw_inverse_transform", "T_ar0234_from_stereo_left_raw")
    if not np.allclose(inverse, rigid_inverse(raw), atol=1e-9):
        raise ExtrinsicSolveError("candidate stored inverse does not match rigid inverse")
    rotation = raw[:3, :3]
    if abs(float(np.linalg.det(rotation)) - 1.0) > 1e-6 or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise ExtrinsicSolveError("candidate rotation is not rigid")
    return candidate, raw, inverse


def _choose_validation_corner_order(
    pair: PairCorners,
    candidate_raw_from_ar_mm: np.ndarray,
    ar_intrinsics: Intrinsics,
    left_intrinsics: Intrinsics,
) -> tuple[np.ndarray, str, np.ndarray, float, float]:
    options = (("as_detected", pair.stereo_left_raw), ("reversed_180", reverse_corners_180(pair.stereo_left_raw)))
    scored = []
    for label, corners in options:
        relative = pnp_relative_transform(pair.ar, corners, ar_intrinsics, left_intrinsics)
        translation_delta = float(np.linalg.norm(relative[:3, 3] - candidate_raw_from_ar_mm[:3, 3]))
        rotation_delta = float(math.degrees(rotation_angle_rad(relative[:3, :3], candidate_raw_from_ar_mm[:3, :3])))
        scored.append((translation_delta + 100.0 * math.radians(rotation_delta), label, corners, relative, translation_delta, rotation_delta))
    _, label, corners, relative, translation_delta, rotation_delta = min(scored, key=lambda item: item[0])
    return corners, label, relative, translation_delta, rotation_delta


def validate_candidate(
    dataset_root: Path = VALIDATION_DATASET_ROOT,
    candidate_path: Path = ARTIFACT_PATH,
    candidate_sha256: str | None = None,
) -> dict[str, Any]:
    candidate, raw_from_ar_mm, ar_from_raw_mm = load_candidate(candidate_path, candidate_sha256)
    ar_intrinsics = load_ar_intrinsics()
    left_intrinsics, _, stereo_calibration_id = load_stereo_left_intrinsics()
    pairs, dataset_provenance = load_verified_validation_pairs(dataset_root)
    per_pair: list[dict[str, Any]] = []
    all_errors: list[float] = []
    translation_deltas: list[float] = []
    rotation_deltas: list[float] = []
    for pair in pairs:
        left_corners, order, relative, translation_delta, rotation_delta = _choose_validation_corner_order(pair, raw_from_ar_mm, ar_intrinsics, left_intrinsics)
        ar_rotation, ar_translation = _pnp_object_to_camera(pair.ar, ar_intrinsics)
        left_rotation, left_translation = _pnp_object_to_camera(left_corners, left_intrinsics)
        ar_to_left = _project_direction(ar_rotation, ar_translation, raw_from_ar_mm, left_intrinsics, left_corners)
        left_to_ar = _project_direction(left_rotation, left_translation, ar_from_raw_mm, ar_intrinsics, pair.ar)
        # Recompute exact corner errors for the aggregate rather than combining summaries.
        for rotation, translation, transform, intrinsics, observed in ((ar_rotation, ar_translation, raw_from_ar_mm, left_intrinsics, left_corners), (left_rotation, left_translation, ar_from_raw_mm, ar_intrinsics, pair.ar)):
            source_points = (rotation @ object_points_mm().T + translation.reshape(3, 1)).T
            target_points = (transform[:3, :3] @ source_points.T + transform[:3, 3:4]).T
            projected, _ = cv2.projectPoints(target_points, np.zeros((3, 1)), np.zeros((3, 1)), intrinsics.camera_matrix, intrinsics.distortion)
            all_errors.extend(np.linalg.norm(projected.reshape(-1, 2) - observed.reshape(-1, 2), axis=1).tolist())
        translation_deltas.append(translation_delta)
        rotation_deltas.append(rotation_delta)
        per_pair.append({
            "pair_id": pair.pair_id,
            "checkerboard": {"ar0234_corner_count": int(len(pair.ar)), "stereo_left_raw_corner_count": int(len(left_corners)), "stereo_left_corner_order": order},
            "ar0234_to_stereo_left_raw": ar_to_left,
            "stereo_left_raw_to_ar0234": left_to_ar,
            "independent_pnp_relative_transform": {"T_stereo_left_raw_from_ar0234_mm": _matrix(relative), "translation_delta_from_candidate_mm": translation_delta, "rotation_delta_from_candidate_deg": rotation_delta},
        })
    aggregate = _error_statistics(np.asarray(all_errors, dtype=np.float64))
    thresholds = {"aggregate_median_px_max": 1.5, "aggregate_p95_px_max": 3.0, "aggregate_max_px_max": 5.0, "rotation_delta_max_deg": 1.5, "translation_delta_max_mm": 30.0, "required_corner_count": 54, "required_positive_depth_count": 54}
    corners_ok = all(row["checkerboard"]["ar0234_corner_count"] == 54 and row["checkerboard"]["stereo_left_raw_corner_count"] == 54 for row in per_pair)
    depths_ok = all(row[direction]["source_positive_depth_count"] == 54 and row[direction]["target_positive_depth_count"] == 54 for row in per_pair for direction in ("ar0234_to_stereo_left_raw", "stereo_left_raw_to_ar0234"))
    rigid = candidate["rigid_rotation_check"]
    gate = corners_ok and depths_ok and aggregate["median_px"] <= thresholds["aggregate_median_px_max"] and aggregate["p95_px"] <= thresholds["aggregate_p95_px_max"] and aggregate["max_px"] <= thresholds["aggregate_max_px_max"] and max(rotation_deltas) <= thresholds["rotation_delta_max_deg"] and max(translation_deltas) <= thresholds["translation_delta_max_mm"] and rigid.get("determinant_near_one") is True and rigid.get("orthonormal_near_identity") is True
    artifact = {
        "schema_version": "sie.ar0234_stereo_extrinsic_physical_validation.v1",
        "status": "PHYSICAL_EXTRINSIC_VALIDATION_PASS" if gate else "PHYSICAL_EXTRINSIC_VALIDATION_FAIL",
        "candidate": {"path": str(candidate_path), "sha256": candidate_sha256 or "0d9b4a2f4110eecbc4994eddc0f7be267868520920988cfb426fdf034041689e", "calibration_id": candidate["calibration_id"]},
        "validation_dataset": {**dataset_provenance, "pair_ids": [pair.pair_id for pair in pairs]},
        "stereo_v6_calibration": {"path": str(STEREO_CALIBRATION), "sha256": STEREO_CALIBRATION_SHA256, "calibration_id": stereo_calibration_id},
        "ar0234_intrinsic": {"path": str(AR_INTRINSIC), "sha256": AR_INTRINSIC_SHA256, "model": "opencv_standard_5"},
        "no_refit_performed": True,
        "per_pair": per_pair,
        "aggregate_reprojection": aggregate,
        "pose_deltas_from_candidate": {"translation_mm": {"per_pair": translation_deltas, "max": float(max(translation_deltas)), "median": float(np.median(translation_deltas))}, "rotation_deg": {"per_pair": rotation_deltas, "max": float(max(rotation_deltas)), "median": float(np.median(rotation_deltas))}},
        "candidate_rigid_rotation_check": candidate["rigid_rotation_check"],
        "acceptance_thresholds": thresholds,
        "acceptance_evaluation": {"all_checkerboards_54_of_54": corners_ok, "all_directional_positive_depths_54_of_54": depths_ok, "aggregate_reprojection_gate": aggregate["median_px"] <= thresholds["aggregate_median_px_max"] and aggregate["p95_px"] <= thresholds["aggregate_p95_px_max"] and aggregate["max_px"] <= thresholds["aggregate_max_px_max"], "pose_delta_gate": max(rotation_deltas) <= thresholds["rotation_delta_max_deg"] and max(translation_deltas) <= thresholds["translation_delta_max_mm"], "candidate_rotation_rigid": rigid.get("determinant_near_one") is True and rigid.get("orthonormal_near_identity") is True},
        "software": {"opencv": cv2.__version__, "numpy": np.__version__, "python": sys.version.split()[0]},
        "limitation": "Offline validation on three independent static checkerboard views. No runtime, human ROI/depth fusion, thermal challenge, or motion validation was performed.",
    }
    _assert_json_finite(artifact)
    return artifact


def solve_candidate(dataset_root: Path = DATASET_ROOT, ar_path: Path = AR_INTRINSIC, stereo_path: Path = STEREO_CALIBRATION) -> dict[str, Any]:
    ar_intrinsics = load_ar_intrinsics(ar_path)
    left_intrinsics, r1, stereo_calibration_id = load_stereo_left_intrinsics(stereo_path)
    pairs, dataset_provenance = load_verified_pairs(dataset_root)
    selected_left, decisions, pnp_transforms = choose_corner_order(pairs, ar_intrinsics, left_intrinsics)
    training = [pair for pair in pairs if pair.pair_id in TRAINING_PAIR_IDS]
    holdout = [pair for pair in pairs if pair.pair_id in HOLDOUT_PAIR_IDS]
    if [pair.pair_id for pair in training] != list(TRAINING_PAIR_IDS) or [pair.pair_id for pair in holdout] != list(HOLDOUT_PAIR_IDS):
        raise ExtrinsicSolveError("training/holdout split mismatch")
    training_rms, training_rotation, training_translation_mm = stereo_calibrate_fixed(training, selected_left, ar_intrinsics, left_intrinsics)
    training_raw_from_ar_mm = rigid_matrix(training_rotation, training_translation_mm)
    holdout_rows = [reprojection_metrics(training_raw_from_ar_mm, pair, selected_left[pair.pair_id], ar_intrinsics, left_intrinsics) for pair in holdout]
    holdout_corner_errors = []
    for pair in holdout:
        ar_rotation, ar_translation = _pnp_object_to_camera(pair.ar, ar_intrinsics)
        ar_points = (ar_rotation @ object_points_mm().T + ar_translation.reshape(3, 1)).T
        raw_points = (training_rotation @ ar_points.T + training_translation_mm.reshape(3, 1)).T
        projected, _ = cv2.projectPoints(raw_points, np.zeros((3, 1)), np.zeros((3, 1)), left_intrinsics.camera_matrix, left_intrinsics.distortion)
        holdout_corner_errors.extend(np.linalg.norm(projected.reshape(-1, 2) - selected_left[pair.pair_id].reshape(-1, 2), axis=1).tolist())
    holdout_errors = np.asarray(holdout_corner_errors, dtype=np.float64)
    final_rms, final_rotation, final_translation_mm = stereo_calibrate_fixed(pairs, selected_left, ar_intrinsics, left_intrinsics)
    final_raw_from_ar_mm = rigid_matrix(final_rotation, final_translation_mm)
    final_raw_from_ar_m = rigid_matrix(final_rotation, final_translation_mm / 1000.0)
    final_rect_from_ar_m = rectified_from_raw(final_raw_from_ar_m, r1)
    determinant = float(np.linalg.det(final_rotation))
    orthogonality = float(np.max(np.abs(final_rotation.T @ final_rotation - np.eye(3))))
    final_per_view = [reprojection_metrics(final_raw_from_ar_mm, pair, selected_left[pair.pair_id], ar_intrinsics, left_intrinsics) for pair in pairs]
    artifact: dict[str, Any] = {
        "schema_version": "sie.ar0234_stereo_extrinsic_candidate.v1",
        "status": "CANDIDATE_OFFLINE_SOLVED",
        "calibration_id": "ar0234_to_stereo_v6_extrinsic_candidate_v1",
        "calibration_version": "v1",
        "source_frame": "ar0234_optical_frame",
        "target_frame": "stereo_left_raw_optical_frame",
        "coordinate_convention": {"standard": "OpenCV", "x": "right", "y": "down", "z": "forward"},
        "translation_units": "m",
        "raw_transform": {"name": "T_stereo_left_raw_from_ar0234", "matrix_4x4": _matrix(final_raw_from_ar_m), "rotation_3x3": _matrix(final_rotation), "translation_m": [float(item) for item in final_translation_mm / 1000.0]},
        "raw_inverse_transform": {"name": "T_ar0234_from_stereo_left_raw", "matrix_4x4": _matrix(rigid_inverse(final_raw_from_ar_m))},
        "rectified_transform": {"name": "T_rectified_left_from_ar0234", "source_frame": "ar0234_optical_frame", "target_frame": "rectified_left_optical_frame", "matrix_4x4": _matrix(final_rect_from_ar_m), "rotation_3x3": _matrix(final_rect_from_ar_m[:3, :3]), "translation_m": [float(item) for item in final_rect_from_ar_m[:3, 3]], "chain": "R_rectified_left_from_ar0234 = R1 @ R_stereo_left_raw_from_ar0234; t_rectified_left_from_ar0234 = R1 @ t_stereo_left_raw_from_ar0234"},
        "rectified_inverse_transform": {"name": "T_ar0234_from_rectified_left", "matrix_4x4": _matrix(rigid_inverse(final_rect_from_ar_m))},
        "inputs": {"ar0234_intrinsic": {"path": str(ar_path), "sha256": AR_INTRINSIC_SHA256, "model": "opencv_standard_5", "image_size": [1920, 1200]}, "stereo_v6": {"path": str(stereo_path), "sha256": STEREO_CALIBRATION_SHA256, "calibration_id": stereo_calibration_id, "raw_physical_left": {"K1": _matrix(left_intrinsics.camera_matrix), "D1": _matrix(left_intrinsics.distortion), "image_size": [1280, 800]}, "rectification": {"R1": _matrix(r1), "P1": _matrix(np.load(stereo_path, allow_pickle=False)["P1"])}}, "dataset": dataset_provenance},
        "checkerboard": {"type": "checkerboard", "inner_corners": [9, 6], "square_size": 24.5, "units_during_solve": "mm", "object_point_order": "row-major: x=0..8 then y=0..5"},
        "pair_ids": [pair.pair_id for pair in pairs],
        "corner_order_decisions": decisions,
        "split": {"training_pair_ids": list(TRAINING_PAIR_IDS), "holdout_pair_ids": list(HOLDOUT_PAIR_IDS)},
        "training_solve": {"stereoCalibrate_flags": ["CALIB_FIX_INTRINSIC"], "camera1": "AR0234", "camera2": "raw physical-left OV9281", "rms_px": training_rms, "T_stereo_left_raw_from_ar0234_mm": _matrix(training_raw_from_ar_mm)},
        "holdout_validation": {"method": "solvePnP in AR0234, transform to raw physical-left, project with K1/D1", "per_view": holdout_rows, "all_holdout_corners": {"count": int(holdout_errors.size), "mean_px": float(np.mean(holdout_errors)), "median_px": float(np.median(holdout_errors)), "rms_px": float(np.sqrt(np.mean(holdout_errors * holdout_errors))), "p95_px": float(np.percentile(holdout_errors, 95)), "max_px": float(np.max(holdout_errors))}},
        "final_refit_all_12": {"stereoCalibrate_rms_px": final_rms, "per_view_reprojection": final_per_view, "per_view_summary": summarize_metrics(final_per_view)},
        "pnp_relative_transform_dispersion": transform_dispersion(pnp_transforms),
        "rigid_rotation_check": {"determinant": determinant, "max_abs_rt_r_minus_i": orthogonality, "determinant_near_one": bool(abs(determinant - 1.0) < 1e-6), "orthonormal_near_identity": bool(orthogonality < 1e-6)},
        "cheirality": {"training_and_holdout_positive_depth_required": 54, "per_view": [{"pair_id": row["pair_id"], "ar_positive_depth_count": row["ar_positive_depth_count"], "stereo_left_raw_positive_depth_count": row["stereo_left_raw_positive_depth_count"]} for row in final_per_view]},
        "optical_center_baseline_magnitude_m": float(np.linalg.norm(final_translation_mm) / 1000.0),
        "software": {"opencv": cv2.__version__, "numpy": np.__version__, "python": sys.version.split()[0]},
        "limitation": "Offline geometric candidate only. Physical extrinsic validation has not yet been completed; this candidate is not ACTIVE and is not production-ready.",
    }
    _assert_json_finite(artifact)
    return artifact


def write_candidate_artifact(artifact: dict[str, Any], path: Path = ARTIFACT_PATH) -> None:
    _assert_json_finite(artifact)
    if path.exists() or path.is_symlink():
        raise ExtrinsicSolveError(f"refusing to overwrite candidate artifact: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = (json.dumps(artifact, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()
    fd, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, path)
        directory = os.open(path.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    except FileExistsError as error:
        raise ExtrinsicSolveError(f"refusing to overwrite candidate artifact: {path}") from error
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
