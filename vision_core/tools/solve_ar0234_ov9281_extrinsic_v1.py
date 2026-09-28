#!/usr/bin/env python3
"""Solve a non-active AR0234-v5 to OV9281-left extrinsic calibration candidate."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def json_default(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(type(value).__name__)


def find_corners(image: np.ndarray, board: tuple[int, int]) -> np.ndarray | None:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCornersSB(gray, board, flags=flags)
    return None if not found or corners is None else corners.astype(np.float32)


def object_points(board: tuple[int, int], square_size_mm: float) -> np.ndarray:
    cols, rows = board
    values = np.zeros((cols * rows, 3), dtype=np.float32)
    values[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    values *= square_size_mm
    return values


def rms_px(observed: np.ndarray, projected: np.ndarray) -> float:
    error = observed.reshape(-1, 2).astype(np.float64) - projected.reshape(-1, 2).astype(np.float64)
    return float(np.sqrt(np.mean(np.sum(error * error, axis=1))))


def stats(values: list[float]) -> dict[str, float]:
    array = np.asarray(values, dtype=np.float64)
    return {
        "count": int(len(array)),
        "min": float(np.min(array)),
        "median": float(np.median(array)),
        "p95": float(np.percentile(array, 95)),
        "max": float(np.max(array)),
    }


def load_ar_intrinsic(path: Path) -> tuple[np.ndarray, np.ndarray, tuple[int, int], tuple[int, int], float]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("schema") != "sie.camera_intrinsic.calibration.fullres.v2":
        raise RuntimeError("Unexpected AR0234 intrinsic schema")
    if payload.get("camera") != "AR0234":
        raise RuntimeError("Unexpected AR0234 intrinsic camera")
    image = payload.get("image", {})
    checkerboard = payload.get("checkerboard", {})
    K = np.asarray(payload.get("camera_matrix"), dtype=np.float64)
    D = np.asarray(payload.get("distortion_coefficients"), dtype=np.float64).reshape(1, -1)
    size = (int(image.get("width")), int(image.get("height")))
    board = tuple(int(value) for value in checkerboard.get("inner_corners", []))
    square_size_mm = float(checkerboard.get("square_size_mm"))
    if K.shape != (3, 3) or D.shape != (1, 5) or len(board) != 2:
        raise RuntimeError("Malformed AR0234 intrinsic calibration")
    return K, D, size, board, square_size_mm


def load_sessions(
    roots: list[Path],
    board: tuple[int, int],
    ar_size: tuple[int, int],
    eye_size: tuple[int, int],
    expected_ar_intrinsic_sha: str,
) -> tuple[list[dict[str, Any]], dict[str, str]]:
    pairs: list[dict[str, Any]] = []
    metadata_hashes: dict[str, str] = {}
    for root in roots:
        metadata_path = root / "capture_metadata.json"
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        if metadata.get("status") != "COMPLETE":
            raise RuntimeError(f"Capture is not complete: {root}")
        if tuple(metadata["target"]["checkerboard_inner_corners"]) != board:
            raise RuntimeError(f"Checkerboard mismatch: {root}")
        if float(metadata["target"]["square_size_mm"]) != 24.5:
            raise RuntimeError(f"Unexpected square size: {root}")
        if metadata["ar0234"]["intrinsic_sha256"] != expected_ar_intrinsic_sha:
            raise RuntimeError(f"AR0234 intrinsic SHA mismatch: {root}")
        if (metadata["ar0234"]["image_size_px"]["width"], metadata["ar0234"]["image_size_px"]["height"]) != ar_size:
            raise RuntimeError(f"AR0234 size mismatch: {root}")
        if (metadata["ov9281_stereo"]["eye_image_size_px"]["width"], metadata["ov9281_stereo"]["eye_image_size_px"]["height"]) != eye_size:
            raise RuntimeError(f"OV9281 eye size mismatch: {root}")
        metadata_hashes[str(root)] = sha256_file(metadata_path)
        for pair in metadata["pairs"]:
            pairs.append({"root": root, **pair})
    return pairs, metadata_hashes


def read_image(path: Path, size: tuple[int, int]) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None or image.shape[:2] != (size[1], size[0]):
        raise RuntimeError(f"Invalid image: {path}")
    return image


def pair_reprojection_errors(
    obj: np.ndarray,
    ar: np.ndarray,
    left: np.ndarray,
    right: np.ndarray,
    K_ar: np.ndarray, D_ar: np.ndarray,
    K_left: np.ndarray, D_left: np.ndarray,
    K_right: np.ndarray, D_right: np.ndarray,
    R_ar_left: np.ndarray, T_ar_left: np.ndarray,
    R_left_right: np.ndarray, T_left_right: np.ndarray,
) -> tuple[float, float]:
    ok, rvec_left, tvec_left = cv2.solvePnP(obj, left, K_left, D_left, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise RuntimeError("solvePnP failed for stereo left")
    R_left_board, _ = cv2.Rodrigues(rvec_left)
    R_left_ar = R_ar_left.T
    T_left_ar = -R_ar_left.T @ T_ar_left
    R_ar_board = R_left_ar @ R_left_board
    T_ar_board = R_left_ar @ tvec_left + T_left_ar
    rvec_ar, _ = cv2.Rodrigues(R_ar_board)
    predicted_ar, _ = cv2.projectPoints(obj, rvec_ar, T_ar_board, K_ar, D_ar)
    ar_error = rms_px(ar, predicted_ar)

    R_right_board = R_left_right @ R_left_board
    T_right_board = R_left_right @ tvec_left + T_left_right
    rvec_right, _ = cv2.Rodrigues(R_right_board)
    predicted_right, _ = cv2.projectPoints(obj, rvec_right, T_right_board, K_right, D_right)
    right_error = rms_px(right, predicted_right)
    return ar_error, right_error


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path, action="append", required=True)
    parser.add_argument("--capture-audit", type=Path, required=True)
    parser.add_argument("--ar-intrinsic", type=Path, required=True)
    parser.add_argument("--stereo-calibration", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--minimum-pairs", type=int, default=40)
    parser.add_argument(
        "--captured-second-is-physical-left", action="store_true",
        help="Use stereo_right/*.png as physical_left and stereo_left/*.png as physical_right.",
    )
    args = parser.parse_args()

    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        raise RuntimeError(f"Output directory is not empty: {args.output_dir}")
    audit = json.loads(args.capture_audit.read_text(encoding="utf-8"))
    if audit.get("result") != "PASS":
        raise RuntimeError("Capture audit must PASS before solving")

    K_ar, D_ar, ar_size, board, square_size_mm = load_ar_intrinsic(args.ar_intrinsic)
    ar_sha = sha256_file(args.ar_intrinsic)
    with np.load(args.stereo_calibration, allow_pickle=False) as stereo:
        K_left, D_left = stereo["K1"].astype(np.float64), stereo["D1"].astype(np.float64)
        K_right, D_right = stereo["K2"].astype(np.float64), stereo["D2"].astype(np.float64)
        R_left_right, T_left_right = stereo["R"].astype(np.float64), stereo["T"].astype(np.float64)
        eye_size = tuple(int(value) for value in stereo["size"])
        calibration_id = str(stereo["calibration_id"].item())
        left_semantics = str(stereo["camera_1_semantics"].item())
        right_semantics = str(stereo["camera_2_semantics"].item())
    if eye_size != (1280, 800) or left_semantics != "physical_left" or right_semantics != "physical_right":
        raise RuntimeError("Stereo calibration is not the expected OV9281 physical-left/right model")

    roots = [path.resolve() for path in args.dataset_root]
    pairs, metadata_hashes = load_sessions(roots, board, ar_size, eye_size, ar_sha)
    if len(pairs) < args.minimum_pairs:
        raise RuntimeError(f"Too few input pairs: {len(pairs)}")
    obj = object_points(board, square_size_mm)

    object_sets, ar_sets, left_sets, right_sets, records = [], [], [], [], []
    for pair in pairs:
        root, filename = Path(pair["root"]), str(pair["filename"])
        ar = read_image(root / "ar0234" / filename, ar_size)
        captured_first = read_image(root / "stereo_left" / filename, eye_size)
        captured_second = read_image(root / "stereo_right" / filename, eye_size)
        if args.captured_second_is_physical_left:
            left, right = captured_second, captured_first
        else:
            left, right = captured_first, captured_second
        ar_corners, left_corners, right_corners = (
            find_corners(ar, board), find_corners(left, board), find_corners(right, board)
        )
        if any(corners is None for corners in (ar_corners, left_corners, right_corners)):
            raise RuntimeError(f"Checkerboard lost during solve: {root.name}/{filename}")
        object_sets.append(obj.copy())
        ar_sets.append(ar_corners)
        left_sets.append(left_corners)
        right_sets.append(right_corners)
        records.append({"session": root.name, "filename": filename, "sequential_skew_ms": pair["sequential_skew_ms"]})

    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 300, 1e-10)
    rms, _, _, _, _, R_ar_left, T_ar_left, _, _ = cv2.stereoCalibrate(
        object_sets, ar_sets, left_sets, K_ar.copy(), D_ar.copy(), K_left.copy(), D_left.copy(),
        ar_size, criteria=criteria, flags=cv2.CALIB_FIX_INTRINSIC,
    )
    initial_ar_errors = []
    for ar, left, right in zip(ar_sets, left_sets, right_sets):
        ar_error, _ = pair_reprojection_errors(
            obj, ar, left, right, K_ar, D_ar, K_left, D_left, K_right, D_right,
            R_ar_left, T_ar_left, R_left_right, T_left_right,
        )
        initial_ar_errors.append(ar_error)
    initial_median = float(np.median(initial_ar_errors))
    initial_mad = float(np.median(np.abs(np.asarray(initial_ar_errors) - initial_median)))
    outlier_limit = min(3.0, initial_median + 3.5 * max(initial_mad, 0.1))
    kept_indices = [index for index, value in enumerate(initial_ar_errors) if value <= outlier_limit]
    if len(kept_indices) < args.minimum_pairs:
        raise RuntimeError(
            f"Too few pairs after robust filtering: {len(kept_indices)} < {args.minimum_pairs}"
        )

    filtered_obj = [object_sets[index] for index in kept_indices]
    filtered_ar = [ar_sets[index] for index in kept_indices]
    filtered_left = [left_sets[index] for index in kept_indices]
    rms, _, _, _, _, R_ar_left, T_ar_left, _, _ = cv2.stereoCalibrate(
        filtered_obj, filtered_ar, filtered_left,
        K_ar.copy(), D_ar.copy(), K_left.copy(), D_left.copy(),
        ar_size, criteria=criteria, flags=cv2.CALIB_FIX_INTRINSIC,
    )

    ar_errors, right_errors = [], []
    kept_index_set = set(kept_indices)
    for index, (record, ar, left, right) in enumerate(zip(records, ar_sets, left_sets, right_sets)):
        ar_error, right_error = pair_reprojection_errors(
            obj, ar, left, right, K_ar, D_ar, K_left, D_left, K_right, D_right,
            R_ar_left, T_ar_left, R_left_right, T_left_right,
        )
        record["used_for_solution"] = index in kept_index_set
        record["ar0234_cross_reprojection_rms_px"] = ar_error
        record["stereo_right_validation_rms_px"] = right_error
        ar_errors.append(ar_error)
        right_errors.append(right_error)

    args.output_dir.mkdir(parents=True)
    output_npz = args.output_dir / "ar0234_to_ov9281_left_extrinsic_candidate_v1.npz"
    np.savez(
        output_npz,
        calibration_id=np.asarray("ar0234_v5_to_ov9281_v6_extrinsic_candidate_v1"),
        activation_eligible=np.asarray(False),
        source_ar_intrinsic_sha256=np.asarray(ar_sha),
        source_stereo_calibration_sha256=np.asarray(sha256_file(args.stereo_calibration)),
        R_ar0234_to_physical_left=R_ar_left,
        T_ar0234_to_physical_left_mm=T_ar_left,
        K_ar0234=K_ar, D_ar0234=D_ar, K_physical_left=K_left, D_physical_left=D_left,
        image_size_ar0234=np.asarray(ar_size), image_size_physical_left=np.asarray(eye_size),
        board_size=np.asarray(board), square_size_mm=np.asarray(square_size_mm),
    )
    report = {
        "schema_version": "sie.ar0234_ov9281.extrinsic_candidate_report.v1",
        "calibration_id": "ar0234_v5_to_ov9281_v6_extrinsic_candidate_v1",
        "status": "CANDIDATE_NOT_ACTIVE",
        "activation_eligible": False,
        "activation_blockers": [
            "independent_quality_review_pending",
            "physical_distance_validation_pending",
            "ROS2 metric-measurement integration_pending",
        ],
        "source": {
            "capture_audit_sha256": sha256_file(args.capture_audit),
            "capture_metadata_sha256": metadata_hashes,
            "ar0234_intrinsic_path": str(args.ar_intrinsic),
            "ar0234_intrinsic_sha256": ar_sha,
            "stereo_calibration_path": str(args.stereo_calibration),
            "stereo_calibration_sha256": sha256_file(args.stereo_calibration),
            "stereo_calibration_id": calibration_id,
        },
        "input_pair_count": len(records),
        "used_pair_count": len(kept_indices),
        "robust_outlier_limit_ar0234_cross_reprojection_px": outlier_limit,
        "initial_ar0234_cross_reprojection_rms_px": stats(initial_ar_errors),
        "captured_stream_identity": (
            {"captured_first": "physical_right", "captured_second": "physical_left"}
            if args.captured_second_is_physical_left
            else {"captured_first": "physical_left", "captured_second": "physical_right"}
        ),
        "stereo_calibrate_rms_px": float(rms),
        "ar0234_cross_reprojection_rms_px": stats(ar_errors),
        "stereo_right_validation_rms_px": stats(right_errors),
        "R_ar0234_to_physical_left": R_ar_left.tolist(),
        "T_ar0234_to_physical_left_mm": T_ar_left.reshape(-1).tolist(),
        "per_pair": records,
        "candidate_npz": str(output_npz),
    }
    report_path = args.output_dir / "extrinsic_candidate_report.json"
    report_path.write_text(json.dumps(report, indent=2, default=json_default), encoding="utf-8")
    print(json.dumps(report, indent=2, default=json_default))
    print(f"Saved: {output_npz}")
    print(f"Saved: {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
