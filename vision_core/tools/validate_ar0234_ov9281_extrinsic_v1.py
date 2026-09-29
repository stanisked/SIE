#!/usr/bin/env python3
"""Independent holdout validation for an AR0234-to-OV9281 extrinsic candidate.

The holdout capture must be a separate session and must not reuse images that
contributed to the candidate.  The combined OV9281 stream is explicitly mapped:
captured first half = physical right; captured second half = physical left.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from datetime import UTC, datetime
from pathlib import Path

import cv2
import numpy as np


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def scalar_text(value: object) -> str:
    array = np.asarray(value)
    return str(array.item() if array.shape == () else array.reshape(-1)[0])


def rms(observed: np.ndarray, predicted: np.ndarray) -> float:
    delta = observed.reshape(-1, 2) - predicted.reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum(delta * delta, axis=1))))


def summary(values: list[float]) -> dict[str, float | int]:
    data = np.asarray(values, dtype=np.float64)
    return {
        "count": int(data.size),
        "min_px": float(np.min(data)),
        "median_px": float(np.median(data)),
        "p95_px": float(np.percentile(data, 95)),
        "max_px": float(np.max(data)),
    }


def find_corners(image_path: Path, board_size: tuple[int, int]) -> np.ndarray | None:
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        return None
    found, corners = cv2.findChessboardCornersSB(
        image,
        board_size,
        flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY,
    )
    return corners.astype(np.float64) if found else None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--ar-intrinsic", required=True, type=Path)
    parser.add_argument("--stereo-calibration", required=True, type=Path)
    parser.add_argument("--extrinsic-candidate", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--inner-corners", nargs=2, type=int, default=(9, 6))
    parser.add_argument("--square-size-mm", type=float, default=24.5)
    parser.add_argument("--min-pairs", type=int, default=10)
    parser.add_argument("--max-ar-cross-median-px", type=float, default=1.5)
    parser.add_argument("--max-ar-cross-p95-px", type=float, default=3.0)
    parser.add_argument("--max-right-median-px", type=float, default=0.5)
    parser.add_argument("--max-right-p95-px", type=float, default=1.0)
    parser.add_argument("--max-rectified-dy-median-px", type=float, default=0.35)
    parser.add_argument("--max-rectified-dy-p95-px", type=float, default=0.75)
    args = parser.parse_args()

    root = args.dataset_root
    ar_dir, captured_first_dir, captured_second_dir = (
        root / "ar0234",
        root / "stereo_left",
        root / "stereo_right",
    )
    if not all(directory.is_dir() for directory in (ar_dir, captured_first_dir, captured_second_dir)):
        raise RuntimeError("dataset must contain ar0234/, stereo_left/, and stereo_right/")

    with args.ar_intrinsic.open(encoding="utf-8") as handle:
        ar_json = json.load(handle)
    k_ar = np.asarray(ar_json["camera_matrix"], dtype=np.float64)
    d_ar = np.asarray(ar_json["distortion_coefficients"], dtype=np.float64)

    stereo = np.load(args.stereo_calibration, allow_pickle=False)
    candidate = np.load(args.extrinsic_candidate, allow_pickle=False)
    required_stereo = ("K1", "D1", "K2", "D2", "R", "T", "R1", "R2", "P1", "P2")
    required_candidate = ("R_ar0234_to_physical_left", "T_ar0234_to_physical_left_mm")
    for key in required_stereo:
        if key not in stereo:
            raise RuntimeError(f"stereo calibration is missing {key}")
    for key in required_candidate:
        if key not in candidate:
            raise RuntimeError(f"extrinsic candidate is missing {key}")

    ar_sha = sha256_file(args.ar_intrinsic)
    stereo_sha = sha256_file(args.stereo_calibration)
    candidate_sha = sha256_file(args.extrinsic_candidate)
    blockers: list[str] = []
    for key, expected in (
        ("source_ar_intrinsic_sha256", ar_sha),
        ("source_stereo_calibration_sha256", stereo_sha),
    ):
        if key not in candidate:
            blockers.append(f"candidate_missing_{key}")
        elif scalar_text(candidate[key]).lower() != expected:
            blockers.append(f"candidate_{key}_mismatch")

    board_size = tuple(args.inner_corners)
    grid = np.zeros((board_size[0] * board_size[1], 3), np.float64)
    grid[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
    grid *= args.square_size_mm / 1000.0

    k_left, d_left = stereo["K1"], stereo["D1"]
    k_right, d_right = stereo["K2"], stereo["D2"]
    r_left_to_right = stereo["R"]
    # v7 was solved with checkerboard coordinates in millimetres; PnP uses metres.
    t_left_to_right = stereo["T"] / 1000.0
    r1, r2, p1, p2 = stereo["R1"], stereo["R2"], stereo["P1"], stereo["P2"]
    r_ar_to_left = candidate["R_ar0234_to_physical_left"]
    t_ar_to_left = candidate["T_ar0234_to_physical_left_mm"].reshape(3, 1) / 1000.0

    names = sorted(
        set(path.name for path in ar_dir.glob("*.png"))
        & set(path.name for path in captured_first_dir.glob("*.png"))
        & set(path.name for path in captured_second_dir.glob("*.png"))
    )
    ar_errors: list[float] = []
    right_errors: list[float] = []
    rectified_dy: list[float] = []
    rows: list[dict[str, object]] = []

    for name in names:
        corners_ar = find_corners(ar_dir / name, board_size)
        # Captured second half is the physical left OV9281 stream.
        corners_left = find_corners(captured_second_dir / name, board_size)
        corners_right = find_corners(captured_first_dir / name, board_size)
        if any(corners is None for corners in (corners_ar, corners_left, corners_right)):
            rows.append({"image": name, "accepted": False, "reason": "checkerboard_not_found_all_streams"})
            continue

        ok, rvec_left, tvec_left = cv2.solvePnP(
            grid, corners_left, k_left, d_left, flags=cv2.SOLVEPNP_ITERATIVE
        )
        if not ok:
            rows.append({"image": name, "accepted": False, "reason": "physical_left_solvepnp_failed"})
            continue
        r_left_board, _ = cv2.Rodrigues(rvec_left)
        r_ar_board = r_ar_to_left.T @ r_left_board
        t_ar_board = r_ar_to_left.T @ (tvec_left - t_ar_to_left)
        rvec_ar, _ = cv2.Rodrigues(r_ar_board)
        projected_ar, _ = cv2.projectPoints(grid, rvec_ar, t_ar_board, k_ar, d_ar)

        r_right_board = r_left_to_right @ r_left_board
        t_right_board = r_left_to_right @ tvec_left + t_left_to_right
        rvec_right, _ = cv2.Rodrigues(r_right_board)
        projected_right, _ = cv2.projectPoints(grid, rvec_right, t_right_board, k_right, d_right)

        rect_left = cv2.undistortPoints(corners_left, k_left, d_left, R=r1, P=p1)
        rect_right = cv2.undistortPoints(corners_right, k_right, d_right, R=r2, P=p2)
        dy = np.abs(rect_left.reshape(-1, 2)[:, 1] - rect_right.reshape(-1, 2)[:, 1])

        ar_error = rms(corners_ar, projected_ar)
        right_error = rms(corners_right, projected_right)
        dy_median = float(np.median(dy))
        ar_errors.append(ar_error)
        right_errors.append(right_error)
        rectified_dy.append(dy_median)
        rows.append(
            {
                "image": name,
                "accepted": True,
                "ar0234_cross_reprojection_rms_px": ar_error,
                "physical_right_reprojection_rms_px": right_error,
                "rectified_median_abs_dy_px": dy_median,
            }
        )

    metrics = {
        "ar0234_cross_reprojection_rms_px": summary(ar_errors) if ar_errors else None,
        "physical_right_reprojection_rms_px": summary(right_errors) if right_errors else None,
        "rectified_median_abs_dy_px": summary(rectified_dy) if rectified_dy else None,
    }
    if len(ar_errors) < args.min_pairs:
        blockers.append(f"insufficient_independent_pairs:{len(ar_errors)}<{args.min_pairs}")
    elif not blockers:
        checks = (
            ("ar_cross_median", metrics["ar0234_cross_reprojection_rms_px"]["median_px"], args.max_ar_cross_median_px),
            ("ar_cross_p95", metrics["ar0234_cross_reprojection_rms_px"]["p95_px"], args.max_ar_cross_p95_px),
            ("right_median", metrics["physical_right_reprojection_rms_px"]["median_px"], args.max_right_median_px),
            ("right_p95", metrics["physical_right_reprojection_rms_px"]["p95_px"], args.max_right_p95_px),
            ("rectified_dy_median", metrics["rectified_median_abs_dy_px"]["median_px"], args.max_rectified_dy_median_px),
            ("rectified_dy_p95", metrics["rectified_median_abs_dy_px"]["p95_px"], args.max_rectified_dy_p95_px),
        )
        blockers.extend(f"{name}:{value:.4f}>{limit:.4f}" for name, value, limit in checks if value > limit)

    report = {
        "schema_version": "sie.ar0234_ov9281.extrinsic_independent_validation.v1",
        "generated_utc": datetime.now(UTC).isoformat(),
        "status": "PASS" if not blockers else "REJECT",
        "holdout_dataset_root": str(root),
        "stream_identity": {
            "captured_first_half": "physical_right",
            "captured_second_half": "physical_left",
        },
        "source_sha256": {
            "ar0234_intrinsic": ar_sha,
            "ov9281_stereo_calibration": stereo_sha,
            "extrinsic_candidate": candidate_sha,
        },
        "input_pair_count": len(names),
        "independent_valid_pair_count": len(ar_errors),
        "thresholds_px": {
            "ar0234_cross_median": args.max_ar_cross_median_px,
            "ar0234_cross_p95": args.max_ar_cross_p95_px,
            "physical_right_median": args.max_right_median_px,
            "physical_right_p95": args.max_right_p95_px,
            "rectified_dy_median": args.max_rectified_dy_median_px,
            "rectified_dy_p95": args.max_rectified_dy_p95_px,
        },
        "metrics": metrics,
        "blockers": blockers,
        "pairs": rows,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"Independent extrinsic validation: {report['status']}; valid_pairs={len(ar_errors)}/{len(names)}")
    print(f"Saved: {args.output}")
    return 0 if not blockers else 2


if __name__ == "__main__":
    raise SystemExit(main())
