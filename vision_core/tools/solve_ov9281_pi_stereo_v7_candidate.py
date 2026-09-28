#!/usr/bin/env python3
"""Build a non-active Pi-specific OV9281 stereo calibration candidate from audited captures."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def corners(image: np.ndarray, board: tuple[int, int]) -> np.ndarray:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, result = cv2.findChessboardCornersSB(gray, board, flags=flags)
    if not found or result is None:
        raise RuntimeError("checkerboard not found")
    return result.astype(np.float32)


def object_template(board: tuple[int, int], square_mm: float) -> np.ndarray:
    cols, rows = board
    result = np.zeros((cols * rows, 3), np.float32)
    result[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    return result * square_mm


def stats(values: list[float]) -> dict[str, float]:
    array = np.asarray(values, dtype=float)
    return {
        "count": int(len(array)),
        "min": float(np.min(array)),
        "median": float(np.median(array)),
        "p95": float(np.percentile(array, 95)),
        "max": float(np.max(array)),
    }


def reprojection_rms(observed: np.ndarray, projected: np.ndarray) -> float:
    error = observed.reshape(-1, 2).astype(float) - projected.reshape(-1, 2).astype(float)
    return float(np.sqrt(np.mean(np.sum(error * error, axis=1))))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path, action="append", required=True)
    parser.add_argument("--capture-audit", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--holdout-every", type=int, default=4)
    parser.add_argument("--physical-baseline-mm", type=float, default=65.1)
    args = parser.parse_args()

    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        raise RuntimeError(f"Output directory is not empty: {args.output_dir}")
    audit = json.loads(args.capture_audit.read_text())
    if audit.get("result") != "PASS":
        raise RuntimeError("combined capture audit must PASS")
    if args.holdout_every < 3:
        raise ValueError("--holdout-every must be >= 3")

    samples: list[dict[str, Any]] = []
    board: tuple[int, int] | None = None
    square_mm: float | None = None
    size: tuple[int, int] | None = None
    source_metadata_sha: dict[str, str] = {}
    for root in (path.resolve() for path in args.dataset_root):
        metadata_path = root / "capture_metadata.json"
        metadata = json.loads(metadata_path.read_text())
        if metadata.get("status") != "COMPLETE":
            raise RuntimeError(f"capture incomplete: {root}")
        current_board = tuple(int(value) for value in metadata["target"]["checkerboard_inner_corners"])
        current_square = float(metadata["target"]["square_size_mm"])
        current_size = (
            int(metadata["ov9281_stereo"]["eye_image_size_px"]["width"]),
            int(metadata["ov9281_stereo"]["eye_image_size_px"]["height"]),
        )
        if board is None:
            board, square_mm, size = current_board, current_square, current_size
        if (current_board, current_square, current_size) != (board, square_mm, size):
            raise RuntimeError(f"capture contract mismatch: {root}")
        source_metadata_sha[str(root)] = sha256(metadata_path)
        for pair in metadata["pairs"]:
            samples.append({"root": root, "filename": pair["filename"]})

    assert board is not None and square_mm is not None and size is not None
    obj = object_template(board, square_mm)
    prepared = []
    for sample in samples:
        root, filename = sample["root"], sample["filename"]
        # Pi stream identity was established independently: second half is physical_left.
        physical_left = cv2.imread(str(root / "stereo_right" / filename), cv2.IMREAD_COLOR)
        physical_right = cv2.imread(str(root / "stereo_left" / filename), cv2.IMREAD_COLOR)
        if physical_left is None or physical_right is None:
            raise RuntimeError(f"unreadable pair: {root.name}/{filename}")
        if physical_left.shape[:2] != (size[1], size[0]) or physical_right.shape[:2] != (size[1], size[0]):
            raise RuntimeError(f"unexpected image size: {root.name}/{filename}")
        prepared.append({
            "session": root.name,
            "filename": filename,
            "left": corners(physical_left, board),
            "right": corners(physical_right, board),
        })

    train = [item for index, item in enumerate(prepared) if (index + 1) % args.holdout_every]
    holdout = [item for index, item in enumerate(prepared) if not (index + 1) % args.holdout_every]
    if len(train) < 40 or len(holdout) < 10:
        raise RuntimeError(f"insufficient train/holdout split: {len(train)}/{len(holdout)}")

    obj_train = [obj.copy() for _ in train]
    left_train = [item["left"] for item in train]
    right_train = [item["right"] for item in train]
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 300, 1e-10)
    left_rms, K1, D1, _, _ = cv2.calibrateCamera(obj_train, left_train, size, None, None, criteria=criteria)
    right_rms, K2, D2, _, _ = cv2.calibrateCamera(obj_train, right_train, size, None, None, criteria=criteria)
    stereo_rms, K1, D1, K2, D2, R, T, E, F = cv2.stereoCalibrate(
        obj_train, left_train, right_train, K1, D1, K2, D2, size,
        criteria=criteria, flags=cv2.CALIB_FIX_INTRINSIC,
    )
    R1, R2, P1, P2, Q, _, _, = cv2.stereoRectify(
        K1, D1, K2, D2, size, R, T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0
    )

    holdout_right_errors, rectified_vertical_errors, per_pair = [], [], []
    for item in holdout:
        ok, rvec, tvec = cv2.solvePnP(obj, item["left"], K1, D1, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            raise RuntimeError(f"solvePnP failed: {item['session']}/{item['filename']}")
        R_left_board, _ = cv2.Rodrigues(rvec)
        R_right_board = R @ R_left_board
        T_right_board = R @ tvec + T
        rvec_right, _ = cv2.Rodrigues(R_right_board)
        predicted, _ = cv2.projectPoints(obj, rvec_right, T_right_board, K2, D2)
        right_error = reprojection_rms(item["right"], predicted)
        left_rectified = cv2.undistortPoints(item["left"], K1, D1, R=R1, P=P1).reshape(-1, 2)
        right_rectified = cv2.undistortPoints(item["right"], K2, D2, R=R2, P=P2).reshape(-1, 2)
        vertical_error = float(np.median(np.abs(left_rectified[:, 1] - right_rectified[:, 1])))
        holdout_right_errors.append(right_error)
        rectified_vertical_errors.append(vertical_error)
        per_pair.append({
            "session": item["session"], "filename": item["filename"],
            "right_reprojection_rms_px": right_error,
            "rectified_median_abs_dy_px": vertical_error,
        })

    args.output_dir.mkdir(parents=True)
    candidate_path = args.output_dir / "ov9281_pi_stereo_candidate_v7.npz"
    np.savez(
        candidate_path,
        calibration_id=np.asarray("ov9281_pi_stereo_candidate_v7"),
        activation_eligible=np.asarray(False),
        camera_1_semantics=np.asarray("physical_left"),
        camera_2_semantics=np.asarray("physical_right"),
        captured_stream_identity=np.asarray("captured_second_is_physical_left"),
        K1=K1, D1=D1, K2=K2, D2=D2, R=R, T=T, E=E, F=F,
        R1=R1, R2=R2, P1=P1, P2=P2, Q=Q, size=np.asarray(size),
        board_size=np.asarray(board), square_size_mm=np.asarray(square_mm),
    )
    baseline_mm = float(np.linalg.norm(T))
    report = {
        "schema_version": "sie.ov9281.pi_stereo_candidate_report.v1",
        "status": "CANDIDATE_NOT_ACTIVE",
        "activation_eligible": False,
        "activation_blockers": [
            "independent_holdout_review_pending",
            "physical_depth_validation_pending",
            "runtime_policy_review_pending",
        ],
        "input_pair_count": len(prepared),
        "train_pair_count": len(train),
        "holdout_pair_count": len(holdout),
        "captured_stream_identity": {
            "captured_first": "physical_right",
            "captured_second": "physical_left",
        },
        "source_capture_metadata_sha256": source_metadata_sha,
        "source_audit_sha256": sha256(args.capture_audit),
        "left_intrinsic_rms_px": float(left_rms),
        "right_intrinsic_rms_px": float(right_rms),
        "stereo_rms_px": float(stereo_rms),
        "baseline_mm": baseline_mm,
        "physical_baseline_mm": args.physical_baseline_mm,
        "physical_baseline_difference_mm": abs(baseline_mm - args.physical_baseline_mm),
        "holdout_right_reprojection_rms_px": stats(holdout_right_errors),
        "holdout_rectified_median_abs_dy_px": stats(rectified_vertical_errors),
        "candidate_path": str(candidate_path),
        "holdout_per_pair": per_pair,
    }
    report_path = args.output_dir / "stereo_candidate_report.json"
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print(f"Saved: {candidate_path}")
    print(f"Saved: {report_path}")


if __name__ == "__main__":
    main()
