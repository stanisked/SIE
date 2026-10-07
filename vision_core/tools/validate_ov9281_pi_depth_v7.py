#!/usr/bin/env python3
"""Headless physical depth check for the Pi-specific OV9281 stereo candidate."""

from __future__ import annotations

import argparse
import json
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np


def set_auto_exposure(device: str) -> None:
    subprocess.run(["v4l2-ctl", "-d", device, "-c", "auto_exposure=3"], check=True)


def find_corners(image: np.ndarray, board: tuple[int, int]) -> np.ndarray | None:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCornersSB(gray, board, flags=flags)
    return None if not found or corners is None else corners.astype(np.float32)


def depth_stats(values: list[float]) -> dict[str, float]:
    data = np.asarray(values, dtype=float)
    return {
        "count": int(len(data)),
        "min_m": float(np.min(data)),
        "p05_m": float(np.percentile(data, 5)),
        "p25_m": float(np.percentile(data, 25)),
        "median_m": float(np.median(data)),
        "p75_m": float(np.percentile(data, 75)),
        "mad_m": float(np.median(np.abs(data - np.median(data)))),
        "p95_m": float(np.percentile(data, 95)),
        "max_m": float(np.max(data)),
    }


def disparity_stats(values: list[float]) -> dict[str, float]:
    data = np.asarray(values, dtype=float)
    return {
        "count": int(len(data)),
        "min_px": float(np.min(data)),
        "p05_px": float(np.percentile(data, 5)),
        "p25_px": float(np.percentile(data, 25)),
        "median_px": float(np.median(data)),
        "p75_px": float(np.percentile(data, 75)),
        "mad_px": float(np.median(np.abs(data - np.median(data)))),
        "p95_px": float(np.percentile(data, 95)),
        "max_px": float(np.max(data)),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stereo-calibration", type=Path, required=True)
    parser.add_argument("--ground-truth-m", type=float, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--device",
        default="/dev/v4l/by-id/usb-TSTC_Web_Camera_TSTC_Web_Camera-video-index0",
    )
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--warmup-frames", type=int, default=20)
    parser.add_argument(
        "--board-inner-corners",
        type=int,
        nargs=2,
        metavar=("COLUMNS", "ROWS"),
        help="override the calibration board size for a diagnostic checkerboard",
    )
    parser.add_argument(
        "--evidence-dir",
        type=Path,
        help="optional directory for accepted raw pairs and corner/disparity arrays",
    )
    args = parser.parse_args()
    if args.ground_truth_m <= 0 or args.frames < 5 or args.warmup_frames <= 0:
        raise ValueError("invalid ground truth or frame count")
    if args.evidence_dir is not None:
        args.evidence_dir.mkdir(parents=True, exist_ok=True)

    with np.load(args.stereo_calibration, allow_pickle=False) as values:
        if str(values["camera_1_semantics"].item()) != "physical_left":
            raise RuntimeError("candidate is not bound to physical_left")
        K1, D1, K2, D2 = values["K1"], values["D1"], values["K2"], values["D2"]
        R1, R2, P1, P2 = values["R1"], values["R2"], values["P1"], values["P2"]
        calibration_board = tuple(int(value) for value in values["board_size"])
        size = tuple(int(value) for value in values["size"])
    board = (
        tuple(args.board_inner_corners)
        if args.board_inner_corners is not None
        else calibration_board
    )
    if board[0] < 2 or board[1] < 2:
        raise ValueError("checkerboard inner-corner dimensions must be at least 2x2")

    set_auto_exposure(args.device)
    capture = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, size[0] * 2)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
    capture.set(cv2.CAP_PROP_FPS, 60)
    if not capture.isOpened():
        raise RuntimeError(f"cannot open {args.device}")

    for _ in range(args.warmup_frames):
        capture.read()
    depths: list[float] = []
    disparities: list[float] = []
    per_frame = []
    try:
        while len(per_frame) < args.frames:
            ok, combined = capture.read()
            if not ok or combined is None or combined.shape[:2] != (size[1], size[0] * 2):
                continue
            # Pi identity: first half physical_right; second half physical_left.
            physical_right = combined[:, :size[0]]
            physical_left = combined[:, size[0]:]
            left = find_corners(physical_left, board)
            right = find_corners(physical_right, board)
            if left is None or right is None:
                continue
            left_rectified = cv2.undistortPoints(left, K1, D1, R=R1, P=P1).reshape(-1, 2)
            right_rectified = cv2.undistortPoints(right, K2, D2, R=R2, P=P2).reshape(-1, 2)
            disparity = left_rectified[:, 0] - right_rectified[:, 0]
            valid = disparity > 0.5
            if valid.sum() < len(disparity) * 0.9:
                continue
            depth = (np.abs(P2[0, 3]) / disparity[valid]) / 1000.0
            frame_median = float(np.median(depth))
            depths.extend(depth.tolist())
            disparities.extend(disparity[valid].tolist())
            evidence: dict[str, str] | None = None
            if args.evidence_dir is not None:
                frame_name = f"accepted_{len(per_frame) + 1:03d}"
                combined_path = args.evidence_dir / f"{frame_name}_combined_source.jpg"
                corners_path = args.evidence_dir / f"{frame_name}_corners_and_disparity.npz"
                if not cv2.imwrite(str(combined_path), combined):
                    raise OSError(f"could not write evidence image: {combined_path}")
                np.savez_compressed(
                    corners_path,
                    physical_left_raw_corners=left,
                    physical_right_raw_corners=right,
                    physical_left_rectified_corners=left_rectified,
                    physical_right_rectified_corners=right_rectified,
                    disparity_px=disparity,
                    valid_disparity_mask=valid,
                )
                evidence = {
                    "combined_source": combined_path.name,
                    "corners_and_disparity": corners_path.name,
                }
            per_frame.append({
                "captured_at_utc": datetime.now(timezone.utc).isoformat(),
                "valid_corner_count": int(valid.sum()),
                "median_depth_m": frame_median,
                "median_disparity_px": float(np.median(disparity[valid])),
                "median_abs_vertical_error_px": float(
                    np.median(np.abs(left_rectified[:, 1] - right_rectified[:, 1]))
                ),
                "evidence": evidence,
            })
            print(f"ACCEPTED {len(per_frame):02d}/{args.frames}: depth={frame_median:.4f} m")
    finally:
        capture.release()

    depth_summary = depth_stats(depths)
    disparity_summary = disparity_stats(disparities)
    raw_cross_frame_difference_m = depth_summary["median_m"] - args.ground_truth_m
    report = {
        "schema_version": "sie.ov9281.pi_stereo_planar_depth_diagnostic.v2",
        "status": "DIAGNOSTIC_NOT_ACTIVATED",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "ground_truth_m": args.ground_truth_m,
        "ground_truth_reference_frame": "physical_left_lens_front_rim_frame",
        "depth_reference_frame": "rectified_left_optical_frame",
        "depth_m": depth_summary,
        "disparity_px": disparity_summary,
        "raw_cross_frame_difference_m": raw_cross_frame_difference_m,
        "raw_cross_frame_absolute_difference_m": abs(raw_cross_frame_difference_m),
        "cross_frame_comparison_note": (
            "Ground truth is measured from the physical-left lens front rim, "
            "while stereo depth is rectified-left optical-frame z. The difference "
            "is diagnostic evidence, not a scale or offset correction."
        ),
        "capture": {
            "warmup_frames": args.warmup_frames,
            "evidence_dir": None if args.evidence_dir is None else str(args.evidence_dir),
            "calibration_board_inner_corners": list(calibration_board),
            "test_board_inner_corners": list(board),
        },
        "per_frame": per_frame,
    }
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({
        "ground_truth_m": args.ground_truth_m,
        "median_depth_m": depth_summary["median_m"],
        "raw_cross_frame_absolute_difference_m": abs(raw_cross_frame_difference_m),
        "depth_mad_m": depth_summary["mad_m"],
        "depth_p05_p95_m": [depth_summary["p05_m"], depth_summary["p95_m"]],
        "disparity_p05_p95_px": [
            disparity_summary["p05_px"], disparity_summary["p95_px"]
        ],
    }, indent=2))
    print(f"Saved: {args.output}")


if __name__ == "__main__":
    main()
