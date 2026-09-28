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


def stats(values: list[float]) -> dict[str, float]:
    data = np.asarray(values, dtype=float)
    return {
        "count": int(len(data)),
        "min_m": float(np.min(data)),
        "median_m": float(np.median(data)),
        "mad_m": float(np.median(np.abs(data - np.median(data)))),
        "p95_m": float(np.percentile(data, 95)),
        "max_m": float(np.max(data)),
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
    args = parser.parse_args()
    if args.ground_truth_m <= 0 or args.frames < 5:
        raise ValueError("invalid ground truth or frame count")

    with np.load(args.stereo_calibration, allow_pickle=False) as values:
        if str(values["camera_1_semantics"].item()) != "physical_left":
            raise RuntimeError("candidate is not bound to physical_left")
        K1, D1, K2, D2 = values["K1"], values["D1"], values["K2"], values["D2"]
        R1, R2, P1, P2 = values["R1"], values["R2"], values["P1"], values["P2"]
        board = tuple(int(value) for value in values["board_size"])
        size = tuple(int(value) for value in values["size"])

    set_auto_exposure(args.device)
    capture = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, size[0] * 2)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
    capture.set(cv2.CAP_PROP_FPS, 60)
    if not capture.isOpened():
        raise RuntimeError(f"cannot open {args.device}")

    for _ in range(20):
        capture.read()
    depths: list[float] = []
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
            per_frame.append({
                "captured_at_utc": datetime.now(timezone.utc).isoformat(),
                "valid_corner_count": int(valid.sum()),
                "median_depth_m": frame_median,
                "median_abs_vertical_error_px": float(
                    np.median(np.abs(left_rectified[:, 1] - right_rectified[:, 1]))
                ),
            })
            print(f"ACCEPTED {len(per_frame):02d}/{args.frames}: depth={frame_median:.4f} m")
    finally:
        capture.release()

    summary = stats(depths)
    error_m = summary["median_m"] - args.ground_truth_m
    report = {
        "schema_version": "sie.ov9281.pi_stereo_physical_depth_check.v1",
        "status": "MEASURED_NOT_ACTIVATED",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "ground_truth_m": args.ground_truth_m,
        "depth_m": summary,
        "median_error_m": error_m,
        "median_absolute_error_m": abs(error_m),
        "per_frame": per_frame,
    }
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({
        "ground_truth_m": args.ground_truth_m,
        "median_depth_m": summary["median_m"],
        "median_absolute_error_m": abs(error_m),
        "depth_mad_m": summary["mad_m"],
    }, indent=2))
    print(f"Saved: {args.output}")


if __name__ == "__main__":
    main()
