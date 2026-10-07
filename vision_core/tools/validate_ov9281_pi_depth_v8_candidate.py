#!/usr/bin/env python3
"""Headless, evidence-preserving physical depth check for an OV9281 v8 candidate.

The script reads a candidate only.  It never activates, copies, or modifies a
calibration artifact.  Ground truth and reconstructed depth intentionally keep
their different reference frames explicit in the report.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def set_auto_exposure(device: str) -> str:
    subprocess.run(["v4l2-ctl", "-d", device, "-c", "auto_exposure=3"], check=True)
    result = subprocess.run(
        ["v4l2-ctl", "-d", device, "-C", "auto_exposure"],
        check=True, text=True, capture_output=True,
    )
    if ": 3" not in result.stdout:
        raise RuntimeError(f"auto_exposure=3 was not retained: {result.stdout.strip()}")
    return result.stdout.strip()


def find_corners(image: np.ndarray, board: tuple[int, int]) -> np.ndarray | None:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCornersSB(gray, board, flags=flags)
    return None if not found or corners is None else corners.astype(np.float32)


def stats(values: list[float]) -> dict[str, float]:
    data = np.asarray(values, dtype=np.float64)
    median = float(np.median(data))
    return {
        "count": int(data.size), "min": float(np.min(data)),
        "p05": float(np.percentile(data, 5)), "p25": float(np.percentile(data, 25)),
        "p50": median, "p75": float(np.percentile(data, 75)),
        "p95": float(np.percentile(data, 95)), "max": float(np.max(data)),
        "mad": float(np.median(np.abs(data - median))),
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
    parser.add_argument("--warmup-frames", type=int, default=180)
    parser.add_argument("--evidence-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.ground_truth_m <= 0 or args.frames < 5 or args.warmup_frames < 1:
        raise ValueError("invalid ground truth, frame count, or warmup count")
    if not args.stereo_calibration.is_file():
        raise FileNotFoundError(args.stereo_calibration)
    if args.output.exists():
        raise FileExistsError(f"output already exists: {args.output}")
    if args.evidence_dir.exists() and any(args.evidence_dir.iterdir()):
        raise RuntimeError(f"evidence directory is not empty: {args.evidence_dir}")

    with np.load(args.stereo_calibration, allow_pickle=False) as values:
        calibration_id = str(values["calibration_id"].item())
        if not re.fullmatch(r"ov9281_pi_stereo_candidate_v8[a-z0-9_]*", calibration_id):
            raise RuntimeError(f"expected a v8-series candidate, got {calibration_id}")
        if str(values["camera_1_semantics"].item()) != "physical_left":
            raise RuntimeError("candidate is not bound to physical_left")
        K1, D1, K2, D2 = values["K1"], values["D1"], values["K2"], values["D2"]
        R1, R2, P1, P2 = values["R1"], values["R2"], values["P1"], values["P2"]
        board = tuple(int(value) for value in values["board_size"])
        size = tuple(int(value) for value in values["size"])

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.evidence_dir.mkdir(parents=True, exist_ok=True)
    exposure = set_auto_exposure(args.device)
    capture = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, size[0] * 2)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
    capture.set(cv2.CAP_PROP_FPS, 60)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not capture.isOpened():
        raise RuntimeError(f"cannot open {args.device}")

    try:
        for _ in range(args.warmup_frames):
            ok, frame = capture.read()
            if not ok or frame is None:
                raise RuntimeError("camera returned no frame during warmup")
        depths: list[float] = []
        disparities: list[float] = []
        vertical_errors: list[float] = []
        per_frame: list[dict] = []
        while len(per_frame) < args.frames:
            ok, combined = capture.read()
            if not ok or combined is None or combined.shape[:2] != (size[1], size[0] * 2):
                continue
            physical_right, physical_left = combined[:, :size[0]], combined[:, size[0]:]
            left, right = find_corners(physical_left, board), find_corners(physical_right, board)
            if left is None or right is None:
                continue
            left_rectified = cv2.undistortPoints(left, K1, D1, R=R1, P=P1).reshape(-1, 2)
            right_rectified = cv2.undistortPoints(right, K2, D2, R=R2, P=P2).reshape(-1, 2)
            disparity = left_rectified[:, 0] - right_rectified[:, 0]
            valid = disparity > 0.5
            if int(np.count_nonzero(valid)) < int(0.9 * disparity.size):
                continue
            depth_m = (abs(float(P2[0, 3])) / disparity[valid]) / 1000.0
            frame_index = len(per_frame) + 1
            stem = f"accepted_{frame_index:03d}"
            image_path = args.evidence_dir / f"{stem}_combined_source.jpg"
            arrays_path = args.evidence_dir / f"{stem}_corners_and_disparity.npz"
            if not cv2.imwrite(str(image_path), combined):
                raise OSError(f"could not save {image_path}")
            np.savez_compressed(
                arrays_path,
                physical_left_raw_corners=left,
                physical_right_raw_corners=right,
                physical_left_rectified_corners=left_rectified,
                physical_right_rectified_corners=right_rectified,
                disparity_px=disparity,
                valid_disparity_mask=valid,
            )
            frame_median = float(np.median(depth_m))
            frame_disparity = float(np.median(disparity[valid]))
            frame_vertical = float(np.median(np.abs(left_rectified[:, 1] - right_rectified[:, 1])))
            depths.extend(depth_m.tolist())
            disparities.extend(disparity[valid].tolist())
            vertical_errors.append(frame_vertical)
            per_frame.append({
                "captured_at_utc": utc_now(), "median_depth_m": frame_median,
                "median_disparity_px": frame_disparity,
                "median_abs_vertical_error_px": frame_vertical,
                "combined_source": image_path.name,
                "corners_and_disparity": arrays_path.name,
            })
            print(f"ACCEPTED {frame_index:02d}/{args.frames}: depth={frame_median:.4f} m")
    finally:
        capture.release()

    depth = stats(depths)
    report = {
        "schema_version": "sie.ov9281.pi_stereo_planar_depth_diagnostic.v8",
        "status": "CANDIDATE_DIAGNOSTIC_NOT_ACTIVATED",
        "calibration_id": calibration_id,
        "ground_truth_m": args.ground_truth_m,
        "ground_truth_reference_frame": "physical_left_lens_front_rim_frame",
        "depth_reference_frame": "rectified_left_optical_frame",
        "cross_frame_difference_m": depth["p50"] - args.ground_truth_m,
        "cross_frame_absolute_difference_m": abs(depth["p50"] - args.ground_truth_m),
        "depth_m": depth, "disparity_px": stats(disparities),
        "median_abs_vertical_error_px": stats(vertical_errors),
        "capture": {"auto_exposure": exposure, "warmup_frames": args.warmup_frames,
                    "frames": args.frames, "board_inner_corners": list(board),
                    "evidence_dir": str(args.evidence_dir)},
        "per_frame": per_frame,
    }
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({
        "ground_truth_m": args.ground_truth_m, "median_depth_m": depth["p50"],
        "cross_frame_absolute_difference_m": report["cross_frame_absolute_difference_m"],
        "depth_mad_m": depth["mad"], "depth_p05_p95_m": [depth["p05"], depth["p95"]],
        "disparity_p05_p95_px": [report["disparity_px"]["p05"], report["disparity_px"]["p95"]],
    }, indent=2))
    print(f"Saved: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
