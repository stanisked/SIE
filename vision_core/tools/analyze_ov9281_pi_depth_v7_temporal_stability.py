#!/usr/bin/env python3
"""Analyse temporal stability in pose-aware OV9281 v7 diagnostic reports.

This is an offline diagnostic.  It reads already recorded per-frame checkerboard
depth, disparity, and PnP pose data to distinguish a moving board pose from a
relative stereo-disparity change.  It never changes a calibration, runtime
policy, camera control, or SIE Measurement state.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


SOURCE_SCHEMA = "sie.ov9281.pi_stereo_pose_aware_planar_depth_diagnostic.v1"
OUTPUT_SCHEMA = "sie.ov9281.pi_stereo_temporal_stability_diagnostic.v1"


def parse_timestamp(value: Any, path: Path, index: int) -> float:
    if not isinstance(value, str):
        raise ValueError(f"{path}: per_frame[{index}] captured_at_utc is missing")
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as error:
        raise ValueError(f"{path}: invalid captured_at_utc at frame {index}") from error
    if parsed.tzinfo is None:
        raise ValueError(f"{path}: captured_at_utc at frame {index} is timezone-naive")
    return parsed.timestamp()


def finite_number(value: Any, description: str) -> float:
    if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ValueError(f"{description} must be finite")
    return float(value)


def correlation(left: list[float], right: list[float]) -> float | None:
    if len(left) != len(right) or len(left) < 2:
        raise ValueError("correlation requires equally sized series with at least two values")
    left_mean = statistics.fmean(left)
    right_mean = statistics.fmean(right)
    numerator = sum((x - left_mean) * (y - right_mean) for x, y in zip(left, right))
    denominator = math.sqrt(
        sum((x - left_mean) ** 2 for x in left)
        * sum((y - right_mean) ** 2 for y in right)
    )
    return None if denominator == 0.0 else numerator / denominator


def series_summary(values: list[float], elapsed_s: list[float], unit: str) -> dict[str, float | str]:
    if len(values) != len(elapsed_s) or len(values) < 2:
        raise ValueError("series requires at least two aligned samples")
    mean_t = statistics.fmean(elapsed_s)
    mean_value = statistics.fmean(values)
    denominator = sum((sample_t - mean_t) ** 2 for sample_t in elapsed_s)
    slope = (
        sum((sample_t - mean_t) * (value - mean_value) for sample_t, value in zip(elapsed_s, values))
        / denominator
    ) if denominator > 0.0 else 0.0
    return {
        "unit": unit,
        "first": values[0],
        "last": values[-1],
        "first_to_last_delta": values[-1] - values[0],
        "minimum": min(values),
        "maximum": max(values),
        "peak_to_peak": max(values) - min(values),
        "median": statistics.median(values),
        "mad": statistics.median(abs(value - statistics.median(values)) for value in values),
        "linear_slope_per_s": slope,
    }


def load_run(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("schema_version") != SOURCE_SCHEMA:
        raise ValueError(f"{path}: unexpected source schema")
    if payload.get("status") != "DIAGNOSTIC_NOT_ACTIVATED":
        raise ValueError(f"{path}: source report is not diagnostic-only")
    if payload.get("validation_mode") != "relative_axis_translation":
        raise ValueError(f"{path}: source report is not relative-axis validation")
    if payload.get("depth_reference_frame") != "rectified_left_optical_frame":
        raise ValueError(f"{path}: unexpected depth reference frame")
    calibration = payload.get("calibration")
    reference = payload.get("relative_axis_reference")
    frames = payload.get("per_frame")
    if not isinstance(calibration, dict) or not isinstance(reference, dict):
        raise ValueError(f"{path}: missing calibration or relative-axis reference")
    if not isinstance(frames, list) or len(frames) < 2:
        raise ValueError(f"{path}: at least two accepted per-frame records are required")

    timestamps: list[float] = []
    depth_m: list[float] = []
    disparity_px: list[float] = []
    pnp_z_m: list[float] = []
    for index, frame in enumerate(frames):
        if not isinstance(frame, dict):
            raise ValueError(f"{path}: per_frame[{index}] is not an object")
        timestamps.append(parse_timestamp(frame.get("captured_at_utc"), path, index))
        depth_m.append(finite_number(frame.get("median_depth_m"), f"{path}: depth at frame {index}"))
        disparity_px.append(
            finite_number(frame.get("median_disparity_px"), f"{path}: disparity at frame {index}")
        )
        pose = frame.get("board_pose_capture_gate")
        if not isinstance(pose, dict):
            raise ValueError(f"{path}: missing PnP pose at frame {index}")
        center = pose.get("board_center_m")
        if not isinstance(center, list) or len(center) != 3:
            raise ValueError(f"{path}: invalid PnP board center at frame {index}")
        pnp_z_m.append(finite_number(center[2], f"{path}: PnP z at frame {index}"))

    if any(later <= earlier for earlier, later in zip(timestamps, timestamps[1:])):
        raise ValueError(f"{path}: accepted frame timestamps must be strictly increasing")
    elapsed_s = [timestamp - timestamps[0] for timestamp in timestamps]
    return {
        "source_report": str(path),
        "calibration_sha256": calibration.get("sha256"),
        "axis_reference_id": reference.get("axis_reference_id"),
        "axis_position_m": finite_number(
            reference.get("board_plane_axis_position_m"), f"{path}: axis position"
        ),
        "capture": payload.get("capture"),
        "sample_count": len(frames),
        "elapsed_s": elapsed_s[-1],
        "series": {
            "stereo_depth_m": series_summary(depth_m, elapsed_s, "m"),
            "median_disparity_px": series_summary(disparity_px, elapsed_s, "px"),
            "pnp_board_center_z_m": series_summary(pnp_z_m, elapsed_s, "m"),
        },
        "correlations": {
            "stereo_depth_vs_median_disparity": correlation(depth_m, disparity_px),
            "stereo_depth_vs_pnp_board_center_z": correlation(depth_m, pnp_z_m),
            "median_disparity_vs_pnp_board_center_z": correlation(disparity_px, pnp_z_m),
        },
        "interpretation": (
            "A changing disparity together with a stable PnP board-center z is "
            "evidence of temporal stereo correspondence or relative image-geometry "
            "drift, not proof of physical board translation."
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    runs = [load_run(path) for path in args.report]
    calibration_hashes = {run["calibration_sha256"] for run in runs}
    reference_ids = {run["axis_reference_id"] for run in runs}
    if len(calibration_hashes) != 1 or None in calibration_hashes:
        raise ValueError("all reports must use one declared calibration SHA-256")
    if len(reference_ids) != 1 or None in reference_ids:
        raise ValueError("all reports must use one declared relative-axis reference")
    report = {
        "schema_version": OUTPUT_SCHEMA,
        "status": "DIAGNOSTIC_NOT_ACTIVATED",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "calibration_sha256": next(iter(calibration_hashes)),
        "axis_reference_id": next(iter(reference_ids)),
        "depth_reference_frame": "rectified_left_optical_frame",
        "runs": runs,
        "interpretation_boundary": (
            "This offline diagnostic analyses existing v7 evidence only. It does not "
            "change a calibration, runtime profile, camera setting, Measurement policy, "
            "navigation state, or actuator authorization."
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"runs": len(runs), "results": runs}, indent=2))
    print(f"Saved: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
