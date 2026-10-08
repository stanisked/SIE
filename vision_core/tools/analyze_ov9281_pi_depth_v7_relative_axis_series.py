#!/usr/bin/env python3
"""Analyse relative depth translation from pose-aware OV9281 v7 point reports.

This tool deliberately evaluates only changes between board positions on one
declared physical axis.  It does not infer the unknown constant transform from
the left lens front rim to the left optical center, and it cannot activate a
calibration or an SIE Measurement policy.
"""

from __future__ import annotations

import argparse
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


SOURCE_SCHEMA = "sie.ov9281.pi_stereo_pose_aware_planar_depth_diagnostic.v1"


def load_point(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("schema_version") != SOURCE_SCHEMA:
        raise ValueError(f"{path}: unexpected source schema")
    if payload.get("status") != "DIAGNOSTIC_NOT_ACTIVATED":
        raise ValueError(f"{path}: source report is not a diagnostic")
    if payload.get("validation_mode") != "relative_axis_translation":
        raise ValueError(f"{path}: report is not relative_axis_translation")
    reference = payload.get("relative_axis_reference")
    if not isinstance(reference, dict):
        raise ValueError(f"{path}: missing relative axis reference")
    calibration = payload.get("calibration")
    depth = payload.get("stereo_depth_m")
    if not isinstance(calibration, dict) or not isinstance(depth, dict):
        raise ValueError(f"{path}: missing calibration or depth summary")
    axis_position = reference.get("board_plane_axis_position_m")
    uncertainty = reference.get("standard_uncertainty_m")
    median_depth = depth.get("median_m")
    depth_mad = depth.get("mad_m")
    if not all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in (
        axis_position,
        uncertainty,
        median_depth,
        depth_mad,
    )):
        raise ValueError(f"{path}: non-finite axis or depth values")
    return {
        "source_report": str(path),
        "axis_reference_id": reference.get("axis_reference_id"),
        "axis_position_m": float(axis_position),
        "axis_position_standard_uncertainty_m": float(uncertainty),
        "median_depth_m": float(median_depth),
        "depth_mad_m": float(depth_mad),
        "calibration_sha256": calibration.get("sha256"),
        "calibration_path": calibration.get("path"),
        "depth_reference_frame": payload.get("depth_reference_frame"),
        "pose_capture_gate": payload.get("pose_capture_gate"),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--max-relative-translation-error",
        type=float,
        help="optional criterion for |delta_z - delta_axis| / |delta_axis|",
    )
    args = parser.parse_args()
    if len(args.report) < 2:
        raise ValueError("at least two relative point reports are required")
    if args.max_relative_translation_error is not None and args.max_relative_translation_error < 0.0:
        raise ValueError("relative translation criterion must be non-negative")

    points = [load_point(path) for path in args.report]
    reference_ids = {point["axis_reference_id"] for point in points}
    calibration_hashes = {point["calibration_sha256"] for point in points}
    frames = {point["depth_reference_frame"] for point in points}
    if len(reference_ids) != 1 or None in reference_ids:
        raise ValueError("all reports must use one declared axis_reference_id")
    if len(calibration_hashes) != 1 or None in calibration_hashes:
        raise ValueError("all reports must use the exact same calibration SHA-256")
    if frames != {"rectified_left_optical_frame"}:
        raise ValueError("all reports must express depth in rectified_left_optical_frame")
    points.sort(key=lambda item: item["axis_position_m"])
    if any(
        later["axis_position_m"] <= earlier["axis_position_m"]
        for earlier, later in zip(points, points[1:])
    ):
        raise ValueError("axis positions must be distinct and strictly increasing")

    anchor = points[0]
    deltas: list[dict[str, Any]] = []
    for point in points[1:]:
        physical_delta_m = point["axis_position_m"] - anchor["axis_position_m"]
        stereo_delta_m = point["median_depth_m"] - anchor["median_depth_m"]
        translation_error_m = stereo_delta_m - physical_delta_m
        relative_error = abs(translation_error_m) / physical_delta_m
        criterion: dict[str, Any] = {"status": "NOT_DECLARED"}
        if args.max_relative_translation_error is not None:
            criterion = {
                "status": "PASS" if relative_error <= args.max_relative_translation_error else "FAIL",
                "maximum_relative_translation_error": args.max_relative_translation_error,
            }
        deltas.append(
            {
                "from_source_report": anchor["source_report"],
                "to_source_report": point["source_report"],
                "physical_axis_translation_m": physical_delta_m,
                "physical_axis_translation_standard_uncertainty_m": math.hypot(
                    anchor["axis_position_standard_uncertainty_m"],
                    point["axis_position_standard_uncertainty_m"],
                ),
                "stereo_depth_translation_m": stereo_delta_m,
                "stereo_translation_minus_physical_translation_m": translation_error_m,
                "absolute_relative_translation_error": relative_error,
                "declared_criterion": criterion,
            }
        )
    report = {
        "schema_version": "sie.ov9281.pi_stereo_relative_axis_translation_diagnostic.v1",
        "status": "DIAGNOSTIC_NOT_ACTIVATED",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "axis_reference_id": anchor["axis_reference_id"],
        "calibration_sha256": anchor["calibration_sha256"],
        "depth_reference_frame": "rectified_left_optical_frame",
        "points": points,
        "anchor_source_report": anchor["source_report"],
        "translation_deltas": deltas,
        "interpretation_boundary": (
            "This report evaluates relative axial translation only. The unknown "
            "constant transform from the physical-left lens front rim to the "
            "optical center is not estimated or applied. No calibration, runtime "
            "policy, Measurement policy, navigation, or actuator authorization is changed."
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"points": len(points), "translation_deltas": deltas}, indent=2))
    print(f"Saved: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
