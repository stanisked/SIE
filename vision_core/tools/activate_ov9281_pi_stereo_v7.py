#!/usr/bin/env python3
"""Create a traceable supervised-only activation bundle for Pi OV9281 stereo v7."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from datetime import datetime, timezone
from pathlib import Path


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_depth_check(path: Path, expected_gt: float) -> dict:
    report = json.loads(path.read_text())
    if report.get("schema_version") != "sie.ov9281.pi_stereo_physical_depth_check.v1":
        raise RuntimeError(f"unexpected depth-check schema: {path}")
    if abs(float(report.get("ground_truth_m")) - expected_gt) > 1e-9:
        raise RuntimeError(f"ground-truth mismatch: {path}")
    if float(report.get("median_absolute_error_m")) > 0.03:
        raise RuntimeError(f"depth error exceeds 30 mm: {path}")
    if float(report["depth_m"]["mad_m"]) > 0.02:
        raise RuntimeError(f"depth MAD exceeds 20 mm: {path}")
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--depth-check-0p5m", type=Path, required=True)
    parser.add_argument("--depth-check-1m", type=Path, required=True)
    parser.add_argument("--depth-check-2m", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        raise RuntimeError(f"output directory is not empty: {args.output_dir}")
    if not args.candidate.is_file():
        raise FileNotFoundError(args.candidate)

    checks = {
        "0.5m": (args.depth_check_0p5m, load_depth_check(args.depth_check_0p5m, 0.5)),
        "1.0m": (args.depth_check_1m, load_depth_check(args.depth_check_1m, 1.0)),
        "2.0m": (args.depth_check_2m, load_depth_check(args.depth_check_2m, 2.0)),
    }
    args.output_dir.mkdir(parents=True)
    candidate_copy = args.output_dir / "ov9281_pi_stereo_v7.npz"
    shutil.copy2(args.candidate, candidate_copy)

    validation = {}
    for name, (path, report) in checks.items():
        validation[name] = {
            "report_path": str(path),
            "report_sha256": sha256(path),
            "ground_truth_m": report["ground_truth_m"],
            "median_depth_m": report["depth_m"]["median_m"],
            "median_absolute_error_m": report["median_absolute_error_m"],
            "depth_mad_m": report["depth_m"]["mad_m"],
        }

    activation = {
        "schema_version": "sie.stereo.calibration_activation.v1",
        "calibration_id": "ov9281_pi_stereo_candidate_v7",
        "status": "ACTIVE_CONDITIONAL_SUPERVISED_METRIC_ONLY",
        "activated_at_utc": datetime.now(timezone.utc).isoformat(),
        "calibration_path": str(candidate_copy),
        "calibration_sha256": sha256(candidate_copy),
        "validated_range_m": [0.5, 2.0],
        "physical_depth_validation": validation,
        "constraints": {
            "capture_mode": "MJPG 2560x800 @ 60 FPS",
            "auto_exposure": 3,
            "captured_first_semantics": "physical_right",
            "captured_second_semantics": "physical_left",
            "metric_measurement_permitted": True,
            "navigation_decision_permitted": False,
            "actuator_command_permitted": False,
            "hidden_depth_scale_or_offset_correction_allowed": False,
        },
    }
    activation_path = args.output_dir / "activation_record.json"
    activation_path.write_text(json.dumps(activation, indent=2), encoding="utf-8")
    print(json.dumps(activation, indent=2))
    print(f"Saved: {candidate_copy}")
    print(f"Saved: {activation_path}")


if __name__ == "__main__":
    main()
