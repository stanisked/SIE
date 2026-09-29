#!/usr/bin/env python3
"""Create a separate 0.5–4.5 m supervised decision-recommendation profile.

The original 0.5–2.0 m activation remains untouched.  This tool accepts only
the frozen v7 stereo profile, the activated AR0234-to-physical-left extrinsic,
and independently repeated far-range physical-depth evidence.  It never
authorizes execution or actuator access.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from datetime import UTC, datetime
from pathlib import Path


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def report(path: Path, ground_truth_m: float, max_error_m: float, max_mad_m: float) -> dict:
    item = json.loads(path.read_text(encoding="utf-8"))
    if item.get("schema_version") != "sie.ov9281.pi_stereo_physical_depth_check.v1":
        raise RuntimeError(f"unexpected depth report schema: {path}")
    if abs(float(item.get("ground_truth_m")) - ground_truth_m) > 1e-9:
        raise RuntimeError(f"wrong ground truth in {path}")
    error = float(item.get("median_absolute_error_m"))
    mad = float(item["depth_m"]["mad_m"])
    if error > max_error_m or mad > max_mad_m:
        raise RuntimeError(
            f"far-range gate rejected {path}: abs_error={error:.4f}, mad={mad:.4f}"
        )
    return {
        "path": str(path),
        "sha256": sha256(path),
        "ground_truth_m": ground_truth_m,
        "median_depth_m": float(item["depth_m"]["median_m"]),
        "median_absolute_error_m": error,
        "depth_mad_m": mad,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-stereo-activation", required=True, type=Path)
    parser.add_argument("--base-decision-activation", required=True, type=Path)
    parser.add_argument("--depth-check-3m", required=True, type=Path)
    parser.add_argument("--depth-check-3p5m-v1", required=True, type=Path)
    parser.add_argument("--depth-check-3p5m-v2", required=True, type=Path)
    parser.add_argument("--depth-check-4m", required=True, type=Path)
    parser.add_argument("--depth-check-4p5m-v1", required=True, type=Path)
    parser.add_argument("--depth-check-4p5m-v2", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()

    if args.output_dir.exists():
        raise RuntimeError(f"output directory already exists: {args.output_dir}")
    stereo = json.loads(args.base_stereo_activation.read_text(encoding="utf-8"))
    if stereo.get("status") != "ACTIVE_CONDITIONAL_SUPERVISED_METRIC_ONLY":
        raise RuntimeError("base stereo activation is not active conditional metric-only")
    if stereo.get("validated_range_m") != [0.5, 2.0]:
        raise RuntimeError("base stereo activation is not the frozen 0.5–2.0 profile")
    calibration_path = Path(str(stereo.get("calibration_path", "")))
    calibration_sha = str(stereo.get("calibration_sha256", ""))
    if not calibration_path.is_file() or sha256(calibration_path) != calibration_sha:
        raise RuntimeError("base stereo calibration is absent or SHA-mismatched")

    base = json.loads(args.base_decision_activation.read_text(encoding="utf-8"))
    if base.get("status") != "ACTIVE_CONDITIONAL_SUPERVISED_DECISION_RECOMMENDATION_ONLY":
        raise RuntimeError("base decision activation is not active")
    profile = base.get("calibration_profile", {})
    sources = profile.get("source_sha256", {})
    if sources.get("ov9281_stereo_calibration") != calibration_sha:
        raise RuntimeError("base decision/stereo SHA mismatch")
    extrinsic = profile.get("ar0234_to_ov9281_physical_left_extrinsic", {})
    extrinsic_path = Path(str(extrinsic.get("path", "")))
    extrinsic_sha = str(extrinsic.get("sha256", ""))
    if not extrinsic_path.is_file() or sha256(extrinsic_path) != extrinsic_sha:
        raise RuntimeError("base extrinsic is absent or SHA-mismatched")

    evidence = {
        "3.0m": [report(args.depth_check_3m, 3.0, 0.10, 0.04)],
        "3.5m": [
            report(args.depth_check_3p5m_v1, 3.5, 0.18, 0.04),
            report(args.depth_check_3p5m_v2, 3.5, 0.18, 0.04),
        ],
        "4.0m": [report(args.depth_check_4m, 4.0, 0.20, 0.04)],
        "4.5m": [
            report(args.depth_check_4p5m_v1, 4.5, 0.30, 0.04),
            report(args.depth_check_4p5m_v2, 4.5, 0.30, 0.04),
        ],
    }
    for label, maximum_difference_m in (("3.5m", 0.06), ("4.5m", 0.06)):
        first, second = evidence[label]
        if abs(first["median_depth_m"] - second["median_depth_m"]) > maximum_difference_m:
            raise RuntimeError(f"{label} independent-repeat disagreement exceeds {maximum_difference_m} m")

    args.output_dir.mkdir(parents=True)
    copied_extrinsic = args.output_dir / "ar0234_to_ov9281_physical_left_extrinsic.npz"
    shutil.copy2(extrinsic_path, copied_extrinsic)
    if sha256(copied_extrinsic) != extrinsic_sha:
        raise RuntimeError("copied extrinsic SHA mismatch")

    activation = {
        "schema_version": "sie.calibration.decision_chain_activation.v2",
        "generated_utc": datetime.now(UTC).isoformat(),
        "status": "ACTIVE_CONDITIONAL_SUPERVISED_DECISION_RECOMMENDATION_ONLY",
        "supersedes": {
            "path": str(args.base_decision_activation),
            "sha256": sha256(args.base_decision_activation),
            "validated_range_m": [0.5, 2.0],
        },
        "calibration_profile": {
            "ar0234_to_ov9281_physical_left_extrinsic": {
                "path": str(copied_extrinsic),
                "sha256": extrinsic_sha,
                "status": "INDEPENDENTLY_VALIDATED",
            },
            "source_sha256": sources,
            "base_stereo_activation": {
                "path": str(args.base_stereo_activation),
                "sha256": sha256(args.base_stereo_activation),
                "calibration_path": str(calibration_path),
                "calibration_sha256": calibration_sha,
            },
        },
        "extended_range_evidence": {
            "range_m": [0.5, 4.5],
            "far_physical_depth_checks": evidence,
            "limits": {
                "3.0m_max_absolute_error_m": 0.10,
                "3.5m_max_absolute_error_m": 0.18,
                "4.0m_max_absolute_error_m": 0.20,
                "4.5m_max_absolute_error_m": 0.30,
                "max_depth_mad_m": 0.04,
                "max_independent_repeat_difference_m": 0.06,
            },
            "five_m_status": "DIAGNOSTIC_ONLY_OUTSIDE_PROFILE",
        },
        "authorization": {
            "observation": True,
            "target_suitability_interpretation": True,
            "metric_measurement_range_m": [0.5, 4.5],
            "navigation_decision_recommendation": True,
            "execution_authorized": False,
            "actuator_bridge": "DISABLED_PHASE_1",
            "motor_command_performed": False,
        },
        "mandatory_runtime_gates": [
            "calibration_sha256_exact_match",
            "fresh_measurement_with_time_and_reference_frame",
            "single_static_3d_association",
            "metric_range_0.5_to_4.5_m",
            "supervised_operator_review",
        ],
        "refusals": [
            "No physical actuator command is authorized.",
            "Dynamic fusion while moving remains prohibited.",
            "Five metres is diagnostic only and outside this profile.",
        ],
    }
    destination = args.output_dir / "ar0234_ov9281_decision_chain_activation.json"
    destination.write_text(json.dumps(activation, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"Activated 0.5–4.5 m supervised decision profile: {destination}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
