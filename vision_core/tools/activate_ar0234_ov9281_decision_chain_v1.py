#!/usr/bin/env python3
"""Activate a validated AR0234+OV9281 calibration profile for supervised decisions.

This tool never enables physical motion.  It authorizes only traceable ROS2
decision recommendations after all runtime evidence gates have been satisfied.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from datetime import UTC, datetime
from pathlib import Path

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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stereo-activation", required=True, type=Path)
    parser.add_argument("--independent-validation", required=True, type=Path)
    parser.add_argument("--extrinsic-candidate", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()

    with args.stereo_activation.open(encoding="utf-8") as handle:
        stereo_activation = json.load(handle)
    with args.independent_validation.open(encoding="utf-8") as handle:
        validation = json.load(handle)
    if stereo_activation.get("status") != "ACTIVE_CONDITIONAL_SUPERVISED_METRIC_ONLY":
        raise RuntimeError("stereo activation is not the approved conditional metric profile")
    if validation.get("status") != "PASS":
        raise RuntimeError("independent extrinsic validation must be PASS")

    candidate_sha = sha256_file(args.extrinsic_candidate)
    validation_sha = validation.get("source_sha256", {}).get("extrinsic_candidate")
    if candidate_sha != validation_sha:
        raise RuntimeError("extrinsic candidate SHA does not match independent validation")

    candidate = np.load(args.extrinsic_candidate, allow_pickle=False)
    for key in ("source_ar_intrinsic_sha256", "source_stereo_calibration_sha256"):
        if key not in candidate:
            raise RuntimeError(f"extrinsic candidate missing {key}")
    sources = validation.get("source_sha256", {})
    if scalar_text(candidate["source_stereo_calibration_sha256"]) != sources.get("ov9281_stereo_calibration"):
        raise RuntimeError("extrinsic/stereo validation source mismatch")
    if scalar_text(candidate["source_ar_intrinsic_sha256"]) != sources.get("ar0234_intrinsic"):
        raise RuntimeError("extrinsic/AR validation source mismatch")

    output = args.output_dir
    output.mkdir(parents=True, exist_ok=False)
    candidate_copy = output / "ar0234_to_ov9281_physical_left_extrinsic.npz"
    shutil.copy2(args.extrinsic_candidate, candidate_copy)
    copied_sha = sha256_file(candidate_copy)
    if copied_sha != candidate_sha:
        raise RuntimeError("copied extrinsic candidate hash mismatch")

    record = {
        "schema_version": "sie.calibration.decision_chain_activation.v1",
        "generated_utc": datetime.now(UTC).isoformat(),
        "status": "ACTIVE_CONDITIONAL_SUPERVISED_DECISION_RECOMMENDATION_ONLY",
        "calibration_profile": {
            "ar0234_to_ov9281_physical_left_extrinsic": {
                "path": str(candidate_copy),
                "sha256": copied_sha,
                "independent_validation_path": str(args.independent_validation),
                "independent_validation_sha256": sha256_file(args.independent_validation),
                "status": "INDEPENDENTLY_VALIDATED",
            },
            "source_sha256": {
                "ar0234_intrinsic": sources["ar0234_intrinsic"],
                "ov9281_stereo_calibration": sources["ov9281_stereo_calibration"],
            },
        },
        "authorization": {
            "observation": True,
            "target_suitability_interpretation": True,
            "metric_measurement_range_m": [0.5, 2.0],
            "navigation_decision_recommendation": True,
            "execution_authorized": False,
            "actuator_bridge": "DISABLED_PHASE_1",
            "motor_command_performed": False,
        },
        "mandatory_runtime_gates": [
            "calibration_sha256_exact_match",
            "fresh_measurement_with_time_and_reference_frame",
            "single_complete_target",
            "metric_range_0.5_to_2.0_m",
            "stop_measure_decide_cycle",
            "supervised_operator_review",
        ],
        "refusals": [
            "No physical actuator command is authorized by this activation.",
            "AR0234 Observation is not itself a metric Measurement.",
            "Multiple, truncated, stale, unlinked, or out-of-range targets must refuse decision recommendation.",
        ],
    }
    record_path = output / "ar0234_ov9281_decision_chain_activation.json"
    record_path.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"Activated supervised decision recommendation profile: {record_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
