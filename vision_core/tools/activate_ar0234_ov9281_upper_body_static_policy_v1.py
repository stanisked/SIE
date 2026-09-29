#!/usr/bin/env python3
"""Activate a narrow static upper-body geometry policy from live Pi evidence.

The supplied base profile remains immutable. This creates a separately
SHA-bound recommendation-only activation that permits only bottom truncation
for a single static upper-body target. It cannot authorize a motor, actuator
bridge, or dynamic fusion.
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


def summary(lines: list[dict], activation_sha: str, extrinsic_sha: str) -> dict:
    if len(lines) < 20:
        raise RuntimeError(f"need at least 20 complete JSON cycles, got {len(lines)}")
    statuses = {str(item.get("status")) for item in lines}
    if not statuses <= {"SUCCESS", "NO_TARGET"}:
        raise RuntimeError(f"unexpected live-window statuses: {sorted(statuses)}")
    no_target = [item for item in lines if item.get("status") == "NO_TARGET"]
    if any(item.get("reason") != "EDGE_TRUNCATED:bottom" for item in no_target):
        raise RuntimeError("NO_TARGET evidence is not exclusively bottom truncation")
    success = [item for item in lines if item.get("status") == "SUCCESS"]
    if len(success) < 10:
        raise RuntimeError(f"need at least 10 SUCCESS cycles, got {len(success)}")
    for item in success:
        if item.get("reason") != "UNIQUE_STATIC_3D_ASSOCIATION":
            raise RuntimeError("SUCCESS evidence is not static 3D association")
        if item.get("calibration", {}).get("sha256") != extrinsic_sha:
            raise RuntimeError("live-window extrinsic SHA mismatch")
        if item.get("provenance", {}).get("activation_profile_sha256") != activation_sha:
            raise RuntimeError("live-window activation SHA mismatch")
    ranges = [float(item["range_m"]) for item in success]
    bearings = [float(item["bearing_deg"]) for item in success]
    mads = [float(item["depth_mad_m"]) for item in success]
    if max(ranges) - min(ranges) > 0.05:
        raise RuntimeError("live SUCCESS range span exceeds 0.05 m")
    if max(bearings) - min(bearings) > 0.20:
        raise RuntimeError("live SUCCESS bearing span exceeds 0.20 deg")
    if max(mads) > 0.05:
        raise RuntimeError("live SUCCESS depth MAD exceeds 0.05 m")
    return {
        "cycle_count": len(lines),
        "status_counts": {"SUCCESS": len(success), "NO_TARGET": len(no_target)},
        "required_no_target_reason": "EDGE_TRUNCATED:bottom",
        "success_range_m": {"min": min(ranges), "max": max(ranges), "span": max(ranges) - min(ranges)},
        "success_bearing_deg": {"min": min(bearings), "max": max(bearings), "span": max(bearings) - min(bearings)},
        "success_depth_mad_m": {"min": min(mads), "max": max(mads)},
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-decision-activation", required=True, type=Path)
    parser.add_argument("--live-window", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()

    if args.output_dir.exists():
        raise RuntimeError(f"output directory already exists: {args.output_dir}")
    base = json.loads(args.base_decision_activation.read_text(encoding="utf-8"))
    if base.get("status") != "ACTIVE_CONDITIONAL_SUPERVISED_DECISION_RECOMMENDATION_ONLY":
        raise RuntimeError("base decision activation is not active")
    authorization = base.get("authorization", {})
    if (
        authorization.get("metric_measurement_range_m") != [0.5, 4.5]
        or authorization.get("execution_authorized") is not False
        or authorization.get("actuator_bridge") != "DISABLED_PHASE_1"
        or authorization.get("motor_command_performed") is not False
    ):
        raise RuntimeError("base must be the 0.5–4.5 m recommendation-only profile")
    profile = base.get("calibration_profile", {})
    extrinsic = profile.get("ar0234_to_ov9281_physical_left_extrinsic", {})
    extrinsic_path = Path(str(extrinsic.get("path", "")))
    extrinsic_sha = str(extrinsic.get("sha256", ""))
    if not extrinsic_path.is_file() or sha256(extrinsic_path) != extrinsic_sha:
        raise RuntimeError("base extrinsic is absent or SHA-mismatched")

    parsed: list[dict] = []
    for number, raw in enumerate(args.live_window.read_text(encoding="utf-8").splitlines(), 1):
        if not raw.strip():
            continue
        try:
            parsed.append(json.loads(raw))
        except json.JSONDecodeError as error:
            raise RuntimeError(f"invalid JSON at live-window line {number}") from error
    evidence = summary(parsed, sha256(args.base_decision_activation), extrinsic_sha)

    args.output_dir.mkdir(parents=True)
    copied_extrinsic = args.output_dir / "ar0234_to_ov9281_physical_left_extrinsic.npz"
    shutil.copy2(extrinsic_path, copied_extrinsic)
    if sha256(copied_extrinsic) != extrinsic_sha:
        raise RuntimeError("copied extrinsic SHA mismatch")
    evidence_copy = args.output_dir / "live_static_upper_body_window.jsonl"
    shutil.copy2(args.live_window, evidence_copy)
    if sha256(evidence_copy) != sha256(args.live_window):
        raise RuntimeError("copied live evidence SHA mismatch")

    activation = {
        "schema_version": "sie.calibration.decision_chain_activation.v3",
        "generated_utc": datetime.now(UTC).isoformat(),
        "status": "ACTIVE_CONDITIONAL_SUPERVISED_DECISION_RECOMMENDATION_ONLY",
        "supersedes": {
            "path": str(args.base_decision_activation),
            "sha256": sha256(args.base_decision_activation),
            "validated_range_m": [0.5, 4.5],
        },
        "calibration_profile": {
            "ar0234_to_ov9281_physical_left_extrinsic": {
                "path": str(copied_extrinsic),
                "sha256": extrinsic_sha,
                "status": "INDEPENDENTLY_VALIDATED",
            },
            "source_sha256": profile["source_sha256"],
            "base_stereo_activation": profile["base_stereo_activation"],
        },
        "target_geometry_policy": {
            "schema_version": "sie.ar0234.upper_body_static_gate.v1",
            "allow_bottom_truncation": True,
            "require_untruncated_edges": ["top", "left", "right"],
            "require_single_target": True,
            "static_scene_only": True,
            "dynamic_fusion_permitted": False,
        },
        "live_static_upper_body_evidence": {
            "path": str(evidence_copy),
            "sha256": sha256(evidence_copy),
            "summary": evidence,
            "limits": {
                "minimum_cycle_count": 20,
                "minimum_success_count": 10,
                "success_range_span_m": 0.05,
                "success_bearing_span_deg": 0.20,
                "max_success_depth_mad_m": 0.05,
            },
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
            "upper_body_only_bottom_edge_exception",
            "metric_range_0.5_to_4.5_m",
            "supervised_operator_review",
        ],
        "refusals": [
            "Top, left, or right truncation remains rejected.",
            "Multiple targets remain rejected.",
            "No physical actuator command is authorized.",
            "Dynamic fusion while moving remains prohibited.",
        ],
    }
    destination = args.output_dir / "ar0234_ov9281_decision_chain_activation.json"
    destination.write_text(json.dumps(activation, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"Activated static upper-body recommendation profile: {destination}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
