#!/usr/bin/env python3
"""Create an immutable static-only successor of an AR0234+OV9281 activation profile.

The source file is never changed. The successor preserves calibration sources,
range, and execution prohibition while making the already-implemented
upper-body geometry and temporal-static policies explicit.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path


GEOMETRY_POLICY = {
    "schema_version": "sie.ar0234.upper_body_static_gate.v1",
    "allow_bottom_truncation": True,
    "require_untruncated_edges": ["top", "left", "right"],
    "require_single_target": True,
    "static_scene_only": True,
    "dynamic_fusion_permitted": False,
}

TEMPORAL_STATIC_GATE = {
    "schema_version": "sie.temporal.static_gate.v1",
    "apply_before_stereo_fusion": True,
    "minimum_consecutive_observations": 2,
    "max_center_delta_px": 12.0,
    "max_area_relative_change": 0.10,
    "candidate_max_age_s": 5.0,
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-profile", required=True, type=Path)
    parser.add_argument("--output-profile", required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    source = args.source_profile.expanduser()
    output = args.output_profile.expanduser()
    if not source.is_file():
        raise FileNotFoundError(f"source profile is missing: {source}")
    if output.exists():
        raise RuntimeError(f"refusing to overwrite existing profile: {output}")

    source_sha256 = _sha256(source)
    profile = json.loads(source.read_text(encoding="utf-8"))
    if (
        profile.get("status")
        != "ACTIVE_CONDITIONAL_SUPERVISED_DECISION_RECOMMENDATION_ONLY"
    ):
        raise RuntimeError("source profile is not the expected active supervised profile")
    authorization = profile.get("authorization")
    if type(authorization) is not dict or authorization.get("execution_authorized") is not False:
        raise RuntimeError("source profile must explicitly prohibit execution")
    if authorization.get("actuator_bridge") != "DISABLED_PHASE_1":
        raise RuntimeError("source profile must keep the phase-1 bridge disabled")
    range_m = authorization.get("metric_measurement_range_m")
    if (
        type(range_m) is not list
        or len(range_m) != 2
        or [float(value) for value in range_m] != [0.5, 4.5]
    ):
        raise RuntimeError("source profile must retain the validated 0.5..4.5 m range")
    if profile.get("target_geometry_policy") is not None:
        raise RuntimeError("source already contains a target_geometry_policy")
    if profile.get("temporal_static_gate") is not None:
        raise RuntimeError("source already contains a temporal_static_gate")

    successor = copy.deepcopy(profile)
    successor["target_geometry_policy"] = GEOMETRY_POLICY
    successor["temporal_static_gate"] = TEMPORAL_STATIC_GATE
    successor["activation_successor"] = {
        "schema_version": "sie.activation.successor.v1",
        "source_profile_path": str(source),
        "source_profile_sha256": source_sha256,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "change": (
            "make existing static-only upper-body bottom-truncation and "
            "temporal gate policies explicit; execution remains prohibited"
        ),
    }

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(successor, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"Created static-only successor profile: {output}")
    print(f"Source SHA-256: {source_sha256}")
    print(f"Successor SHA-256: {_sha256(output)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
