#!/usr/bin/env python3
"""Re-evaluate historical RGB-anchored metadata freshness at an explicit time.

Read-only offline diagnostic. No ROS, sensor, wall-clock, or motor I/O.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path

from sie_ros2.temporal_association_freshness_v1 import (
    FreshnessTrialPolicy, evaluate_association_freshness,
)


def _verified_json(path: Path, expected: str):
    raw = path.read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    if digest != expected:
        raise RuntimeError(f"INPUT_SHA256_MISMATCH: {path.name}")
    return json.loads(raw)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", type=Path, required=True)
    parser.add_argument("--expect-capture-sha256", required=True)
    parser.add_argument("--associations", type=Path, required=True)
    parser.add_argument("--expect-associations-sha256", required=True)
    parser.add_argument("--evaluation-lag-ms", type=int, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    if args.evaluation_lag_ms < 0:
        raise ValueError("evaluation lag must be nonnegative")
    capture = _verified_json(args.capture, args.expect_capture_sha256)
    associations = _verified_json(args.associations, args.expect_associations_sha256)
    if (capture["boot_id"] != associations["boot_id"]
            or associations["source_evidence_sha256"] != args.expect_capture_sha256):
        raise RuntimeError("ASSOCIATION_CAPTURE_MISMATCH")

    bridge = capture["bridge"]
    # The trial evaluation time is historical, not current Pi time.
    evaluation_ns = bridge["last_sample_monotonic_ns"] + args.evaluation_lag_ms * 1_000_000

    policy = FreshnessTrialPolicy(
        policy_id="pi-recorded-250ms-source-age-1s-bridge-TRIAL-ONLY",
        max_rgb_age_ns=250_000_000,
        max_stereo_age_ns=250_000_000,
        max_ld19_ros_header_age_ns=250_000_000,
        max_bridge_holdover_ns=1_000_000_000,
        max_future_skew_ns=1_000_000,
    )
    result = evaluate_association_freshness(
        associations, bridge, evaluation_monotonic_ns=evaluation_ns,
        boot_id=capture["boot_id"], policy=policy,
    )
    result["source_association_report_sha256"] = args.expect_associations_sha256
    result["historical_offline_evaluation"] = True
    result["evaluation_lag_from_bracket_window_end_ms"] = args.evaluation_lag_ms

    payload = (json.dumps(
        result, sort_keys=True, separators=(",", ":"), allow_nan=False
    ) + "\n").encode()
    digest = hashlib.sha256(payload).hexdigest()

    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        fd = os.open(args.output, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
        with os.fdopen(fd, "wb") as file:
            file.write(payload)
        print("PRIVATE_OUTPUT_PATH:", args.output)
        print("PRIVATE_OUTPUT_SHA256:", digest)
    else:
        print("OUTPUT_NOT_SAVED: no --output")

    print("SOURCE_SHA256: PASS")
    print("HISTORICAL_EVALUATION: TRUE")
    print("EVALUATION_LAG_FROM_BRIDGE_END_MS:", args.evaluation_lag_ms)
    print("FRESHNESS_STATUS:", result["status"])
    for key, value in result["summary"].items():
        print(key.upper() + ":", value)
    print("AGE_FIELD_PRESENT:", bool(result["rows"]) and all(
        isinstance(row.get("age_ns"), dict) for row in result["rows"]
    ))
    print("PHYSICAL_LASER_ACQUISITION_TIME: UNKNOWN")
    print("MEASUREMENT_ADMISSION: DISABLED")
    print("WORLD_STATE_UPDATE: DISABLED")
    print("MOTOR_CONTROL: DISABLED")
    print("SIEE_TEMPORAL_FRESHNESS_V1: COMPLETE")


if __name__ == "__main__":
    main()
