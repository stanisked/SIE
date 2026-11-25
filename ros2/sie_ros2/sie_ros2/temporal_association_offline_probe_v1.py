#!/usr/bin/env python3
"""Offline CLI for a private Pi three-sensor metadata capture, no sensor I/O."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path

from sie_ros2.temporal_association_diagnostic_v1 import (
    TemporalAssociationTrialPolicy,
    associate_warmed_metadata,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--expect-sha256", required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    data = args.input.read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    if digest != args.expect_sha256:
        raise RuntimeError("INPUT_SHA256_MISMATCH")
    evidence = json.loads(data)

    # Experiment-reproduction thresholds, NOT a production policy.
    policy = TemporalAssociationTrialPolicy(
        policy_id="pi-rgb-anchor-25ms-stereo-50ms-ld19-TRIAL-ONLY",
        max_rgb_stereo_delta_ns=25_000_000,
        max_rgb_ld19_header_delta_ns=50_000_000,
    )
    result = associate_warmed_metadata(
        evidence, source_sha256=digest, policy=policy,
    )
    payload = (json.dumps(
        result, sort_keys=True, allow_nan=False, separators=(",", ":")
    ) + "\n").encode()
    report_sha = hashlib.sha256(payload).hexdigest()

    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        fd = os.open(
            args.output, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600,
        )
        with os.fdopen(fd, "wb") as file:
            file.write(payload)
        print("PRIVATE_OUTPUT_PATH:", args.output)
        print("PRIVATE_OUTPUT_SHA256:", report_sha)
    else:
        print("OUTPUT_NOT_SAVED: no --output")

    print("INPUT_SHA256: PASS")
    print("BOOT_ID:", result["boot_id"])
    print("ASSOCIATION_STATUS:", result["status"])
    for key, value in result["summary"].items():
        print(key.upper() + ":", value)
    print("PHYSICAL_LASER_ACQUISITION_TIME: UNKNOWN")
    print("MEASUREMENT_ADMISSION: DISABLED")
    print("WORLD_STATE_UPDATE: DISABLED")
    print("MOTOR_CONTROL: DISABLED")
    print("SIEE_TEMPORAL_ASSOCIATION_DIAGNOSTIC_V1: COMPLETE")


if __name__ == "__main__":
    main()
