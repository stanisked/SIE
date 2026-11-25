#!/usr/bin/env python3
"""Read-only Pi host-clock sampling for diagnostic Timebase Bridge v1.

Not a ROS node and not a physical sensor acquisition time validator.
Writes private JSON only when --output is explicitly supplied.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import time

from sie_ros2.host_timebase_bridge_v1 import (
    HostBridgeTestPolicy, make_host_timebase_bridge,
)


def collect_host_clock_brackets(*, count: int, interval_ms: float) -> list[dict]:
    if type(count) is not int or not 3 <= count <= 1000:
        raise ValueError("count must be within [3, 1000]")
    if type(interval_ms) not in (int, float) or not 0 <= interval_ms <= 1000:
        raise ValueError("interval_ms must be between 0 and 1000")
    rows = []
    for i in range(count):
        before = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        realtime = time.clock_gettime_ns(time.CLOCK_REALTIME)
        after = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        rows.append({
            "monotonic_before_ns": before,
            "realtime_ns": realtime,
            "monotonic_after_ns": after,
        })
        if i + 1 < count:
            time.sleep(interval_ms / 1000)
    return rows


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=200)
    parser.add_argument("--interval-ms", type=float, default=10.0)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    # These are diagnostic *trial* limits, not SIE admission thresholds.
    policy = HostBridgeTestPolicy(
        policy_id="host-pi-clock-probe-v1-EXPERIMENTAL",
        max_bracket_ns=100_000,
        max_sample_gap_ns=100_000_000,
        max_offset_span_ns=100_000,
        max_consecutive_offset_jump_ns=50_000,
        max_between_window_offset_change_ns=100_000,
        max_age_ns=1_000_000_000,
        max_extrapolation_ns=250_000_000,
        max_future_skew_ns=1_000_000,
    )

    boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    if not boot_id:
        raise RuntimeError("Pi boot_id is unavailable")
    created_at = datetime.now(timezone.utc).isoformat()
    brackets = collect_host_clock_brackets(
        count=args.samples, interval_ms=args.interval_ms,
    )
    bridge = make_host_timebase_bridge(
        brackets, boot_id=boot_id, policy=policy,
    )
    audit = {
        "schema_version": "sie.host.clock_probe.v1.diagnostic",
        "created_at_utc": created_at,
        "boot_id": boot_id,
        "policy": asdict(policy),
        "sample_count": len(brackets),
        "samples": brackets,
        "bridge": bridge,
        "physical_acquisition_time": "UNKNOWN",
        "measurement_authorized": False,
        "world_state_update_authorized": False,
        "motor_control_authorized": False,
    }
    data = (json.dumps(audit, sort_keys=True, allow_nan=False, separators=(",", ":"))
            + "\n").encode()
    digest = hashlib.sha256(data).hexdigest()

    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        # Create-only private file; never overwrite a prior evidence artifact.
        fd = os.open(args.output, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, "wb") as handle:
            handle.write(data)
        print("PRIVATE_EVIDENCE_PATH:", args.output)
        print("PRIVATE_EVIDENCE_SHA256:", digest)
    else:
        print("EVIDENCE_NOT_WRITTEN: no --output")

    print("SAMPLES:", len(brackets))
    print("BOOT_ID:", boot_id)
    print("BRIDGE_STATUS:", bridge["status"])
    print("OFFSET_NS:", bridge["offset_ns"])
    print("OBSERVED_SPAN_NS:", bridge["observed_offset_span_ns"])
    print("EMPIRICAL_ENVELOPE_NS:", bridge["empirical_offset_envelope_ns"])
    print("PHYSICAL_ACQUISITION_TIME: UNKNOWN")
    print("MEASUREMENT_ADMISSION: DISABLED")
    print("WORLD_STATE_UPDATE: DISABLED")
    print("MOTOR_CONTROL: DISABLED")
    print("SIEE_HOST_TIMEBASE_PROBE_V1: COMPLETE")


if __name__ == "__main__":
    main()
