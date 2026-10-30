#!/usr/bin/env python3
"""Summarize sampled person/depth availability from a frozen Shadow replay.

The audit timestamps are host capture times, not exposure synchronization.
This diagnostic does not establish person identity, physical range accuracy,
continuous target availability between samples, Measurement or motor authority.
"""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
from statistics import median


COHERENT = "COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def analyze(root: Path, replay_path: Path, scenario: str) -> dict:
    replay = json.loads(replay_path.read_text(encoding="utf-8"))
    if (replay.get("status") != "OFFLINE_DIAGNOSTIC_ONLY"
            or Path(replay["source_root"]).resolve() != root.resolve()
            or not isinstance(replay.get("calibration_sha256"), dict)
            or not isinstance(replay.get("cycles"), list)):
        raise ValueError("replay/root or diagnostic provenance mismatch")
    saved = replay["cycles"]
    files = sorted(root.glob("*/*/audit.json"))
    if not saved or len(files) != len(saved):
        raise ValueError("every captured audit must have exactly one replay row")
    by_id = {row["cycle_id"]: row for row in saved}
    if len(by_id) != len(saved) or set(by_id) != {p.parent.name for p in files}:
        raise ValueError("audit/replay cycle identity mismatch")

    rows = []
    for file in files:
        audit = json.loads(file.read_text(encoding="utf-8"))
        cid = file.parent.name
        raw = by_id[cid]
        if (audit.get("cycle_id") != cid or audit.get("calibration") != replay["calibration_sha256"]
                or raw.get("source_status") != audit.get("status")
                or raw.get("source_reason") != audit.get("reason")
                or audit.get("metric_measurement_authorized") is not False
                or audit.get("navigation_eligible") is not False
                or audit.get("actuator_access") is not False):
            raise ValueError(f"audit/replay or diagnostic safety mismatch: {cid}")
        if "mask_sha256" in raw:
            mask = file.parent / f"{cid}.png"
            if not mask.is_file() or sha256(mask) != raw["mask_sha256"]:
                raise ValueError(f"replayed mask changed or missing: {cid}")
        timestamp = datetime.fromisoformat(audit["timestamp"])
        if timestamp.tzinfo is None:
            raise ValueError(f"timestamp has no timezone: {cid}")
        target = (audit.get("ar0234_dynamic_target") or {}).get("candidate")
        center = target.get("center_x_px") if isinstance(target, dict) else None
        if center is not None and not math.isfinite(center):
            raise ValueError(f"nonfinite RGB center: {cid}")
        depth = raw["mask_depth"]
        if depth["status"] == COHERENT and not isinstance(target, dict):
            raise ValueError(f"coherent depth has no current RGB person candidate: {cid}")
        pairing = audit.get("pairing") or {}
        if pairing.get("exposure_synchronization_proven") is not False:
            raise ValueError(f"unexpected exposure synchronization claim: {cid}")
        rows.append({"cycle_id": cid, "timestamp": timestamp.isoformat(),
                     "audit_sha256": sha256(file), "source_status": audit["status"],
                     "source_reason": audit["reason"], "rgb_person_candidate": isinstance(target, dict),
                     "rgb_center_x_px": center, "mask_depth_status": depth["status"],
                     "depth_m": depth.get("candidate_depth_m"),
                     "mad_m": depth.get("candidate_mad_m"),
                     "selected_sample_count": depth.get("selected_sample_count"),
                     "capture_pair_skew_ms": pairing.get("capture_pair_skew_ms"),
                     "host_capture_to_result_s": (audit.get("processing_timing") or {}).get(
                         "host_capture_start_to_result_ready_s")})
    rows.sort(key=lambda row: datetime.fromisoformat(row["timestamp"]))
    times = [datetime.fromisoformat(row["timestamp"]) for row in rows]
    intervals = [(b - a).total_seconds() for a, b in zip(times, times[1:])]
    if any(value <= 0 for value in intervals):
        raise ValueError("nonmonotonic or repeated host capture times")
    statuses = [row["mask_depth_status"] for row in rows]
    coherent = [index for index, status in enumerate(statuses) if status == COHERENT]
    gaps = [{"from_cycle_id": rows[a]["cycle_id"], "to_cycle_id": rows[b]["cycle_id"],
             "intervening_refusal_count": b - a - 1,
             "between_coherent_observations_s": (times[b] - times[a]).total_seconds()}
            for a, b in zip(coherent, coherent[1:]) if b > a + 1]
    transitions = Counter(f"{a} -> {b}" for a, b in zip(statuses, statuses[1:]))
    centers = [row["rgb_center_x_px"] for row in rows if row["rgb_center_x_px"] is not None]
    target_count = sum(row["rgb_person_candidate"] for row in rows)
    skews = [row["capture_pair_skew_ms"] for row in rows]
    if any(type(value) not in (int, float) or not math.isfinite(value) or value < 0
           for value in skews):
        raise ValueError("missing or invalid host capture skew")
    return {
        "schema_version": "sie.shadow_temporal_depth_diagnostic.v1",
        "status": "SAMPLED_TEMPORAL_AVAILABILITY_DIAGNOSTIC_ONLY",
        "operator_scenario_annotation": scenario,
        "source_root": str(root.resolve()), "replay_sha256": sha256(replay_path),
        "calibration_sha256": replay["calibration_sha256"],
        "diagnostic_depth_range_m": replay["diagnostic_depth_range_m"],
        "activated_depth_range_m": replay["activated_depth_range_m"],
        "reference_frame": "ov9281_physical_left_optical_frame", "unit": "m",
        "confidence": None, "metric_measurement_authorized": False,
        "navigation_eligible": False, "actuator_access": False,
        "summary": {
            "cycle_count": len(rows), "sampled_duration_s": (times[-1] - times[0]).total_seconds(),
            "median_intercycle_s": median(intervals) if intervals else None,
            "max_intercycle_s": max(intervals) if intervals else None,
            "rgb_person_candidate_cycles": target_count,
            "coherent_depth_cycles": len(coherent),
            "coherent_fraction_of_rgb_candidate_cycles": len(coherent) / target_count if target_count else None,
            "depth_status_counts": dict(Counter(statuses)), "depth_status_transitions": dict(transitions),
            "max_gap_between_coherent_observations_s": max(
                (gap["between_coherent_observations_s"] for gap in gaps), default=None),
            "gaps_between_coherent_observations": gaps,
            "prefix_without_coherent_cycles": coherent[0] if coherent else len(rows),
            "suffix_without_coherent_cycles": len(rows) - 1 - coherent[-1] if coherent else len(rows),
            "rgb_center_x_span_px": max(centers) - min(centers) if centers else None,
            "capture_pair_skew_ms_median": median(skews),
        },
        "cycles": rows,
        "limitations": [
            "Host timestamps and capture skew do not prove exposure synchronization or fresh-camera latency.",
            "Intercycle gaps are unobserved; the depth state is known only at sampled cycles.",
            "Adjacent RGB boxes and center motion do not prove persistent person identity.",
            "Candidate optical Z is not robot-front clearance or independent distance truth.",
            "The diagnostic range does not expand the activated Measurement range or authorize motion.",
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence-root", required=True, type=Path)
    parser.add_argument("--replay-report", required=True, type=Path)
    parser.add_argument("--operator-scenario", required=True)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output already exists")
    report = analyze(args.evidence_root, args.replay_report, args.operator_scenario)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print("SUMMARY", json.dumps(report["summary"], allow_nan=False))
    print("Saved:", args.output)


if __name__ == "__main__":
    main()
