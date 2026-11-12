#!/usr/bin/env python3
"""Summarize the motor-off Shadow fast-preview journal without granting motion."""
from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import statistics


def summarize(rows: list[dict]) -> dict:
    cycles = [r for r in rows if r.get("cycle_id") is not None]
    follows = [r for r in cycles if r.get("decision") == "FOLLOW_ELIGIBLE"]
    ages = {sensor: [r["driver_to_decision_s"][sensor] for r in cycles
                     if isinstance(r.get("driver_to_decision_s"), dict)
                     and type(r["driver_to_decision_s"].get(sensor)) in (int, float)
                     and math.isfinite(r["driver_to_decision_s"][sensor])]
            for sensor in ("ar0234", "ov9281")}
    valid_age_cycles = [r for r in cycles if all(
        type((r.get("driver_to_decision_s") or {}).get(sensor)) in (int, float)
        and math.isfinite(r["driver_to_decision_s"][sensor])
        for sensor in ("ar0234", "ov9281"))]
    age_over_budget = [r["cycle_id"] for r in valid_age_cycles if any(
        not 0 <= r["driver_to_decision_s"][sensor] <= .5
        for sensor in ("ar0234", "ov9281"))]
    tracks = sorted({str(r["track_id"]) for r in follows if r.get("track_id")})
    def time_stats(key: str) -> dict:
        values = [r[key] for r in cycles if type(r.get(key)) in (int, float)
                  and math.isfinite(r[key])]
        return ({"count": len(values), "median_ms": round(1000 * statistics.median(values), 2),
                 "max_ms": round(1000 * max(values), 2)} if values
                else {"count": 0})
    return {
        "scope": "MOTOR_OFF_DIAGNOSTIC; no Pi or physical validation inferred from this summary",
        "camera_cycles": len(cycles),
        "no_new_camera_ticks": len(rows) - len(cycles),
        "decisions": dict(Counter(r.get("decision") for r in cycles)),
        "adapter_reasons": dict(Counter(r.get("adapter_reason") for r in cycles)),
        "audit_statuses": dict(Counter(r.get("audit_status") for r in cycles)),
        "mask_statuses": dict(Counter(r.get("mask_status") for r in cycles)),
        "v3_mask_matches": dict(Counter(r.get("v3_mask_match") for r in cycles)),
        "v3_detection_counts": dict(Counter(r.get("v3_detection_count") for r in cycles)),
        "v3_inference": time_stats("v3_inference_s"),
        "host_processing": time_stats("host_processing_s"),
        "cycles_with_two_eligible_people": sum((r.get("current_eligible_count") or 0) >= 2
                                              for r in cycles),
        "cycles_with_two_segmentation_instances": sum((r.get("segmentation_instance_count") or 0) >= 2
                                                      for r in cycles),
        "cycles_with_missing_driver_age": len(cycles) - len(valid_age_cycles),
        "cycles_exceeding_500ms_to_decision": len(age_over_budget),
        "driver_age_to_decision_ms": {
            k: {"count": len(v), "median": round(1000 * statistics.median(v), 2),
                "max": round(1000 * max(v), 2)} if v else {"count": 0}
            for k, v in ages.items()},
        "follow_eligible_cycles": len(follows),
        "follow_track_ids": tracks,
        "note": "Check the chronological JSONL for actual disappearance/reappearance and two-person scenes. Driver SOE is declared, not independently verified physical exposure. FOLLOW_ELIGIBLE is not motion authorization.",
    }


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("journal", type=Path)
    args = p.parse_args()
    rows = []
    with args.journal.open() as stream:
        for number, line in enumerate(stream, 1):
            row = json.loads(line)
            if not isinstance(row, dict) or row.get("source") != "fast_mask":
                p.error(f"unexpected source or record at line {number}")
            rows.append(row)
    print(json.dumps(summarize(rows), indent=2, ensure_ascii=False, allow_nan=False))


if __name__ == "__main__":
    main()
