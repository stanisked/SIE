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
    def distance_stats(key: str) -> dict:
        values = [r[key] for r in cycles if type(r.get(key)) in (int, float)
                  and math.isfinite(r[key])]
        return ({"count": len(values), "median_mm": round(1000 * statistics.median(values), 2),
                 "max_mm": round(1000 * max(values), 2)} if values
                else {"count": 0})
    def torso_distance_stats() -> dict:
        values = [r["distance_m"] for r in cycles
                  if r.get("adapter_reason") == "CURRENT_STABLE_TORSO_CORE_PREVIEW"
                  and type(r.get("distance_m")) in (int, float)
                  and math.isfinite(r["distance_m"])]
        return ({"count": len(values), "min_mm": round(1000 * min(values), 2),
                 "median_mm": round(1000 * statistics.median(values), 2),
                 "max_mm": round(1000 * max(values), 2)} if values
                else {"count": 0})
    return {
        "scope": "MOTOR_OFF_DIAGNOSTIC; no Pi or physical validation inferred from this summary",
        "camera_cycles": len(cycles),
        "no_new_camera_ticks": len(rows) - len(cycles),
        "decisions": dict(Counter(r.get("decision") for r in cycles)),
        "adapter_reasons": dict(Counter(r.get("adapter_reason") for r in cycles)),
        "audit_statuses": dict(Counter(r.get("audit_status") for r in cycles)),
        "mask_statuses": dict(Counter(r.get("mask_status") for r in cycles)),
        "shape_aware_quality_statuses": dict(Counter(
            r.get("shape_aware_quality_status") for r in cycles)),
        "exact_local_surface_cycles": sum(
            r.get("local_surface_binding") == "EXACT_SOURCE_FRAME_AND_MASK"
            and r.get("local_surface_status") == "OBSERVED_NOT_ADMITTED"
            for r in cycles),
        "mad_refusals_with_exact_local_surface": sum(
            r.get("mask_status") == "DEPTH_MAD_EXCEEDED"
            and r.get("local_surface_binding") == "EXACT_SOURCE_FRAME_AND_MASK"
            and r.get("local_surface_status") == "OBSERVED_NOT_ADMITTED"
            for r in cycles),
        "local_surface_residual_mad": distance_stats("local_surface_residual_mad_m"),
        "local_surface_residual_p90": distance_stats("local_surface_residual_p90_m"),
        "local_candidate_front_statuses": dict(Counter(
            r.get("local_candidate_front_status") for r in cycles)),
        "mad_refusals_with_local_candidate_front": sum(
            r.get("mask_status") == "DEPTH_MAD_EXCEEDED"
            and r.get("local_candidate_front_status") == "LOCAL_FITTED_CANDIDATE_FRONT_DIAGNOSTIC_ONLY"
            for r in cycles),
        "stable_front_statuses": dict(Counter(
            r.get("stable_front_status") for r in cycles)),
        "mad_refusals_with_stable_front": sum(
            r.get("mask_status") == "DEPTH_MAD_EXCEEDED"
            and r.get("stable_front_status") == "STABLE_CANDIDATE_FRONT_DIAGNOSTIC_ONLY"
            for r in cycles),
        "local_preview_eligible_cycles": sum(
            r.get("adapter_reason") in ("CURRENT_STABLE_LOCAL_PREVIEW",
                                         "CURRENT_STABLE_TORSO_CORE_PREVIEW")
            and r.get("decision") == "FOLLOW_ELIGIBLE" for r in cycles),
        "torso_core_preview_eligible_cycles": sum(
            r.get("adapter_reason") == "CURRENT_STABLE_TORSO_CORE_PREVIEW"
            and r.get("decision") == "FOLLOW_ELIGIBLE" for r in cycles),
        "stable_front_points": {
            "count": len(stable_points), "min": min(stable_points),
            "median": statistics.median(stable_points)
        } if (stable_points := [r["stable_front_point_count"] for r in cycles
                              if type(r.get("stable_front_point_count")) is int])
          else {"count": 0},
        "local_candidate_front_points": {
            "count": len(points), "min": min(points), "median": statistics.median(points)
        } if (points := [r["local_candidate_front_point_count"] for r in cycles
                       if type(r.get("local_candidate_front_point_count")) is int])
          else {"count": 0},
        "torso_core_front_statuses": dict(Counter(
            r.get("torso_core_front_status") for r in cycles)),
        "torso_core_front_points": {
            "count": len(points), "min": min(points), "median": statistics.median(points)
        } if (points := [r["torso_core_front_point_count"] for r in cycles
                       if type(r.get("torso_core_front_point_count")) is int])
          else {"count": 0},
        "torso_core_front_known_median": distance_stats("torso_core_front_known_median_m"),
        "torso_core_selected_distance": torso_distance_stats(),
        "v3_mask_matches": dict(Counter(r.get("v3_mask_match") for r in cycles)),
        "v3_detection_counts": dict(Counter(r.get("v3_detection_count") for r in cycles)),
        "v3_raw_detection_counts": dict(Counter(r.get("v3_raw_detection_count")
                                                 if r.get("v3_raw_detection_count") is not None
                                                 else (len(r["v3_detections"])
                                                       if isinstance(r.get("v3_detections"), list)
                                                       else None)
                                                 for r in cycles)),
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
