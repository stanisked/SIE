#!/usr/bin/env python3
"""Offline multidimensional quality profiles; no new acceptance threshold.

Preserve existing decisions. Compare only frames in the same saved run using
Pareto ordering, without a weighted score, threshold fitting or motion output.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import statistics


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def profile(path):
    audit = json.loads(path.read_text())
    shadow = audit.get("shadow_local_surfaces") or {}
    row = {"cycle_id": audit["cycle_id"], "run_directory": str(path.parent.parent),
           "source_audit": str(path), "source_audit_sha256": digest(path),
           "original_status": audit.get("status"), "original_reason": audit.get("reason"),
           "mask_status": (shadow.get("mask_depth") or {}).get("status"),
           "comparison_status": "NOT_COMPARABLE", "reasons": [],
           "metric_measurement_authorized": False, "navigation_eligible": False,
           "actuator_access": False, "confidence": None,
           "reference_frame": "rectified_left_optical_frame", "unit": "m"}
    if shadow.get("status") != "SHAPE_AWARE_SHADOW_DIAGNOSTIC_ONLY":
        row["reasons"] = ["NO_CURRENT_SURFACE_DIAGNOSTIC"]
        return row
    try:
        report_path = path.parent / "shadow_local_surfaces/report.json"
        if (shadow.get("report_file") != "shadow_local_surfaces/report.json"
                or digest(report_path) != shadow.get("report_sha256")):
            raise ValueError("surface report binding mismatch")
        report = json.loads(report_path.read_text())
        full = report["result"]
        if (report["cycle_id"] != audit["cycle_id"]
                or report["calibration_sha256"] != audit["calibration"]
                or full["cycle_id"] != audit["cycle_id"]
                or full["mask_depth"] != shadow["mask_depth"]
                or full["source_status"] != audit["status"]
                or full["source_reason"] != audit["reason"]
                or report["reference_frame"] != "rectified_left_optical_frame"):
            raise ValueError("same-cycle/calibration/decision mismatch")
        sources = {"source_audit_sha256": "shadow_surface_input.json",
                   "source_mask_sha256": f"{audit['cycle_id']}.png",
                   "source_disparity_sha256": "disparity_lr_consistent_px.npy",
                   "source_rgb_sha256": "ar0234_source.jpg"}
        for key, filename in sources.items():
            if digest(path.parent / filename) != full[key]:
                raise ValueError(f"frozen source changed: {filename}")
        local = full["local_surface"]
        if any(local.get(k) != v for k, v in shadow["local_surface"].items()):
            raise ValueError("compact/full local metrics mismatch")
        depth = full["mask_depth"]
        residual = local.get("pooled_held_out_axial_residual") or {}
        coverage = local.get("rgb_coverage") or {}
        row.update({"diagnostic_depth_range_m": report["diagnostic_depth_range_m"],
                    "activated_depth_range_m": report["activated_depth_range_m"],
                    "selected_sample_count": depth.get("selected_sample_count"),
                    "candidate_sample_count": depth.get("candidate_sample_count"),
                    "plausible_layer_count": depth.get("plausible_layer_count"),
                    "depth_m": depth.get("candidate_depth_m"),
                    "global_mad_m": depth.get("candidate_mad_m"),
                    "local_mad_m": residual.get("mad_m"),
                    "local_p90_absolute_m": residual.get("absolute_p90_m"),
                    "local_max_absolute_m": residual.get("absolute_max_m"),
                    "local_signed_median_m": residual.get("median_m"),
                    "held_out_sample_count": residual.get("sample_count"),
                    "coarse_rgb_coverage_fraction": coverage.get("mask_area_in_fitted_bins_fraction"),
                    "fitted_sample_fraction": local.get("fitted_sample_fraction"),
                    "unfitted_patch_fraction": local.get("unfitted_patch_fraction"),
                    "patch_status_counts": local.get("patch_status_counts"),
                    "host_latency_s": (audit.get("processing_timing") or {}).get("host_capture_start_to_result_ready_s"),
                    "calibration_sha256": audit["calibration"],
                    "method": report["method"], "code_sha256": report["code_sha256"],
                    "exposure_synchronization_proven": (audit.get("pairing") or {}).get("exposure_synchronization_proven")})
        values = [row.get(k) for k in AXES]
        if (any(type(v) not in (int, float) or not math.isfinite(v) or v < 0 for v in values)
                or any(row[k] > 1 for k in FRACTIONS)
                or depth.get("plausible_layer_count") != 1
                or not residual.get("sample_count", 0)
                or not depth.get("candidate_sample_count", 0)):
            row["reasons"].append("MISSING_INVALID_OR_AMBIGUOUS_SURFACE_SUPPORT")
        else:
            row["comparison_status"] = "PROFILE_COMPLETE_DIAGNOSTIC_ONLY"
    except (OSError, ValueError, KeyError, TypeError) as error:
        row["comparison_status"] = "EVIDENCE_INTEGRITY_ERROR"
        row["reasons"].append(str(error))
    return row


FRACTIONS = ("coarse_rgb_coverage_fraction", "fitted_sample_fraction", "unfitted_patch_fraction")
AXES = ("local_mad_m", "local_p90_absolute_m", "local_max_absolute_m", *FRACTIONS)


def dominates(a, b):
    """No worse on EVERY declared axis, strictly better on at least one.

    This is only relative to a fitted model. It never establishes accuracy,
    semantic ownership, freshness, synchronisation or approved range.
    """
    if any(r.get("comparison_status") != "PROFILE_COMPLETE_DIAGNOSTIC_ONLY" for r in (a, b)):
        return False
    # Do not compare different runs, calibrations, methods or analysis versions.
    if any(a.get(k) != b.get(k) for k in ("run_directory", "calibration_sha256", "method", "code_sha256", "diagnostic_depth_range_m")):
        return False
    def vector(r):
        return [(-r[k] if k in ("coarse_rgb_coverage_fraction", "fitted_sample_fraction") else r[k]) for k in AXES]
    av, bv = vector(a), vector(b)
    return all(x <= y for x, y in zip(av, bv)) and any(x < y for x, y in zip(av, bv))


def summarize(rows):
    contradictions = [{"refused_cycle": a["cycle_id"], "accepted_cycle": b["cycle_id"]}
        for a in rows if a.get("mask_status") == "DEPTH_MAD_EXCEEDED"
        for b in rows if b.get("mask_status") == "COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY"
        if dominates(a, b)]
    groups = {}
    for status in sorted({r.get("mask_status") or "ABSENT" for r in rows}):
        selected = [r for r in rows if (r.get("mask_status") or "ABSENT") == status]
        metrics = {}
        for key in (*AXES, "global_mad_m", "host_latency_s"):
            values = [r[key] for r in selected if r.get("comparison_status") == "PROFILE_COMPLETE_DIAGNOSTIC_ONLY" and r.get(key) is not None]
            if values:
                metrics[key] = {"count": len(values), "minimum": min(values),
                                "median": statistics.median(values), "maximum": max(values)}
        groups[status] = {"cycle_count": len(selected), "metrics": metrics}
    return {"cycles": len(rows), "profile_statuses": dict(Counter(r["comparison_status"] for r in rows)),
            "mask_status_groups": groups, "refused_dominates_accepted": contradictions,
            "ordering_note": "No contradiction does not validate global MAD; the partial order can be inconclusive."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence-root", type=Path, action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output exists; use a new report filename")
    files = sorted({p.resolve() for root in args.evidence_root for p in root.rglob("audit.json")})
    if not files:
        parser.error("no saved audit.json files")
    rows = [profile(p) for p in files]
    runs = {}
    for row in rows:
        runs.setdefault(row["run_directory"], []).append(row)
    summaries = {run: summarize(values) for run, values in runs.items()}
    for run, summary in summaries.items():
        print("RUN:", run)
        print(json.dumps(summary, ensure_ascii=False, allow_nan=False), flush=True)
    report = {"status": "QUALITY_CRITERION_RESEARCH_ONLY", "profiles": rows, "runs": summaries,
              "axes": AXES, "acceptance_policy": "NOT_DEFINED_OR_VALIDATED",
              "metric_measurement_authorized": False, "navigation_eligible": False, "actuator_access": False,
              "tool_sha256": digest(Path(__file__)),
              "limitations": ["Residuals are relative to a model, not physical range errors.",
                              "Coverage uses coarse bins and does not verify semantic ownership.",
                              "No new numeric thresholds, weighted score or live acceptance rule.",
                              "Temporal freshness, independent accuracy and motion applicability remain unvalidated."]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print("Saved:", args.output)
    if any(r["comparison_status"] == "EVIDENCE_INTEGRITY_ERROR" for r in rows):
        raise SystemExit(2)


if __name__ == "__main__":
    main()
