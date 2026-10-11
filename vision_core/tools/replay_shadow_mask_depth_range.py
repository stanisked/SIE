#!/usr/bin/env python3
"""Replay saved AR instance masks and stereo disparity at an explicit diagnostic range.

This reads only frozen evidence. It neither changes the activated Measurement
range nor publishes a Measurement or motor command. The range is a hypothesis
for analysis, not an approved operating envelope.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import statistics

import cv2
import numpy as np

from analyze_ar0234_stereo_component_ambiguity import load_calibrations, sha256
from vision_core.person_localization.instance_mask_stereo_fusion import masked_depth_hypothesis


def replay(path: Path, calibration: dict, minimum: float, maximum: float) -> dict:
    audit = json.loads(path.read_text(encoding="utf-8"))
    row = {"cycle_id": audit["cycle_id"], "source_status": audit["status"],
           "source_reason": audit["reason"]}
    if audit["calibration"] != calibration["sha"]:
        raise ValueError(f"calibration SHA mismatch: {path}")
    geometry = audit.get("stereo_geometry")
    if not isinstance(geometry, dict):
        row["mask_depth"] = {"status": "NO_STEREO_GEOMETRY"}
        return row
    cycle_dir = path.parent
    mask_path = cycle_dir / f'{audit["cycle_id"]}.png'
    metadata_path = cycle_dir / f'{audit["cycle_id"]}.json'
    disparity_path = cycle_dir / "disparity_lr_consistent_px.npy"
    if not all(p.is_file() for p in (mask_path, metadata_path, disparity_path)):
        row["mask_depth"] = {"status": "EVIDENCE_MISSING"}
        return row
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    worker = audit.get("shadow_instance_mask") or {}
    if (worker.get("status") != "MASK_SAVED_DIAGNOSTIC_ONLY"
            or metadata.get("cycle_id") != audit["cycle_id"]
            or metadata.get("reference_frame") != "ar0234_image_frame"
            or metadata.get("source_ar0234_sha256") != sha256(cycle_dir / "ar0234_source.jpg")
            or metadata.get("mask_sha256") != sha256(mask_path)
            or metadata.get("model_sha256") != worker.get("worker_model_sha256")):
        raise ValueError(f"mask provenance mismatch: {path}")
    disparity = np.load(disparity_path, allow_pickle=False)
    width, height = calibration["size"]
    if disparity.shape != (height, width):
        raise ValueError(f"stereo disparity size mismatch: {path}")
    mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
    if mask is None or mask.shape != (1200, 1920) or not np.isin(mask, (0, 255)).all():
        raise ValueError(f"invalid binary mask: {mask_path}")
    stride = int(geometry.get("sample_stride_px", 4))
    if stride < 1:
        raise ValueError(f"invalid stereo grid stride: {path}")
    # Sample the complete rectified frame: the saved live ROI was built for
    # the activated 0.5–2.0 m interval and may clip the 2.3 m hypothesis.
    v, u = np.mgrid[0:height:stride, 0:width:stride]
    d = disparity[::stride, ::stride]
    good = np.isfinite(d) & (d > 0.5)
    u, v, d = u[good].astype(np.float64), v[good].astype(np.float64), d[good]
    z = abs(float(calibration["P2"][0, 3])) / d / 1000.0
    inside_range = (z >= minimum) & (z <= maximum)
    row["lr_consistent_sample_count"] = int(z.size)
    row["diagnostic_in_range_sample_count"] = int(inside_range.sum())
    u, v, z = u[inside_range], v[inside_range], z[inside_range]
    p1 = calibration["P1"]
    rect = np.column_stack(((u - p1[0, 2]) * z / p1[0, 0],
                            (v - p1[1, 2]) * z / p1[1, 1], z))
    left = (calibration["R1"].T @ rect.T).T
    ar = (calibration["R"].T @ (left - calibration["T"]).T).T
    front = ar[:, 2] > 0
    z, ar = z[front], ar[front]
    if ar.size:
        projected, _ = cv2.projectPoints(ar.reshape(-1, 1, 3), np.zeros(3),
                                         np.zeros(3), calibration["K"], calibration["D"])
        projected = projected.reshape(-1, 2)
    else:
        projected = np.empty((0, 2), dtype=np.float64)
    # Keep the same torso-oriented AR inner ROI as the earlier 21-cycle replay.
    roi = geometry.get("ar_inner_roi_xyxy_px")
    if not isinstance(roi, list) or len(roi) != 4:
        raise ValueError(f"missing AR inner ROI: {path}")
    x1, y1, x2, y2 = (float(x) for x in roi)
    within_roi = ((projected[:, 0] >= x1) & (projected[:, 0] <= x2)
                  & (projected[:, 1] >= y1) & (projected[:, 1] <= y2))
    row["ar_inner_roi_sample_count"] = int(within_roi.sum())
    depth = masked_depth_hypothesis(projected[within_roi], z[within_roi], mask == 255)
    row["mask_depth"] = depth
    row["mask_sha256"] = sha256(mask_path)
    return row


def summary(rows: list[dict]) -> dict:
    depths = [r["mask_depth"]["candidate_depth_m"] for r in rows
              if r["mask_depth"].get("status") == "COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY"]
    return {"cycle_count": len(rows),
            "statuses": dict(Counter(r["mask_depth"]["status"] for r in rows)),
            "coherent_depth_median_m": statistics.median(depths) if depths else None,
            "coherent_depth_span_m": max(depths) - min(depths) if depths else None}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence-root", type=Path, required=True)
    parser.add_argument("--ar-intrinsic", type=Path, required=True)
    parser.add_argument("--stereo-calibration", type=Path, required=True)
    parser.add_argument("--activation", type=Path, required=True)
    parser.add_argument("--diagnostic-range-m", nargs=2, type=float, required=True,
                        metavar=("MIN", "MAX"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    lower, upper = args.diagnostic_range_m
    if lower <= 0 or upper <= lower or upper > 4.5:
        parser.error("diagnostic range must be positive, ordered, and at most 4.5 m")
    if args.output.exists():
        parser.error(f"output already exists: {args.output}")
    activation = json.loads(args.activation.read_text(encoding="utf-8"))
    source = activation["calibration_profile"]["source_sha256"]
    extrinsic = activation["calibration_profile"]["ar0234_to_ov9281_physical_left_extrinsic"]
    extrinsic_path = Path(extrinsic["path"]).expanduser()
    if (sha256(args.ar_intrinsic) != source["ar0234_intrinsic"]
            or sha256(args.stereo_calibration) != source["ov9281_stereo_calibration"]
            or sha256(extrinsic_path) != extrinsic["sha256"]):
        raise ValueError("activation calibration SHA mismatch")
    calibration = load_calibrations(args.ar_intrinsic, args.stereo_calibration, extrinsic_path)
    files = sorted(args.evidence_root.rglob("audit.json"))
    if not files:
        parser.error("no saved audit.json files")
    rows = [replay(path, calibration, lower, upper) for path in files]
    report = {"status": "OFFLINE_DIAGNOSTIC_ONLY", "diagnostic_depth_range_m": [lower, upper],
              "activated_depth_range_m": activation["authorization"]["metric_measurement_range_m"],
              "source_root": str(args.evidence_root), "calibration_sha256": calibration["sha"],
              "summary": summary(rows), "cycles": rows,
              "limitations": ["No exposure synchronization or ground-truth human distance.",
                              "This replay does not authorize Measurement or motion.",
                              "Whole-frame stereo sampling differs from the earlier ROI-limited replay."]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    for row in rows:
        d = row["mask_depth"]
        print(row["cycle_id"][-8:], row["source_status"], row["source_reason"],
              d["status"], "n=", d.get("candidate_sample_count"),
              "depth_m=", round(d["candidate_depth_m"], 4) if "candidate_depth_m" in d else None,
              "MAD_mm=", round(d["candidate_mad_m"] * 1000, 1) if "candidate_mad_m" in d else None)
    print("Summary:", report["summary"])
    print("Saved:", args.output)


if __name__ == "__main__":
    main()
