#!/usr/bin/env python3
"""Compare frozen shadow depth with an owned, valid-only spatial median 3x3.

Offline diagnostic only. A previously saved baseline must be reproduced first.
No cameras, model inference, ROS publication, activation edits or motion.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import time

import cv2
import numpy as np

from replay_shadow_mask_depth_range import replay, summary
from analyze_ar0234_stereo_component_ambiguity import load_calibrations, sha256
from vision_core.person_localization.diagnostic_depth_median import masked_median3x3
from vision_core.person_localization.instance_mask_stereo_fusion import masked_depth_hypothesis


def project(u: np.ndarray, v: np.ndarray, z: np.ndarray, cal: dict) -> np.ndarray:
    """Return original-order RGB pixels; behind-camera points remain NaN."""
    result = np.full((len(z), 2), np.nan, dtype=np.float64)
    if not len(z):
        return result
    p1 = cal["P1"]
    rect = np.column_stack(((u - p1[0, 2]) * z / p1[0, 0],
                            (v - p1[1, 2]) * z / p1[1, 1], z))
    left = (cal["R1"].T @ rect.T).T
    ar = (cal["R"].T @ (left - cal["T"]).T).T
    front = np.isfinite(ar).all(axis=1) & (ar[:, 2] > 0)
    if front.any():
        xy, _ = cv2.projectPoints(ar[front].reshape(-1, 1, 3), np.zeros(3),
                                   np.zeros(3), cal["K"], cal["D"])
        result[front] = xy.reshape(-1, 2)
    return result


def in_roi(xy: np.ndarray, roi: list) -> np.ndarray:
    x1, y1, x2, y2 = roi
    return (np.isfinite(xy).all(axis=1) & (xy[:, 0] >= x1) & (xy[:, 0] <= x2)
            & (xy[:, 1] >= y1) & (xy[:, 1] <= y2))


def owned(xy: np.ndarray, interior: np.ndarray, roi: list) -> np.ndarray:
    good = in_roi(xy, roi)
    rounded = np.rint(np.where(np.isfinite(xy), xy, -1)).astype(np.int64)
    x, y = rounded.T
    h, w = interior.shape
    good &= (x >= 0) & (x < w) & (y >= 0) & (y < h)
    selected = np.zeros(len(xy), dtype=bool)
    selected[good] = interior[y[good], x[good]] != 0
    return selected


def verify_baseline(current: dict, saved: dict) -> None:
    for key in ("cycle_id", "source_status", "source_reason"):
        if current.get(key) != saved.get(key):
            raise ValueError(f"baseline identity mismatch: {current['cycle_id']} {key}")
    if current["mask_depth"] != saved["mask_depth"]:
        raise ValueError(f"baseline numerical replay mismatch: {current['cycle_id']}")


def compare(path: Path, raw: dict, cal: dict, lower: float, upper: float,
            output_dir: Path) -> dict:
    started = time.monotonic()
    if "mask_sha256" not in raw:
        return {**raw, "filtered_mask_depth": dict(raw["mask_depth"]),
                "filter": {"status": "NOT_APPLIED_NO_VERIFIED_DEPTH_EVIDENCE"}}
    audit = json.loads(path.read_text(encoding="utf-8"))
    g = audit["stereo_geometry"]
    stride = int(g["sample_stride_px"])
    cycle_id = raw["cycle_id"]
    disparity_path = path.parent / "disparity_lr_consistent_px.npy"
    disparity = np.load(disparity_path, allow_pickle=False)
    mask = cv2.imread(str(path.parent / f"{cycle_id}.png"), cv2.IMREAD_GRAYSCALE)
    # Same two-pixel interior margin as masked_depth_hypothesis.
    interior = cv2.erode(mask, np.ones((5, 5), dtype=np.uint8),
                         borderType=cv2.BORDER_CONSTANT, borderValue=0)
    valid = np.isfinite(disparity) & (disparity > 0.5)
    v, u = np.nonzero(valid)
    z = abs(float(cal["P2"][0, 3])) / disparity[valid] / 1000.0
    within = (z >= lower) & (z <= upper)
    u, v, z = u[within], v[within], z[within]
    depth = np.full(disparity.shape, np.nan, dtype=np.float64)
    depth[v, u] = z
    xy = project(u.astype(np.float64), v.astype(np.float64), z, cal)
    original_owned = owned(xy, interior, g["ar_inner_roi_xyxy_px"])
    eligible = np.zeros(depth.shape, dtype=bool)
    eligible[v[original_owned], u[original_owned]] = True
    filtered, changed, stats = masked_median3x3(depth, eligible)
    # Preserve exactly the original stereo grid and LR-valid sample indices.
    sample = ((u % stride) == 0) & ((v % stride) == 0)
    su, sv = u[sample], v[sample]
    filtered_z = filtered[sv, su]
    filtered_xy = project(su.astype(np.float64), sv.astype(np.float64), filtered_z, cal)
    filtered_roi = in_roi(filtered_xy, g["ar_inner_roi_xyxy_px"])
    hypothesis = masked_depth_hypothesis(filtered_xy[filtered_roi],
                                         filtered_z[filtered_roi], mask == 255)
    # Also retain a fixed-support comparison to expose any mask-membership
    # change caused by reprojecting a corrected depth through the extrinsic.
    original_xy = xy[sample]
    original_roi = in_roi(original_xy, g["ar_inner_roi_xyxy_px"])
    fixed_support = masked_depth_hypothesis(original_xy[original_roi],
                                            filtered_z[original_roi], mask == 255)
    before_owned = original_owned[sample]
    after_owned = owned(filtered_xy, interior, g["ar_inner_roi_xyxy_px"])
    stats.update({"status": "APPLIED_DIAGNOSTIC_ONLY", "sample_stride_px": stride,
                  "changed_grid_point_count": int(changed[sv, su].sum()),
                  "reprojection_entered_mask_count": int((~before_owned & after_owned).sum()),
                  "reprojection_left_mask_count": int((before_owned & ~after_owned).sum())})
    delta = filtered[changed] - depth[changed]
    cy, cx = np.nonzero(changed)
    evidence_path = output_dir / f"{cycle_id}_changes.npz"
    np.savez_compressed(evidence_path, stereo_v=cy, stereo_u=cx,
                        raw_depth_m=depth[changed], filtered_depth_m=filtered[changed])
    artifacts = {"changed_points": evidence_path.name,
                 "changed_points_sha256": sha256(evidence_path)}
    source_image = cv2.imread(str(path.parent / "ov9281_physical_left_rectified.jpg"))
    if source_image is not None:
        if source_image.shape[:2] != disparity.shape:
            raise ValueError(f"rectified evidence image shape mismatch: {path}")
        overlay = source_image.copy()
        contours, _ = cv2.findContours(eligible.astype(np.uint8), cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(overlay, contours, -1, (0, 255, 0), 1)
        overlay[cy, cx] = (0, 255, 255)
        large = np.abs(delta) > 0.05  # Visualization only; not a filter/acceptance gate.
        overlay[cy[large], cx[large]] = (0, 0, 255)
        cv2.putText(overlay, "green: eligible; yellow: changed; red: change >50mm",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        image_path = output_dir / f"{cycle_id}_changes.jpg"
        if not cv2.imwrite(str(image_path), overlay):
            raise OSError(f"cannot save {image_path}")
        artifacts["overlay"] = image_path.name
    diff = {}
    for name in ("candidate_depth_m", "candidate_mad_m", "selected_sample_count"):
        if name in raw["mask_depth"] and name in hypothesis:
            diff[name] = hypothesis[name] - raw["mask_depth"][name]
    return {**raw, "filtered_mask_depth": hypothesis,
            "fixed_support_filtered_mask_depth": fixed_support,
            "filtered_minus_raw": diff, "filter": stats, "artifacts": artifacts,
            "source_audit_sha256": sha256(path),
            "source_disparity_sha256": sha256(disparity_path),
            "comparison_elapsed_s": time.monotonic() - started}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-report", type=Path, required=True)
    parser.add_argument("--ar-intrinsic", type=Path, required=True)
    parser.add_argument("--stereo-calibration", type=Path, required=True)
    parser.add_argument("--activation", type=Path, required=True)
    parser.add_argument("--diagnostic-range-m", type=float, nargs=2, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    lower, upper = args.diagnostic_range_m
    if not np.isfinite([lower, upper]).all() or not 0 < lower < upper <= 4.5:
        parser.error("explicit diagnostic range must be within 0..4.5 m")
    evidence_dir = args.output.parent / f"{args.output.stem}_evidence"
    if args.output.exists() or evidence_dir.exists():
        parser.error("output or comparison evidence directory already exists; choose a new output")
    baseline = json.loads(args.baseline_report.read_text(encoding="utf-8"))
    if baseline["diagnostic_depth_range_m"] != [lower, upper]:
        raise ValueError("diagnostic range differs from baseline")
    activation = json.loads(args.activation.read_text(encoding="utf-8"))
    source = activation["calibration_profile"]["source_sha256"]
    extrinsic = activation["calibration_profile"]["ar0234_to_ov9281_physical_left_extrinsic"]
    extrinsic_path = Path(extrinsic["path"]).expanduser()
    if (sha256(args.ar_intrinsic) != source["ar0234_intrinsic"]
            or sha256(args.stereo_calibration) != source["ov9281_stereo_calibration"]
            or sha256(extrinsic_path) != extrinsic["sha256"]):
        raise ValueError("activation/calibration SHA mismatch")
    cal = load_calibrations(args.ar_intrinsic, args.stereo_calibration, extrinsic_path)
    if baseline["calibration_sha256"] != cal["sha"]:
        raise ValueError("baseline/calibration SHA mismatch")
    root = Path(baseline["source_root"])
    files = sorted(root.rglob("audit.json"))
    saved_rows = {row["cycle_id"]: row for row in baseline["cycles"]}
    if not files or len(files) != len(saved_rows) or len(saved_rows) != len(baseline["cycles"]):
        raise ValueError("baseline/evidence cycle count mismatch")
    raw_rows = [replay(path, cal, lower, upper) for path in files]
    if set(saved_rows) != {r["cycle_id"] for r in raw_rows}:
        raise ValueError("baseline/evidence cycle identities mismatch")
    for row in raw_rows:
        verify_baseline(row, saved_rows[row["cycle_id"]])
    print(f"BASELINE_REPLAY_VERIFIED {len(raw_rows)}/{len(raw_rows)}", flush=True)
    evidence_dir.mkdir(parents=True, exist_ok=False)
    rows = []
    for path, raw in zip(files, raw_rows):
        row = compare(path, raw, cal, lower, upper, evidence_dir)
        rows.append(row)
        a, b = row["mask_depth"], row["filtered_mask_depth"]
        def mm(d: dict, key: str) -> str:
            return f"{d[key] * 1000:.2f}" if key in d else "None"
        print(row["cycle_id"][-8:], a["status"], "->", b["status"],
              "MAD_mm=", mm(a, "candidate_mad_m"), "/", mm(b, "candidate_mad_m"),
              "depth_mm=", mm(a, "candidate_depth_m"), "/", mm(b, "candidate_depth_m"),
              "n=", a.get("selected_sample_count"), "/", b.get("selected_sample_count"),
              "changed_grid=", row["filter"].get("changed_grid_point_count", 0), flush=True)
    filtered_rows = [{"mask_depth": r["filtered_mask_depth"]} for r in rows]
    report = {"status": "OFFLINE_MEDIAN_COMPARISON_DIAGNOSTIC_ONLY",
              "metric_measurement_authorized": False, "navigation_eligible": False,
              "actuator_access": False, "reference_frame": "rectified_left_optical_frame",
              "unit": "m", "confidence": None, "confidence_status": "NOT_VALIDATED",
              "baseline_report": str(args.baseline_report),
              "baseline_report_sha256": sha256(args.baseline_report),
              "baseline_replay_verified": True, "source_root": str(root),
              "diagnostic_depth_range_m": [lower, upper],
              "activated_depth_range_m": activation["authorization"]["metric_measurement_range_m"],
              "calibration_sha256": cal["sha"],
              "maximum_depth_mad_m": 0.05, "minimum_samples": 100,
              "raw_summary": summary(rows), "filtered_summary": summary(filtered_rows),
              "transitions": dict(Counter(f"{r['mask_depth']['status']} -> "
                                           f"{r['filtered_mask_depth']['status']}" for r in rows)),
              "evidence_dir": str(evidence_dir), "cycles": rows,
              "code_sha256": {"comparison": sha256(Path(__file__)),
                              "baseline_replay": sha256(Path(__file__).with_name("replay_shadow_mask_depth_range.py")),
                              "median_filter": sha256(Path(__file__).resolve().parents[1] /
                                                      "person_localization/diagnostic_depth_median.py")},
              "limitations": ["No physical ground-truth depth or exposure synchronization.",
                              "Median filtering may erase thin surfaces or internal depth edges.",
                              "Eligibility uses original geometry; filtered depths are reprojected for final selection.",
                              "Filtered values are derived estimates, not new LR-validated correspondences.",
                              "Lower MAD or a changed diagnostic status does not establish accuracy or motion eligibility."]}
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print("RAW:", report["raw_summary"])
    print("FILTERED:", report["filtered_summary"])
    print("TRANSITIONS:", report["transitions"])
    print("Saved:", args.output)


if __name__ == "__main__":
    main()
