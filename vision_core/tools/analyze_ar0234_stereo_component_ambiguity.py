#!/usr/bin/env python3
"""Replay saved stereo components into AR0234 images without choosing a target.

Diagnostic only. Uses saved LR-consistent disparities and SHA-checked calibration;
does not open a camera, run a detector, alter a gate, or publish a Measurement.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from vision_core.person_localization.instance_mask_stereo_fusion import (  # noqa: E402
    masked_depth_hypothesis,
)


def bbox_gap(a: list[int], b: list[int]) -> float:
    """Distance between inclusive image rectangles; overlap has zero gap."""
    dx = max(a[0] - b[2], b[0] - a[2], 0)
    dy = max(a[1] - b[3], b[1] - a[3], 0)
    return math.hypot(dx, dy)


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def load_calibrations(ar: Path, stereo: Path, extrinsic: Path) -> dict:
    intrinsic = json.loads(ar.read_text(encoding="utf-8"))
    with np.load(stereo, allow_pickle=False) as s, np.load(extrinsic, allow_pickle=False) as e:
        if str(s["camera_1_semantics"].item()) != "physical_left":
            raise ValueError("stereo camera 1 is not physical_left")
        if str(s["camera_2_semantics"].item()) != "physical_right":
            raise ValueError("stereo camera 2 is not physical_right")
        if str(e["source_stereo_calibration_sha256"].item()) != sha256(stereo):
            raise ValueError("extrinsic/stereo SHA mismatch")
        if str(e["source_ar_intrinsic_sha256"].item()) != sha256(ar):
            raise ValueError("extrinsic/AR SHA mismatch")
        return {
            "K": np.asarray(intrinsic["camera_matrix"], dtype=np.float64),
            "D": np.asarray(intrinsic["distortion_coefficients"], dtype=np.float64),
            "R1": s["R1"].copy(), "P1": s["P1"].copy(), "P2": s["P2"].copy(),
            "size": tuple(int(v) for v in s["size"]),
            "R": e["R_ar0234_to_physical_left"].copy(),
            "T": e["T_ar0234_to_physical_left_mm"].reshape(3) / 1000.0,
            "sha": {"ar0234_intrinsic_sha256": sha256(ar),
                    "ov9281_stereo_sha256": sha256(stereo),
                    "ar0234_to_physical_left_extrinsic_sha256": sha256(extrinsic)},
        }


def replay(path: Path, calibration: dict, output_dir: Path,
           instance_mask_dir: Path | None = None) -> dict:
    audit = json.loads(path.read_text(encoding="utf-8"))
    result = {"cycle_id": audit["cycle_id"], "source_status": audit["status"],
              "source_reason": audit["reason"], "components": []}
    if isinstance(audit.get("association"), dict):
        result["source_association"] = {
            key: audit["association"].get(key)
            for key in ("range_m", "depth_mad_m", "depth_sample_count")
        }
    g = audit.get("stereo_geometry")
    if not isinstance(g, dict) or "spatial_components" not in g:
        result["status"] = "NO_SAVED_COMPONENTS"
        return result
    if audit["calibration"] != calibration["sha"]:
        raise ValueError(f"calibration SHA mismatch in {path}")
    disparity = np.load(path.parent / "disparity_lr_consistent_px.npy", allow_pickle=False)
    width, height = calibration["size"]
    if disparity.shape != (height, width):
        raise ValueError(f"disparity size mismatch in {path}")
    x1, y1, x2, y2 = g["physical_left_frustum_roi_xyxy_px"]
    ax1, ay1, ax2, ay2 = g["ar_inner_roi_xyxy_px"]
    stride = int(g["sample_stride_px"])
    yy, xx = np.mgrid[y1:y2:stride, x1:x2:stride]
    values = disparity[yy, xx]
    diag = g["pre_range_depth_diagnostic"]
    zmin, zmax = map(float, diag["configured_depth_range_m"])
    valid = np.isfinite(values) & (values > 0.5)
    u, v, dv = xx[valid].astype(np.float64), yy[valid].astype(np.float64), values[valid]
    z = abs(float(calibration["P2"][0, 3])) / dv / 1000.0
    valid_z = (z >= zmin) & (z <= zmax)
    u, v, z = u[valid_z], v[valid_z], z[valid_z]
    p1 = calibration["P1"]
    rect = np.column_stack(((u - p1[0, 2]) * z / p1[0, 0],
                            (v - p1[1, 2]) * z / p1[1, 1], z))
    left = (calibration["R1"].T @ rect.T).T
    ar = (calibration["R"].T @ (left - calibration["T"]).T).T
    front = ar[:, 2] > 0
    u, v, z, ar = u[front], v[front], z[front], ar[front]
    projected, _ = cv2.projectPoints(ar.reshape(-1, 1, 3), np.zeros(3),
                                      np.zeros(3), calibration["K"], calibration["D"])
    projected = projected.reshape(-1, 2)
    inside = ((projected[:, 0] >= ax1) & (projected[:, 0] <= ax2)
              & (projected[:, 1] >= ay1) & (projected[:, 1] <= ay2))
    u, v, z, projected = u[inside], v[inside], z[inside], projected[inside]
    if instance_mask_dir is not None:
        mask_file = instance_mask_dir / f'{audit["cycle_id"]}.png'
        metadata_file = instance_mask_dir / f'{audit["cycle_id"]}.json'
        if not mask_file.is_file() or not metadata_file.is_file():
            result["instance_mask_depth"] = {"status": "MASK_MISSING"}
        else:
            metadata = json.loads(metadata_file.read_text(encoding="utf-8"))
            if (metadata.get("cycle_id") != audit["cycle_id"]
                    or metadata.get("reference_frame") != "ar0234_image_frame"
                    or metadata.get("source_ar0234_sha256") != sha256(
                        path.parent / "ar0234_source.jpg")
                    or metadata.get("mask_sha256") != sha256(mask_file)):
                raise ValueError(f"instance mask is not bound to source cycle {path}")
            mask = cv2.imread(str(mask_file), cv2.IMREAD_GRAYSCALE)
            if mask is None or mask.shape != (1200, 1920) or not np.isin(
                mask, (0, 255)).all():
                raise ValueError(f"invalid binary AR instance mask {mask_file}")
            result["instance_mask_depth"] = {
                **masked_depth_hypothesis(projected, z, mask == 255),
                "mask_file": str(mask_file), "mask_sha256": sha256(mask_file),
                "mask_source": metadata.get("mask_source", "UNDECLARED"),
            }
    gw = (x2 - x1 + stride - 1) // stride
    gh = (y2 - y1 + stride - 1) // stride
    gx = ((u.astype(np.int32) - x1) // stride).clip(0, gw - 1)
    gy = ((v.astype(np.int32) - y1) // stride).clip(0, gh - 1)
    occupied = np.zeros((gh, gw), dtype=np.uint8)
    occupied[gy, gx] = 1
    _, labels, stats, _ = cv2.connectedComponentsWithStats(occupied, connectivity=8)
    sample_labels = labels[gy, gx]
    source = g["spatial_components"]
    frame = cv2.imread(str(path.parent / "ar0234_source.jpg"))
    if frame is None:
        raise ValueError(f"AR source image missing in {path.parent}")
    image = frame.copy()
    cv2.rectangle(image, (round(ax1), round(ay1)), (round(ax2), round(ay2)),
                  (255, 255, 255), 2)
    colors = [(0, 255, 255), (255, 80, 80), (80, 255, 80), (255, 80, 255),
              (80, 160, 255), (255, 255, 80)]
    reconstructed = []
    for label in range(1, int(stats.shape[0])):
        where = sample_labels == label
        if not np.any(where):
            continue
        left_cell, top_cell, w, h, area = map(int, stats[label])
        bbox = [x1 + left_cell * stride, y1 + top_cell * stride,
                min(x2, x1 + (left_cell + w) * stride),
                min(y2, y1 + (top_cell + h) * stride)]
        reconstructed.append((where, bbox, area))
    expected = sorted((c["sample_count"], c["bbox_xyxy_px"]) for c in source)
    actual = sorted((int(w.sum()), b) for w, b, _ in reconstructed)
    if actual != expected or len(u) != int(g["reprojected_inside_ar_inner_roi_count"]):
        raise ValueError(f"replayed spatial components do not match audit in {path}")
    if len(reconstructed) != len(source):
        raise ValueError(f"component order mismatch in {path}")
    for index, (where, bbox, area) in enumerate(reconstructed, 1):
        if (int(where.sum()), bbox) != (source[index - 1]["sample_count"],
                                      source[index - 1]["bbox_xyxy_px"]):
            raise ValueError(f"component label order mismatch in {path}")
        color = colors[(index - 1) % len(colors)]
        points = np.rint(projected[where]).astype(np.int32)
        for px, py in points:
            if 0 <= px < frame.shape[1] and 0 <= py < frame.shape[0]:
                cv2.circle(image, (int(px), int(py)), 2, color, -1)
        xlo, ylo = points.min(axis=0)
        xhi, yhi = points.max(axis=0)
        cv2.rectangle(image, (int(xlo), int(ylo)), (int(xhi), int(yhi)), color, 2)
        cv2.putText(image, f"C{index}: {int(where.sum())} points",
                    (int(xlo), max(20, int(ylo) - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                    .55, color, 2)
        depth = z[where]
        median = float(np.median(depth))
        component = source[index - 1]
        plausible = (component["sample_count"] >= g["spatial_component_min_samples"]
                     and component["coverage_area_px"] >=
                     g["spatial_component_min_footprint_px"]
                     and component["bbox_width_px"] >= g["spatial_component_min_width_px"]
                     and component["bbox_height_px"] >= g["spatial_component_min_height_px"])
        result["components"].append({
            "component_id": index, "sample_count": int(where.sum()),
            "plausible_by_source_gates": bool(plausible),
            "stereo_bbox_xyxy_px": bbox, "occupied_grid_cells": area,
            "ar_projected_bbox_xyxy_px": [int(xlo), int(ylo), int(xhi), int(yhi)],
            "median_depth_m": median,
            "depth_mad_m": float(np.median(np.abs(depth - median))),
        })
    if sum(c["plausible_by_source_gates"] for c in result["components"]) != int(
        g["plausible_spatial_component_count"]
    ):
        raise ValueError(f"plausible component count mismatch in {path}")
    plausible = [i for i, c in enumerate(result["components"])
                 if c["plausible_by_source_gates"]]
    if len(plausible) in (1, 2):
        selected = [reconstructed[i][0] for i in plausible]
        member_depths = [z[w] for w in selected]
        union_depth = np.concatenate(member_depths)
        median = float(np.median(union_depth))
        candidate = {
            "status": "SINGLE_COMPONENT_REFERENCE" if len(plausible) == 1
                      else "DISCONNECTED_PAIR_HYPOTHESIS_UNVERIFIED",
            "component_ids": [i + 1 for i in plausible],
            "sample_count": int(union_depth.size),
            "median_depth_m": median,
            "depth_mad_m": float(np.median(np.abs(union_depth - median))),
            "depth_p10_p90_m": [float(v) for v in np.percentile(union_depth, [10, 90])],
            "semantic_ownership_verified": False,
        }
        if len(plausible) == 2:
            first, second = [result["components"][i] for i in plausible]
            candidate.update({
                "component_median_depth_difference_m": abs(
                    first["median_depth_m"] - second["median_depth_m"]),
                "projected_ar_bbox_gap_px": bbox_gap(
                    first["ar_projected_bbox_xyxy_px"],
                    second["ar_projected_bbox_xyxy_px"]),
                "rectified_stereo_bbox_gap_px": bbox_gap(
                    first["stereo_bbox_xyxy_px"], second["stereo_bbox_xyxy_px"]),
            })
        result["candidate"] = candidate
    name = f'{audit["cycle_id"][-8:]}_components_ar0234.jpg'
    if not cv2.imwrite(str(output_dir / name), image):
        raise OSError(f"could not write {name}")
    result["overlay"] = name
    result["status"] = "REPLAY_VERIFIED_DIAGNOSTIC_ONLY"
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence-root", type=Path, required=True)
    parser.add_argument("--ar-intrinsic", type=Path, required=True)
    parser.add_argument("--stereo-calibration", type=Path, required=True)
    parser.add_argument("--extrinsic", type=Path, required=True)
    parser.add_argument("--instance-mask-dir", type=Path,
                        help="optional cycle-id.png masks with SHA-bound cycle-id.json metadata")
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.output_dir.exists():
        parser.error(f"output already exists: {args.output_dir}")
    paths = sorted(args.evidence_root.rglob("audit.json"))
    if not paths:
        parser.error("no audit.json in evidence root")
    cal = load_calibrations(args.ar_intrinsic, args.stereo_calibration, args.extrinsic)
    args.output_dir.mkdir(parents=True)
    rows = [replay(path, cal, args.output_dir, args.instance_mask_dir) for path in paths]
    singleton = [r["candidate"]["median_depth_m"] for r in rows
                 if r.get("candidate", {}).get("status") == "SINGLE_COMPONENT_REFERENCE"]
    pairs = [r["candidate"]["median_depth_m"] for r in rows
             if r.get("candidate", {}).get("status") ==
             "DISCONNECTED_PAIR_HYPOTHESIS_UNVERIFIED"]
    series = [r["candidate"]["median_depth_m"] for r in rows if "candidate" in r]
    report = {"status": "DIAGNOSTIC_ONLY_NO_TARGET_SELECTION",
              "source_root": str(args.evidence_root), "calibration_sha256": cal["sha"],
              "limitations": [
                  "The source RGB observation has a box, not a person instance mask.",
                  "Both components projecting inside the box does not prove ownership.",
                  "A union can contain foreground and background; no gate is changed.",
                  "Temporal changes can include subject motion and camera pair skew.",
                  "Mask-based status is emitted only for image- and cycle-bound masks.",
              ],
              "temporal_comparison": {
                  "singleton_count": len(singleton), "pair_count": len(pairs),
                  "singleton_median_depth_m": float(np.median(singleton)) if singleton else None,
                  "pair_median_depth_m": float(np.median(pairs)) if pairs else None,
                  "singleton_depth_span_m": max(singleton)-min(singleton) if singleton else None,
                  "pair_depth_span_m": max(pairs)-min(pairs) if pairs else None,
                  "adjacent_candidate_depth_delta_median_m": float(np.median(
                      np.abs(np.diff(series)))) if len(series) > 1 else None,
              },
              "cycles": rows}
    (args.output_dir / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    for row in rows:
        candidate = row.get("candidate", {})
        print(row["cycle_id"][-8:], row["source_reason"],
              "plausible/total=",
              f'{sum(c["plausible_by_source_gates"] for c in row["components"])}/'
              f'{len(row["components"])}',
              "ids=", candidate.get("component_ids"),
              "joint_depth_m=", round(candidate["median_depth_m"], 4) if candidate else None,
              "joint_MAD_mm=", round(candidate["depth_mad_m"] * 1000, 1)
              if candidate else None,
              "AR_gap_px=", round(candidate["projected_ar_bbox_gap_px"], 1)
              if "projected_ar_bbox_gap_px" in candidate else None)
    print("Temporal comparison:", report["temporal_comparison"])
    if args.instance_mask_dir is not None:
        from collections import Counter
        print("Mask depth statuses:", dict(Counter(
            row.get("instance_mask_depth", {}).get("status", "NOT_EVALUATED")
            for row in rows)))
    print("Saved:", args.output_dir / "report.json")


if __name__ == "__main__":
    main()
