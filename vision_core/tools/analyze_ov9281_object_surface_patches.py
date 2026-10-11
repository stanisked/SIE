#!/usr/bin/env python3
"""Offline diagnostic of local depth patches in saved RGB/stereo audits.

Use the exact selected component mask where recorded. Older evidence has only
the component bbox, which is explicitly reported as an approximation. This
tool never changes the live node's quality gates or publishes ROS data.
"""

from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np


def median_mad(values: np.ndarray) -> tuple[float, float]:
    median = float(np.median(values))
    return median, float(np.median(np.abs(values - median)))


def analyse_cycle(path: Path, tile_px: int, min_points: int) -> dict:
    audit = json.loads(path.read_text(encoding="utf-8"))
    geometry = audit.get("stereo_geometry")
    if not isinstance(geometry, dict) or "selected_spatial_component" not in geometry:
        return {"cycle_id": audit.get("cycle_id"), "source_status": audit.get("status"),
                "status": "NO_SELECTED_COMPONENT"}
    disparity = np.load(path.parent / "disparity_lr_consistent_px.npy", allow_pickle=False)
    if disparity.shape != (800, 1280):
        raise ValueError(f"unexpected disparity shape in {path.parent}: {disparity.shape}")

    x1, y1, x2, y2 = geometry["physical_left_frustum_roi_xyxy_px"]
    bx1, by1, bx2, by2 = geometry["selected_spatial_component"]["bbox_xyxy_px"]
    stride = int(geometry["sample_stride_px"])
    if stride <= 0 or not (0 <= x1 < x2 <= 1280 and 0 <= y1 < y2 <= 800):
        raise ValueError(f"invalid frustum grid in {path}")
    yy, xx = np.mgrid[y1:y2:stride, x1:x2:stride]
    values = disparity[yy, xx]
    diagnostic = geometry["pre_range_depth_diagnostic"]
    range_min, range_max = map(float, diagnostic["configured_depth_range_m"])
    disp_min, disp_max = map(float, diagnostic["configured_disparity_range_px"])
    focal_baseline = disp_min * range_max
    if not math.isclose(focal_baseline, disp_max * range_min, rel_tol=1e-5):
        raise ValueError(f"inconsistent calibrated disparity range in {path}")
    mask_record = geometry.get("selected_spatial_component_mask")
    mask_path = path.parent / "selected_spatial_component_mask.npy"
    if mask_record is None and mask_path.exists():
        raise ValueError(f"unreferenced component mask in {path.parent}")
    if mask_record is not None:
        if (mask_record.get("file") != mask_path.name
                or mask_record.get("coordinate_frame") != "rectified_physical_left"):
            raise ValueError(f"invalid component mask metadata in {path}")
        mask = np.load(mask_path, allow_pickle=False)
        if mask.shape != disparity.shape or mask.dtype != np.uint8:
            raise ValueError(f"invalid component mask shape or dtype in {mask_path}")
        if int(np.count_nonzero(mask)) != int(mask_record["sample_count"]):
            raise ValueError(f"component mask count mismatch in {path}")
        component = mask[yy, xx] == 1
        mode = "EXACT_COMPONENT_SAMPLES"
    else:
        component = ((xx >= bx1) & (xx < bx2) & (yy >= by1) & (yy < by2))
        mode = "BBOX_APPROXIMATION_LEGACY"
    valid = (np.isfinite(values) & (values >= disp_min) & (values <= disp_max)
             & component)
    x, y, depth = xx[valid], yy[valid], focal_baseline / values[valid]
    if not len(depth):
        raise ValueError(f"empty component bbox in {path}")
    if mask_record is not None and len(depth) != int(mask_record["sample_count"]):
        raise ValueError(f"component mask and valid disparity disagree in {path}")

    bbox_median, bbox_mad = median_mad(depth)
    groups: dict[tuple[int, int], list[float]] = defaultdict(list)
    for px, py, z in zip(x, y, depth):
        groups[(int(px) // tile_px, int(py) // tile_px)].append(float(z))
    patches = []
    for (tx, ty), samples in sorted(groups.items()):
        if len(samples) < min_points:
            continue
        median, mad = median_mad(np.asarray(samples))
        patches.append({"grid_xy": [tx, ty], "bbox_xyxy_px": [tx * tile_px, ty * tile_px,
                        (tx + 1) * tile_px, (ty + 1) * tile_px],
                        "sample_count": len(samples), "median_depth_m": median, "mad_m": mad})
    source_cluster = geometry["depth_clusters"][0]
    return {
        "cycle_id": audit["cycle_id"], "source_status": audit["status"],
        "status": "PATCH_DIAGNOSTIC_ONLY", "selection_mode": mode,
        "calibration": audit["calibration"],
        "source_cluster": source_cluster,
        "selected_samples": {"sample_count": len(depth), "median_depth_m": bbox_median,
                               "mad_m": bbox_mad, "selected_component_bbox_xyxy_px":
                               [bx1, by1, bx2, by2]},
        "patches": patches,
    }


def analyse(root: Path, tile_px: int = 32, min_points: int = 24) -> dict:
    paths = sorted(root.rglob("audit.json"))
    if not paths:
        raise ValueError(f"no audit.json under {root}")
    cycles = [analyse_cycle(p, tile_px, min_points) for p in paths]
    hashes = {c["calibration"]["ov9281_stereo_sha256"] for c in cycles
              if c["status"] == "PATCH_DIAGNOSTIC_ONLY"}
    if not hashes:
        raise ValueError(f"no cycle with a selected spatial component under {root}")
    if len(hashes) != 1:
        raise ValueError(f"calibration changed within run: {hashes}")
    by_position: dict[tuple[str, int, int], list[float]] = defaultdict(list)
    for cycle in cycles:
        for patch in cycle.get("patches", []):
            by_position[(cycle["selection_mode"], *patch["grid_xy"])].append(
                patch["median_depth_m"])
    temporal = []
    for (mode, tx, ty), samples in sorted(by_position.items()):
        if len(samples) < 2:
            continue
        median, mad = median_mad(np.asarray(samples))
        temporal.append({"selection_mode": mode, "grid_xy": [tx, ty],
                         "cycle_count": len(samples),
                         "median_depth_m": median, "temporal_mad_m": mad,
                         "peak_to_peak_m": max(samples) - min(samples)})
    return {
        "schema_version": "sie.ov9281.object_surface_patch_offline_diagnostic.v2",
        "status": "DIAGNOSTIC_ONLY_NOT_A_MEASUREMENT",
        "limitations": [
            "Older cycles without a mask use a bbox approximation containing extra pixels.",
            "Exact component samples are still geometric candidates, not semantic ownership.",
            "A low local MAD does not establish semantic ownership or valid target range.",
            "Fixed image patches do not compensate target or camera motion.",
            "No live gate, calibration, runtime profile, or actuator policy is modified.",
        ],
        "source_root": str(root), "calibration_sha256": next(iter(hashes)),
        "tile_size_px": tile_px, "minimum_samples_per_patch": min_points,
        "cycles": cycles, "temporal_patches": temporal,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error(f"output already exists: {args.output}")
    result = analyse(args.evidence_root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print("cycle | source | selection | selected_MAD_mm | patches | local_MAD_le_50mm | patch_depth_span_mm")
    for cycle in result["cycles"]:
        patches = cycle.get("patches", [])
        medians = [p["median_depth_m"] for p in patches]
        span = (max(medians) - min(medians)) * 1000 if medians else float("nan")
        selected = cycle.get("selected_samples", {})
        print(f'{cycle["cycle_id"][-8:]} | {cycle["source_status"]} | '
              f'{cycle.get("selection_mode", "NONE")} | '
              f'{selected.get("mad_m", float("nan"))*1000:.1f} | {len(patches)} | '
              f'{sum(p["mad_m"] <= .05 for p in patches)} | {span:.1f}')
    stable = [p for p in result["temporal_patches"] if p["cycle_count"] >= 12]
    print(f'patches_seen_in_at_least_12_cycles={len(stable)}')
    if stable:
        print('their_temporal_MAD_mm_median='
              f'{np.median([p["temporal_mad_m"] for p in stable])*1000:.1f}')
    print(f'Saved diagnostic: {args.output}')
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
