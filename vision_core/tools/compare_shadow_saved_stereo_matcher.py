#!/usr/bin/env python3
"""Compare wider SGBM and StereoBM on frozen Shadow stereo pairs, offline only.

Saved rectified JPEGs were encoded after the original in-memory stereo match.
The saved NPY is verified against its baseline; both new matchers consume the
same decoded JPEGs so their comparison is paired, not a replacement replay.
No camera, ROS, controller, policy or Measurement access.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import platform
import time

import cv2
import numpy as np

from analyze_ar0234_stereo_component_ambiguity import load_calibrations, sha256
from compare_shadow_mask_depth_median import in_roi, owned, project, verify_baseline
from replay_shadow_mask_depth_range import replay
from vision_core.person_localization.instance_mask_stereo_fusion import masked_depth_hypothesis

try:
    import scipy
    from vision_core.person_localization.local_surface_diagnostic import analyze_local_surfaces
except ModuleNotFoundError as error:
    if error.name != "scipy":
        raise
    scipy = None
    analyze_local_surfaces = None


def matcher(profile: str, min_disparity: int):
    if profile not in ("sgbm7", "sgbm11", "bm15") or min_disparity not in (0, -128):
        raise ValueError("only the declared reference and candidate are supported")
    if profile == "bm15":
        result = cv2.StereoBM_create(numDisparities=128, blockSize=15)
        result.setMinDisparity(min_disparity)
        result.setPreFilterType(cv2.STEREO_BM_PREFILTER_XSOBEL)
        result.setPreFilterSize(9)
        result.setPreFilterCap(31)
        result.setTextureThreshold(10)
        result.setUniquenessRatio(10)
        result.setSpeckleWindowSize(100)
        result.setSpeckleRange(2)
        result.setDisp12MaxDiff(1)
        return result
    block = 7 if profile == "sgbm7" else 11
    return cv2.StereoSGBM_create(
        minDisparity=min_disparity, numDisparities=128, blockSize=block,
        P1=8 * 3 * block * block, P2=32 * 3 * block * block,
        uniquenessRatio=10, speckleWindowSize=100, speckleRange=2,
        disp12MaxDiff=1, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)


def lr_pair(left: np.ndarray, right: np.ndarray, profile: str) -> tuple[np.ndarray, np.ndarray, float]:
    start = time.monotonic()
    d = matcher(profile, 0).compute(left, right).astype(np.float32) / 16.0
    reverse = matcher(profile, -128).compute(right, left).astype(np.float32) / 16.0
    y, x = np.indices(d.shape)
    xr = np.rint(x - d).astype(np.int32)
    lookup = np.isfinite(d) & (d > .5) & (xr >= 0) & (xr < d.shape[1])
    matched = np.full(d.shape, np.nan, dtype=np.float32)
    matched[lookup] = reverse[y[lookup], xr[lookup]]
    consistent = lookup & (matched < -.5) & (np.abs(d + matched) <= 1.5)
    return d, consistent, time.monotonic() - start


def evaluate(disparity: np.ndarray, consistent: np.ndarray, mask: np.ndarray,
             roi: list, stride: int, cal: dict, lower: float, upper: float) -> tuple[dict, np.ndarray]:
    if disparity.shape != consistent.shape or disparity.shape != tuple(reversed(cal["size"])):
        raise ValueError("matcher evidence shape/calibration mismatch")
    v, u = np.mgrid[0:disparity.shape[0]:stride, 0:disparity.shape[1]:stride]
    d = disparity[::stride, ::stride]
    ok = consistent[::stride, ::stride] & np.isfinite(d) & (d > .5)
    u, v, d = u[ok], v[ok], d[ok]
    z = abs(float(cal["P2"][0, 3])) / d / 1000.0
    valid_z = (z >= lower) & (z <= upper)
    u, v, d, z = u[valid_z], v[valid_z], d[valid_z], z[valid_z]
    xy = project(u.astype(np.float64), v.astype(np.float64), z, cal)
    inside = in_roi(xy, roi)
    hypothesis = masked_depth_hypothesis(xy[inside], z[inside], mask == 255)
    interior = cv2.erode(mask, np.ones((5, 5), dtype=np.uint8),
                         borderType=cv2.BORDER_CONSTANT, borderValue=0)
    selected = owned(xy, interior, roi)
    if int(selected.sum()) != hypothesis.get("selected_sample_count"):
        raise ValueError("projected mask support differs from hypothesis")
    grid = np.zeros(disparity.shape, dtype=bool)
    grid[v[selected], u[selected]] = True
    if selected.sum() >= 12 and analyze_local_surfaces is not None:
        surface, _ = analyze_local_surfaces(u[selected], v[selected], d[selected],
            abs(float(cal["P2"][0, 3])) / 1000.0, stride=stride)
        local = {key: surface[key] for key in (
            "patch_count", "fitted_patch_count", "fitted_sample_fraction",
            "patch_status_counts", "pooled_held_out_axial_residual")}
    elif analyze_local_surfaces is None:
        local = {"status": "UNAVAILABLE_SCIPY_NOT_INSTALLED"}
    else:
        local = {"status": "INSUFFICIENT_OWNED_STEREO_SAMPLES"}
    return {"mask_depth": hypothesis, "local_surface": local,
            "grid_lr_consistent_count": int(ok.sum()),
            "grid_in_diagnostic_range_count": int(len(z))}, grid


def compare_grids(a: np.ndarray, b: np.ndarray, da: np.ndarray, db: np.ndarray) -> dict:
    common = a & b
    union = a | b
    delta = db[common] - da[common]
    return {"a_count": int(a.sum()), "b_count": int(b.sum()),
            "common_count": int(common.sum()),
            "left_a_count": int((a & ~b).sum()), "entered_b_count": int((~a & b).sum()),
            "jaccard": float(common.sum() / union.sum()) if union.any() else None,
            "median_signed_disparity_delta_px": float(np.median(delta)) if delta.size else None,
            "median_absolute_disparity_delta_px": float(np.median(np.abs(delta))) if delta.size else None,
            "p90_absolute_disparity_delta_px": float(np.percentile(np.abs(delta), 90)) if delta.size else None}


def analyze(path: Path, baseline: dict, cal: dict, lower: float, upper: float,
            output_dir: Path) -> dict:
    audit_path = path / "audit.json"
    audit = json.loads(audit_path.read_text(encoding="utf-8"))
    cid = audit["cycle_id"]
    if cid != baseline["cycle_id"] or not isinstance(audit.get("stereo_geometry"), dict):
        raise ValueError("selected cycle has no verified stereo geometry")
    roi = audit["stereo_geometry"]["ar_inner_roi_xyxy_px"]
    stride = int(audit["stereo_geometry"]["sample_stride_px"])
    if stride < 1:
        raise ValueError("invalid saved sample stride")
    files = {"audit": audit_path, "mask": path / f"{cid}.png",
             "saved_raw": path / "disparity_raw_px.npy",
             "saved_lr": path / "disparity_lr_consistent_px.npy",
             "rectified_left": path / "ov9281_physical_left_rectified.jpg",
             "rectified_right": path / "ov9281_physical_right_rectified.jpg"}
    source_hashes = {key: sha256(file) for key, file in files.items()}
    mask = cv2.imread(str(files["mask"]), cv2.IMREAD_GRAYSCALE)
    left_bgr = cv2.imread(str(files["rectified_left"]))
    right_bgr = cv2.imread(str(files["rectified_right"]))
    if (mask is None or mask.shape != (1200, 1920) or
            not np.isin(mask, (0, 255)).all() or
            left_bgr is None or right_bgr is None or
            left_bgr.shape != right_bgr.shape or
            left_bgr.shape[:2] != tuple(reversed(cal["size"]))):
        raise ValueError("saved image shape or mask invalid")
    left = cv2.cvtColor(left_bgr, cv2.COLOR_BGR2GRAY)
    right = cv2.cvtColor(right_bgr, cv2.COLOR_BGR2GRAY)
    saved_d = np.load(files["saved_raw"], allow_pickle=False)
    saved_lr_d = np.load(files["saved_lr"], allow_pickle=False)
    if saved_d.shape != left.shape or saved_lr_d.shape != left.shape:
        raise ValueError("saved disparity shape invalid")
    saved_mask = np.isfinite(saved_lr_d)
    if not np.array_equal(saved_lr_d[saved_mask], saved_d[saved_mask]):
        raise ValueError("saved LR disparity differs from saved raw")
    results = {}
    arrays = {}
    for label, profile in (("jpeg_reference_7px", "sgbm7"),
                           ("jpeg_candidate_11px", "sgbm11"),
                           ("jpeg_stereo_bm_15px", "bm15")):
        d, lr, elapsed = lr_pair(left, right, profile)
        metrics, grid = evaluate(d, lr, mask, roi, stride, cal, lower, upper)
        metrics["matching_elapsed_s"] = elapsed
        results[label] = metrics
        arrays[label] = (d, lr, grid)
    saved_metrics, saved_grid = evaluate(saved_d, saved_mask, mask, roi, stride,
                                        cal, lower, upper)
    if saved_metrics["mask_depth"] != baseline["mask_depth"]:
        raise ValueError("saved matcher evidence does not reproduce the baseline")
    ref_d, ref_lr, ref_grid = arrays["jpeg_reference_7px"]
    cand_d, cand_lr, cand_grid = arrays["jpeg_candidate_11px"]
    bm_d, bm_lr, bm_grid = arrays["jpeg_stereo_bm_15px"]
    # The saved in-memory output and the JPEG rerun are never called identical.
    valid_raw = (saved_d > .5) & (ref_d > .5)
    delta_raw = np.abs(saved_d[valid_raw] - ref_d[valid_raw])
    artifact = output_dir / f"{cid}_matcher_arrays.npz"
    np.savez_compressed(artifact, reference_disparity_px=ref_d,
                        reference_lr_consistent=ref_lr,
                        candidate_disparity_px=cand_d,
                        candidate_lr_consistent=cand_lr,
                        bm_disparity_px=bm_d, bm_lr_consistent=bm_lr)
    return {"cycle_id": cid, "saved_original": saved_metrics,
            "jpeg_reference_7px": results["jpeg_reference_7px"],
            "jpeg_candidate_11px": results["jpeg_candidate_11px"],
            "jpeg_stereo_bm_15px": results["jpeg_stereo_bm_15px"],
            "saved_vs_jpeg_reference": {
                "shared_raw_valid_count": int(valid_raw.sum()),
                "raw_median_absolute_disparity_delta_px": float(np.median(delta_raw)) if delta_raw.size else None,
                "raw_exact_to_one_sixteenth_fraction": float(np.mean(delta_raw <= 1/16)) if delta_raw.size else None,
                "owned_grid": compare_grids(saved_grid, ref_grid, saved_d, ref_d)},
            "reference_vs_candidate": {
                "owned_grid": compare_grids(ref_grid, cand_grid, ref_d, cand_d),
                "lr_grid": compare_grids(ref_lr, cand_lr, ref_d, cand_d)},
            "reference_vs_bm": {
                "owned_grid": compare_grids(ref_grid, bm_grid, ref_d, bm_d),
                "lr_grid": compare_grids(ref_lr, bm_lr, ref_d, bm_d)},
            "source_sha256": source_hashes, "artifact": artifact.name,
            "artifact_sha256": sha256(artifact)}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-report", type=Path, required=True)
    parser.add_argument("--ar-intrinsic", type=Path, required=True)
    parser.add_argument("--stereo-calibration", type=Path, required=True)
    parser.add_argument("--activation", type=Path, required=True)
    parser.add_argument("--cycles", nargs=3, default=["00000002", "00000008", "00000010"])
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.output_dir.exists():
        parser.error("output directory already exists; choose a new path")
    baseline = json.loads(args.baseline_report.read_text(encoding="utf-8"))
    lower, upper = baseline["diagnostic_depth_range_m"]
    if not np.isfinite([lower, upper]).all() or not 0 < lower < upper <= 4.5:
        raise ValueError("invalid baseline diagnostic range")
    activation = json.loads(args.activation.read_text(encoding="utf-8"))
    profile = activation["calibration_profile"]
    extrinsic = profile["ar0234_to_ov9281_physical_left_extrinsic"]
    if (sha256(args.ar_intrinsic) != profile["source_sha256"]["ar0234_intrinsic"]
            or sha256(args.stereo_calibration) != profile["source_sha256"]["ov9281_stereo_calibration"]
            or sha256(Path(extrinsic["path"]).expanduser()) != extrinsic["sha256"]):
        raise ValueError("activation calibration SHA mismatch")
    cal = load_calibrations(args.ar_intrinsic, args.stereo_calibration,
                            Path(extrinsic["path"]).expanduser())
    if cal["sha"] != baseline["calibration_sha256"]:
        raise ValueError("baseline/calibration SHA mismatch")
    source_root = Path(baseline["source_root"])
    selected = []
    if len(set(args.cycles)) != 3:
        raise ValueError("three distinct cycles required")
    for suffix in args.cycles:
        matches = [row for row in baseline["cycles"] if row["cycle_id"].endswith(suffix)]
        if len(matches) != 1:
            raise ValueError(f"ambiguous or missing baseline cycle: {suffix}")
        saved = matches[0]
        paths = list(source_root.rglob(f"{saved['cycle_id']}/audit.json"))
        if len(paths) != 1:
            raise ValueError(f"ambiguous or missing source: {suffix}")
        current = replay(paths[0], cal, lower, upper)
        verify_baseline(current, saved)
        selected.append((paths[0].parent, saved))
    cv2.setNumThreads(1)
    print(f"BASELINE_REPLAY_VERIFIED {len(selected)}/{len(selected)}", flush=True)
    args.output_dir.mkdir(parents=True, exist_ok=False)
    rows = []
    for path, saved in selected:
        row = analyze(path, saved, cal, lower, upper, args.output_dir)
        rows.append(row)
        def short(label):
            result = row[label]["mask_depth"]
            return (result["status"], result.get("candidate_sample_count"),
                    round(result["candidate_depth_m"], 4) if "candidate_depth_m" in result else None,
                    round(result["candidate_mad_m"] * 1000, 1) if "candidate_mad_m" in result else None)
        print(row["cycle_id"][-8:], "saved", short("saved_original"),
              "jpeg7", short("jpeg_reference_7px"),
              "jpeg11", short("jpeg_candidate_11px"),
              "jpegBM15", short("jpeg_stereo_bm_15px"), flush=True)
    report = {"status": "SAVED_JPEG_STEREO_MATCHER_COMPARISON_DIAGNOSTIC_ONLY",
              "reference_frame": "rectified_left_optical_frame", "unit": "m",
              "confidence": None, "confidence_status": "NOT_VALIDATED",
              "metric_measurement_authorized": False, "navigation_eligible": False,
              "actuator_access": False, "calibration_sha256": cal["sha"],
              "activated_depth_range_m": activation["authorization"]["metric_measurement_range_m"],
              "diagnostic_depth_range_m": [lower, upper],
              "baseline_report_sha256": sha256(args.baseline_report),
              "source_root": str(source_root),
              "method": {"reference": "saved original plus paired JPEG SGBM 7px",
                         "candidate": "paired JPEG SGBM 11px (P1/P2 rescaled with block area)",
                         "alternate": "paired JPEG StereoBM 15px, XSOBEL 9/31, texture 10",
                         "common": "128 disparities, uniqueness 10, speckle 100/2, LR 1.5px"},
              "environment": {"python": platform.python_version(), "opencv": cv2.__version__,
                              "numpy": np.__version__,
                              "scipy": scipy.__version__ if scipy is not None else None},
              "code_sha256": {"tool": sha256(Path(__file__)),
                              "replay": sha256(Path(__file__).with_name("replay_shadow_mask_depth_range.py")),
                              "mask_depth": sha256(Path(__file__).resolve().parents[1] /
                                                   "person_localization/instance_mask_stereo_fusion.py")},
              "cycles": rows,
              "limitations": ["Saved rectified JPEGs were re-encoded after in-memory stereo matching; exact original disparity reproduction is not expected.",
                              "Wider blocks can erase thin features or mix background with a moving subject; StereoBM may refuse low-texture areas.",
                              "Local surface fits are unavailable when SciPy is not installed; masked-depth and LR comparisons still run.",
                              "No independent physical range truth or exposure synchronization.",
                              "A lower MAD, local residual or changed diagnostic status cannot authorize Measurement or motion."]}
    destination = args.output_dir / "report.json"
    destination.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print("Saved:", destination)


if __name__ == "__main__":
    main()
