"""Internal companion-worker request; reuses the verified offline algorithms."""

from __future__ import annotations

import json
from pathlib import Path
import platform
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def process_surface_request(request: dict) -> dict:
    # Lazy imports keep SciPy outside the ROS Python environment.
    import numpy as np
    import scipy
    import cv2
    from analyze_ar0234_stereo_component_ambiguity import load_calibrations, sha256
    from replay_shadow_mask_depth_range import replay
    from analyze_shadow_local_surfaces import analyze_cycle

    started = time.monotonic()
    path = Path(request["snapshot"])
    cid = str(request["cycle_id"])
    if path.name != "shadow_surface_input.json" or path.parent.name != cid:
        raise ValueError("surface snapshot does not belong to cycle")
    if sha256(path) != request["snapshot_sha256"]:
        raise ValueError("surface snapshot SHA mismatch")
    audit = json.loads(path.read_text())
    if audit["cycle_id"] != cid:
        raise ValueError("surface cycle mismatch")
    lower, upper = request["diagnostic_depth_range_m"]
    if not np.isfinite([lower, upper]).all() or not 0 < lower < upper <= 4.5:
        raise ValueError("invalid separate diagnostic range")
    disparity = path.parent / "disparity_lr_consistent_px.npy"
    if sha256(disparity) != request["disparity_sha256"]:
        raise ValueError("surface disparity SHA mismatch")
    activation_path = Path(request["activation"])
    if sha256(activation_path) != request["activation_sha256"]:
        raise ValueError("activation SHA mismatch")
    activation = json.loads(activation_path.read_text())
    cp = activation["calibration_profile"]
    extrinsic = cp["ar0234_to_ov9281_physical_left_extrinsic"]
    ar, stereo = Path(request["ar_intrinsic"]), Path(request["stereo_calibration"])
    ex = Path(extrinsic["path"]).expanduser()
    if (sha256(ar) != cp["source_sha256"]["ar0234_intrinsic"]
            or sha256(stereo) != cp["source_sha256"]["ov9281_stereo_calibration"]
            or sha256(ex) != extrinsic["sha256"]):
        raise ValueError("activation calibration binding mismatch")
    cal = load_calibrations(ar, stereo, ex)
    raw = replay(path, cal, lower, upper)
    # Do not export PLY in a live cycle. Preserve original samples for replay.
    output = path.parent / "shadow_local_surfaces"
    output.mkdir(exist_ok=False)
    row = analyze_cycle(path, raw, cal, lower, upper, output, False)
    local = row["local_surface"]
    patches = local.get("patch_count", 0)
    local["unfitted_patch_fraction"] = (1 - local.get("fitted_patch_count", 0) / patches
                                           if patches else None)
    result = {
        "cycle_id": cid, "status": "SHAPE_AWARE_SHADOW_DIAGNOSTIC_ONLY",
        "reference_frame": "rectified_left_optical_frame", "unit": "m",
        "confidence": None, "confidence_status": "NOT_VALIDATED",
        "metric_measurement_authorized": False, "navigation_eligible": False,
        "actuator_access": False,
        "diagnostic_depth_range_m": [lower, upper],
        "activated_depth_range_m": activation["authorization"]["metric_measurement_range_m"],
        "calibration_sha256": cal["sha"], "activation_sha256": sha256(activation_path),
        "method": {"tile_px": 32, "minimum_points": 24, "mask_erosion_px": 2,
                   "mask_minimum_samples": 100, "mask_maximum_mad_m": 0.05,
                   "test_split": "alternating original stereo-grid cells",
                   "acceptance_rule_added": False},
        "environment": {"python": platform.python_version(), "numpy": np.__version__,
                        "scipy": scipy.__version__, "opencv": cv2.__version__},
        "code_sha256": {"request": sha256(Path(__file__)),
                        "local_analysis": sha256(Path(__file__).with_name("analyze_shadow_local_surfaces.py")),
                        "replay": sha256(Path(__file__).with_name("replay_shadow_mask_depth_range.py")),
                        "projection": sha256(Path(__file__).with_name("compare_shadow_mask_depth_median.py")),
                        "mask_fusion": sha256(ROOT / "vision_core/person_localization/instance_mask_stereo_fusion.py"),
                        "surface_model": sha256(ROOT / "vision_core/person_localization/local_surface_diagnostic.py")},
        "result": row,
    }
    report = output / "report.json"
    with report.open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    return {**{k: v for k, v in result.items() if k != "result"},
            "mask_depth": row["mask_depth"],
            "local_surface": {k: v for k, v in local.items() if k != "patches"},
            "report_file": str(report.relative_to(path.parent)),
            "report_sha256": sha256(report),
            "elapsed_s": time.monotonic() - started}
