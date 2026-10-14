#!/usr/bin/env python3
"""Offline SGBM scheduling comparison on saved rectified JPEG pairs. No cameras.

Two independent matcher objects, identical inputs and runtime parameters.
Exact comparison covers both signed int16 maps and the bilateral-consistency
mask. This benchmark does not change the ROS node or authorize measurements.
"""
from __future__ import annotations

import argparse
import ast
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import platform
import statistics
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def literal(node):
    if isinstance(node, ast.Constant) and type(node.value) is int:
        return node.value
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.USub):
        return -literal(node.operand)
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Mult):
        return literal(node.left) * literal(node.right)
    if ast.unparse(node) == "self.cv2.STEREO_SGBM_MODE_SGBM_3WAY":
        return cv2.STEREO_SGBM_MODE_SGBM_3WAY
    raise ValueError("runtime matcher parameter is not a supported constant")


def runtime_parameters(root=ROOT):
    configs, hashes = [], {}
    directory = root / "ros2/sie_ros2/sie_ros2"
    for filename, target in (("ar0234_ov9281_metric_measurement_node.py", "self.sgbm"),
                             ("ar0234_ov9281_dynamic_association_audit_node.py", "self.right_sgbm")):
        path = directory / filename
        matches = [n.value for n in ast.walk(ast.parse(path.read_text()))
                   if isinstance(n, ast.Assign) and any(ast.unparse(t) == target for t in n.targets)]
        if (len(matches) != 1 or not isinstance(matches[0], ast.Call)
                or ast.unparse(matches[0].func) != "self.cv2.StereoSGBM_create"
                or matches[0].args or any(k.arg is None for k in matches[0].keywords)):
            raise ValueError("cannot identify exact runtime matcher configuration")
        configs.append({k.arg: literal(k.value) for k in matches[0].keywords})
        hashes[filename] = sha256(path)
    return configs, hashes


def compute_pair(matchers, left, right, executor=None):
    started = time.perf_counter()
    if executor is None:
        a = matchers[0].compute(left, right)
        b = matchers[1].compute(right, left)
    else:
        # Each matcher has one owner/task. Never share one SGBM object.
        first = executor.submit(matchers[0].compute, left, right)
        second = executor.submit(matchers[1].compute, right, left)
        a, b = first.result(), second.result()
    return (a, b), time.perf_counter() - started


def consistency(maps, tolerance=1.5):
    left, right = (d.astype(np.float32) / 16. for d in maps)
    h, w = left.shape
    y, x = np.indices((h, w))
    xr = np.rint(x - left).astype(np.int32)
    lookup = np.isfinite(left) & (left > .5) & (xr >= 0) & (xr < w)
    matched = np.full(left.shape, np.nan, np.float32)
    matched[lookup] = right[y[lookup], xr[lookup]]
    return lookup & np.isfinite(matched) & (matched < -.5) & (np.abs(left + matched) <= tolerance)


def compare(reference, candidate):
    result = {}
    for name, expected, actual in zip(("left", "right"), reference, candidate):
        if expected.shape != actual.shape or expected.dtype != actual.dtype:
            raise ValueError("matcher output shape/dtype changed")
        result[name + "_different_pixels"] = int(np.count_nonzero(expected != actual))
        result[name + "_max_absolute_delta_px"] = float(np.abs(
            expected.astype(np.int32) - actual.astype(np.int32)).max()) / 16.
    result["lr_mask_different_pixels"] = int(np.count_nonzero(consistency(reference) != consistency(candidate)))
    result["exact_match"] = all(result[k] == 0 for k in ("left_different_pixels", "right_different_pixels", "lr_mask_different_pixels"))
    return result


def load_cases(root, count):
    cases = []
    for audit_path in sorted(root.rglob("audit.json")):
        audit = json.loads(audit_path.read_text())
        folder = audit_path.parent
        if folder.name != audit["cycle_id"]:
            raise ValueError("audit/cycle directory mismatch")
        paths = [folder / f"ov9281_physical_{side}_rectified.jpg" for side in ("left", "right")]
        if not all(p.is_file() for p in paths):
            continue
        # Match the live BGR->gray conversion, after decoding frozen JPEGs.
        color = [cv2.imread(str(p), cv2.IMREAD_COLOR) for p in paths]
        if any(a is None or a.shape != (800, 1280, 3) for a in color):
            raise ValueError("expected physical left/right rectified 1280x800 JPEGs")
        images = [cv2.cvtColor(a, cv2.COLOR_BGR2GRAY) for a in color]
        for image in images:
            image.flags.writeable = False
        cases.append({"cycle_id": audit["cycle_id"], "images": images,
                      "source_sha256": {str(p): sha256(p) for p in [audit_path, *paths]},
                      "calibration": audit.get("calibration")})
        if len(cases) == count:
            break
    if not cases:
        raise ValueError("no saved rectified pairs; do not capture new frames for this benchmark")
    return cases


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--cycles", type=int, default=2)
    parser.add_argument("--repeats", type=int, default=2)
    parser.add_argument("--opencv-thread-counts", type=int, nargs="+", default=[0, 1, 2],
                        help="0 preserves initial OpenCV thread count; other values are explicit")
    args = parser.parse_args()
    if args.cycles < 1 or args.repeats < 1 or any(n < 0 for n in args.opencv_thread_counts):
        parser.error("invalid benchmark counts")
    if args.output.exists():
        parser.error("output exists; use a new report filename")
    configs, source_hashes = runtime_parameters()
    cases = load_cases(args.evidence_root, args.cycles)
    initial_threads = cv2.getNumThreads()
    input_hashes = [tuple(hashlib.sha256(a.tobytes()).hexdigest() for a in c["images"]) for c in cases]
    rows, references = [], []
    print(f"Offline pairs={len(cases)} repeats={args.repeats} OpenCV={cv2.__version__} initial_threads={initial_threads}", flush=True)
    try:
        reference_matchers = [cv2.StereoSGBM_create(**c) for c in configs]
        for case in cases:
            maps, _ = compute_pair(reference_matchers, *case["images"])
            references.append(tuple(a.copy() for a in maps))
        with ThreadPoolExecutor(max_workers=2) as executor:
            seen = set()
            for requested in args.opencv_thread_counts:
                threads = initial_threads if requested == 0 else requested
                if threads in seen:
                    continue
                seen.add(threads)
                cv2.setNumThreads(threads)  # Only while no tasks are running.
                matchers = [cv2.StereoSGBM_create(**c) for c in configs]
                warm_checks = {}
                for parallel in (False, True):
                    mode = "parallel" if parallel else "sequential"
                    pool = executor if parallel else None
                    warm, _ = compute_pair(matchers, *cases[0]["images"], executor=pool)
                    warm_checks[mode] = compare(references[0], warm)
                for repeat in range(args.repeats):
                    for index, case in enumerate(cases):
                        # Alternate AB/BA order to reduce drift/order bias.
                        order = (False, True) if (repeat + index) % 2 == 0 else (True, False)
                        for parallel in order:
                            mode = "parallel" if parallel else "sequential"
                            pool = executor if parallel else None
                            maps, elapsed = compute_pair(matchers, *case["images"], executor=pool)
                            row = {"mode": mode, "opencv_threads": cv2.getNumThreads(),
                                   "cycle_id": case["cycle_id"], "repeat": repeat,
                                   "matcher_pair_wall_s": elapsed, "warmup_check": warm_checks[mode],
                                   **compare(references[index], maps)}
                            rows.append(row)
                for mode in ("sequential", "parallel"):
                    subset = [r for r in rows if r["mode"] == mode and r["opencv_threads"] == threads]
                    print(f"{mode} threads={threads}: median_s={statistics.median(r['matcher_pair_wall_s'] for r in subset):.4f} "
                          f"exact={sum(r['exact_match'] for r in subset)}/{len(subset)}", flush=True)
    finally:
        cv2.setNumThreads(initial_threads)
    unchanged = all(before == tuple(hashlib.sha256(a.tobytes()).hexdigest() for a in case["images"])
                    for before, case in zip(input_hashes, cases))
    sources_unchanged = all(sha256(Path(p)) == digest for c in cases for p, digest in c["source_sha256"].items())
    report = {"status": "OFFLINE_SCHEDULING_COMPARISON_ONLY", "rows": rows,
              "all_exact": all(r["exact_match"] and r["warmup_check"]["exact_match"] for r in rows),
              "inputs_unchanged": unchanged and sources_unchanged,
              "matcher_parameters": configs, "runtime_source_sha256": source_hashes,
              "tool_sha256": sha256(Path(__file__)),
              "sources": [{k: v for k, v in c.items() if k != "images"} for c in cases],
              "environment": {"python": platform.python_version(), "opencv": cv2.__version__,
                              "numpy": np.__version__, "machine": platform.machine(),
                              "logical_cpus": os.cpu_count(), "initial_opencv_threads": initial_threads,
                              "opencv_build_information": cv2.getBuildInformation()},
              "limitations": ["JPEG-decoded rectified evidence differs from original live arrays.",
                              "Matcher time only; LR comparison, file reads, matcher creation and pool warmup excluded.",
                              "Small benchmark with alternating AB/BA order; load and thermal effects can affect timing.",
                              "No automatic runtime change; numerical equality is not physical accuracy."]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print("ALL_EXACT:", report["all_exact"], "INPUTS_UNCHANGED:", report["inputs_unchanged"], flush=True)
    print("Saved:", args.output, flush=True)
    if not report["all_exact"] or not report["inputs_unchanged"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
