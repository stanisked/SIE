#!/usr/bin/env python3
"""Build a non-active OV9281 v8 candidate from audited raw v8 captures.

The numerical solver is deliberately reused from the reviewed v7 candidate
implementation.  It runs only in a temporary directory, then this wrapper
creates a distinct v8 artifact and report with explicit provenance.  Neither
the v7 runtime artifact nor any activation file is touched.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
import tempfile
from pathlib import Path

import numpy as np

import solve_ov9281_pi_stereo_v7_candidate as v7_solver


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path, action="append", required=True)
    parser.add_argument("--capture-audit", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--holdout-every", type=int, default=4)
    parser.add_argument("--physical-baseline-mm", type=float, default=65.1)
    parser.add_argument("--calibration-id", default="ov9281_pi_stereo_candidate_v8")
    args = parser.parse_args()

    if not re.fullmatch(r"ov9281_pi_stereo_candidate_v8[a-z0-9_]*", args.calibration_id):
        raise ValueError("--calibration-id must start with ov9281_pi_stereo_candidate_v8")
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        raise RuntimeError(f"Output directory is not empty: {args.output_dir}")
    audit = json.loads(args.capture_audit.read_text(encoding="utf-8"))
    if audit.get("schema_version") != "sie.ov9281.pi_stereo_capture_audit.v8":
        raise RuntimeError("v8 solver requires sie.ov9281.pi_stereo_capture_audit.v8")
    if audit.get("result") != "PASS":
        raise RuntimeError("capture audit must PASS")
    args.output_dir.parent.mkdir(parents=True, exist_ok=True)

    legacy_source = Path(v7_solver.__file__).resolve()
    with tempfile.TemporaryDirectory(prefix="ov9281_v8_staging_", dir=args.output_dir.parent) as staging:
        # The reviewed v7 solver deliberately creates --output-dir itself.
        # Use a child path, not the already-created TemporaryDirectory root.
        stage = Path(staging) / "solver_output"
        legacy_argv = [
            str(legacy_source),
            *sum((["--dataset-root", str(root)] for root in args.dataset_root), []),
            "--capture-audit", str(args.capture_audit),
            "--output-dir", str(stage),
            "--holdout-every", str(args.holdout_every),
            "--physical-baseline-mm", str(args.physical_baseline_mm),
        ]
        old_argv = sys.argv
        try:
            sys.argv = legacy_argv
            v7_solver.main()
        finally:
            sys.argv = old_argv
        legacy_candidate = stage / "ov9281_pi_stereo_candidate_v7.npz"
        legacy_report = json.loads((stage / "stereo_candidate_report.json").read_text(encoding="utf-8"))
        with np.load(legacy_candidate, allow_pickle=False) as source:
            arrays = {name: source[name] for name in source.files}

    args.output_dir.mkdir(parents=True, exist_ok=False)
    candidate_path = args.output_dir / f"{args.calibration_id}.npz"
    arrays["calibration_id"] = np.asarray(args.calibration_id)
    arrays["activation_eligible"] = np.asarray(False)
    np.savez(candidate_path, **arrays)
    report = dict(legacy_report)
    report.update({
        "schema_version": "sie.ov9281.pi_stereo_candidate_report.v8",
        "status": "CANDIDATE_NOT_ACTIVE",
        "calibration_id": args.calibration_id,
        "candidate_path": str(candidate_path),
        "candidate_sha256": sha256_file(candidate_path),
        "solver_implementation": {
            "wrapper": "solve_ov9281_pi_stereo_v8_candidate.py",
            "numerical_solver": str(legacy_source),
            "numerical_solver_sha256": sha256_file(legacy_source),
        },
        "v8_capture_audit_sha256": sha256_file(args.capture_audit),
    })
    report_path = args.output_dir / "stereo_candidate_report.json"
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print(f"Saved: {candidate_path}")
    print(f"Saved: {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
