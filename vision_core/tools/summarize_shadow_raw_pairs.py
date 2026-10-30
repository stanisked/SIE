#!/usr/bin/env python3
"""Inventory exact-cycle mask/disparity pairs from the full camera audit."""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path


def summarize(root: Path):
    counts = Counter()
    examples = {"COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY": [],
                "DEPTH_MAD_EXCEEDED": []}
    incomplete = []
    for audit_file in sorted(root.glob("*/*/audit.json")):
        row = json.loads(audit_file.read_text(encoding="utf-8"))
        cycle = row.get("cycle_id")
        if cycle != audit_file.parent.name:
            raise ValueError(f"cycle directory mismatch: {audit_file}")
        depth = (row.get("stereo_geometry") or {}).get("shadow_mask_depth") or {}
        status = depth.get("status", "NO_MASK_DEPTH_RESULT")
        counts[status] += 1
        if status not in examples or len(examples[status]) >= 5:
            continue
        directory = audit_file.parent
        mask = directory / (cycle + ".png")
        metadata = directory / (cycle + ".json")
        disparity = directory / "disparity_lr_consistent_px.npy"
        rgb = directory / "ar0234_source.jpg"
        stereo = directory / "ov9281_combined_source.jpg"
        required = (mask, metadata, disparity, rgb, stereo)
        missing = [path.name for path in required if not path.is_file()]
        if missing:
            incomplete.append({"cycle_id": cycle, "missing": missing})
            continue
        meta = json.loads(metadata.read_text(encoding="utf-8"))
        digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
        if (meta.get("cycle_id") != cycle
                or meta.get("mask_sha256") != digest(mask)
                or meta.get("source_ar0234_sha256") != digest(rgb)
                or meta.get("model_sha256") != (row.get("shadow_instance_mask") or {}).get("worker_model_sha256")):
            raise ValueError(f"mask/source binding mismatch: {cycle}")
        examples[status].append({
            "cycle_id": cycle, "directory": str(directory),
            "mask_depth": depth, "processing_timing": row.get("processing_timing"),
            "pairing": row.get("pairing"), "calibration": row.get("calibration"),
            "disparity_sha256": digest(disparity),
            "files": [path.name for path in required],
        })
    return {"schema_version": "sie.shadow_raw_pairs_inventory.v1",
            "scope": "FULL_RESOLUTION_CAMERA_AUDIT_DIAGNOSTIC_ONLY",
            "motor_requests": 0, "records": sum(counts.values()),
            "mask_depth_status_counts": dict(counts), "examples": examples,
            "incomplete_examples": incomplete,
            "note": "Full-resolution audit differs from the half-resolution control preview; compare failures, do not treat outcomes as interchangeable."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("evidence_root", type=Path)
    args = parser.parse_args()
    report = summarize(args.evidence_root)
    out = args.evidence_root / "shadow_raw_pairs_summary.json"
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"summary": str(out), "records": report["records"],
                      "status_counts": report["mask_depth_status_counts"],
                      "complete_examples": {key: len(value) for key, value in report["examples"].items()},
                      "incomplete_examples": len(report["incomplete_examples"])}, ensure_ascii=False))
    return 0 if report["records"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
