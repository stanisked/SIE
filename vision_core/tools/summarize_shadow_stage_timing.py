#!/usr/bin/env python3
"""Print saved host-stage timings; never opens cameras or changes evidence."""
import argparse
import json
from pathlib import Path
import statistics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("evidence_root", type=Path)
    args = parser.parse_args()
    runs = {}
    for path in sorted(args.evidence_root.rglob("audit.json")):
        audit = json.loads(path.read_text())
        timing = audit.get("processing_timing", {})
        if "stages" in timing:
            runs.setdefault(path.parent.parent, []).append(timing)
    rows = [row for run in runs.values() for row in run[1:]]
    print("TIMING_SUMMARY: first saved cycle of each run excluded; cycles=", len(rows))
    if not rows:
        return
    def show(label, values):
        if values:
            print(f"{label}: n={len(values)} median_s={statistics.median(values):.4f} max_s={max(values):.4f}")
    show("total_before_final_audit", [r["host_capture_start_to_result_ready_s"] for r in rows])
    names = set().union(*(r["stages"] for r in rows))
    stages = {name: [r["stages"][name]["exclusive_s"] for r in rows if name in r["stages"]] for name in names}
    for name in sorted(stages, key=lambda n: statistics.median(stages[n]), reverse=True):
        show(name, stages[name])
    show("unattributed", [r["unattributed_s"] for r in rows])
    print("Exclusive times only; stage n may differ on refused/missing-target cycles.")


if __name__ == "__main__":
    main()
