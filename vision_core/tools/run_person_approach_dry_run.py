"""Dry-run stop-and-measure person-approach decisions; no actuator integration."""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from vision_core.person_approach.decision import PersonApproachDecisionEngine
from vision_core.person_depth_fusion.live import LiveFusionError, build_live_runtime


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--project-root", type=Path, required=True)
    parser.add_argument("--person-threshold", type=float, choices=(.4, .5), required=True)
    parser.add_argument("--max-decisions", type=int, required=True)
    parser.add_argument("--cycle-interval-s", type=float, default=.0)
    args = parser.parse_args()
    if args.max_decisions <= 0 or args.cycle_interval_s < 0:
        parser.error("--max-decisions must be positive and --cycle-interval-s non-negative")
    runtime = None
    try:
        runtime = build_live_runtime(model=args.model, reference=args.reference, project_root=args.project_root, person_threshold=args.person_threshold)
        runtime.start(); engine = PersonApproachDecisionEngine()
        for index in range(1, args.max_decisions+1):
            decision = engine.ingest(runtime.cycle(f"approach-{index:06d}"))
            print(json.dumps(decision.to_dict(), allow_nan=False, sort_keys=True), flush=True)
            if args.cycle_interval_s and index < args.max_decisions: time.sleep(args.cycle_interval_s)
    except KeyboardInterrupt:
        return 0
    except (LiveFusionError, ValueError, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr); return 2
    finally:
        if runtime is not None: runtime.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
