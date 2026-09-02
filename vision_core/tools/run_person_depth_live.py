"""Headless live AR0234 + OV9281 person-depth fusion; no actuation."""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from vision_core.person_depth_fusion.live import LiveFusionError, build_live_runtime


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--project-root", type=Path, required=True)
    parser.add_argument("--max-cycles", type=int, required=True)
    parser.add_argument("--cycle-interval-s", type=float, default=0.0)
    parser.add_argument("--person-threshold", type=float, choices=(0.4, 0.5), default=0.5)
    args = parser.parse_args(argv)
    if args.max_cycles <= 0 or args.cycle_interval_s < 0:
        parser.error("--max-cycles must be positive and --cycle-interval-s non-negative")
    return args


def main() -> int:
    args = parse_args()
    runtime = None
    try:
        runtime = build_live_runtime(model=args.model, reference=args.reference, project_root=args.project_root, person_threshold=args.person_threshold)
        runtime.start()
        for number in range(1, args.max_cycles + 1):
            print(json.dumps(runtime.cycle(f"live-{number:06d}"), allow_nan=False, sort_keys=True), flush=True)
            if args.cycle_interval_s and number < args.max_cycles:
                time.sleep(args.cycle_interval_s)
    except KeyboardInterrupt:
        return 0
    except (LiveFusionError, ValueError, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    finally:
        if runtime is not None:
            runtime.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
