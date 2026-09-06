#!/usr/bin/env python3
"""Explicitly launched supervised SIE demo; it never performs a motor command."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from vision_core.person_approach.supervised_demo import (  # noqa: E402
    AWAITING_OPERATOR_CONFIRMATION,
    SupervisedPersonApproachDemo,
)
from vision_core.person_depth_fusion.live import LiveFusionError, build_live_runtime  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--boot-session-id", required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--project-root", type=Path, required=True)
    parser.add_argument("--person-threshold", type=float, choices=(.4, .5), required=True)
    parser.add_argument("--max-cycles", type=int, default=5)
    args = parser.parse_args()
    if args.max_cycles < 5:
        parser.error("--max-cycles must be at least 5")
    runtime = None
    try:
        runtime = build_live_runtime(
            model=args.model,
            reference=args.reference,
            project_root=args.project_root,
            person_threshold=args.person_threshold,
        )
        runtime.start()
        demo = SupervisedPersonApproachDemo(boot_session_id=args.boot_session_id)
        record = None
        for index in range(1, args.max_cycles + 1):
            record = demo.process_cycle(runtime.cycle(f"supervised-demo-{index:06d}"))
            if record["stage"] in {AWAITING_OPERATOR_CONFIRMATION, "NO_ACTION"} and index >= 5:
                break
        assert record is not None
        print(json.dumps(record, allow_nan=False, sort_keys=True, separators=(",", ":")))
        return 0
    except (LiveFusionError, ValueError, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    finally:
        if runtime is not None:
            runtime.close()


if __name__ == "__main__":
    raise SystemExit(main())
