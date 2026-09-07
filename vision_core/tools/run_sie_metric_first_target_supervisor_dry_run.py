#!/usr/bin/env python3
"""Run one metric-first five-cycle target window without action execution."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from vision_core.person_approach.metric_first_target_supervisor import MetricFirstTargetSupervisor  # noqa: E402
from vision_core.person_depth_fusion.live import LiveFusionError, build_live_runtime  # noqa: E402
from vision_core.tools.prepare_ar0234_yaw_observations import load_optical_axis_cx  # noqa: E402


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--project-root", type=Path, required=True)
    parser.add_argument("--ar-intrinsic", type=Path, required=True)
    parser.add_argument("--center-tolerance-px", type=float, required=True)
    parser.add_argument("--person-threshold", type=float, choices=(0.4, 0.5), default=0.5)
    return parser.parse_args(argv)


def main() -> int:
    args = parse_args()
    runtime = None
    try:
        runtime = build_live_runtime(model=args.model, reference=args.reference, project_root=args.project_root, person_threshold=args.person_threshold)
        runtime.start()
        runner = MetricFirstTargetSupervisor(
            live_runtime=runtime, optical_axis_cx_px=load_optical_axis_cx(args.ar_intrinsic),
            center_tolerance_px=args.center_tolerance_px,
        )
        print(json.dumps(runner.run_live_window(), allow_nan=False, sort_keys=True, separators=(",", ":")))
    except (LiveFusionError, ValueError, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    finally:
        if runtime is not None:
            runtime.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
