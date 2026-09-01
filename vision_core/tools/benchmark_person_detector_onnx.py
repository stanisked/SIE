#!/usr/bin/env python3
"""Run an externally stored, explicitly approved ONNX CPU benchmark.

The production approval and adapter registries are intentionally empty in this
scaffold. Therefore a valid manifest exits before an ONNX loader can run.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from vision_core.person_detector_benchmark.benchmark import (
    BenchmarkBlockedError,
    ReportPublishedDurabilityUncertainError,
    canonical_repository_root,
    discover_worktree_roots,
    run_offline_benchmark,
    write_report_atomic,
)


def _absolute_path(value: str) -> Path:
    path = Path(value)
    if not path.is_absolute():
        raise argparse.ArgumentTypeError("path must be absolute")
    return path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=_absolute_path, required=True, help="external ONNX model path")
    parser.add_argument("--manifest", type=_absolute_path, required=True, help="external strict JSON manifest")
    parser.add_argument("--images", type=_absolute_path, required=True, help="external image directory")
    parser.add_argument("--warmup-count", type=int, default=5)
    parser.add_argument("--opencv-threads", type=int, default=1)
    parser.add_argument("--report", type=_absolute_path, required=True, help="external JSON report path")
    parser.add_argument("--overwrite", action="store_true", help="atomically replace an existing external report")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = canonical_repository_root()
    roots = discover_worktree_roots(root)
    report = run_offline_benchmark(
        artifact_path=args.model,
        manifest_path=args.manifest,
        image_directory=args.images,
        warmup_count=args.warmup_count,
        opencv_threads=args.opencv_threads,
        canonical_root_provider=lambda: root,
        root_discovery=lambda _root: roots,
    )
    write_report_atomic(args.report, report, worktree_roots=roots, overwrite=args.overwrite)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ReportPublishedDurabilityUncertainError as error:
        print(
            "REPORT_PUBLISHED_DURABILITY_UNCERTAIN: "
            f"target={error.target_path}; published={error.published}; "
            f"durability_confirmed={error.durability_confirmed}; overwrite={error.overwrite}; "
            "the target is visible, but crash durability was not confirmed",
            file=sys.stderr,
        )
        raise SystemExit(3) from error
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(2) from error
