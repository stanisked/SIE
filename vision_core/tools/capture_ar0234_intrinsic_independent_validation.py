#!/usr/bin/env python3
"""Capture immutable AR0234 checkerboard evidence for independent intrinsic validation."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from vision_core.ar0234_intrinsic_independent_validation import IntrinsicValidationError, capture_independent_evidence


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--session-id", required=True)
    parser.add_argument("--intrinsic", type=Path, required=True)
    parser.add_argument("--operator-pose-label", required=True)
    parser.add_argument("--static-target-affirmed", action="store_true")
    parser.add_argument("--frame-count", type=int, default=12)
    arguments = parser.parse_args()
    try:
        capture_independent_evidence(
            output_root=arguments.output_root,
            session_id=arguments.session_id,
            intrinsic=arguments.intrinsic,
            operator_pose_label=arguments.operator_pose_label,
            static_target_affirmed=arguments.static_target_affirmed,
            frame_count=arguments.frame_count,
        )
    except IntrinsicValidationError as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
