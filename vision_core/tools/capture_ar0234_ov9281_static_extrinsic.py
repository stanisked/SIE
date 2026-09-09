#!/usr/bin/env python3
"""Capture static-only AR0234 and OV9281 checkerboard evidence."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from vision_core.ar0234_ov9281_static_extrinsic import (
    AR_DEVICE, OV_CALIBRATION_DEFAULT, OV_DEVICE, StaticExtrinsicError,
    capture_static_pairs,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--ar-intrinsic", type=Path, required=True)
    parser.add_argument("--ov-calibration", type=Path, default=OV_CALIBRATION_DEFAULT)
    parser.add_argument("--ar-device", type=Path, default=AR_DEVICE)
    parser.add_argument("--ov-device", type=Path, default=OV_DEVICE)
    parser.add_argument("--pair-count", type=int, default=12)
    parser.add_argument("--static-target-affirmed", action="store_true")
    arguments = parser.parse_args()
    try:
        capture_static_pairs(output_root=arguments.output_root, ar_intrinsic=arguments.ar_intrinsic, ov_calibration=arguments.ov_calibration, pair_count=arguments.pair_count, static_target_affirmed=arguments.static_target_affirmed, ar_device=arguments.ar_device, ov_device=arguments.ov_device)
    except StaticExtrinsicError as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
