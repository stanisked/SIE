#!/usr/bin/env python3
"""Run an offline forensic review of AR0234 to OV9281 extrinsic geometry."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from vision_core.ar0234_ov9281_extrinsic_forensics import (
    forensic_geometry_review,
    write_forensic_outputs,
)
from vision_core.ar0234_ov9281_static_extrinsic import OV_CALIBRATION_DEFAULT, StaticExtrinsicError


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-manifest", type=Path, required=True)
    parser.add_argument("--ar-intrinsic", type=Path, required=True)
    parser.add_argument("--ov-calibration", type=Path, default=OV_CALIBRATION_DEFAULT)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    arguments = parser.parse_args()
    try:
        report = forensic_geometry_review(
            capture_manifest=arguments.capture_manifest,
            ar_intrinsic=arguments.ar_intrinsic,
            ov_calibration=arguments.ov_calibration,
            candidate=arguments.candidate,
        )
        write_forensic_outputs(output_dir=arguments.output_dir, report=report)
    except StaticExtrinsicError as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
