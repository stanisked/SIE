"""Capture one AR0234 + OV9281 combined raw pair for offline fusion."""
from __future__ import annotations

import argparse
from pathlib import Path

from vision_core.person_depth_fusion.capture_pair import (
    DEFAULT_OUTPUT_ROOT, PersonDepthPairCaptureError, capture_pair,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--countdown-s", type=float, default=10.0)
    parser.add_argument("--preview", action="store_true", help="show AR0234 and physical-left optical axes")
    parser.add_argument("--ar-intrinsic", type=Path, required=False, help="AR0234 intrinsic calibration JSON (required with --preview)")
    parser.add_argument("--stereo-calibration", type=Path, required=False, help="Stereo V6 NPZ calibration (required with --preview)")
    args = parser.parse_args()
    if args.preview and (args.ar_intrinsic is None or args.stereo_calibration is None):
        parser.error("--preview requires --ar-intrinsic and --stereo-calibration")
    try:
        record = capture_pair(output_root=args.output_root, countdown_s=args.countdown_s, preview=args.preview,
                              ar_intrinsic=args.ar_intrinsic or Path(), stereo_calibration=args.stereo_calibration or Path())
    except PersonDepthPairCaptureError as error:
        print(error.status, error.detail)
        return 2
    if not args.preview:
        print(record["status"], args.output_root)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
