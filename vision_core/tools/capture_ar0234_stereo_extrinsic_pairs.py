"""Interactive raw paired capture for AR0234-to-OV9281 extrinsic calibration."""
from __future__ import annotations

import argparse
from pathlib import Path

from vision_core.rgb_stereo_extrinsic.capture import (
    AR0234_BY_ID, DATASET_ROOT, STEREO_BY_ID, ExtrinsicCaptureError, capture_runtime,
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", type=Path, default=DATASET_ROOT)
    parser.add_argument("--pair-limit", type=int, default=12)
    parser.add_argument("--ar-device", type=Path, default=AR0234_BY_ID)
    parser.add_argument("--stereo-device", type=Path, default=STEREO_BY_ID)
    args = parser.parse_args()
    try:
        capture_runtime(root=args.output_root, pair_limit=args.pair_limit, ar_device=args.ar_device, stereo_device=args.stereo_device)
    except ExtrinsicCaptureError as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
