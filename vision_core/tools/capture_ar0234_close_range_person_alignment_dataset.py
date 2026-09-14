#!/usr/bin/env python3
"""Capture lossless AR0234 frames for manual person_upper_body annotation."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from vision_core.close_range_person_alignment_dataset.capture import (
    AR0234_BY_ID,
    DatasetCaptureError,
    SessionTags,
    capture_interactively,
    create_session_metadata,
    validate_stable_device_path,
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--session-id", required=True)
    parser.add_argument("--ar-intrinsic", type=Path, required=True)
    parser.add_argument("--distance-band", required=True)
    parser.add_argument("--pose", required=True)
    parser.add_argument("--lateral-position", required=True)
    parser.add_argument("--lighting", required=True)
    parser.add_argument("--scene-type", required=True)
    parser.add_argument("--contains-person", choices=("true", "false"), required=True)
    parser.add_argument("--device", type=Path, default=AR0234_BY_ID)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    validate_stable_device_path(args.device)
    tags = SessionTags(args.distance_band, args.pose, args.lateral_position, args.lighting, args.scene_type, args.contains_person == "true")
    create_session_metadata(args.output_root, session_id=args.session_id, tags=tags, intrinsic_path=args.ar_intrinsic)
    print("Press SPACE or S to save a raw PNG. Press Q or Esc to exit.", flush=True)
    capture_interactively(args.output_root, session_id=args.session_id)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (DatasetCaptureError, OSError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(2) from error
