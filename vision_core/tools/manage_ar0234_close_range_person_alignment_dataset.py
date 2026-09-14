#!/usr/bin/env python3
"""Validate LabelImg YOLO labels and create session-isolated dataset splits."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from vision_core.close_range_person_alignment_dataset.capture import DatasetCaptureError
from vision_core.close_range_person_alignment_dataset.labels import create_session_splits, inspect_labelimg_layout, validate_yolo_labels


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    commands = parser.add_mutually_exclusive_group(required=True)
    commands.add_argument("--inspect-labelimg-layout", action="store_true")
    commands.add_argument("--validate-labels", action="store_true")
    commands.add_argument("--create-session-splits", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.create_session_splits:
        result = create_session_splits(args.output_root)
    elif args.validate_labels:
        result = validate_yolo_labels(args.output_root)
    else:
        result = inspect_labelimg_layout(args.output_root)
    print(json.dumps(result, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (DatasetCaptureError, OSError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(2) from error
