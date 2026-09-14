#!/usr/bin/env python3
"""Local, deterministic LabelMe-to-YOLO workflow for the frozen AR0234 package."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from vision_core.close_range_person_alignment_dataset.annotation import (
    convert_labelme_to_yolo,
    validate_annotation_package,
)
from vision_core.close_range_person_alignment_dataset.capture import DatasetCaptureError


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--annotations-dir", type=Path, required=True)
    parser.add_argument("--frozen-inventory", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--mode", choices=("dry-run", "apply", "validate"), required=True)
    parser.add_argument("--overwrite", action="store_true", help="allow replacement of existing YOLO labels only in apply mode")
    parser.add_argument("--require-complete", action="store_true", help="make incomplete validation report BLOCKED_INCOMPLETE_ANNOTATION")
    return parser


def _write_report(path: Path, report: dict[str, object]) -> None:
    if path.is_symlink() or path.exists():
        raise DatasetCaptureError("report path already exists; choose a new immutable report filename")
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n"
    path.write_text(encoded, encoding="utf-8")


def main() -> int:
    args = _parser().parse_args()
    try:
        if args.mode == "validate":
            report = validate_annotation_package(
                args.dataset_root,
                annotations_dir=args.annotations_dir,
                inventory_path=args.frozen_inventory,
                require_complete=args.require_complete,
            )
        else:
            if args.require_complete:
                raise DatasetCaptureError("--require-complete is valid only with --mode validate")
            report = convert_labelme_to_yolo(
                args.dataset_root,
                annotations_dir=args.annotations_dir,
                inventory_path=args.frozen_inventory,
                dry_run=args.mode == "dry-run",
                overwrite=args.overwrite,
            )
        _write_report(args.report, report)
    except DatasetCaptureError as error:
        raise SystemExit(f"ERROR: {error}") from error
    print(json.dumps(report, ensure_ascii=False, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
