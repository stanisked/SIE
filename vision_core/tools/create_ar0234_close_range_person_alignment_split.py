#!/usr/bin/env python3
"""Create or validate a deterministic session-isolated AR0234 dataset split."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from vision_core.close_range_person_alignment_dataset.capture import DatasetCaptureError
from vision_core.close_range_person_alignment_dataset.splits import create_session_split, validate_session_split


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--frozen-inventory", type=Path, required=True)
    parser.add_argument("--expected-source-snapshot-id", required=True)
    parser.add_argument("--seed", type=int, default=20260915)
    parser.add_argument("--mode", choices=("create", "validate"), required=True)
    args = parser.parse_args()
    try:
        if args.mode == "create":
            report = create_session_split(
                args.dataset_root,
                inventory_path=args.frozen_inventory,
                expected_source_snapshot_id=args.expected_source_snapshot_id,
                seed=args.seed,
            )
        else:
            report = validate_session_split(
                args.dataset_root,
                inventory_path=args.frozen_inventory,
                manifest_path=args.dataset_root / "splits" / "SPLIT_MANIFEST_v1.json",
            )
    except DatasetCaptureError as error:
        raise SystemExit(f"ERROR: {error}") from error
    print(json.dumps(report, ensure_ascii=False, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
