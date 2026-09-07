#!/usr/bin/env python3
"""Coordinate local supervised yaw and range-acquisition records without execution."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from vision_core.person_approach.supervised_acquire_range import coordinate_supervised_acquire_range  # noqa: E402


def _read(path: Path) -> object:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("temporal_alignment", type=Path, help="local temporal yaw result JSON")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--person-depth-record", type=Path, help="local person-depth live/offline JSON")
    group.add_argument("--person-approach-decision", type=Path, help="local existing decision JSON")
    args = parser.parse_args()
    try:
        envelope: dict[str, object] = {"temporal_alignment": _read(args.temporal_alignment)}
        if args.person_depth_record is not None:
            envelope["person_depth"] = _read(args.person_depth_record)
        if args.person_approach_decision is not None:
            envelope["person_approach_decision"] = _read(args.person_approach_decision)
    except (OSError, json.JSONDecodeError):
        envelope = {}
    print(json.dumps(coordinate_supervised_acquire_range(envelope), allow_nan=False, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
