#!/usr/bin/env python3
"""Plan one local temporal AR0234 yaw-alignment dry-run from JSONL observations."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from vision_core.person_approach.temporal_yaw_alignment import evaluate_temporal_yaw_alignment  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="local JSONL observations, one object per line")
    args = parser.parse_args()
    try:
        with args.input.open("r", encoding="utf-8") as stream:
            observations = [json.loads(line) for line in stream if line.strip()]
    except (OSError, json.JSONDecodeError):
        observations = {"invalid": True}
    print(json.dumps(evaluate_temporal_yaw_alignment(observations), allow_nan=False, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
