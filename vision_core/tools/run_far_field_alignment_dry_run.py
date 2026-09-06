#!/usr/bin/env python3
"""Evaluate one local JSON envelope for offline far-field image alignment."""
from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from vision_core.person_approach.far_field_alignment import evaluate_far_field_alignment  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="local JSON envelope")
    args = parser.parse_args()
    try:
        with args.input.open("r", encoding="utf-8") as stream:
            envelope = json.load(stream)
        result = evaluate_far_field_alignment(envelope)
    except (OSError, json.JSONDecodeError) as error:
        result = evaluate_far_field_alignment({"center_tolerance_px": None}, now_utc=lambda: datetime.now(timezone.utc))
        result["semantic_decision"] = "BLOCKED_INVALID_INPUT"
        result["block_reason"] = f"INPUT_READ_FAILED_{type(error).__name__}"
    print(json.dumps(result, allow_nan=False, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
