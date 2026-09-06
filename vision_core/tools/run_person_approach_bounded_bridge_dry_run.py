#!/usr/bin/env python3
"""Read one local bridge envelope and print one offline-only JSON result."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from vision_core.person_approach.bounded_bridge import plan_bounded_command


def _read_input(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def main() -> int:
    parser = argparse.ArgumentParser(description="Plan one bounded command without network or actuation")
    parser.add_argument("input", type=Path, help="local JSON envelope")
    args = parser.parse_args()
    try:
        envelope = _read_input(args.input)
        result = plan_bounded_command(envelope).to_dict()
    except (OSError, json.JSONDecodeError) as error:
        result = {
            "schema_version": "sie.person_approach_bounded_bridge.v1",
            "result": "BLOCKED_NO_COMMAND",
            "decision_id": None,
            "method": None,
            "endpoint": None,
            "query": None,
            "command_id": None,
            "evidence_cycle_ids": [],
            "evidence_measurement_ids": [],
            "reference_frame": None,
            "units": None,
            "network_performed": False,
            "reobserve_required": False,
            "block_reason": f"INPUT_READ_FAILED: {type(error).__name__}",
        }
    print(json.dumps(result, allow_nan=False, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
