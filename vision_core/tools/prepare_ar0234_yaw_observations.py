#!/usr/bin/env python3
"""Adapt local AR0234 person-localization JSONL to temporal yaw JSONL evidence."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from datetime import datetime
from pathlib import Path
from typing import Any


IMAGE_FRAME = "ar0234_image_frame"
IMAGE_UNITS = "px"


def _finite(value: object) -> float | None:
    if type(value) not in (int, float) or not math.isfinite(value):
        return None
    return float(value)


def _text(value: object) -> str | None:
    return value if type(value) is str and value else None


def _timezone_aware_timestamp(value: object) -> str | None:
    if not isinstance(value, str) or not value:
        return None
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None
    return value if parsed.tzinfo is not None else None


def load_optical_axis_cx(path: Path) -> float:
    """Read the AR0234 principal point from intrinsic K, never from a constant."""
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
        matrix = document.get("K", document.get("camera_matrix"))
    except (OSError, json.JSONDecodeError, AttributeError) as error:
        raise ValueError(f"invalid AR intrinsic JSON: {error}") from error
    if type(matrix) is not list or len(matrix) != 3 or any(type(row) is not list or len(row) != 3 for row in matrix):
        raise ValueError("AR intrinsic K must be a 3x3 array")
    values = [[_finite(item) for item in row] for row in matrix]
    if any(item is None for row in values for item in row):
        raise ValueError("AR intrinsic K must be finite")
    cx = values[0][2]
    assert cx is not None
    return cx


def _evidence_id(raw: dict[str, Any], line_number: int, *, prefix: str) -> str:
    provided = _text(raw.get("evidence_id"))
    if provided is not None:
        return provided
    canonical = json.dumps(raw, allow_nan=False, sort_keys=True, separators=(",", ":"))
    digest = hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:16]
    return f"{prefix}-{line_number:06d}-{digest}"


def prepare_observation(
    raw: object,
    *,
    line_number: int,
    optical_axis_cx_px: float,
    center_tolerance_px: float,
) -> dict[str, Any]:
    """Create one planner-compatible observation; invalid perception remains offset-free."""
    if type(raw) is not dict:
        raise ValueError("each raw JSONL record must be an object")
    if _finite(optical_axis_cx_px) is None or _finite(center_tolerance_px) is None or center_tolerance_px < 0:
        raise ValueError("optical axis and center tolerance must be finite; tolerance non-negative")
    person_status = _text(raw.get("status")) or "INVALID_PERSON_EVIDENCE"
    observation: dict[str, Any] = {
        "evidence_id": _evidence_id(raw, line_number, prefix="ar0234-localization"),
        "timestamp": _timezone_aware_timestamp(raw.get("timestamp")),
        "reference_frame": IMAGE_FRAME,
        "units": IMAGE_UNITS,
        "person_status": person_status,
        "center_tolerance_px": float(center_tolerance_px),
    }
    bbox = raw.get("bbox")
    if person_status != "SINGLE_PERSON" or type(bbox) is not list or len(bbox) != 4:
        return observation
    x1, y1, x2, y2 = (_finite(value) for value in bbox)
    if None in (x1, y1, x2, y2) or x2 <= x1 or y2 <= y1:
        return observation
    center_x = (x1 + x2) / 2.0
    observation["image_offset_px"] = center_x - optical_axis_cx_px
    return observation


def _live_cycle_evidence_id(cycle: dict[str, Any], line_number: int) -> str:
    measurement = cycle.get("measurement")
    if type(measurement) is dict:
        for key in ("person_evidence_id", "person_observation_id", "measurement_id"):
            identifier = _text(measurement.get(key))
            if identifier is not None:
                return identifier
    return _evidence_id(cycle, line_number, prefix="person-depth-live-cycle")


def prepare_person_depth_live_cycle_observation(
    cycle: object,
    *,
    line_number: int,
    optical_axis_cx_px: float,
    center_tolerance_px: float,
) -> dict[str, Any]:
    """Create temporal evidence from one existing person-depth live-cycle record."""
    if type(cycle) is not dict:
        raise ValueError("each live-cycle JSONL record must be an object")
    if _finite(optical_axis_cx_px) is None or _finite(center_tolerance_px) is None or center_tolerance_px < 0:
        raise ValueError("optical axis and center tolerance must be finite; tolerance non-negative")
    person = cycle.get("person") if type(cycle.get("person")) is dict else {}
    person_status = _text(person.get("status")) or "INVALID_PERSON_EVIDENCE"
    if cycle.get("schema_version") != "sie.person_depth_live_cycle.v1":
        person_status = "INVALID_PERSON_EVIDENCE"
    if person_status == "SINGLE_PERSON" and cycle.get("status") != "SUCCESS":
        person_status = "INVALID_PERSON_EVIDENCE"
    observation: dict[str, Any] = {
        "evidence_id": _live_cycle_evidence_id(cycle, line_number),
        "timestamp": _timezone_aware_timestamp(cycle.get("captured_at_utc")),
        "reference_frame": IMAGE_FRAME,
        "units": IMAGE_UNITS,
        "person_status": person_status,
        "center_tolerance_px": float(center_tolerance_px),
    }
    bbox = person.get("bbox_xyxy_px")
    if person_status != "SINGLE_PERSON" or type(bbox) is not list or len(bbox) != 4:
        return observation
    x1, y1, x2, y2 = (_finite(value) for value in bbox)
    if None in (x1, y1, x2, y2) or x2 <= x1 or y2 <= y1:
        return observation
    observation["image_offset_px"] = (x1 + x2) / 2.0 - optical_axis_cx_px
    return observation


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw_input", type=Path, help="local source JSONL")
    parser.add_argument("output", type=Path, help="new planner-compatible JSONL path")
    parser.add_argument("--ar-intrinsic", type=Path, required=True)
    parser.add_argument("--center-tolerance-px", type=float, required=True)
    parser.add_argument(
        "--source-kind",
        choices=("ar0234-localization", "person-depth-live-cycle"),
        default="ar0234-localization",
        help="explicit source record contract; default preserves raw AR0234 localization input",
    )
    parser.add_argument("--overwrite", action="store_true", help="explicitly replace an existing output file")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.raw_input.resolve() == args.output.resolve():
        raise ValueError("output path must differ from raw input")
    cx = load_optical_axis_cx(args.ar_intrinsic)
    tolerance = _finite(args.center_tolerance_px)
    if tolerance is None or tolerance < 0:
        raise ValueError("--center-tolerance-px must be finite and non-negative")
    try:
        with args.raw_input.open("r", encoding="utf-8") as stream:
            raw_records = [json.loads(line) for line in stream if line.strip()]
        prepare = prepare_observation if args.source_kind == "ar0234-localization" else prepare_person_depth_live_cycle_observation
        observations = [prepare(raw, line_number=index, optical_axis_cx_px=cx, center_tolerance_px=tolerance) for index, raw in enumerate(raw_records, start=1)]
        output_mode = "w" if args.overwrite else "x"
        with args.output.open(output_mode, encoding="utf-8", newline="\n") as stream:
            for observation in observations:
                stream.write(json.dumps(observation, allow_nan=False, sort_keys=True, separators=(",", ":")) + "\n")
    except (OSError, json.JSONDecodeError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
