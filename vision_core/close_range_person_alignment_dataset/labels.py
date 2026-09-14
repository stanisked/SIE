"""Offline YOLO-label validation and session-isolated split generation."""

from __future__ import annotations

import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Any

from .capture import CLASS_NAME, DATASET_ID, DatasetCaptureError, _paths, _validate_output_root


def validate_yolo_labels(root: Path) -> dict[str, Any]:
    _validate_output_root(root)
    paths = _paths(root)
    _require_classes(paths["classes"])
    images = sorted(paths["images"].glob("*.png"))
    if not images:
        raise DatasetCaptureError("images/ contains no PNG files")
    validated: list[str] = []
    for image in images:
        label = paths["labels"] / f"{image.stem}.txt"
        if label.is_symlink() or not label.is_file():
            raise DatasetCaptureError(f"missing LabelImg YOLO label: {label.name}")
        _validate_label_file(label)
        validated.append(image.stem)
    known_stems = set(validated)
    extra = sorted(path.name for path in paths["labels"].glob("*.txt") if path.stem not in known_stems)
    if extra:
        raise DatasetCaptureError(f"labels without matching images: {extra}")
    result = {
        "schema_version": "sie.ar0234_close_range_person_alignment_label_validation.v1",
        "dataset_id": DATASET_ID,
        "status": "VALID",
        "validated_image_count": len(validated),
        "class_name": CLASS_NAME,
    }
    json.dumps(result, allow_nan=False)
    return result


def create_session_splits(root: Path) -> dict[str, Any]:
    """Create immutable train/val/test lists only after every image is annotated."""
    validation = validate_yolo_labels(root)
    paths = _paths(root)
    records = _load_capture_records(paths["capture_records"])
    image_names = {path.name for path in paths["images"].glob("*.png")}
    if set(records) != image_names:
        raise DatasetCaptureError("capture records and images/ must match before split generation")
    by_session: dict[str, list[str]] = defaultdict(list)
    for filename, record in records.items():
        by_session[record["session_id"]].append(filename)
    sessions = sorted(by_session)
    if len(sessions) < 3:
        raise DatasetCaptureError("at least three capture sessions are required for session-isolated splits")
    assignment = {session: "train" for session in sessions[:-2]}
    assignment[sessions[-2]] = "val"
    assignment[sessions[-1]] = "test"
    result: dict[str, Any] = {
        "schema_version": "sie.ar0234_close_range_person_alignment_splits.v1",
        "dataset_id": DATASET_ID,
        "label_validation": validation,
        "session_assignment": assignment,
        "splits": {},
    }
    for split in ("train", "val", "test"):
        target = paths["splits"] / f"{split}.txt"
        if target.exists() or target.is_symlink():
            raise DatasetCaptureError(f"refusing overwrite: {target}")
        members = sorted(
            f"images/{filename}"
            for session, files in by_session.items()
            if assignment[session] == split
            for filename in files
        )
        target.write_text("\n".join(members) + "\n", encoding="utf-8")
        result["splits"][split] = {
            "session_ids": sorted(session for session, assigned in assignment.items() if assigned == split),
            "image_count": len(members),
        }
    report = paths["reports"] / "split_report.json"
    if report.exists() or report.is_symlink():
        raise DatasetCaptureError(f"refusing overwrite: {report}")
    report.write_text(json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
    return result


def _require_classes(path: Path) -> None:
    if path.is_symlink() or not path.is_file() or path.read_text(encoding="utf-8") != f"{CLASS_NAME}\n":
        raise DatasetCaptureError("classes.txt must contain exactly person_upper_body")


def _validate_label_file(path: Path) -> None:
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        fields = line.split()
        if len(fields) != 5 or fields[0] != "0":
            raise DatasetCaptureError(f"{path.name}:{number} must be class_id 0 plus four normalized values")
        try:
            center_x, center_y, width, height = (float(value) for value in fields[1:])
        except ValueError as error:
            raise DatasetCaptureError(f"{path.name}:{number} contains a non-numeric YOLO value") from error
        values = (center_x, center_y, width, height)
        if any(not math.isfinite(value) or value <= 0.0 or value > 1.0 for value in values):
            raise DatasetCaptureError(f"{path.name}:{number} values must be finite and in (0, 1]")
        if center_x - width / 2.0 < 0.0 or center_x + width / 2.0 > 1.0 or center_y - height / 2.0 < 0.0 or center_y + height / 2.0 > 1.0:
            raise DatasetCaptureError(f"{path.name}:{number} bbox extends outside the image")


def _load_capture_records(path: Path) -> dict[str, dict[str, str]]:
    if path.is_symlink() or not path.is_file():
        raise DatasetCaptureError("capture_records.jsonl is required before split generation")
    output: dict[str, dict[str, str]] = {}
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        try:
            record = json.loads(line)
        except json.JSONDecodeError as error:
            raise DatasetCaptureError(f"invalid capture record line {number}") from error
        expected = {"schema_version", "dataset_id", "session_id", "image_filename", "image_sha256", "png_bytes", "captured_at_utc"}
        if not isinstance(record, dict) or set(record) != expected:
            raise DatasetCaptureError(f"capture record line {number} schema mismatch")
        if record["dataset_id"] != DATASET_ID or not isinstance(record["image_filename"], str) or not isinstance(record["session_id"], str):
            raise DatasetCaptureError(f"capture record line {number} identity mismatch")
        if record["image_filename"] in output:
            raise DatasetCaptureError("duplicate image filename in capture records")
        output[record["image_filename"]] = {"session_id": record["session_id"]}
    return output
