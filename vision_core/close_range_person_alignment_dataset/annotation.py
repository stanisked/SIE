"""Deterministic, local LabelMe-to-YOLO conversion for the frozen capture package."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any

import cv2

from .capture import DATASET_ID, DatasetCaptureError, _paths, _validate_output_root
from .labels import _load_capture_records, _load_session_metadata


ANNOTATION_CLASS = "person_upper_body"
ANNOTATION_SCHEMA_VERSION = "sie.ar0234_close_range_person_alignment_annotation.v1"
EXPECTED_WIDTH = 1920
EXPECTED_HEIGHT = 1200


def convert_labelme_to_yolo(
    root: Path,
    *,
    annotations_dir: Path,
    inventory_path: Path,
    dry_run: bool,
    overwrite: bool,
) -> dict[str, Any]:
    """Convert only complete single-person LabelMe rectangles into YOLO labels."""
    context = _load_context(root, annotations_dir, inventory_path)
    converted: list[dict[str, str]] = []
    failures: list[dict[str, str]] = [
        {"image_filename": path.name, "reason": "labels directory may contain only YOLO .txt files"}
        for path in context["unexpected_label_files"]
    ]
    negatives_without_json = 0
    for filename, record in context["records"].items():
        stem = Path(filename).stem
        annotation = annotations_dir / f"{stem}.json"
        positive = context["contains_person"][record["session_id"]]
        try:
            if not annotation.exists():
                if positive:
                    raise DatasetCaptureError("positive image has no LabelMe JSON")
                negatives_without_json += 1
                continue
            result = _parse_annotation(annotation, filename=filename, positive=positive)
            if result is None:
                continue
            yolo = _to_yolo(result)
            target = context["paths"]["labels"] / f"{stem}.txt"
            if target.exists() or target.is_symlink():
                if not overwrite:
                    raise DatasetCaptureError("existing YOLO label requires --overwrite")
            converted.append({"image_filename": filename, "label_filename": target.name, "yolo": yolo})
        except (DatasetCaptureError, ValueError, json.JSONDecodeError) as error:
            failures.append({"image_filename": filename, "reason": str(error)})
    status = "READY_TO_WRITE" if not failures else "BLOCKED_ANNOTATION_FAILURES"
    if not dry_run and not failures:
        for item in converted:
            target = context["paths"]["labels"] / item["label_filename"]
            target.write_text(item["yolo"] + "\n", encoding="utf-8")
        status = "CONVERTED"
    report = {
        "schema_version": ANNOTATION_SCHEMA_VERSION,
        "dataset_id": DATASET_ID,
        "source_snapshot_id": context["snapshot_id"],
        "dry_run": dry_run,
        "overwrite": overwrite,
        "status": status,
        "image_count": len(context["records"]),
        "converted_positive_count": len(converted),
        "negative_images_without_json": negatives_without_json,
        "failure_count": len(failures),
        "failures": failures,
        "converted": converted,
    }
    _json_safe(report)
    return report


def validate_annotation_package(
    root: Path,
    *,
    annotations_dir: Path,
    inventory_path: Path,
    require_complete: bool,
) -> dict[str, Any]:
    """Check LabelMe/YOLO/image stems without making split files or edits."""
    context = _load_context(root, annotations_dir, inventory_path)
    pending_positive: list[str] = []
    failures: list[dict[str, str]] = []
    negative_with_bbox: list[str] = []
    yolo_missing: list[str] = []
    for filename, record in context["records"].items():
        stem = Path(filename).stem
        annotation = annotations_dir / f"{stem}.json"
        yolo = context["paths"]["labels"] / f"{stem}.txt"
        positive = context["contains_person"][record["session_id"]]
        try:
            result = None if not annotation.exists() else _parse_annotation(annotation, filename=filename, positive=positive)
            if positive and result is None:
                pending_positive.append(filename)
                continue
            if not positive and result is not None:
                negative_with_bbox.append(filename)
                continue
            if result is not None:
                expected = _to_yolo(result)
                if not yolo.is_file() or yolo.is_symlink():
                    yolo_missing.append(filename)
                elif yolo.read_text(encoding="utf-8").strip() != expected:
                    failures.append({"image_filename": filename, "reason": "YOLO label differs from LabelMe rectangle"})
            elif yolo.exists():
                if yolo.is_symlink() or yolo.read_text(encoding="utf-8").strip():
                    failures.append({"image_filename": filename, "reason": "negative image must not have a YOLO bbox"})
        except (DatasetCaptureError, ValueError, json.JSONDecodeError) as error:
            failures.append({"image_filename": filename, "reason": str(error)})
    extra_json = sorted(path.name for path in annotations_dir.glob("*.json") if f"{path.stem}.png" not in context["records"])
    extra_yolo = sorted(path.name for path in context["paths"]["labels"].glob("*.txt") if f"{path.stem}.png" not in context["records"])
    unexpected_label_files = [path.name for path in context["unexpected_label_files"]]
    complete = not (pending_positive or failures or negative_with_bbox or yolo_missing or extra_json or extra_yolo or unexpected_label_files)
    status = "VALID" if complete else "PENDING_ANNOTATION"
    if require_complete and not complete:
        status = "BLOCKED_INCOMPLETE_ANNOTATION"
    report = {
        "schema_version": ANNOTATION_SCHEMA_VERSION,
        "dataset_id": DATASET_ID,
        "source_snapshot_id": context["snapshot_id"],
        "status": status,
        "split_created": False,
        "positive_pending_annotation": len(pending_positive),
        "negative_with_bbox": negative_with_bbox,
        "yolo_missing": yolo_missing,
        "failures": failures,
        "extra_annotation_json": extra_json,
        "extra_yolo_labels": extra_yolo,
        "unexpected_label_files": unexpected_label_files,
    }
    _json_safe(report)
    return report


def _load_context(root: Path, annotations_dir: Path, inventory_path: Path) -> dict[str, Any]:
    _validate_output_root(root)
    paths = _paths(root)
    if annotations_dir != root / "annotations_labelme" or annotations_dir.is_symlink() or not annotations_dir.is_dir():
        raise DatasetCaptureError("annotations_dir must be the real dataset annotations_labelme/ directory")
    if inventory_path.is_symlink() or not inventory_path.is_file():
        raise DatasetCaptureError("frozen inventory JSON is required")
    inventory = _strict_json(inventory_path)
    if inventory.get("dataset_id") != DATASET_ID or not isinstance(inventory.get("snapshot_id"), str):
        raise DatasetCaptureError("frozen inventory identity is invalid")
    expected_raw = {entry.get("path"): entry.get("sha256") for entry in inventory.get("raw_images", []) if isinstance(entry, dict)}
    actual_raw = {f"raw/{path.name}": hashlib.sha256(path.read_bytes()).hexdigest() for path in paths["raw"].glob("*.png") if path.is_file() and not path.is_symlink()}
    if expected_raw != actual_raw:
        raise DatasetCaptureError("raw images do not match the frozen inventory snapshot")
    records = _load_capture_records(paths["capture_records"])
    image_paths = {path.name: path for path in paths["images"].glob("*.png") if path.is_file() and not path.is_symlink()}
    image_names = set(image_paths)
    if set(records) != image_names:
        raise DatasetCaptureError("images/ and capture records must match")
    for image_path in image_paths.values():
        image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
        if image is None or image.shape[:2] != (EXPECTED_HEIGHT, EXPECTED_WIDTH):
            raise DatasetCaptureError("source image must decode as 1920x1200")
    contains_person = _load_session_metadata(paths["manifests"], {record["session_id"] for record in records.values()})
    unexpected_label_files = sorted(
        path for path in paths["labels"].iterdir() if path.is_symlink() or not path.is_file() or path.suffix != ".txt"
    )
    return {
        "paths": paths,
        "records": records,
        "contains_person": contains_person,
        "snapshot_id": inventory["snapshot_id"],
        "unexpected_label_files": unexpected_label_files,
    }


def _parse_annotation(path: Path, *, filename: str, positive: bool) -> tuple[float, float, float, float] | None:
    if path.is_symlink() or not path.is_file():
        raise DatasetCaptureError("annotation JSON is unsafe or missing")
    payload = _strict_json(path)
    if payload.get("imagePath") != filename:
        raise DatasetCaptureError("imagePath does not match the source image filename")
    if payload.get("imageWidth") != EXPECTED_WIDTH or payload.get("imageHeight") != EXPECTED_HEIGHT:
        raise DatasetCaptureError("LabelMe image dimensions must be 1920x1200")
    if payload.get("imageData") is not None:
        raise DatasetCaptureError("LabelMe imageData embedding is forbidden")
    shapes = payload.get("shapes")
    if not isinstance(shapes, list):
        raise DatasetCaptureError("LabelMe shapes must be a list")
    if not positive:
        if shapes:
            raise DatasetCaptureError("negative image must have no shapes")
        return None
    if len(shapes) != 1:
        raise DatasetCaptureError("positive image requires exactly one rectangle")
    shape = shapes[0]
    if not isinstance(shape, dict) or shape.get("label") != ANNOTATION_CLASS or shape.get("shape_type") != "rectangle":
        raise DatasetCaptureError("only rectangle person_upper_body is permitted")
    points = shape.get("points")
    if not isinstance(points, list) or len(points) != 2:
        raise DatasetCaptureError("rectangle must have exactly two points")
    try:
        (x1, y1), (x2, y2) = ((float(point[0]), float(point[1])) for point in points)
    except (TypeError, ValueError, IndexError) as error:
        raise DatasetCaptureError("rectangle points are invalid") from error
    values = (x1, y1, x2, y2)
    if any(not math.isfinite(value) for value in values) or not (0.0 <= x1 < x2 <= EXPECTED_WIDTH and 0.0 <= y1 < y2 <= EXPECTED_HEIGHT):
        raise DatasetCaptureError("rectangle must be finite, non-empty and inside image bounds")
    return x1, y1, x2, y2


def _to_yolo(rectangle: tuple[float, float, float, float]) -> str:
    x1, y1, x2, y2 = rectangle
    width, height = x2 - x1, y2 - y1
    return f"0 {(x1 + x2) / 2.0 / EXPECTED_WIDTH:.10f} {(y1 + y2) / 2.0 / EXPECTED_HEIGHT:.10f} {width / EXPECTED_WIDTH:.10f} {height / EXPECTED_HEIGHT:.10f}"


def _strict_json(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"), parse_constant=lambda value: (_ for _ in ()).throw(ValueError(f"non-finite JSON value: {value}")))
    except (OSError, json.JSONDecodeError) as error:
        raise DatasetCaptureError(f"invalid JSON: {path}") from error
    if not isinstance(payload, dict):
        raise DatasetCaptureError("JSON root must be an object")
    return payload


def _json_safe(value: object) -> None:
    try:
        json.dumps(value, allow_nan=False)
    except (TypeError, ValueError) as error:
        raise DatasetCaptureError("annotation report must be JSON-safe") from error
