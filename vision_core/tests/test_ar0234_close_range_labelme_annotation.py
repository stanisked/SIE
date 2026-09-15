from __future__ import annotations

import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from vision_core.close_range_person_alignment_dataset.annotation import (
    convert_labelme_to_yolo,
    validate_annotation_package,
)
from vision_core.close_range_person_alignment_dataset.capture import (
    SessionTags,
    create_session_metadata,
    save_raw_frame,
)


NOW = datetime(2026, 9, 14, tzinfo=timezone.utc)


def _intrinsic(tmp_path: Path) -> Path:
    path = tmp_path / "ar_intrinsic.json"
    path.write_text('{"K": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}\n', encoding="utf-8")
    return path


def _tags(contains_person: bool) -> SessionTags:
    if contains_person:
        return SessionTags("1.5-2.0m", "standing_front", "center", "daylight", "indoor", True)
    return SessionTags("not_applicable", "not_applicable", "varied", "daylight", "hard_negative", False)


def _prepare_frozen_package(tmp_path: Path) -> tuple[Path, Path, Path, dict[str, str]]:
    root = tmp_path / "ar0234_close_range_person_alignment_v1"
    intrinsic = _intrinsic(tmp_path)
    create_session_metadata(root, session_id="positive", tags=_tags(True), intrinsic_path=intrinsic, now=lambda: NOW)
    create_session_metadata(root, session_id="negative", tags=_tags(False), intrinsic_path=intrinsic, now=lambda: NOW)
    frame = np.zeros((1200, 1920, 3), dtype=np.uint8)
    positive = save_raw_frame(root, session_id="positive", frame_bgr=frame, captured_at_utc=NOW)["image_filename"]
    negative = save_raw_frame(root, session_id="negative", frame_bgr=frame, captured_at_utc=NOW)["image_filename"]
    raw_images = []
    for image in sorted((root / "raw").glob("*.png")):
        raw_images.append({"path": f"raw/{image.name}", "sha256": hashlib.sha256(image.read_bytes()).hexdigest()})
    inventory = root / "frozen_inventory.json"
    inventory.write_text(json.dumps({"dataset_id": root.name, "snapshot_id": "frozen-test-001", "raw_images": raw_images}), encoding="utf-8")
    annotations = root / "annotations_labelme"
    annotations.mkdir()
    return root, annotations, inventory, {"positive": positive, "negative": negative}


def _labelme(
    filename: str,
    *,
    label: str = "person_upper_body",
    shapes: list[dict[str, object]] | None = None,
    image_path: str | None = None,
) -> dict[str, object]:
    return {
        "version": "5.0.0",
        "flags": {},
        "shapes": shapes if shapes is not None else [{"label": label, "points": [[100.0, 120.0], [900.0, 1100.0]], "shape_type": "rectangle", "flags": {}}],
        "imagePath": image_path if image_path is not None else f"../images/{filename}",
        "imageData": None,
        "imageHeight": 1200,
        "imageWidth": 1920,
    }


def test_dry_run_then_apply_is_deterministic_and_keeps_negative_without_json(tmp_path: Path) -> None:
    root, annotations, inventory, names = _prepare_frozen_package(tmp_path)
    stem = Path(names["positive"]).stem
    (annotations / f"{stem}.json").write_text(json.dumps(_labelme(names["positive"])), encoding="utf-8")
    dry = convert_labelme_to_yolo(root, annotations_dir=annotations, inventory_path=inventory, dry_run=True, overwrite=False)
    assert dry["status"] == "READY_TO_WRITE"
    assert dry["negative_images_without_json"] == 1
    assert not (root / "labels" / f"{stem}.txt").exists()
    applied = convert_labelme_to_yolo(root, annotations_dir=annotations, inventory_path=inventory, dry_run=False, overwrite=False)
    assert applied["status"] == "CONVERTED"
    assert (root / "labels" / f"{stem}.txt").read_text(encoding="utf-8").startswith("0 ")
    assert validate_annotation_package(root, annotations_dir=annotations, inventory_path=inventory, require_complete=True)["status"] == "VALID"


def test_invalid_or_multiple_labelme_shapes_fail_closed_without_writing_label(tmp_path: Path) -> None:
    root, annotations, inventory, names = _prepare_frozen_package(tmp_path)
    stem = Path(names["positive"]).stem
    bad = _labelme(names["positive"], shapes=[
        {"label": "person_upper_body", "points": [[100, 100], [900, 1000]], "shape_type": "rectangle", "flags": {}},
        {"label": "person_upper_body", "points": [[50, 50], [100, 100]], "shape_type": "rectangle", "flags": {}},
    ])
    (annotations / f"{stem}.json").write_text(json.dumps(bad), encoding="utf-8")
    report = convert_labelme_to_yolo(root, annotations_dir=annotations, inventory_path=inventory, dry_run=False, overwrite=False)
    assert report["status"] == "BLOCKED_ANNOTATION_FAILURES"
    assert report["failure_count"] == 1
    assert not (root / "labels" / f"{stem}.txt").exists()


def test_relative_image_path_and_reversed_rectangle_points_are_normalized_in_dry_run(tmp_path: Path) -> None:
    root, annotations, inventory, names = _prepare_frozen_package(tmp_path)
    stem = Path(names["positive"]).stem
    payload = _labelme(
        names["positive"],
        shapes=[{"label": "person_upper_body", "points": [[900, 1100], [100, -2.842170943040401e-14]], "shape_type": "rectangle", "flags": {}}],
    )
    (annotations / f"{stem}.json").write_text(json.dumps(payload), encoding="utf-8")
    report = convert_labelme_to_yolo(root, annotations_dir=annotations, inventory_path=inventory, dry_run=True, overwrite=False)
    assert report["status"] == "READY_TO_WRITE"
    assert report["normalized_point_order_records"] == 1
    assert report["converted"][0]["yolo"] == "0 0.2604166667 0.4583333333 0.4166666667 0.9166666667"


def test_image_path_escape_fails_closed(tmp_path: Path) -> None:
    root, annotations, inventory, names = _prepare_frozen_package(tmp_path)
    stem = Path(names["positive"]).stem
    payload = _labelme(names["positive"], image_path=f"../raw/{names['positive']}")
    (annotations / f"{stem}.json").write_text(json.dumps(payload), encoding="utf-8")
    report = convert_labelme_to_yolo(root, annotations_dir=annotations, inventory_path=inventory, dry_run=True, overwrite=False)
    assert report["status"] == "BLOCKED_ANNOTATION_FAILURES"
    assert report["failure_count"] == 1
    assert "escapes the dataset images directory" in report["failures"][0]["reason"]


def test_validator_reports_non_yolo_file_in_labels_without_deleting_it(tmp_path: Path) -> None:
    root, annotations, inventory, _ = _prepare_frozen_package(tmp_path)
    unexpected = root / "labels" / "preserved_annotation.json"
    unexpected.write_text("{}\n", encoding="utf-8")
    report = validate_annotation_package(root, annotations_dir=annotations, inventory_path=inventory, require_complete=False)
    assert report["status"] == "PENDING_ANNOTATION"
    assert report["unexpected_label_files"] == ["preserved_annotation.json"]
    assert unexpected.read_text(encoding="utf-8") == "{}\n"
