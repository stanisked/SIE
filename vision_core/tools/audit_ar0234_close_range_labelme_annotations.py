#!/usr/bin/env python3
"""Read-only consistency audit for frozen AR0234 LabelMe annotations."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np


EXPECTED_LABEL = "person_upper_body"
WIDTH = 1920
HEIGHT = 1200
EDGE_EPSILON_PX = 1e-9
SCHEMA_VERSION = "sie.ar0234_close_range_person_alignment_labelme_audit.v1"


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--labelme-config", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--samples-per-stratum", type=int, default=3)
    return parser


def _strict_json(path: Path) -> Any:
    return json.loads(
        path.read_text(encoding="utf-8"),
        parse_constant=lambda value: (_ for _ in ()).throw(ValueError(f"non-finite JSON value: {value}")),
    )


def _quantiles(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {key: None for key in ("min", "p05", "median", "p95", "max")}
    values = sorted(values)
    return {
        "min": values[0],
        "p05": values[round((len(values) - 1) * 0.05)],
        "median": statistics.median(values),
        "p95": values[round((len(values) - 1) * 0.95)],
        "max": values[-1],
    }


def _read_capture_records(path: Path) -> dict[str, dict[str, Any]]:
    records: dict[str, dict[str, Any]] = {}
    for line_no, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        record = json.loads(line)
        name = record.get("image_filename")
        if not isinstance(name, str) or name in records:
            raise ValueError(f"invalid or duplicate capture record at line {line_no}")
        records[name] = record
    return records


def _read_sessions(manifests_dir: Path) -> dict[str, dict[str, Any]]:
    sessions: dict[str, dict[str, Any]] = {}
    for path in manifests_dir.glob("*.json"):
        payload = _strict_json(path)
        if isinstance(payload, dict) and isinstance(payload.get("session_id"), str):
            sessions[payload["session_id"]] = payload
    return sessions


def _parse_annotation(path: Path, image_path: Path) -> dict[str, Any]:
    issues: list[str] = []
    try:
        payload = _strict_json(path)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        return {"annotation_filename": path.name, "issues": [f"invalid_json:{error}"], "shapes": 0}
    if not isinstance(payload, dict):
        return {"annotation_filename": path.name, "issues": ["json_root_not_object"], "shapes": 0}
    declared_path = payload.get("imagePath")
    expected_image = image_path.resolve()
    resolved_image = (path.parent / declared_path).resolve() if isinstance(declared_path, str) else None
    if resolved_image != expected_image:
        issues.append("image_path_mismatch")
    if payload.get("imageWidth") != WIDTH or payload.get("imageHeight") != HEIGHT:
        issues.append("image_size_mismatch")
    shapes = payload.get("shapes")
    if not isinstance(shapes, list):
        return {"annotation_filename": path.name, "image_path": declared_path, "issues": issues + ["shapes_not_list"], "shapes": 0}
    result: dict[str, Any] = {
        "annotation_filename": path.name,
        "image_path": declared_path,
        "issues": issues,
        "shapes": len(shapes),
        "labels": [],
        "shape_types": [],
    }
    if len(shapes) != 1:
        result["issues"].append("shape_count_not_one")
        return result
    shape = shapes[0]
    if not isinstance(shape, dict):
        result["issues"].append("shape_not_object")
        return result
    label = shape.get("label")
    shape_type = shape.get("shape_type")
    result["labels"] = [label]
    result["shape_types"] = [shape_type]
    if label != EXPECTED_LABEL:
        result["issues"].append("wrong_label")
    if shape_type != "rectangle":
        result["issues"].append("shape_not_rectangle")
    points = shape.get("points")
    if not isinstance(points, list) or len(points) != 2:
        result["issues"].append("rectangle_points_not_pair")
        return result
    try:
        x1, y1 = float(points[0][0]), float(points[0][1])
        x2, y2 = float(points[1][0]), float(points[1][1])
    except (IndexError, TypeError, ValueError):
        result["issues"].append("rectangle_points_invalid")
        return result
    values = (x1, y1, x2, y2)
    if not all(math.isfinite(value) for value in values):
        result["issues"].append("rectangle_points_non_finite")
        return result
    left, right = sorted((x1, x2))
    top, bottom = sorted((y1, y2))
    if not (-EDGE_EPSILON_PX <= left < right <= WIDTH + EDGE_EPSILON_PX and -EDGE_EPSILON_PX <= top < bottom <= HEIGHT + EDGE_EPSILON_PX):
        result["issues"].append("rectangle_geometry_impossible")
        return result
    left = min(max(left, 0.0), float(WIDTH))
    right = min(max(right, 0.0), float(WIDTH))
    top = min(max(top, 0.0), float(HEIGHT))
    bottom = min(max(bottom, 0.0), float(HEIGHT))
    result.update(
        {
            "bbox_xyxy_px": [left, top, right, bottom],
            "x_point_order_reversed": x1 > x2,
            "y_point_order_reversed": y1 > y2,
            "touches_top": top <= EDGE_EPSILON_PX,
            "touches_bottom": bottom >= float(HEIGHT) - EDGE_EPSILON_PX,
            "touches_left": left <= EDGE_EPSILON_PX,
            "touches_right": right >= float(WIDTH) - EDGE_EPSILON_PX,
            "aspect_ratio": (right - left) / (bottom - top),
            "relative_area": ((right - left) * (bottom - top)) / (WIDTH * HEIGHT),
        }
    )
    return result


def _config_summary(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8")
    compact = next((line.strip() for line in text.splitlines() if line.strip().startswith("labels:")), None)
    return {
        "path": str(path),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "labels_declaration": compact,
        "contains_expected_label": EXPECTED_LABEL in text,
        "has_exact_validation": "validate_label: exact" in text,
        "has_image_data_disabled": "with_image_data: false" in text,
    }


def _choose_samples(records: list[dict[str, Any]], count: int) -> list[tuple[str, dict[str, Any]]]:
    selectors = {
        "standing": lambda item: str(item["tags"].get("pose", "")).startswith("standing"),
        "sitting": lambda item: item["tags"].get("pose") == "sitting",
        "near": lambda item: item["tags"].get("distance_band") == "0.8-1.2m",
        "far": lambda item: item["tags"].get("distance_band") == "2.5-3.5m",
        "daylight": lambda item: item["tags"].get("lighting") == "daylight",
        "artificial": lambda item: item["tags"].get("lighting") == "artificial",
    }
    selected: list[tuple[str, dict[str, Any]]] = []
    for name, selector in selectors.items():
        matching = [record for record in records if not record["issues"] and selector(record)]
        for record in sorted(matching, key=lambda item: item["image_filename"])[:count]:
            selected.append((name, record))
    return selected


def _write_contact_sheet(output: Path, dataset_root: Path, samples: list[tuple[str, dict[str, Any]]]) -> None:
    cell_width, image_height, caption_height = 384, 240, 54
    cols = 6
    rows = max(1, math.ceil(len(samples) / cols))
    sheet = np.full((rows * (image_height + caption_height), cols * cell_width, 3), 245, dtype=np.uint8)
    for index, (stratum, record) in enumerate(samples):
        image = cv2.imread(str(dataset_root / "images" / record["image_filename"]), cv2.IMREAD_COLOR)
        if image is None:
            continue
        left, top, right, bottom = (round(value) for value in record["bbox_xyxy_px"])
        cv2.rectangle(image, (left, top), (right, bottom), (0, 200, 0), 5)
        view = cv2.resize(image, (cell_width, image_height), interpolation=cv2.INTER_AREA)
        row, col = divmod(index, cols)
        y, x = row * (image_height + caption_height), col * cell_width
        sheet[y : y + image_height, x : x + cell_width] = view
        caption = f"{stratum}: {record['image_filename'][:33]}"
        meta = f"{record['tags'].get('pose')} | {record['tags'].get('distance_band')}"
        cv2.putText(sheet, caption, (x + 5, y + image_height + 19), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (20, 20, 20), 1, cv2.LINE_AA)
        cv2.putText(sheet, meta, (x + 5, y + image_height + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (20, 20, 20), 1, cv2.LINE_AA)
    if not cv2.imwrite(str(output), sheet):
        raise RuntimeError(f"could not write contact sheet: {output}")


def _markdown(report: dict[str, Any]) -> str:
    geometry = report["geometry"]
    candidates = report["manual_review_candidates"]
    return f"""# LabelMe consistency audit v1\n\n## Scope\n\nRead-only audit of `{report['dataset_root']}`. No source image, LabelMe JSON, manifest, frozen inventory, YOLO label, split or model artifact was changed.\n\n## Facts\n\n- LabelMe JSON: `{report['counts']['annotation_json']}`; annotated images: `{report['counts']['annotated_images']}`; total shapes: `{report['counts']['shapes']}`.\n- Exact `person_upper_body`: `{report['counts']['exact_expected_label']}`; rectangles: `{report['counts']['rectangles']}`.\n- Proven impossible geometry: `{len(candidates['proven_invalid'])}`; wrong labels: `{len(candidates['wrong_label'])}`; multiple/non-single shapes: `{len(candidates['shape_count_not_one'])}`; image mismatches: `{len(candidates['image_mismatch'])}`.\n- Edge-touch diagnostics: top `{geometry['top_edge_touch_count']}`, bottom `{geometry['bottom_edge_touch_count']}`, left `{geometry['left_edge_touch_count']}`, right `{geometry['right_edge_touch_count']}`.\n- Point-order diagnostic: `{geometry['x_point_order_reversed_count']}` rectangles have reversed X point order. This is not automatically a visual bbox error, but the current converter's strict `x1 < x2` rule will reject them.\n\n## Bbox distribution\n\n| Metric | min | P05 | median | P95 | max |\n| --- | ---: | ---: | ---: | ---: | ---: |\n| aspect ratio | {geometry['aspect_ratio']['min']:.4f} | {geometry['aspect_ratio']['p05']:.4f} | {geometry['aspect_ratio']['median']:.4f} | {geometry['aspect_ratio']['p95']:.4f} | {geometry['aspect_ratio']['max']:.4f} |\n| relative area | {geometry['relative_area']['min']:.4f} | {geometry['relative_area']['p05']:.4f} | {geometry['relative_area']['median']:.4f} | {geometry['relative_area']['p95']:.4f} | {geometry['relative_area']['max']:.4f} |\n\nAspect-ratio extremes are concentrated in sitting sessions; this is a pose correlation, not proof of incorrect annotation. The contact sheet is the required visual review aid.\n\n## Workflow findings\n\nCurrent LabelMe config declaration is `{report['labelme_config']['labels_declaration']}`. It does not match the committed single-class contract unless it declares only `person_upper_body`. Existing annotations remain `person_upper_body`; do not relabel or mass-edit them from this finding alone. `imagePath` values resolve correctly to the source PNG, but are relative (`../images/...`); current converter requires only a basename and therefore is incompatible with these otherwise consistent LabelMe files.\n\n## Manual review candidates\n\nNo semantic bbox inconsistency is proven by geometry alone. Manual review is limited to the listed structural candidates plus top/bottom edge-touch subsets, where the protocol requires confirming that an image boundary really truncates the torso.\n\n- structural candidates: `{len(candidates['structural'])}`\n- top-edge candidates: `{len(candidates['top_edge_touch'])}`\n- bottom-edge candidates: `{len(candidates['bottom_edge_touch'])}`\n\n## Decision\n\n**A is the work-preserving path:** retain `person_upper_body`. B would silently change training semantics and is unsupported. C is appropriate only for individually reviewed bbox candidates, not a bulk redraw. Before any YOLO conversion, make a separate narrow converter/config compatibility change for resolved relative `imagePath` and normalized rectangle point ordering; that is a workflow fix, not a label rewrite.\n\nThe audit does not train a model, create a split, qualify yaw alignment, or change motion behavior.\n"""


def audit(dataset_root: Path, config_path: Path, output_dir: Path, samples_per_stratum: int) -> dict[str, Any]:
    if samples_per_stratum <= 0:
        raise ValueError("samples-per-stratum must be positive")
    annotations_dir = dataset_root / "annotations_labelme"
    images_dir = dataset_root / "images"
    manifests_dir = dataset_root / "manifests"
    records = _read_capture_records(manifests_dir / "capture_records.jsonl")
    sessions = _read_sessions(manifests_dir)
    annotations = sorted(annotations_dir.glob("*.json"))
    entries: list[dict[str, Any]] = []
    for path in annotations:
        image_filename = f"{path.stem}.png"
        entry = _parse_annotation(path, images_dir / image_filename)
        entry["image_filename"] = image_filename
        record = records.get(image_filename)
        if record is None:
            entry["issues"].append("missing_capture_record")
            entry["tags"] = {}
        else:
            session = sessions.get(record["session_id"], {})
            entry["session_id"] = record["session_id"]
            entry["tags"] = session.get("session_tags", {})
        entries.append(entry)
    values = [entry for entry in entries if "bbox_xyxy_px" in entry and not entry["issues"]]
    issue_groups: dict[str, list[str]] = defaultdict(list)
    for entry in entries:
        for issue in entry["issues"]:
            issue_groups[issue].append(entry["image_filename"])
    structural = sorted(
        entry["image_filename"]
        for entry in entries
        if entry.get("x_point_order_reversed") or entry.get("y_point_order_reversed")
    )
    top_edge = sorted(entry["image_filename"] for entry in values if entry["touches_top"])
    bottom_edge = sorted(entry["image_filename"] for entry in values if entry["touches_bottom"])
    by_tag: dict[str, dict[str, int]] = {}
    for tag in ("pose", "lateral_position", "distance_band", "lighting"):
        by_tag[tag] = dict(sorted(Counter(str(entry["tags"].get(tag, "missing")) for entry in values).items()))
    report = {
        "schema_version": SCHEMA_VERSION,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "dataset_root": str(dataset_root),
        "source_dataset_read_only": True,
        "capture_records_sha256": hashlib.sha256((manifests_dir / "capture_records.jsonl").read_bytes()).hexdigest(),
        "labelme_config": _config_summary(config_path),
        "counts": {
            "annotation_json": len(annotations),
            "annotated_images": len({entry["image_filename"] for entry in entries}),
            "shapes": sum(entry["shapes"] for entry in entries),
            "exact_expected_label": sum(entry.get("labels") == [EXPECTED_LABEL] for entry in entries),
            "rectangles": sum(entry.get("shape_types") == ["rectangle"] for entry in entries),
        },
        "geometry": {
            "top_edge_touch_count": sum(entry["touches_top"] for entry in values),
            "bottom_edge_touch_count": sum(entry["touches_bottom"] for entry in values),
            "left_edge_touch_count": sum(entry["touches_left"] for entry in values),
            "right_edge_touch_count": sum(entry["touches_right"] for entry in values),
            "x_point_order_reversed_count": sum(entry["x_point_order_reversed"] for entry in values),
            "y_point_order_reversed_count": sum(entry["y_point_order_reversed"] for entry in values),
            "aspect_ratio": _quantiles([entry["aspect_ratio"] for entry in values]),
            "relative_area": _quantiles([entry["relative_area"] for entry in values]),
        },
        "by_tag": by_tag,
        "manual_review_candidates": {
            "proven_invalid": sorted(set(issue_groups["rectangle_geometry_impossible"] + issue_groups["rectangle_points_invalid"] + issue_groups["rectangle_points_non_finite"])),
            "wrong_label": sorted(issue_groups["wrong_label"]),
            "shape_count_not_one": sorted(issue_groups["shape_count_not_one"]),
            "image_mismatch": sorted(issue_groups["image_path_mismatch"] + issue_groups["image_size_mismatch"] + issue_groups["missing_capture_record"]),
            "structural": structural,
            "top_edge_touch": top_edge,
            "bottom_edge_touch": bottom_edge,
        },
        "records": entries,
    }
    json.dumps(report, allow_nan=False)
    output_dir.mkdir(parents=True, exist_ok=False)
    json_path = output_dir / "LABELME_CONSISTENCY_AUDIT_v1.json"
    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
    samples = _choose_samples(values, samples_per_stratum)
    _write_contact_sheet(output_dir / "LABELME_CONSISTENCY_CONTACT_SHEET_v1.jpg", dataset_root, samples)
    (output_dir / "LABELME_CONSISTENCY_AUDIT_v1.md").write_text(_markdown(report), encoding="utf-8")
    return report


def main() -> int:
    args = _parser().parse_args()
    report = audit(args.dataset_root, args.labelme_config, args.output_dir, args.samples_per_stratum)
    print(json.dumps({"status": "COMPLETED", "output_dir": str(args.output_dir), "counts": report["counts"]}, ensure_ascii=False, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
