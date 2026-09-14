#!/usr/bin/env python3
"""Read-only audit of the AR0234 close-range person-alignment dataset."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in __import__("sys").path:
    __import__("sys").path.insert(0, str(REPOSITORY_ROOT))

from vision_core.close_range_person_alignment_dataset.capture import DATASET_ID  # noqa: E402


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--report-version", choices=("v1", "v2"), default="v1")
    return parser.parse_args(argv)


def audit_dataset(dataset_root: Path, output_dir: Path, *, report_version: str = "v1") -> dict[str, object]:
    root = dataset_root.resolve()
    if not root.is_dir() or root.is_symlink():
        raise ValueError(f"dataset root must be a real directory: {dataset_root}")
    records_path = root / "manifests" / "capture_records.jsonl"
    images_dir = root / "images"
    records = [_strict_json(line, f"capture_records.jsonl:{index}") for index, line in enumerate(records_path.read_text(encoding="utf-8").splitlines(), 1) if line.strip()]
    session_files = sorted(path for path in (root / "manifests").glob("*.json") if path.name != "dataset_manifest.json")
    sessions = {path.stem: _strict_json(path.read_text(encoding="utf-8"), path.name) for path in session_files}
    image_paths = sorted(images_dir.glob("*.png"))
    record_names = [record.get("image_filename") for record in records]
    image_name_set = {path.name for path in image_paths}
    missing = sorted(name for name in record_names if not isinstance(name, str) or name not in image_name_set)
    extra = sorted(name for name in image_name_set if name not in set(record_names))
    duplicate_names = sorted(name for name, count in Counter(record_names).items() if count > 1)
    quality: list[dict[str, object]] = []
    exact: dict[str, list[str]] = defaultdict(list)
    images_by_name: dict[str, np.ndarray] = {}
    sha_mismatches: list[str] = []
    decode_failures: list[str] = []
    wrong_dimensions: list[str] = []
    for record in records:
        filename = record.get("image_filename")
        if not isinstance(filename, str) or filename not in image_name_set:
            continue
        path = images_dir / filename
        payload = path.read_bytes()
        digest = hashlib.sha256(payload).hexdigest()
        exact[digest].append(filename)
        if digest != record.get("image_sha256"):
            sha_mismatches.append(filename)
        image = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            decode_failures.append(filename)
            continue
        images_by_name[filename] = image
        if image.shape != (1200, 1920, 3):
            wrong_dimensions.append(filename)
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        quality.append({
            "image_filename": filename,
            "brightness_mean": float(gray.mean()),
            "clipping_percent": float(np.mean((gray <= 1) | (gray >= 254)) * 100.0),
            "sharpness_laplacian_variance": float(cv2.Laplacian(gray, cv2.CV_64F).var()),
        })
    exact_groups = [sorted(names) for names in exact.values() if len(names) > 1]
    near_pairs = _near_duplicate_pairs(images_by_name)
    session_counts: dict[str, int] = Counter(str(record.get("session_id")) for record in records)
    tag_counts: dict[str, dict[str, int]] = {}
    for field in ("pose", "lateral_position", "distance_band", "lighting", "contains_person"):
        tag_counts[field] = dict(
            sorted(
                Counter(
                    sessions.get(str(record.get("session_id")), {})
                    .get("session_tags", {})
                    .get(field)
                    for record in records
                ).items(),
                key=lambda item: str(item[0]),
            )
        )
    quality_summary = _quality_summary(quality)
    output_dir.mkdir(parents=True, exist_ok=True)
    contact_sheet = output_dir / f"contact_sheet_3_per_session_{report_version}.jpg"
    _write_contact_sheet(images_by_name, records, sessions, contact_sheet)
    report: dict[str, object] = {
        "schema_version": f"sie.ar0234_close_range_person_alignment_audit.{report_version}",
        "dataset_id": DATASET_ID,
        "dataset_root": str(root),
        "read_only": True,
        "manifest_records": len(records),
        "session_count": len(sessions),
        "image_count": len(image_paths),
        "label_count": len(list((root / "labels").glob("*.txt"))),
        "manifest_image_check": {"missing_images": missing, "extra_images": extra, "duplicate_record_filenames": duplicate_names, "unique_filename_count": len(image_name_set)},
        "integrity": {"sha_mismatches": sorted(sha_mismatches), "decode_failures": sorted(decode_failures), "wrong_dimensions": sorted(wrong_dimensions), "all_images_valid": not (missing or extra or duplicate_names or sha_mismatches or decode_failures or wrong_dimensions)},
        "counts": {"by_session_id": dict(sorted(session_counts.items())), "by_pose": tag_counts["pose"], "by_lateral_position": tag_counts["lateral_position"], "by_distance_band": tag_counts["distance_band"], "by_lighting": tag_counts["lighting"], "by_contains_person": tag_counts["contains_person"]},
        "duplicates": {"exact_sha256_groups": exact_groups, "near_duplicate_method": "mean absolute grayscale difference on 64x40 thumbnail <= 2.0; diagnostic only", "near_duplicate_pairs": near_pairs},
        "image_quality": {"metric_definitions": {"brightness": "mean grayscale value 0..255", "clipping": "percent grayscale pixels <=1 or >=254", "sharpness": "variance of Laplacian"}, "aggregate": quality_summary, "per_image": quality},
        "contact_sheet": str(contact_sheet),
        "annotation_status": "NOT_ANNOTATED",
        "training_status": "NOT_TRAINED",
        "notes": ["Raw dataset files were read only.", "Quality, duplicate and near-duplicate findings are diagnostic and do not define an acceptance policy.", "No model qualification for yaw or motion follows from this audit."],
    }
    report_path = output_dir / f"DATASET_AUDIT_{report_version}.json"
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
    _write_summary(report, output_dir / f"DATASET_AUDIT_{report_version}.md", report_version)
    return report


def _near_duplicate_pairs(images: dict[str, np.ndarray]) -> list[dict[str, object]]:
    names = sorted(images)
    thumbnails = {name: cv2.resize(cv2.cvtColor(images[name], cv2.COLOR_BGR2GRAY), (64, 40), interpolation=cv2.INTER_AREA).astype(np.float32) for name in names}
    pairs: list[dict[str, object]] = []
    for index, first in enumerate(names):
        for second in names[index + 1:]:
            distance = float(np.mean(np.abs(thumbnails[first] - thumbnails[second])))
            if distance <= 2.0:
                pairs.append({"first": first, "second": second, "mean_absolute_difference": distance})
    return pairs


def _quality_summary(rows: list[dict[str, object]]) -> dict[str, float | int | None]:
    if not rows:
        return {"count": 0, "brightness_mean": None, "brightness_median": None, "clipping_percent_mean": None, "sharpness_mean": None, "sharpness_median": None}
    brightness = np.array([float(row["brightness_mean"]) for row in rows])
    clipping = np.array([float(row["clipping_percent"]) for row in rows])
    sharpness = np.array([float(row["sharpness_laplacian_variance"]) for row in rows])
    return {"count": len(rows), "brightness_mean": float(brightness.mean()), "brightness_median": float(np.median(brightness)), "brightness_min": float(brightness.min()), "brightness_max": float(brightness.max()), "clipping_percent_mean": float(clipping.mean()), "clipping_percent_max": float(clipping.max()), "sharpness_mean": float(sharpness.mean()), "sharpness_median": float(np.median(sharpness)), "sharpness_min": float(sharpness.min()), "sharpness_max": float(sharpness.max())}


def _write_contact_sheet(images: dict[str, np.ndarray], records: list[dict[str, object]], sessions: dict[str, dict[str, object]], path: Path) -> None:
    by_session: dict[str, list[str]] = defaultdict(list)
    for record in records:
        filename = record.get("image_filename")
        if isinstance(filename, str) and filename in images:
            by_session[str(record.get("session_id"))].append(filename)
    rows: list[np.ndarray] = []
    thumb_width, thumb_height = 480, 300
    for session_id in sorted(by_session):
        filenames = sorted(by_session[session_id])
        picks = [filenames[0], filenames[len(filenames) // 2], filenames[-1]] if filenames else []
        cells: list[np.ndarray] = []
        for filename in picks:
            thumb = cv2.resize(images[filename], (thumb_width, thumb_height), interpolation=cv2.INTER_AREA)
            cv2.putText(thumb, filename, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1, cv2.LINE_AA)
            cells.append(thumb)
        while len(cells) < 3:
            cells.append(np.zeros((thumb_height, thumb_width, 3), dtype=np.uint8))
        row = np.hstack(cells)
        title = np.zeros((36, row.shape[1], 3), dtype=np.uint8)
        tags = sessions.get(session_id, {}).get("session_tags", {})
        cv2.putText(title, f"{session_id} | pose={tags.get('pose')} | lateral={tags.get('lateral_position')}", (8, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 1, cv2.LINE_AA)
        rows.append(np.vstack((title, row)))
    if not rows:
        rows = [np.zeros((thumb_height + 36, thumb_width * 3, 3), dtype=np.uint8)]
    _write_image_exclusive(path, np.vstack(rows))


def _write_summary(report: dict[str, object], path: Path, report_version: str) -> None:
    counts = report["counts"]
    integrity = report["integrity"]
    quality = report["image_quality"]["aggregate"]
    duplicates = report["duplicates"]
    lines = [
        f"# DATASET_AUDIT_{report_version}",
        "",
        f"Dataset: `{report['dataset_root']}` (read-only audit).",
        "",
        f"Records: **{report['manifest_records']}**, sessions: **{report['session_count']}**, images: **{report['image_count']}**, labels: **{report['label_count']}**.",
        "",
        "## Integrity",
        "",
        f"All records/images valid: **{integrity['all_images_valid']}**. Missing={len(report['manifest_image_check']['missing_images'])}, extra={len(report['manifest_image_check']['extra_images'])}, SHA mismatches={len(integrity['sha_mismatches'])}, decode failures={len(integrity['decode_failures'])}, wrong dimensions={len(integrity['wrong_dimensions'])}.",
        "",
        "## Distribution",
        "",
        f"- Sessions: `{json.dumps(counts['by_session_id'], ensure_ascii=False, sort_keys=True)}`",
        f"- Pose: `{json.dumps(counts['by_pose'], ensure_ascii=False, sort_keys=True)}`",
        f"- Lateral position: `{json.dumps(counts['by_lateral_position'], ensure_ascii=False, sort_keys=True)}`",
        f"- Distance band: `{json.dumps(counts['by_distance_band'], ensure_ascii=False, sort_keys=True)}`",
        f"- Lighting: `{json.dumps(counts['by_lighting'], ensure_ascii=False, sort_keys=True)}`",
        f"- Contains person: `{json.dumps(counts['by_contains_person'], ensure_ascii=False, sort_keys=True)}`",
        "",
        "## Duplicates",
        "",
        f"Exact duplicate groups: **{len(duplicates['exact_sha256_groups'])}**. Near-duplicate pairs: **{len(duplicates['near_duplicate_pairs'])}**. Near criterion: mean absolute grayscale difference on 64×40 thumbnail ≤2.0; diagnostic only.",
        "",
        "## Image quality",
        "",
        f"Brightness mean/median: `{quality['brightness_mean']:.3f}` / `{quality['brightness_median']:.3f}`; clipping mean/max: `{quality['clipping_percent_mean']:.5f}%` / `{quality['clipping_percent_max']:.5f}%`; sharpness mean/median: `{quality['sharpness_mean']:.3f}` / `{quality['sharpness_median']:.3f}`.",
        "",
        "## Status",
        "",
        "Кадры ещё **НЕ размечены**: `labels/` пустой. Модель **НЕ обучалась**. Этот аудит не является acceptance policy и не даёт qualification для yaw или движения.",
        "",
        f"Contact sheet: `{report['contact_sheet']}`.",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _strict_json(payload: str | bytes, label: object) -> dict[str, object]:
    value = json.loads(payload, parse_constant=lambda constant: (_ for _ in ()).throw(ValueError(f"non-finite JSON constant: {constant}")))
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a JSON object")
    return value


def _write_image_exclusive(path: Path, image: np.ndarray) -> None:
    if path.exists() or path.is_symlink():
        raise FileExistsError(f"refusing overwrite: {path}")
    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 95])
    if not ok:
        raise ValueError("contact sheet encoding failed")
    path.write_bytes(encoded.tobytes())


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    report = audit_dataset(args.dataset_root, args.output_dir, report_version=args.report_version)
    print(json.dumps({"status": "OK", "report": str(args.output_dir / f"DATASET_AUDIT_{args.report_version}.json"), "sessions": report["session_count"], "images": report["image_count"]}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
