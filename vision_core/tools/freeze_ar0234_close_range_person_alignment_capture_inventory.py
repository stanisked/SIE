#!/usr/bin/env python3
"""Create a read-only SHA-256 snapshot of the captured AR0234 dataset package."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from vision_core.close_range_person_alignment_dataset.capture import DATASET_ID  # noqa: E402


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args(argv)


def create_inventory(dataset_root: Path, output_dir: Path) -> dict[str, object]:
    root = dataset_root.resolve()
    if not root.is_dir() or root.is_symlink():
        raise ValueError("dataset root must be a real directory")
    raw = sorted(path for path in (root / "raw").glob("*.png") if path.is_file() and not path.is_symlink())
    manifests = sorted(path for path in (root / "manifests").iterdir() if path.is_file() and not path.is_symlink())
    labels = sorted(path for path in (root / "labels").glob("*.txt") if path.is_file())
    entries = {
        "raw_images": [_entry(root, path) for path in raw],
        "manifest_files": [_entry(root, path) for path in manifests],
    }
    canonical = json.dumps(entries, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    timestamp = datetime.now(timezone.utc).isoformat()
    snapshot_id = f"ar0234-close-range-person-alignment-v1-{hashlib.sha256(canonical).hexdigest()[:16]}"
    report: dict[str, object] = {
        "schema_version": "sie.ar0234_close_range_person_alignment_capture_inventory.v1",
        "snapshot_id": snapshot_id,
        "created_at_utc": timestamp,
        "dataset_id": DATASET_ID,
        "dataset_root": str(root),
        "capture_package_status": "FROZEN_PENDING_ANNOTATION",
        "annotation_status": "NO_LABELS_PRESENT",
        "training_status": "NOT_TRAINED",
        "raw_image_count": len(raw),
        "manifest_file_count": len(manifests),
        "label_file_count": len(labels),
        **entries,
        "notes": [
            "This inventory reads the persistent capture package without copying or modifying raw originals.",
            "The snapshot fixes capture evidence only; it is not a model, yaw, or motion qualification.",
        ],
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / "FROZEN_CAPTURE_INVENTORY_v1.json"
    markdown_path = output_dir / "FROZEN_CAPTURE_INVENTORY_v1.md"
    _write_new(json_path, json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n")
    _write_new(
        markdown_path,
        "\n".join(
            [
                "# Frozen capture inventory v1",
                "",
                f"Snapshot ID: `{snapshot_id}`.",
                f"Created: `{timestamp}`.",
                f"Dataset: `{root}`.",
                "",
                f"Package: `{DATASET_ID}`, `FROZEN_PENDING_ANNOTATION`.",
                f"Raw PNG: **{len(raw)}**; manifest files: **{len(manifests)}**; labels: **{len(labels)}**.",
                "",
                "SHA-256 for every raw PNG and capture manifest is stored in the paired JSON. Raw originals were neither copied nor modified. The package is still without labels; no model has been trained.",
                "",
            ]
        ),
    )
    return report


def _entry(root: Path, path: Path) -> dict[str, object]:
    payload = path.read_bytes()
    return {"path": str(path.relative_to(root)), "sha256": hashlib.sha256(payload).hexdigest(), "bytes": len(payload)}


def _write_new(path: Path, text: str) -> None:
    if path.exists() or path.is_symlink():
        raise FileExistsError(f"refusing overwrite: {path}")
    path.write_text(text, encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    report = create_inventory(args.dataset_root, args.output_dir)
    print(json.dumps({"snapshot_id": report["snapshot_id"], "raw_image_count": report["raw_image_count"], "manifest_file_count": report["manifest_file_count"]}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
