#!/usr/bin/env python3
"""Build a provenance-bound YOLO training addendum from reviewed Shadow frames.

Only new derived labels are training-ready. The reviewed LabelMe source archive
and its original JPEGs are never modified.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import io
import json
import math
from pathlib import Path, PurePosixPath
import tarfile


MODEL_SHA = "5583d645afebd3d52706b76655662cb4ce16110b2a0b224af2cc806d0292d07b"
SOURCE_SHA = "b8d0437995ff81b9b6ddcd24029ecfb0857571729614566fa97c70796b2b1cbb"
BASELINE_SHA = "9cec0505c3390efc44c59513204469d3f2d84785fff3598092fc5d306dadd6ef"
REVIEWED_SHA = "8b79e947ef381825440bfe2a826b463ac4568d42182b62917e1cb606b792438f"
SOURCE_PREFIX = "shadow_two_people_annotation_review_20261031/"


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def members(path: Path) -> dict[str, bytes]:
    result: dict[str, bytes] = {}
    with tarfile.open(path, "r:gz") as archive:
        for member in archive:
            name = PurePosixPath(member.name)
            if (name.is_absolute() or ".." in name.parts
                    or not (member.isfile() or member.isdir())):
                raise ValueError(f"UNSAFE_ARCHIVE_MEMBER:{member.name}")
            if member.isdir():
                continue
            if member.name in result:
                raise ValueError(f"DUPLICATE_ARCHIVE_MEMBER:{member.name}")
            result[member.name] = archive.extractfile(member).read()
    return result


def labelme_boxes(data: bytes, image_name: str) -> list[list[float]]:
    doc = json.loads(data)
    if (doc.get("imagePath") != image_name or doc.get("imageData") is not None
            or (doc.get("imageWidth"), doc.get("imageHeight")) != (1920, 1200)
            or doc.get("flags", {}).get("sie_person_visible") is not True
            or len(doc.get("shapes", [])) != 2):
        raise ValueError(f"INVALID_REVIEWED_LABELME:{image_name}")
    boxes = []
    for shape in doc["shapes"]:
        if (shape.get("label") != "person_upper_body"
                or shape.get("shape_type") != "rectangle"
                or len(shape.get("points", [])) != 2):
            raise ValueError(f"INVALID_PERSON_SHAPE:{image_name}")
        (x1, y1), (x2, y2) = shape["points"]
        values = (x1, y1, x2, y2)
        if (any(type(v) not in (int, float) or not math.isfinite(v) for v in values)
                or not (0 <= x1 < x2 <= 1920 and 0 <= y1 < y2 <= 1200)):
            raise ValueError(f"INVALID_PERSON_BOUNDS:{image_name}")
        boxes.append(list(values))
    if not (sum(boxes[0][::2]) < sum(boxes[1][::2])):
        raise ValueError(f"EXPECTED_LEFT_AND_RIGHT_PEOPLE:{image_name}")
    return boxes


def yolo_text(boxes: list[list[float]]) -> bytes:
    lines = []
    for x1, y1, x2, y2 in boxes:
        values = ((x1+x2)/3840, (y1+y2)/2400,
                  (x2-x1)/1920, (y2-y1)/1200)
        lines.append("0 " + " ".join(f"{v:.9f}" for v in values) + "\n")
    return "".join(lines).encode("ascii")


def addendum(reviewed: Path, baseline: Path, output: Path) -> dict:
    if output.exists():
        raise FileExistsError(output)
    if sha(reviewed.read_bytes()) != REVIEWED_SHA or sha(baseline.read_bytes()) != BASELINE_SHA:
        raise ValueError("INPUT_ARCHIVE_HASH_MISMATCH")
    source = members(reviewed)
    train = members(baseline)
    manifest = json.loads(source[SOURCE_PREFIX + "manifest.json"])
    baseline_manifest = json.loads(train["manifest.json"])
    if (manifest.get("source_archive_sha256") != SOURCE_SHA
            or manifest.get("model_sha256") != MODEL_SHA
            or manifest.get("status") != "UNREVIEWED_DRAFT"
            or manifest.get("source_frame_count") != 6
            or len(manifest.get("frames", [])) != 6
            or baseline_manifest.get("counts", {}).get("train_pool_images") != 230
            or baseline_manifest.get("class_names") != ["person_upper_body"]):
        raise ValueError("SOURCE_OR_BASELINE_MANIFEST_MISMATCH")
    existing = set()
    for row in baseline_manifest["rows"]:
        if row["kind"] == "ignored":
            continue
        packet = train[row["image"]]
        if sha(packet) != row["source_image_sha256"]:
            raise ValueError(f"BASELINE_IMAGE_HASH_MISMATCH:{row['image']}")
        if sha(train[row["label"]]) != row["yolo_label_sha256"]:
            raise ValueError(f"BASELINE_LABEL_HASH_MISMATCH:{row['label']}")
        existing.add(row["source_image_sha256"])
    if len(existing) != 230:
        raise ValueError("DUPLICATE_BASELINE_IMAGES")
    new: dict[str, bytes] = {}
    rows = []
    for frame in manifest["frames"]:
        image_name = Path(frame["image"]).name
        stem = Path(image_name).stem
        packet = source[SOURCE_PREFIX + frame["image"]]
        if (sha(packet) != frame["source_jpeg_sha256"]
                or not packet.startswith(b"\xff\xd8")
                or frame.get("review_status") != "UNREVIEWED"
                or frame.get("usable_for_training") is not False
                or frame["source_jpeg_sha256"] in existing):
            raise ValueError(f"INVALID_SOURCE_FRAME:{image_name}")
        labelme_name = SOURCE_PREFIX + "images/" + stem + ".json"
        labelme = source[labelme_name]
        boxes = labelme_boxes(labelme, image_name)
        target_image = "images/train/two_" + image_name
        target_label = "labels/train/two_" + stem + ".txt"
        label = yolo_text(boxes)
        new[target_image] = packet
        new[target_label] = label
        rows.append({"source_cycle_id": frame["cycle_id"],
                     "source_image_sha256": sha(packet),
                     "source_labelme_sha256": sha(labelme),
                     "training_image": target_image, "training_label": target_label,
                     "training_label_sha256": sha(label),
                     "box_count": len(boxes)})
        existing.add(sha(packet))
    if len(rows) != 6 or len(existing) != 236:
        raise ValueError("EXPECTED_SIX_UNIQUE_IMAGES")
    report = {
        "schema_version": "sie.shadow_two_person_training_addendum.v1",
        "status": "OPERATOR_REVIEWED_TRAINING_ADDENDUM",
        "review_basis": "operator-confirmed six LabelMe corrections, 2026-11-01",
        "reviewed_archive_sha256": REVIEWED_SHA,
        "base_training_archive_sha256": BASELINE_SHA,
        "source_stereo_archive_sha256": SOURCE_SHA, "detector_model_sha256": MODEL_SHA,
        "class_names": ["person_upper_body"],
        "counts": {"images": 6, "person_boxes": 12,
                   "base_train_images": 230, "combined_train_images": 236},
        "rows": rows,
        "limitations": [
            "The six frames were selected because v2 missed the nearer person.",
            "Both stereo runs are now development data, not independent holdout.",
            "No range, identity, or motor authorization follows from RGB labels.",
        ],
    }
    new["manifest.json"] = (json.dumps(report, indent=2, ensure_ascii=False) + "\n").encode()
    new["README.md"] = (
        "# Shadow two-person training addendum\n\n"
        "Merge the six image/label pairs into the 230-image training pool by their "
        "relative paths. They were checked against the frozen source hashes and "
        "operator-reviewed LabelMe coordinates. Never use these evaluated runs "
        "as an independent test after training. See manifest.json for provenance.\n"
    ).encode()
    with output.open("xb") as destination, gzip.GzipFile(
            fileobj=destination, mode="wb", mtime=0, filename="") as compressed:
        with tarfile.open(fileobj=compressed, mode="w") as archive:
            for name, data in sorted(new.items()):
                item = tarfile.TarInfo(name)
                item.mode = 0o644
                item.mtime = 0
                item.size = len(data)
                archive.addfile(item, io.BytesIO(data))
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reviewed", type=Path, required=True)
    parser.add_argument("--baseline-train", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = addendum(args.reviewed, args.baseline_train, args.output)
    print(json.dumps({"status": report["status"], "counts": report["counts"],
                      "sha256": sha(args.output.read_bytes()),
                      "output": str(args.output)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
