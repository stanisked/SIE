#!/usr/bin/env python3
"""Create SHA-bound person instance masks from saved AR0234 audit frames.

Offline diagnostic. Requires an explicit local segmentation checkpoint and a
separate environment with Ultralytics. It does not alter the existing detector,
capture cameras, publish ROS data, or select a stereo component.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def iou(a: np.ndarray, b: np.ndarray) -> float:
    xy1 = np.maximum(a[:2], b[:2])
    xy2 = np.minimum(a[2:], b[2:])
    intersection = float(np.prod(np.maximum(xy2 - xy1, 0)))
    area_a = float(np.prod(np.maximum(a[2:] - a[:2], 0)))
    area_b = float(np.prod(np.maximum(b[2:] - b[:2], 0)))
    union = area_a + area_b - intersection
    return intersection / union if union > 0 else 0.0


def segment_one(path: Path, model: object, model_sha: str, output_dir: Path,
                minimum_iou: float, minimum_margin: float) -> dict:
    audit = json.loads(path.read_text(encoding="utf-8"))
    cycle_id = audit["cycle_id"]
    record = {"cycle_id": cycle_id, "status": "MASK_UNAVAILABLE"}
    candidate = (audit.get("ar0234_dynamic_target") or {}).get("candidate")
    if not isinstance(candidate, dict) or "bbox_xyxy_px" not in candidate:
        record["reason"] = "NO_CURRENT_RGB_TARGET"
        return record
    image_path = path.parent / "ar0234_source.jpg"
    source = cv2.imread(str(image_path))
    if source is None or source.shape[:2] != (1200, 1920):
        raise ValueError(f"missing or unexpected AR source frame: {image_path}")
    results = model.predict(source=source, imgsz=640, conf=0.25, classes=[0],
                            retina_masks=True, device="cpu", verbose=False)
    if len(results) != 1:
        raise ValueError(f"unexpected segmentation result count in {path}")
    result = results[0]
    if result.boxes is None or result.masks is None or len(result.boxes) == 0:
        record["reason"] = "NO_PERSON_SEGMENTATION"
        return record
    boxes = result.boxes.xyxy.cpu().numpy()
    confidence = result.boxes.conf.cpu().numpy()
    labels = result.boxes.cls.cpu().numpy().astype(int)
    if len(result.masks.data) != len(boxes) or np.any(labels != 0):
        raise ValueError(f"inconsistent person masks in {path}")
    target_box = np.asarray(candidate["bbox_xyxy_px"], dtype=np.float64)
    scores = np.asarray([iou(target_box, b) for b in boxes])
    ranked = np.argsort(scores)[::-1]
    best = int(ranked[0])
    runner_up = float(scores[ranked[1]]) if len(ranked) > 1 else 0.0
    record.update({"best_bbox_iou": float(scores[best]),
                   "runner_up_bbox_iou": runner_up,
                   "person_instance_count": len(boxes)})
    if scores[best] < minimum_iou or scores[best] - runner_up < minimum_margin:
        record["reason"] = "PERSON_INSTANCE_MATCH_AMBIGUOUS"
        return record
    mask = result.masks.data[best].cpu().numpy()
    if mask.shape != source.shape[:2]:
        raise ValueError(f"mask was not returned at AR full resolution in {path}")
    binary = (mask > 0.5).astype(np.uint8) * 255
    if not binary.any():
        record["reason"] = "PERSON_INSTANCE_MASK_EMPTY"
        return record
    mask_file = output_dir / f"{cycle_id}.png"
    if not cv2.imwrite(str(mask_file), binary):
        raise OSError(f"could not write {mask_file}")
    metadata = {
        "schema_version": "sie.ar0234.offline_instance_mask.v1",
        "cycle_id": cycle_id, "reference_frame": "ar0234_image_frame",
        "source_ar0234_sha256": sha256(image_path),
        "mask_sha256": sha256(mask_file),
        "mask_source": "YOLO11N_SEG_COCO_PERSON_OFFLINE_DIAGNOSTIC",
        "model_sha256": model_sha, "model_bbox_xyxy_px": boxes[best].tolist(),
        "source_target_bbox_xyxy_px": target_box.tolist(),
        "model_confidence": float(confidence[best]),
        "match_iou": float(scores[best]),
        "runner_up_iou": runner_up,
        "mask_pixel_count": int(np.count_nonzero(binary)),
        "semantic_ownership_certified": False,
    }
    (output_dir / f"{cycle_id}.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    record.update({"status": "MASK_SAVED_DIAGNOSTIC_ONLY", "mask_file": str(mask_file),
                   "mask_pixel_count": metadata["mask_pixel_count"]})
    return record


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence-root", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True,
                        help="explicit local YOLO11n-seg checkpoint")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--minimum-bbox-iou", type=float, default=0.50)
    parser.add_argument("--minimum-match-margin", type=float, default=0.15)
    args = parser.parse_args()
    if args.output_dir.exists():
        parser.error(f"output already exists: {args.output_dir}")
    if not args.model.is_file():
        parser.error(f"segmentation model absent: {args.model}")
    if not 0 < args.minimum_bbox_iou <= 1 or not 0 <= args.minimum_match_margin <= 1:
        parser.error("invalid match limits")
    paths = sorted(args.evidence_root.rglob("audit.json"))
    if not paths:
        parser.error("no saved audit cycles")
    from ultralytics import YOLO
    model = YOLO(str(args.model), task="segment")
    if model.task != "segment" or model.names.get(0) != "person":
        parser.error("expected a COCO instance segmentation model with person class 0")
    model_sha = sha256(args.model)
    args.output_dir.mkdir(parents=True)
    records = [segment_one(path, model, model_sha, args.output_dir,
                           args.minimum_bbox_iou, args.minimum_match_margin)
               for path in paths]
    summary = {"status": "OFFLINE_MASK_PRODUCTION_DIAGNOSTIC_ONLY",
               "model_sha256": model_sha,
               "evidence_root": str(args.evidence_root),
               "minimum_bbox_iou": args.minimum_bbox_iou,
               "minimum_match_margin": args.minimum_match_margin,
               "records": records}
    (args.output_dir / "producer_report.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    for row in records:
        print(row["cycle_id"][-8:], row["status"], row.get("reason", ""),
              "IoU=", round(row["best_bbox_iou"], 3)
              if "best_bbox_iou" in row else None)
    print("Saved:", args.output_dir / "producer_report.json")


if __name__ == "__main__":
    main()
