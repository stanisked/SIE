#!/usr/bin/env python3
"""Offline person-presence screening on frozen Shadow AR0234 JPEGs.

Score thresholds here are diagnostic; this tool has no camera, UART, ROS or
motor access. A score above threshold is not proof of correct localization.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path


THRESHOLDS = (0.70, 0.40, 0.10)


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def verified_rows(root: Path, annotations: dict) -> list[dict]:
    if annotations.get("schema_version") != "sie.shadow_rgb_visual_labels.v1":
        raise ValueError("INVALID_LABEL_SCHEMA")
    labels = annotations.get("frames")
    if not isinstance(labels, list) or not labels:
        raise ValueError("EMPTY_LABELS")
    rows = []
    seen = set()
    for label in labels:
        cid = label.get("cycle_id")
        if (not isinstance(cid, str) or not cid.startswith("ar0234-ov9281-dynamic-audit-")
                or "/" in cid or cid in seen or type(label.get("person_visible")) is not bool):
            raise ValueError("INVALID_OR_DUPLICATE_LABEL")
        seen.add(cid)
        matches = list(root.glob("*/" + cid))
        if len(matches) != 1:
            raise ValueError(f"MISSING_OR_DUPLICATE_CYCLE:{cid}")
        folder = matches[0]
        image, audit_file = folder / "ar0234_source.jpg", folder / "audit.json"
        if (digest(image) != label.get("image_sha256")
                or digest(audit_file) != label.get("audit_sha256")):
            raise ValueError(f"FROZEN_EVIDENCE_HASH_MISMATCH:{cid}")
        audit = json.loads(audit_file.read_text(encoding="utf-8"))
        if (audit.get("cycle_id") != cid or audit.get("actuator_access") is not False
                or audit.get("metric_measurement_authorized") is not False):
            raise ValueError(f"INVALID_SOURCE_AUDIT:{cid}")
        target = (audit.get("ar0234_dynamic_target") or {})
        rows.append({"cycle_id": cid, "person_visible": label["person_visible"],
                     "image_sha256": label["image_sha256"], "audit_sha256": label["audit_sha256"],
                     "original_rgb_target_status": target.get("source_target_status"),
                     "original_rgb_candidate": isinstance(target.get("candidate"), dict),
                     "original_audit_status": audit.get("status"),
                     "original_audit_reason": audit.get("reason"),
                     "image_path": image})
    return rows


def infer_saved_jpegs(rows: list[dict], model: Path) -> tuple[dict, str]:
    import cv2
    import onnxruntime as ort
    from vision_core.person_localization.yolo11_person_upper_body import (
        MODEL_INPUT_SIZE, decode_yolo11_one_class_candidates, letterbox_bgr,
        nms_detections, verify_model_sha256)

    model_sha = verify_model_sha256(model)
    session = ort.InferenceSession(str(model), providers=["CPUExecutionProvider"])
    inputs = session.get_inputs()
    if len(inputs) != 1 or tuple(inputs[0].shape) != (1, 3, MODEL_INPUT_SIZE, MODEL_INPUT_SIZE):
        raise ValueError("INVALID_MODEL_INPUT")
    results = {}
    for row in rows:
        frame = cv2.imread(str(row["image_path"]), cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError(f"UNREADABLE_IMAGE:{row['cycle_id']}")
        tensor, transform = letterbox_bgr(frame)
        output = session.run(None, {inputs[0].name: tensor})
        if len(output) != 1:
            raise ValueError("INVALID_MODEL_OUTPUT")
        candidates = decode_yolo11_one_class_candidates(output[0], transform=transform)
        max_score = max((c.confidence for c in candidates), default=0.0)
        results[row["cycle_id"]] = {
            "max_confidence": max_score, "raw_proposal_count": len(candidates),
            "post_nms_counts": {str(t): len(nms_detections(
                [c for c in candidates if c.confidence >= t])) for t in THRESHOLDS},
        }
    return results, model_sha


def summarize(rows: list[dict], scores: dict, *, source: str, model_sha: str | None) -> dict:
    if set(scores) - {r["cycle_id"] for r in rows}:
        raise ValueError("SCORE_NOT_IN_LABELS")
    out = []
    for row in rows:
        score = scores.get(row["cycle_id"])
        maximum = None if score is None else score.get("max_confidence")
        if score is not None and (type(maximum) not in (int, float)
                                  or not math.isfinite(maximum) or not 0 <= maximum <= 1):
            raise ValueError("INVALID_SCORE")
        clean = {k: v for k, v in row.items() if k != "image_path"}
        clean["saved_jpeg_max_confidence"] = maximum
        clean["above_threshold"] = (None if maximum is None else
                                    {str(t): maximum >= t for t in THRESHOLDS})
        if score is not None and "post_nms_counts" in score:
            clean["post_nms_counts"] = score["post_nms_counts"]
            clean["raw_proposal_count"] = score["raw_proposal_count"]
        out.append(clean)
    evaluated = [r for r in out if r["above_threshold"] is not None]
    positive = [r for r in evaluated if r["person_visible"]]
    negative = [r for r in evaluated if not r["person_visible"]]
    def counts(group):
        return {str(t): sum(r["above_threshold"][str(t)] for r in group) for t in THRESHOLDS}
    return {"schema_version": "sie.shadow_rgb_detector_offline.v1",
            "status": "OFFLINE_DIAGNOSTIC_ONLY", "score_source": source,
            "model_sha256": model_sha,
            "metric_measurement_authorized": False, "navigation_eligible": False,
            "actuator_access": False,
            "summary": {"labeled_frames": len(out), "evaluated_frames": len(evaluated),
                        "positive_frames_evaluated": len(positive),
                        "negative_frames_evaluated": len(negative),
                        "positive_frames_above_threshold": counts(positive),
                        "negative_frames_above_threshold": counts(negative),
                        "false_positive_rate": None if not negative else {
                            str(t): counts(negative)[str(t)] / len(negative) for t in THRESHOLDS}},
            "frames": out,
            "limitations": [
                "Scores are from redecoded saved JPEGs, not original in-memory frames.",
                "A high proposal score does not establish person localization or identity; no human boxes were annotated.",
                "This selected positive-heavy sample does not estimate operational recall.",
                "False-positive rate requires annotated frames without a person.",
                "No depth, metric Measurement or motion policy is changed.",
            ]}


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--evidence-root", required=True, type=Path)
    p.add_argument("--labels", required=True, type=Path)
    source = p.add_mutually_exclusive_group(required=True)
    source.add_argument("--model", type=Path, help="verified YOLO11n ONNX model for full replay")
    source.add_argument("--reported-scores", type=Path,
                        help="partial operator-reported saved-JPEG max scores")
    p.add_argument("--output", required=True, type=Path)
    args = p.parse_args()
    if args.output.exists():
        p.error("output exists; choose a fresh path")
    rows = verified_rows(args.evidence_root, json.loads(args.labels.read_text(encoding="utf-8")))
    if args.model:
        results, model_sha = infer_saved_jpegs(rows, args.model)
        kind = "verified_ONNX_replay_on_saved_JPEG"
    else:
        reported = json.loads(args.reported_scores.read_text(encoding="utf-8"))
        if reported.get("schema_version") != "sie.shadow_rgb_operator_scores.v1":
            raise ValueError("INVALID_REPORTED_SCORE_SCHEMA")
        results, model_sha = reported["scores"], None
        kind = "operator_reported_partial_JPEG_scores"
    report = summarize(rows, results, source=kind, model_sha=model_sha)
    report["labels_sha256"] = digest(args.labels)
    report["source_root"] = str(args.evidence_root.resolve())
    if args.reported_scores:
        report["reported_scores_sha256"] = digest(args.reported_scores)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print("SUMMARY", json.dumps(report["summary"]))
    print("Saved:", args.output)


if __name__ == "__main__":
    main()
