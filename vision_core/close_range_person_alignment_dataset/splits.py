"""Deterministic, session-isolated train/val/test split for frozen labels."""

from __future__ import annotations

import hashlib
import json
import math
from collections import Counter, defaultdict
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .annotation import _load_context
from .capture import CLASS_NAME, DATASET_ID, DatasetCaptureError
from .labels import _validate_label_file


SPLIT_SCHEMA_VERSION = "sie.ar0234_close_range_person_alignment_session_split.v1"
SPLIT_NAMES = ("train", "val", "test")


@dataclass(frozen=True)
class SessionSummary:
    session_id: str
    image_count: int
    contains_person: bool
    lighting: str


def select_session_assignment(sessions: list[SessionSummary], *, seed: int) -> dict[str, str]:
    """Select held-out sessions deterministically while preserving key strata."""
    if seed < 0:
        raise DatasetCaptureError("split seed must be non-negative")
    positive_day = sorted((s for s in sessions if s.contains_person and s.lighting == "daylight"), key=lambda s: _rank(seed, s.session_id))
    positive_artificial = sorted((s for s in sessions if s.contains_person and s.lighting == "artificial"), key=lambda s: _rank(seed, s.session_id))
    negatives = sorted((s for s in sessions if not s.contains_person), key=lambda s: _rank(seed, s.session_id))
    if len(positive_day) < 2 or len(positive_artificial) < 2 or len(negatives) < 2:
        raise DatasetCaptureError("session strata cannot provide positive daylight/artificial and negative evidence to all splits")
    total = sum(session.image_count for session in sessions)
    target = {"train": total * 0.70, "val": total * 0.15, "test": total * 0.15}
    best: tuple[tuple[float, tuple[str, ...]], dict[str, str]] | None = None
    for val_day in positive_day:
        for val_artificial in positive_artificial:
            for test_day in positive_day:
                if test_day == val_day:
                    continue
                for test_artificial in positive_artificial:
                    if test_artificial == val_artificial:
                        continue
                    for val_negative in negatives:
                        for test_negative in negatives:
                            if test_negative == val_negative:
                                continue
                            assignment = {session.session_id: "train" for session in sessions}
                            for session in (val_day, val_artificial, val_negative):
                                assignment[session.session_id] = "val"
                            for session in (test_day, test_artificial, test_negative):
                                assignment[session.session_id] = "test"
                            counts = Counter()
                            for session in sessions:
                                counts[assignment[session.session_id]] += session.image_count
                            score = sum((counts[name] - target[name]) ** 2 for name in SPLIT_NAMES)
                            tie_break = tuple(sorted((f"{_rank(seed, name)}:{name}", split) for name, split in assignment.items()))
                            candidate = ((score, tie_break), assignment)
                            if best is None or candidate[0] < best[0]:
                                best = candidate
    if best is None:
        raise DatasetCaptureError("no session-isolated split assignment exists")
    return best[1]


def create_session_split(
    root: Path,
    *,
    inventory_path: Path,
    expected_source_snapshot_id: str,
    seed: int,
) -> dict[str, Any]:
    context = _load_context(root, root / "annotations_labelme", inventory_path)
    if context["snapshot_id"] != expected_source_snapshot_id:
        raise DatasetCaptureError("frozen source snapshot ID does not match the explicit expected value")
    sessions, images_by_session, tags_by_session = _session_data(root, context)
    _validate_training_labels(context, images_by_session)
    assignment = select_session_assignment(sessions, seed=seed)
    artifacts = _artifact_paths(context["paths"]["splits"])
    if any(path.exists() or path.is_symlink() for path in artifacts.values()):
        raise DatasetCaptureError("split artifacts already exist; refusing overwrite")
    split_images = {
        split: sorted(
            str(context["paths"]["images"] / filename)
            for session_id, files in images_by_session.items()
            if assignment[session_id] == split
            for filename in files
        )
        for split in SPLIT_NAMES
    }
    for split, path in ((name, artifacts[f"{name}_list"]) for name in SPLIT_NAMES):
        path.write_text("\n".join(split_images[split]) + "\n", encoding="utf-8")
    artifacts["dataset_yaml"].write_text(_dataset_yaml(root), encoding="utf-8")
    split_stats = _split_stats(assignment, images_by_session, tags_by_session)
    _assert_split_coverage(split_stats)
    manifest = {
        "schema_version": SPLIT_SCHEMA_VERSION,
        "dataset_id": DATASET_ID,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "source_snapshot_id": context["snapshot_id"],
        "class_mapping": {"0": CLASS_NAME},
        "strategy": "deterministic_session_stratified_pair_holdout_v1",
        "seed": seed,
        "session_assignment": dict(sorted(assignment.items())),
        "splits": split_stats,
        "split_file_sha256": {
            path.name: hashlib.sha256(path.read_bytes()).hexdigest()
            for key, path in artifacts.items()
            if key != "manifest" and key != "report"
        },
        "training_not_performed": True,
        "generalization_claim": "INTERNAL_HELD_OUT_SINGLE_CAPTURE_ENVIRONMENT_ONLY",
    }
    _write_json_new(artifacts["manifest"], manifest)
    artifacts["report"].write_text(_report_markdown(manifest), encoding="utf-8")
    validation = validate_session_split(root, inventory_path=inventory_path, manifest_path=artifacts["manifest"])
    if validation["status"] != "VALID":
        raise DatasetCaptureError("generated split did not validate")
    manifest["validation"] = validation
    _json_safe(manifest)
    return manifest


def validate_session_split(root: Path, *, inventory_path: Path, manifest_path: Path) -> dict[str, Any]:
    context = _load_context(root, root / "annotations_labelme", inventory_path)
    if manifest_path.is_symlink() or not manifest_path.is_file():
        raise DatasetCaptureError("split manifest is missing or unsafe")
    manifest = _strict_json(manifest_path)
    if manifest.get("schema_version") != SPLIT_SCHEMA_VERSION or manifest.get("dataset_id") != DATASET_ID:
        raise DatasetCaptureError("split manifest identity mismatch")
    if manifest.get("source_snapshot_id") != context["snapshot_id"]:
        raise DatasetCaptureError("split manifest snapshot mismatch")
    assignment = manifest.get("session_assignment")
    if not isinstance(assignment, dict):
        raise DatasetCaptureError("split session assignment is invalid")
    sessions, images_by_session, tags_by_session = _session_data(root, context)
    if set(assignment) != {session.session_id for session in sessions} or set(assignment.values()) - set(SPLIT_NAMES):
        raise DatasetCaptureError("split session assignment does not cover sessions exactly once")
    _validate_training_labels(context, images_by_session)
    artifacts = _artifact_paths(context["paths"]["splits"])
    seen_images: set[str] = set()
    seen_sessions: dict[str, str] = {}
    for split in SPLIT_NAMES:
        path = artifacts[f"{split}_list"]
        if path.is_symlink() or not path.is_file():
            raise DatasetCaptureError(f"missing split list: {split}")
        expected = {
            str(context["paths"]["images"] / filename)
            for session_id, files in images_by_session.items()
            if assignment[session_id] == split
            for filename in files
        }
        listed = [line for line in path.read_text(encoding="utf-8").splitlines() if line]
        if len(listed) != len(set(listed)) or set(listed) != expected:
            raise DatasetCaptureError(f"split image list mismatch: {split}")
        for image_name in listed:
            image = Path(image_name)
            if image.is_symlink() or not image.is_file():
                raise DatasetCaptureError(f"split image is missing or unsafe: {image_name}")
            filename = image.name
            session_id = context["records"][filename]["session_id"]
            if session_id in seen_sessions and seen_sessions[session_id] != split:
                raise DatasetCaptureError("session intersects multiple splits")
            seen_sessions[session_id] = split
            if image_name in seen_images:
                raise DatasetCaptureError("image intersects multiple splits")
            seen_images.add(image_name)
    expected_images = {str(path) for path in context["image_paths"].values()}
    if seen_images != expected_images:
        raise DatasetCaptureError("all dataset images must be assigned exactly once")
    expected_hashes = manifest.get("split_file_sha256")
    actual_hashes = {
        path.name: hashlib.sha256(path.read_bytes()).hexdigest()
        for key, path in artifacts.items()
        if key not in {"manifest", "report"}
    }
    if expected_hashes != actual_hashes:
        raise DatasetCaptureError("split file SHA-256 mismatch")
    split_stats = _split_stats(assignment, images_by_session, tags_by_session)
    _assert_split_coverage(split_stats)
    result = {
        "schema_version": SPLIT_SCHEMA_VERSION,
        "dataset_id": DATASET_ID,
        "status": "VALID",
        "source_snapshot_id": context["snapshot_id"],
        "all_images_assigned_once": len(seen_images),
        "session_assignment": dict(sorted(seen_sessions.items())),
        "splits": split_stats,
    }
    _json_safe(result)
    return result


def _session_data(root: Path, context: dict[str, Any]) -> tuple[list[SessionSummary], dict[str, list[str]], dict[str, dict[str, Any]]]:
    by_session: dict[str, list[str]] = defaultdict(list)
    for filename, record in context["records"].items():
        by_session[record["session_id"]].append(filename)
    tags: dict[str, dict[str, Any]] = {}
    sessions: list[SessionSummary] = []
    for session_id, filenames in by_session.items():
        payload = _strict_json(root / "manifests" / f"{session_id}.json")
        session_tags = payload.get("session_tags")
        if not isinstance(session_tags, dict) or type(session_tags.get("contains_person")) is not bool or session_tags.get("lighting") not in {"daylight", "artificial"}:
            raise DatasetCaptureError(f"session tags are invalid: {session_id}")
        tags[session_id] = session_tags
        sessions.append(SessionSummary(session_id, len(filenames), session_tags["contains_person"], session_tags["lighting"]))
    return sorted(sessions, key=lambda session: session.session_id), by_session, tags


def _validate_training_labels(context: dict[str, Any], images_by_session: dict[str, list[str]]) -> None:
    for session_id, filenames in images_by_session.items():
        for filename in filenames:
            label = context["paths"]["labels"] / f"{Path(filename).stem}.txt"
            if context["contains_person"][session_id]:
                if label.is_symlink() or not label.is_file():
                    raise DatasetCaptureError(f"positive image missing YOLO label: {filename}")
                lines = [line for line in label.read_text(encoding="utf-8").splitlines() if line]
                if len(lines) != 1:
                    raise DatasetCaptureError(f"positive image must have exactly one YOLO bbox: {filename}")
                _validate_label_file(label)
            elif label.exists() or label.is_symlink():
                raise DatasetCaptureError(f"negative image must not have a YOLO label: {filename}")


def _split_stats(assignment: dict[str, str], images_by_session: dict[str, list[str]], tags: dict[str, dict[str, Any]]) -> dict[str, dict[str, Any]]:
    stats: dict[str, dict[str, Any]] = {}
    for split in SPLIT_NAMES:
        session_ids = sorted(session_id for session_id, assigned in assignment.items() if assigned == split)
        images = sum(len(images_by_session[session_id]) for session_id in session_ids)
        positive = sum(len(images_by_session[session_id]) for session_id in session_ids if tags[session_id]["contains_person"])
        lighting = Counter(tags[session_id]["lighting"] for session_id in session_ids)
        stats[split] = {
            "session_ids": session_ids,
            "image_count": images,
            "positive_image_count": positive,
            "negative_image_count": images - positive,
            "lighting_session_count": dict(sorted(lighting.items())),
        }
    return stats


def _assert_split_coverage(stats: dict[str, dict[str, Any]]) -> None:
    for split, data in stats.items():
        if data["positive_image_count"] <= 0 or data["negative_image_count"] <= 0:
            raise DatasetCaptureError(f"split lacks positive or negative evidence: {split}")
        if set(data["lighting_session_count"]) != {"daylight", "artificial"}:
            raise DatasetCaptureError(f"split lacks daylight or artificial session evidence: {split}")


def _artifact_paths(splits: Path) -> dict[str, Path]:
    return {
        "train_list": splits / "train.txt",
        "val_list": splits / "val.txt",
        "test_list": splits / "test.txt",
        "dataset_yaml": splits / "dataset.yaml",
        "manifest": splits / "SPLIT_MANIFEST_v1.json",
        "report": splits / "SPLIT_REPORT_v1.md",
    }


def _dataset_yaml(root: Path) -> str:
    return f"path: {root}\ntrain: splits/train.txt\nval: splits/val.txt\ntest: splits/test.txt\nnames:\n  0: {CLASS_NAME}\n"


def _report_markdown(manifest: dict[str, Any]) -> str:
    rows = "\n".join(
        f"| {name} | {data['image_count']} | {data['positive_image_count']} | {data['negative_image_count']} | {', '.join(data['session_ids'])} |"
        for name, data in manifest["splits"].items()
    )
    return f"""# Session-isolated split v1\n\nSource snapshot: `{manifest['source_snapshot_id']}`. Strategy: `{manifest['strategy']}`, seed `{manifest['seed']}`.\n\n| Split | Images | Positive | Negative | Sessions |\n| --- | ---: | ---: | ---: | --- |\n{rows}\n\nLists reference existing `images/` paths; no image is copied. This is an internal held-out split from one capture environment. It is not evidence of generalization, training quality, yaw qualification or motion safety.\n"""


def _rank(seed: int, session_id: str) -> str:
    return hashlib.sha256(f"{seed}:{session_id}".encode("ascii")).hexdigest()


def _write_json_new(path: Path, payload: dict[str, Any]) -> None:
    encoded = json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n"
    with path.open("x", encoding="utf-8") as stream:
        stream.write(encoded)


def _strict_json(path: Path) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise DatasetCaptureError(f"required JSON is missing or unsafe: {path.name}")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"), parse_constant=lambda value: (_ for _ in ()).throw(ValueError(f"non-finite JSON: {value}")))
    except (OSError, json.JSONDecodeError) as error:
        raise DatasetCaptureError(f"invalid JSON: {path.name}") from error
    if not isinstance(payload, dict):
        raise DatasetCaptureError(f"JSON root must be object: {path.name}")
    return payload


def _json_safe(payload: object) -> None:
    try:
        json.dumps(payload, allow_nan=False)
    except (TypeError, ValueError) as error:
        raise DatasetCaptureError("split output is not JSON-safe") from error
