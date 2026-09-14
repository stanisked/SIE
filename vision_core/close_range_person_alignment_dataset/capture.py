"""Append-only, raw AR0234 capture for manual upper-body annotation.

No detector, model inference, network client, depth, or motion code belongs in
this module. It stores lossless PNG images only after the operator presses
SPACE or S in the capture window.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import tempfile
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

import cv2
import numpy as np

from vision_core.person_localization.ar0234 import AR0234_BY_ID, AR0234Capture, AR0234CaptureConfig


DATASET_ID = "ar0234_close_range_person_alignment_v1"
DATASET_SCHEMA_VERSION = "sie.ar0234_close_range_person_alignment_dataset.v1"
SESSION_SCHEMA_VERSION = "sie.ar0234_close_range_person_alignment_session.v1"
CAPTURE_RECORD_SCHEMA_VERSION = "sie.ar0234_close_range_person_alignment_capture.v1"
CLASS_NAME = "person_upper_body"
FRAME_WIDTH = 1920
FRAME_HEIGHT = 1200
FRAME_FPS = 30.0
FRAME_FOURCC = "MJPG"
FRAME_BUFFER_SIZE = 1
SESSION_ID_PATTERN = re.compile(r"[a-z0-9][a-z0-9_-]{0,63}\Z")


class DatasetCaptureError(RuntimeError):
    """Raised when the append-only capture dataset contract is violated."""


@dataclass(frozen=True)
class SessionTags:
    distance_band: str
    pose: str
    lateral_position: str
    lighting: str
    scene_type: str
    contains_person: bool

    def __post_init__(self) -> None:
        for field in (self.distance_band, self.pose, self.lateral_position, self.lighting, self.scene_type):
            if type(field) is not str or not field.strip():
                raise DatasetCaptureError("every session tag must be a non-empty string")
        if type(self.contains_person) is not bool:
            raise DatasetCaptureError("contains_person must be boolean")


def capture_config() -> AR0234CaptureConfig:
    """Return the one approved AR0234 capture configuration for this dataset."""
    return AR0234CaptureConfig(
        device=AR0234_BY_ID,
        width=FRAME_WIDTH,
        height=FRAME_HEIGHT,
        fps=FRAME_FPS,
        fourcc=FRAME_FOURCC,
        buffer_size=FRAME_BUFFER_SIZE,
    )


def validate_stable_device_path(device: Path) -> Path:
    if device != AR0234_BY_ID:
        raise DatasetCaptureError("capture requires the exact approved AR0234 by-id path")
    return device


def prepare_dataset_root(root: Path) -> dict[str, Path]:
    """Create the fixed, empty-or-append-only directory layout."""
    _validate_output_root(root)
    paths = _paths(root)
    for directory in (paths["raw"], paths["images"], paths["labels"], paths["manifests"], paths["splits"], paths["reports"]):
        directory.mkdir(parents=True, exist_ok=True)
    classes = paths["classes"]
    if classes.exists():
        if classes.is_symlink() or classes.read_text(encoding="utf-8") != f"{CLASS_NAME}\n":
            raise DatasetCaptureError("classes.txt conflicts with the one-class dataset contract")
    else:
        _exclusive_write(classes, f"{CLASS_NAME}\n".encode("utf-8"))
    dataset_manifest = paths["dataset_manifest"]
    if not dataset_manifest.exists():
        _exclusive_json(
            dataset_manifest,
            {
                "schema_version": DATASET_SCHEMA_VERSION,
                "dataset_id": DATASET_ID,
                "class_names": [CLASS_NAME],
                "image_storage": "lossless_png_raw_with_images_hardlinks",
            },
        )
    return paths


def create_session_metadata(
    root: Path,
    *,
    session_id: str,
    tags: SessionTags,
    intrinsic_path: Path,
    now: Callable[[], datetime] = lambda: datetime.now(timezone.utc),
) -> Path:
    """Create immutable capture-session metadata before opening the camera."""
    paths = prepare_dataset_root(root)
    _validate_session_id(session_id)
    if intrinsic_path.is_symlink() or not intrinsic_path.is_file():
        raise DatasetCaptureError("--ar-intrinsic must name an existing regular calibration file")
    session_path = paths["manifests"] / f"{session_id}.json"
    if session_path.exists() or session_path.is_symlink():
        raise DatasetCaptureError(f"session_id already exists: {session_id}")
    payload = {
        "schema_version": SESSION_SCHEMA_VERSION,
        "dataset_id": DATASET_ID,
        "session_id": session_id,
        "created_at_utc": _utc(now()),
        "camera_stable_by_id": str(AR0234_BY_ID),
        "requested_capture_settings": {
            "device": str(AR0234_BY_ID),
            "width": FRAME_WIDTH,
            "height": FRAME_HEIGHT,
            "fps": FRAME_FPS,
            "fourcc": FRAME_FOURCC,
            "buffer_size": FRAME_BUFFER_SIZE,
        },
        "image": {"width": FRAME_WIDTH, "height": FRAME_HEIGHT, "format": "PNG"},
        "ar0234_intrinsic_calibration": {"path": str(intrinsic_path), "sha256": _sha256(intrinsic_path)},
        "capture_purpose": "future_manual_annotation_person_upper_body_close_range_alignment",
        "session_tags": asdict(tags),
    }
    _assert_json_safe(payload)
    _exclusive_json(session_path, payload)
    return session_path


def next_frame_filename(root: Path, session_id: str) -> str:
    _validate_session_id(session_id)
    paths = _paths(root)
    pattern = re.compile(rf"{re.escape(session_id)}_(\d{{6}})\.png\Z")
    numbers: list[int] = []
    for directory in (paths["raw"], paths["images"]):
        if directory.exists():
            numbers += [int(match.group(1)) for entry in directory.iterdir() if (match := pattern.fullmatch(entry.name))]
    return f"{session_id}_{(max(numbers) if numbers else 0) + 1:06d}.png"


def save_raw_frame(root: Path, *, session_id: str, frame_bgr: np.ndarray, captured_at_utc: datetime) -> dict[str, Any]:
    """Write one lossless frame and expose the same inode through images/ for LabelImg."""
    paths = prepare_dataset_root(root)
    _validate_session_id(session_id)
    if not (paths["manifests"] / f"{session_id}.json").is_file():
        raise DatasetCaptureError("capture session metadata must exist before saving frames")
    _validate_frame(frame_bgr)
    filename = next_frame_filename(root, session_id)
    raw_path, image_path = paths["raw"] / filename, paths["images"] / filename
    if raw_path.exists() or image_path.exists() or raw_path.is_symlink() or image_path.is_symlink():
        raise DatasetCaptureError(f"refusing overwrite: {filename}")
    ok, encoded = cv2.imencode(".png", frame_bgr)
    if not ok:
        raise DatasetCaptureError("PNG encoding failed")
    image_bytes = encoded.tobytes()
    _exclusive_write(raw_path, image_bytes)
    try:
        os.link(raw_path, image_path)
    except BaseException as error:
        # The raw evidence survives a failed hardlink and must not be deleted.
        raise DatasetCaptureError(f"could not expose raw frame in images/: {image_path}") from error
    record = {
        "schema_version": CAPTURE_RECORD_SCHEMA_VERSION,
        "dataset_id": DATASET_ID,
        "session_id": session_id,
        "image_filename": filename,
        "image_sha256": hashlib.sha256(image_bytes).hexdigest(),
        "png_bytes": len(image_bytes),
        "captured_at_utc": _utc(captured_at_utc),
    }
    _assert_json_safe(record)
    _append_jsonl(paths["capture_records"], record)
    return record


def capture_interactively(root: Path, *, session_id: str) -> None:
    """Open AR0234 only when an operator explicitly invokes the capture CLI."""
    paths = prepare_dataset_root(root)
    if not (paths["manifests"] / f"{session_id}.json").is_file():
        raise DatasetCaptureError("capture session metadata is missing")
    capture = AR0234Capture(capture_config())
    window_name = "AR0234 close-range person alignment capture"
    try:
        capture.open()
        try:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        except cv2.error as error:
            raise DatasetCaptureError(f"cannot create capture window: {error}") from error
        while True:
            frame = capture.read()
            try:
                cv2.imshow(window_name, frame)
                key = cv2.waitKey(1) & 0xFF
            except cv2.error as error:
                raise DatasetCaptureError(f"capture window failed: {error}") from error
            if key in (ord("q"), ord("Q"), 27):
                return
            if key in (ord(" "), ord("s"), ord("S")):
                record = save_raw_frame(root, session_id=session_id, frame_bgr=frame, captured_at_utc=datetime.now(timezone.utc))
                print(f"SAVED {record['image_filename']}", flush=True)
    finally:
        capture.close()
        cv2.destroyAllWindows()


def _paths(root: Path) -> dict[str, Path]:
    return {"root": root, "raw": root / "raw", "images": root / "images", "labels": root / "labels", "manifests": root / "manifests", "splits": root / "splits", "reports": root / "reports", "classes": root / "classes.txt", "dataset_manifest": root / "manifests" / "dataset_manifest.json", "capture_records": root / "manifests" / "capture_records.jsonl"}


def _validate_output_root(root: Path) -> None:
    if not root.is_absolute() or root.is_symlink():
        raise DatasetCaptureError("--output-root must be an absolute non-symlink directory path")


def _validate_session_id(session_id: str) -> None:
    if type(session_id) is not str or not SESSION_ID_PATTERN.fullmatch(session_id):
        raise DatasetCaptureError("session_id must match [a-z0-9][a-z0-9_-]{0,63}")


def _validate_frame(frame_bgr: object) -> None:
    if not isinstance(frame_bgr, np.ndarray) or frame_bgr.dtype != np.uint8 or frame_bgr.shape != (FRAME_HEIGHT, FRAME_WIDTH, 3):
        raise DatasetCaptureError("frame must be a uint8 1920x1200 BGR image")


def _utc(value: datetime) -> str:
    if not isinstance(value, datetime) or value.tzinfo is None or value.utcoffset() is None:
        raise DatasetCaptureError("timestamp must be timezone-aware")
    return value.astimezone(timezone.utc).isoformat()


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _exclusive_json(path: Path, value: object) -> None:
    _exclusive_write(path, (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode("utf-8"))


def _exclusive_write(path: Path, payload: bytes) -> None:
    if path.exists() or path.is_symlink():
        raise DatasetCaptureError(f"refusing overwrite: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.link(temporary, path)
    except FileExistsError as error:
        raise DatasetCaptureError(f"refusing overwrite: {path}") from error
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def _append_jsonl(path: Path, record: dict[str, Any]) -> None:
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(record, sort_keys=True, allow_nan=False) + "\n")
        handle.flush()
        os.fsync(handle.fileno())


def _assert_json_safe(value: object) -> None:
    try:
        json.dumps(value, allow_nan=False)
    except (TypeError, ValueError) as error:
        raise DatasetCaptureError("metadata must be JSON-safe") from error
