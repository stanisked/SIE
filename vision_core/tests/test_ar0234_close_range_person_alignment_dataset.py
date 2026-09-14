from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import pytest

from vision_core.close_range_person_alignment_dataset.capture import (
    AR0234_BY_ID,
    DatasetCaptureError,
    SessionTags,
    create_session_metadata,
    next_frame_filename,
    prepare_dataset_root,
    save_raw_frame,
    validate_stable_device_path,
)


NOW = datetime(2026, 9, 14, tzinfo=timezone.utc)


def _intrinsic(tmp_path: Path) -> Path:
    path = tmp_path / "ar_intrinsic.json"
    path.write_text('{"K": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}\n', encoding="utf-8")
    return path


def _tags() -> SessionTags:
    return SessionTags("1.5-2.0m", "standing_front", "center", "daylight", "indoor", True)


def _frame() -> np.ndarray:
    return np.zeros((1200, 1920, 3), dtype=np.uint8)


def test_only_exact_approved_stable_path_is_accepted() -> None:
    assert validate_stable_device_path(AR0234_BY_ID) == AR0234_BY_ID
    with pytest.raises(DatasetCaptureError):
        validate_stable_device_path(Path("/dev/video4"))


def test_session_metadata_is_json_safe_and_has_no_raw_pixels(tmp_path: Path) -> None:
    root = tmp_path / "ar0234_close_range_person_alignment_v1"
    metadata_path = create_session_metadata(root, session_id="session-a", tags=_tags(), intrinsic_path=_intrinsic(tmp_path), now=lambda: NOW)
    payload = json.loads(metadata_path.read_text(encoding="utf-8"))
    assert payload["dataset_id"] == "ar0234_close_range_person_alignment_v1"
    assert payload["camera_stable_by_id"] == str(AR0234_BY_ID)
    assert "pixels" not in json.dumps(payload).lower()
    json.dumps(payload, allow_nan=False)


def test_sequential_filename_and_no_overwrite(tmp_path: Path) -> None:
    root = tmp_path / "ar0234_close_range_person_alignment_v1"
    create_session_metadata(root, session_id="session-a", tags=_tags(), intrinsic_path=_intrinsic(tmp_path), now=lambda: NOW)
    first = save_raw_frame(root, session_id="session-a", frame_bgr=_frame(), captured_at_utc=NOW)
    assert first["image_filename"] == "session-a_000001.png"
    assert next_frame_filename(root, "session-a") == "session-a_000002.png"
    assert (root / "raw" / first["image_filename"]).samefile(root / "images" / first["image_filename"])
    (root / "images" / "session-a_000002.png").write_bytes(b"occupied")
    second = save_raw_frame(root, session_id="session-a", frame_bgr=_frame(), captured_at_utc=NOW)
    assert second["image_filename"] == "session-a_000003.png"
    assert (root / "images" / "session-a_000002.png").read_bytes() == b"occupied"


def test_prepare_layout_has_labelimg_directories_and_one_class(tmp_path: Path) -> None:
    paths = prepare_dataset_root(tmp_path / "ar0234_close_range_person_alignment_v1")
    assert all(paths[name].is_dir() for name in ("raw", "images", "labels", "splits", "reports", "manifests"))
    assert paths["classes"].read_text(encoding="utf-8") == "person_upper_body\n"
