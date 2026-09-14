from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from vision_core.person_localization.close_range_locator import (
    CloseRangeDetection,
    OpenCvHaarFaceLocator,
    TemporalFaceTracker,
    build_close_range_target_record,
    default_face_cascade_path,
)


SCRIPT = Path("vision_core/tools/run_ar0234_close_range_locator_preview.py")
spec = importlib.util.spec_from_file_location("ar0234_close_range_locator_preview", SCRIPT)
preview = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = preview
spec.loader.exec_module(preview)


def _record(tracking) -> dict:
    return build_close_range_target_record(
        tracking=tracking,
        evidence_id="close-range-000001",
        captured_at_utc=datetime(2026, 9, 11, tzinfo=timezone.utc),
        image_width=1920,
        image_height=1200,
    )


def _face(x: int, y: int = 120) -> CloseRangeDetection:
    return CloseRangeDetection((x, y, x + 100, y + 140), 0.75)


def test_persistent_face_plus_one_frame_false_detection_is_single_target() -> None:
    tracker = TemporalFaceTracker()
    for index in range(5):
        detections = [_face(1000 + index * 3)]
        if index == 2:
            detections.append(_face(300, 700))
        tracking = tracker.update(detections)
    single = _record(tracking)
    assert single["target_status"] == "SINGLE_TARGET"
    assert single["raw_detection_count"] == 1
    assert single["persistent_track_count"] == 1
    assert single["selected_track_id"] == "face-track-000001"
    assert single["center_x_px"] == 1062.0
    assert single["bbox_xyxy_px"] == [1012, 120, 1112, 260]
    assert single["reference_frame"] == "ar0234_image_frame"
    assert single["units"] == "px"
    assert "pixels" not in single and "frame" not in single
    json.dumps(single, allow_nan=False)


def test_two_persistent_faces_are_multiple_targets() -> None:
    tracker = TemporalFaceTracker()
    for index in range(5):
        tracking = tracker.update([_face(400 + index * 2), _face(1200 + index * 2)])
    multiple = _record(tracking)
    assert multiple["target_status"] == "MULTIPLE_TARGETS"
    assert multiple["raw_detection_count"] == 2
    assert multiple["persistent_track_count"] == 2
    assert multiple["selected_track_id"] is None
    assert multiple["center_x_px"] is None and multiple["confidence"] is None


def test_only_unstable_false_detections_are_no_target() -> None:
    tracker = TemporalFaceTracker()
    for x in (100, 400, 700, 1000, 1300):
        tracking = tracker.update([_face(x, 700)])
    no_target = _record(tracking)
    assert no_target["target_status"] == "NO_TARGET"
    assert no_target["raw_detection_count"] == 1
    assert no_target["persistent_track_count"] == 0
    assert no_target["selected_track_id"] is None
    assert no_target["center_x_px"] is None and no_target["bbox_xyxy_px"] is None


def test_local_bundled_face_cascade_is_available_without_download() -> None:
    cascade = default_face_cascade_path()
    assert cascade.name == "haarcascade_frontalface_default.xml"
    assert cascade.is_file()
    assert OpenCvHaarFaceLocator(cascade).detect(np.zeros((1200, 1920, 3), dtype=np.uint8)) == []


def test_overlay_uses_copy_and_displays_single_no_and_multiple_states() -> None:
    frame = np.zeros((1200, 1920, 3), dtype=np.uint8)
    detection = _face(1000)
    tracker = TemporalFaceTracker()
    for _ in range(3):
        single_tracking = tracker.update([detection])
    for status, raw, tracking, center, confidence in (
        ("SINGLE_TARGET", [detection], single_tracking, 1050.0, 0.75),
        ("NO_TARGET", [], TemporalFaceTracker().update([]), None, None),
        ("MULTIPLE_TARGETS", [detection, _face(1200)], single_tracking, None, None),
    ):
        view = preview.draw_overlay(
            frame,
            target_status=status,
            raw_detections=raw,
            persistent_tracks=tracking.persistent_tracks,
            selected_track_id=("face-track-000001" if status == "SINGLE_TARGET" else None),
            center_x_px=center,
            confidence=confidence,
        )
        assert view.shape == frame.shape
        assert not np.shares_memory(view, frame)
    assert not frame.any()


def test_preview_help_and_stable_by_id_capture_contract_are_available() -> None:
    result = subprocess.run(
        [sys.executable, str(SCRIPT), "--help"], check=True, capture_output=True, text=True
    )
    assert "--preview-width" in result.stdout and "--max-frames" in result.stdout
    source = SCRIPT.read_text(encoding="utf-8")
    assert "device=AR0234_BY_ID" in source
    assert "AR0234CaptureConfig(" in source
    assert "fps=30.0" in source and 'fourcc="MJPG"' in source
