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
    build_close_range_target_record,
    default_face_cascade_path,
)


SCRIPT = Path("vision_core/tools/run_ar0234_close_range_locator_preview.py")
spec = importlib.util.spec_from_file_location("ar0234_close_range_locator_preview", SCRIPT)
preview = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = preview
spec.loader.exec_module(preview)


def _record(detections: list[CloseRangeDetection]) -> dict:
    return build_close_range_target_record(
        detections=detections,
        evidence_id="close-range-000001",
        captured_at_utc=datetime(2026, 9, 11, tzinfo=timezone.utc),
        image_width=1920,
        image_height=1200,
    )


def test_close_range_record_is_json_safe_and_fail_closed_by_target_count() -> None:
    detection = CloseRangeDetection((1000, 120, 1100, 260), 0.75)
    single = _record([detection])
    assert single["target_status"] == "SINGLE_TARGET"
    assert single["center_x_px"] == 1050.0
    assert single["bbox_xyxy_px"] == [1000, 120, 1100, 260]
    assert single["reference_frame"] == "ar0234_image_frame"
    assert single["units"] == "px"
    assert "pixels" not in single and "frame" not in single
    json.dumps(single, allow_nan=False)

    no_target = _record([])
    assert no_target["target_status"] == "NO_TARGET"
    assert no_target["center_x_px"] is None and no_target["bbox_xyxy_px"] is None

    multiple = _record([detection, CloseRangeDetection((1200, 150, 1300, 290), 0.7)])
    assert multiple["target_status"] == "MULTIPLE_TARGETS"
    assert multiple["center_x_px"] is None and multiple["confidence"] is None


def test_local_bundled_face_cascade_is_available_without_download() -> None:
    cascade = default_face_cascade_path()
    assert cascade.name == "haarcascade_frontalface_default.xml"
    assert cascade.is_file()
    assert OpenCvHaarFaceLocator(cascade).detect(np.zeros((1200, 1920, 3), dtype=np.uint8)) == []


def test_overlay_uses_copy_and_displays_single_no_and_multiple_states() -> None:
    frame = np.zeros((1200, 1920, 3), dtype=np.uint8)
    detection = CloseRangeDetection((1000, 120, 1100, 260), 0.75)
    for status, detections, center, confidence in (
        ("SINGLE_TARGET", [detection], 1050.0, 0.75),
        ("NO_TARGET", [], None, None),
        ("MULTIPLE_TARGETS", [detection, CloseRangeDetection((1200, 150, 1300, 290), 0.7)], None, None),
    ):
        view = preview.draw_overlay(
            frame,
            target_status=status,
            detections=detections,
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
