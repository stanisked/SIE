from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import pytest


SCRIPT = Path("vision_core/tools/run_ar0234_alignment_preview.py")
spec = importlib.util.spec_from_file_location("ar0234_alignment_preview", SCRIPT)
preview = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = preview
spec.loader.exec_module(preview)


def intrinsic_document(*, width: int = 1920, height: int = 1200) -> dict:
    return {
        "image": {"width": width, "height": height},
        "camera_matrix": [[2000., 0., 997.365537], [0., 2000., 693.8], [0., 0., 1.]],
    }


@pytest.mark.parametrize(
    ("center", "expected"),
    [(900., "IMAGE_LEFT"), (997.365537, "CENTER_BAND"), (1100., "IMAGE_RIGHT")],
)
def test_pure_image_frame_classification(center: float, expected: str):
    assert preview.classify_image_position(center, 997.365537, 30.) == expected


def test_rolling_median_is_cleared_after_lost_or_multiple():
    rolling = preview.RollingCenters(3)
    center, median = rolling.update("SINGLE_PERSON", [900, 100, 1000, 500])
    assert center == median == 950.
    assert rolling.update("PERSON_LOST", None) == (None, None)
    assert rolling.update("MULTIPLE_PERSONS", None) == (None, None)
    center, median = rolling.update("SINGLE_PERSON", [1000, 100, 1100, 500])
    assert center == median == 1050.


def test_invalid_calibration_and_tolerance_fail_closed(tmp_path: Path):
    source = tmp_path / "invalid.json"
    source.write_text(json.dumps(intrinsic_document(width=1280)))
    with pytest.raises(ValueError, match="1920x1200"):
        preview.load_ar_intrinsic(source)
    source.write_text(json.dumps({"image": {"width": 1920, "height": 1200}, "camera_matrix": [[1, 0], [0, 1]]}))
    with pytest.raises(ValueError, match="3x3"):
        preview.load_ar_intrinsic(source)
    for value in (0., -1., float("nan"), float("inf")):
        with pytest.raises(ValueError, match="finite positive"):
            preview.positive_finite(value, "--center-tolerance-px")


def test_runner_help_is_available_without_opening_camera():
    result = subprocess.run([sys.executable, str(SCRIPT), "--help"], check=True, capture_output=True, text=True)
    for option in ("--model", "--reference", "--ar-intrinsic", "--center-tolerance-px", "--device", "--window-size", "--max-frames"):
        assert option in result.stdout
