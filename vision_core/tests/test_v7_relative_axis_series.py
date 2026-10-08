"""Regression tests for the OV9281 v7 relative-axis diagnostic analyzer."""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from vision_core.tools import analyze_ov9281_pi_depth_v7_relative_axis_series as analyzer


def point(position_m: float, depth_m: float, *, sha256: str = "a" * 64) -> dict:
    return {
        "schema_version": analyzer.SOURCE_SCHEMA,
        "status": "DIAGNOSTIC_NOT_ACTIVATED",
        "validation_mode": "relative_axis_translation",
        "depth_reference_frame": "rectified_left_optical_frame",
        "calibration": {"sha256": sha256, "path": "/tmp/v7.npz"},
        "stereo_depth_m": {"median_m": depth_m, "mad_m": 0.01},
        "relative_axis_reference": {
            "axis_reference_id": "physical_left_lens_front_rim_axis",
            "board_plane_axis_position_m": position_m,
            "standard_uncertainty_m": 0.002,
        },
    }


class RelativeAxisSeriesTest(unittest.TestCase):
    def write(self, directory: Path, name: str, payload: dict) -> Path:
        path = directory / name
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    def test_load_point_accepts_declared_relative_datum(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            path = self.write(Path(raw), "point.json", point(1.5, 1.49))
            result = analyzer.load_point(path)
        self.assertEqual(result["axis_reference_id"], "physical_left_lens_front_rim_axis")
        self.assertEqual(result["median_depth_m"], 1.49)

    def test_load_point_rejects_absolute_report(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            payload = point(1.5, 1.49)
            payload["validation_mode"] = "absolute_optical_z"
            path = self.write(Path(raw), "point.json", payload)
            with self.assertRaisesRegex(ValueError, "not relative_axis_translation"):
                analyzer.load_point(path)

    def test_main_compares_translation_without_optical_offset(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            directory = Path(raw)
            first = self.write(directory, "first.json", point(1.5, 1.50))
            second = self.write(directory, "second.json", point(2.0, 2.05))
            output = directory / "series.json"
            argv = [
                "analyze_ov9281_pi_depth_v7_relative_axis_series.py",
                "--report", str(first),
                "--report", str(second),
                "--output", str(output),
                "--max-relative-translation-error", "0.05",
            ]
            with patch.object(sys, "argv", argv):
                self.assertEqual(analyzer.main(), 0)
            result = json.loads(output.read_text(encoding="utf-8"))
        delta = result["translation_deltas"][0]
        self.assertAlmostEqual(delta["physical_axis_translation_m"], 0.50)
        self.assertAlmostEqual(delta["stereo_depth_translation_m"], 0.55)
        self.assertAlmostEqual(delta["absolute_relative_translation_error"], 0.10)
        self.assertEqual(delta["declared_criterion"]["status"], "FAIL")


if __name__ == "__main__":
    unittest.main()
