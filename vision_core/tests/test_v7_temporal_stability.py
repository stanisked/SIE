"""Regression tests for the offline OV9281 v7 temporal diagnostic."""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from vision_core.tools import analyze_ov9281_pi_depth_v7_temporal_stability as analyzer


def report() -> dict:
    frames = []
    for index, (depth, disparity, pnp_z) in enumerate(
        ((4.00, 10.00, 4.01), (3.99, 10.10, 4.01), (3.98, 10.20, 4.01))
    ):
        frames.append(
            {
                "captured_at_utc": f"2026-10-08T12:00:0{index}+00:00",
                "median_depth_m": depth,
                "median_disparity_px": disparity,
                "board_pose_capture_gate": {"board_center_m": [0.0, 0.0, pnp_z]},
            }
        )
    return {
        "schema_version": analyzer.SOURCE_SCHEMA,
        "status": "DIAGNOSTIC_NOT_ACTIVATED",
        "validation_mode": "relative_axis_translation",
        "depth_reference_frame": "rectified_left_optical_frame",
        "calibration": {"sha256": "a" * 64},
        "relative_axis_reference": {
            "axis_reference_id": "physical_left_lens_front_rim_axis_v1",
            "board_plane_axis_position_m": 4.0,
        },
        "capture": {"warmup_frames": 180},
        "per_frame": frames,
    }


class TemporalStabilityTest(unittest.TestCase):
    def write(self, directory: Path, payload: dict) -> Path:
        path = directory / "run.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    def test_load_run_reports_disparity_drift_with_stable_pnp(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            result = analyzer.load_run(self.write(Path(raw), report()))
        depth = result["series"]["stereo_depth_m"]
        disparity = result["series"]["median_disparity_px"]
        pnp = result["series"]["pnp_board_center_z_m"]
        self.assertAlmostEqual(depth["first_to_last_delta"], -0.02)
        self.assertAlmostEqual(disparity["first_to_last_delta"], 0.20)
        self.assertAlmostEqual(pnp["peak_to_peak"], 0.0)
        self.assertAlmostEqual(result["correlations"]["stereo_depth_vs_median_disparity"], -1.0)
        self.assertIsNone(result["correlations"]["stereo_depth_vs_pnp_board_center_z"])

    def test_main_rejects_mixed_calibration_hashes(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            directory = Path(raw)
            first = self.write(directory, report())
            second_payload = report()
            second_payload["calibration"]["sha256"] = "b" * 64
            second = directory / "second.json"
            second.write_text(json.dumps(second_payload), encoding="utf-8")
            argv = [
                "analyze_ov9281_pi_depth_v7_temporal_stability.py",
                "--report", str(first),
                "--report", str(second),
                "--output", str(directory / "output.json"),
            ]
            with patch.object(sys, "argv", argv):
                with self.assertRaisesRegex(ValueError, "one declared calibration"):
                    analyzer.main()


if __name__ == "__main__":
    unittest.main()
