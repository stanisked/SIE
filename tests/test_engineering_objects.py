import unittest

import numpy as np

from sie_core.api.vision_api import VisionAPI
from sie_core.depth.roi_depth import ROIDepthEstimator
from sie_core.geometry.transforms import GeometryEngine
from sie_core.quality.metrics import QualityEngine
from vision_core.measurement import Measurement
from vision_core.observation import Observation
from vision_core.quality import QualityReport
from vision_core.result import VisionFrameResult


class EngineeringObjectsTest(unittest.TestCase):
    def test_depth_measurement_returns_sie_engineering_objects(self):
        class StereoStub:
            def compute(self, left, right):
                return np.full((24, 24), 16.0)

        class DepthStub:
            def compute(self, disparity):
                return np.full_like(disparity, 2.0, dtype=np.float64)

        api = VisionAPI(
            StereoStub(),
            DepthStub(),
            GeometryEngine(fx=500, fy=500, cx=3, cy=3),
            QualityEngine(),
        )

        result = api.process_depth_measurement(
            np.zeros((24, 24, 3)),
            np.zeros((24, 24, 3)),
            ROIDepthEstimator(kernel_size=6),
            reference_frame="camera_left",
        )

        self.assertIn("observation", result)
        self.assertIn("measurement", result)
        self.assertIn("evidence", result)
        self.assertIn("debug", result)

        measurement = result["measurement"]
        self.assertEqual(measurement["measurement_type"], "roi_depth")
        self.assertEqual(measurement["status"], "VALID")
        self.assertEqual(measurement["unit"], "m")
        self.assertEqual(measurement["reference_frame"], "camera_left")
        self.assertEqual(measurement["value"], 2.0)
        self.assertGreater(measurement["confidence"], 0.0)
        self.assertTrue(measurement["quality"]["valid"])
        self.assertEqual(measurement["quality"]["valid_points"], 144)
        self.assertEqual(measurement["quality"]["roi_size_px"], (12, 12))
        self.assertEqual(measurement["quality"]["depth_std_mm"], 0.0)

        observation = result["observation"]
        self.assertEqual(observation["source"], "vision_core")
        self.assertEqual(observation["observation_type"], "depth_observation")
        self.assertEqual(observation["payload"]["reference_frame"], "camera_left")
        self.assertEqual(observation["payload"]["unit"], "m")
        self.assertEqual(observation["evidence_ids"], ["evidence.stereo_frame.current"])
        self.assertEqual(measurement["source_observations"], [observation["observation_id"]])

        evidence = result["evidence"][0]
        self.assertEqual(evidence["source"], "vision_core")
        self.assertEqual(evidence["kind"], "stereo_depth_pipeline")

    def test_vision_core_contract_classes_are_es004_shaped(self):
        observation = Observation.from_depth_pipeline(
            disparity=np.ones((2, 2)),
            depth=np.ones((2, 2)),
            confidence={"confidence": 0.8},
            reference_frame="camera_left",
            cycle_id="test",
        )
        measurement = Measurement.from_roi_depth(
            value=1.5,
            observation=observation,
            reference_frame="camera_left",
            confidence=0.8,
            quality={"valid": True},
            status="VALID",
            cycle_id="test",
        )

        observation_dict = observation.to_dict()
        measurement_dict = measurement.to_dict()

        for key in (
            "observation_id",
            "source",
            "timestamp",
            "cycle_id",
            "observation_type",
            "payload",
            "confidence",
            "quality",
        ):
            self.assertIn(key, observation_dict)

        for key in (
            "measurement_id",
            "measurement_type",
            "value",
            "unit",
            "reference_frame",
            "confidence",
            "quality",
            "source_observations",
        ):
            self.assertIn(key, measurement_dict)

    def test_vision_frame_result_groups_observations_measurements_and_quality(self):
        observation = Observation.from_depth_pipeline(
            disparity=np.ones((2, 2)),
            depth=np.ones((2, 2)),
            confidence={"confidence": 0.8},
            reference_frame="camera_left",
            cycle_id="test",
        )
        quality = QualityReport(
            valid=True,
            confidence=0.8,
            valid_points=144,
            roi_size_px=(12, 12),
            depth_std_mm=0.0,
        )
        distance_measurement = Measurement.from_roi_depth(
            value=1.5,
            observation=observation,
            reference_frame="camera_left",
            confidence=quality.confidence,
            quality=quality.to_dict(),
            status="VALID",
            cycle_id="test",
        )
        position_measurement = Measurement.from_roi_position(
            value={"u": 10, "v": 20},
            observation=observation,
            reference_frame="depth_image",
            confidence=quality.confidence,
            quality=quality.to_dict(),
            status="VALID",
            cycle_id="test",
        )

        result = VisionFrameResult(
            cycle_id="test",
            timestamp=observation.timestamp,
            observations=[observation],
            measurements=[distance_measurement, position_measurement],
            quality=quality,
            frame_id=1,
        )

        self.assertEqual(result.cycle_id, "test")
        self.assertEqual(len(result.observations), 1)
        self.assertEqual(len(result.measurements), 2)
        self.assertIs(result.quality, quality)


if __name__ == "__main__":
    unittest.main()
