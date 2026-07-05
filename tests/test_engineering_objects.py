import unittest

import numpy as np

from sie_core.api.vision_api import VisionAPI
from sie_core.depth.roi_depth import ROIDepthEstimator
from sie_core.geometry.transforms import GeometryEngine
from sie_core.quality.metrics import QualityEngine
from vision_core.measurement import Measurement
from vision_core.observation import Observation


class EngineeringObjectsTest(unittest.TestCase):
    def test_depth_measurement_returns_sie_engineering_objects(self):
        class StereoStub:
            def compute(self, left, right):
                return np.full((6, 6), 16.0)

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
            np.zeros((6, 6, 3)),
            np.zeros((6, 6, 3)),
            ROIDepthEstimator(kernel_size=2),
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


if __name__ == "__main__":
    unittest.main()
