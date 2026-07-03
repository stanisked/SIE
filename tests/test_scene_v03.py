import unittest

import numpy as np

from sie_core.api.vision_api import VisionAPI
from sie_core.depth.depth_map import DepthEngine
from sie_core.geometry.measurements import MeasurementEngine
from sie_core.geometry.plane import PlaneDetector
from sie_core.geometry.point_cloud import PointCloudEngine
from sie_core.geometry.transforms import GeometryEngine
from sie_core.quality.metrics import QualityEngine
from sie_core.quality.uncertainty import UncertaintyEngine


class SceneV03Test(unittest.TestCase):
    def test_point_cloud_engine_converts_depth_to_points(self):
        depth = np.array([
            [1.0, np.nan, 2.0],
            [0.0, 3.0, 4.0],
            [5.0, 6.0, -1.0],
        ])

        points = PointCloudEngine(fx=10, fy=10, cx=0, cy=0, stride=1).depth_to_points(depth)

        self.assertEqual(len(points), 6)
        np.testing.assert_allclose(points[0], (0.0, 0.0, 1.0))
        np.testing.assert_allclose(points[-1], (0.6, 1.2, 6.0))

    def test_plane_detector_svd_reports_deviation(self):
        points = np.array([
            [0.0, 0.0, 2.000],
            [1.0, 0.0, 2.001],
            [0.0, 1.0, 1.999],
            [1.0, 1.0, 2.000],
        ])
        detector = PlaneDetector()

        centroid, normal = detector.fit_plane_svd(points)
        std, mean_abs = detector.plane_deviation(points, centroid, normal)

        self.assertEqual(centroid.shape, (3,))
        self.assertEqual(normal.shape, (3,))
        self.assertLess(std, 0.002)
        self.assertLess(mean_abs, 0.002)

    def test_measurement_engine_returns_bbox_size(self):
        points = np.array([
            [0.0, 0.0, 0.0],
            [2.0, 3.0, 4.0],
        ])

        bbox = MeasurementEngine().bounding_box_3d(points)

        np.testing.assert_allclose(bbox["size"], (2.0, 3.0, 4.0))

    def test_uncertainty_engine_scores_geometry(self):
        uncertainty = UncertaintyEngine()

        self.assertIsNone(uncertainty.propagate_depth_error(-1.0))
        self.assertAlmostEqual(uncertainty.propagate_depth_error(2.0, 0.5), 2.0)
        self.assertEqual(uncertainty.confidence_from_geometry(np.zeros((10, 3))), 0.1)

        cloud = np.ones((60, 3))
        self.assertAlmostEqual(uncertainty.confidence_from_geometry(cloud), 1.0)

    def test_process_scene_returns_geometry_measurements(self):
        class StereoStub:
            def compute(self, left, right):
                return np.full((4, 4), 16.0)

        api = VisionAPI(
            StereoStub(),
            DepthEngine(fx=500, baseline=0.064),
            GeometryEngine(fx=500, fy=500, cx=2, cy=2),
            QualityEngine(),
            None,
            PointCloudEngine(fx=500, fy=500, cx=2, cy=2, stride=1),
            PlaneDetector(),
            MeasurementEngine(),
            UncertaintyEngine(),
        )

        result = api.process_scene(None, None)

        self.assertIn("depth", result)
        self.assertIn("points", result)
        self.assertIn("plane", result)
        self.assertIn("bbox", result)
        self.assertIn("confidence", result)
        self.assertIsNotNone(result["bbox"])
        self.assertIsNotNone(result["plane"]["deviation"])


if __name__ == "__main__":
    unittest.main()
