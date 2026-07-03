import unittest

import numpy as np

from sie_core.api.vision_api import VisionAPI
from sie_core.depth.roi import ROIStableDepth
from sie_core.depth.roi_depth import ROIDepthEstimator
from sie_core.depth.spatial_filter import SpatialDepthFilter
from sie_core.depth.temporal_filter import TemporalDepthFilter
from sie_core.geometry.safe import SafeGeometry
from sie_core.geometry.transforms import GeometryEngine
from sie_core.quality.accuracy import RulerBenchmark
from sie_core.quality.metrics import QualityEngine


class DepthV02Test(unittest.TestCase):
    def test_vision_api_process_returns_depth_and_confidence(self):
        class StereoStub:
            def compute(self, left, right):
                return np.array([[1.0, 2.0], [np.nan, 4.0]], dtype=np.float32)

        class DepthStub:
            def compute(self, disparity):
                return disparity * 10.0

        class TemporalStub:
            def update(self, depth):
                return depth + 1.0

        api = VisionAPI(
            StereoStub(),
            DepthStub(),
            GeometryEngine(fx=100, fy=100, cx=0, cy=0),
            QualityEngine(),
            TemporalStub(),
        )

        depth, confidence = api.process(None, None)

        np.testing.assert_allclose(
            depth,
            np.array([[11.0, 21.0], [np.nan, 41.0]], dtype=np.float32),
        )
        self.assertIn("confidence", confidence)
        self.assertAlmostEqual(confidence["valid_ratio"], 0.75)

    def test_temporal_filter_uses_nanmedian(self):
        depth_filter = TemporalDepthFilter(window_size=3)

        depth_filter.update(np.array([[1.0, np.nan], [4.0, 10.0]]))
        depth_filter.update(np.array([[3.0, 2.0], [100.0, 20.0]]))
        filtered = depth_filter.update(np.array([[5.0, 4.0], [6.0, 30.0]]))

        np.testing.assert_allclose(filtered, np.array([[3.0, 3.0], [6.0, 20.0]]))

    def test_spatial_filter_reduces_single_pixel_noise(self):
        depth = np.ones((5, 5), dtype=np.float32)
        depth[2, 2] = 100.0

        filtered = SpatialDepthFilter(kernel_size=3).apply(depth)

        self.assertEqual(filtered[2, 2], 1.0)

    def test_roi_stable_depth_returns_median_and_valid_ratio(self):
        depth = np.array([
            [1.0, 1.1, np.nan],
            [0.9, 50.0, 1.0],
            [np.nan, 1.2, 1.0],
        ])

        result = ROIStableDepth(min_valid_ratio=0.5).measure(depth, (0, 0, 3, 3))

        self.assertAlmostEqual(result["depth"], 1.0)
        self.assertAlmostEqual(result["valid_ratio"], 7 / 9)
        self.assertEqual(result["count"], 7)

    def test_center_roi_depth_uses_median_and_ignores_invalid_values(self):
        depth = np.full((7, 7), np.nan, dtype=np.float32)
        depth[2:4, 2:4] = np.array([
            [1.0, 1.1],
            [10.0, 1.2],
        ])

        result = ROIDepthEstimator(kernel_size=1).center_roi_depth(depth)

        self.assertAlmostEqual(result, 1.15, places=6)

    def test_center_roi_depth_returns_none_without_valid_values(self):
        depth = np.full((5, 5), np.nan, dtype=np.float32)

        result = ROIDepthEstimator(kernel_size=2).center_roi_depth(depth)

        self.assertIsNone(result)

    def test_batch_pixel_to_3d(self):
        geometry = GeometryEngine(fx=100, fy=100, cx=10, cy=20)
        points = geometry.pixels_to_3d(
            pixels=np.array([[10, 20], [20, 30]]),
            depths=np.array([2.0, 4.0]),
        )

        np.testing.assert_allclose(points, np.array([[0.0, 0.0, 2.0], [0.4, 0.4, 4.0]]))

    def test_safe_point_returns_none_for_invalid_geometry(self):
        self.assertIsNone(SafeGeometry.safe_point(1.0, np.nan, 2.0))
        self.assertEqual(SafeGeometry.safe_point(1.0, 2.0, 3.0), (1.0, 2.0, 3.0))

    def test_pixel_to_3d_returns_none_for_invalid_depth(self):
        geometry = GeometryEngine(fx=100, fy=100, cx=10, cy=20)

        self.assertIsNone(geometry.pixel_to_3d(10, 20, np.nan))

    def test_batch_pixel_to_3d_marks_invalid_rows_as_nan(self):
        geometry = GeometryEngine(fx=100, fy=100, cx=10, cy=20)
        points = geometry.pixels_to_3d(
            pixels=np.array([[10, 20], [20, 30]]),
            depths=np.array([2.0, np.nan]),
        )

        np.testing.assert_allclose(points[0], np.array([0.0, 0.0, 2.0]))
        self.assertTrue(np.isnan(points[1]).all())

    def test_confidence_map_uses_validity_depth_and_stability(self):
        quality = QualityEngine()
        depth = np.array([[1.0, np.nan], [2.0, 4.0]])
        previous = np.array([[1.0, 1.0], [3.0, 4.0]])

        confidence = quality.confidence_map(depth, previous_depth_map=previous, max_depth=4.0)

        expected = np.array([[0.75, 0.0], [0.25, 0.0]])
        np.testing.assert_allclose(confidence, expected)

    def test_quality_engine_v2_uses_engineering_signals(self):
        quality = QualityEngine()
        disparity = np.array([
            [1.0, 1.0, 1.0],
            [1.0, np.nan, 1.0],
            [1.0, 1.0, 3.0],
        ])
        depth_sequence = [
            np.array([[2.0, 2.0], [2.0, 2.0]]),
            np.array([[2.0, 2.2], [1.8, 2.0]]),
        ]

        result = quality.evaluate(disparity, depth_sequence=depth_sequence)

        self.assertAlmostEqual(result["valid_ratio"], 8 / 9)
        self.assertGreater(result["confidence"], 0.0)
        self.assertLessEqual(result["confidence"], 1.0)
        self.assertIn("noise", result)
        self.assertIn("texture_score", result)
        self.assertIn("temporal_stability", result)
        self.assertIn("local_consistency", result)
        self.assertLess(result["local_consistency"], 1.0)

    def test_ruler_benchmark_reports_errors(self):
        result = RulerBenchmark().evaluate(
            measured_depths=[1.0, 1.2, np.nan, 2.1],
            ruler_depths=[1.0, 1.0, 1.0, 2.0],
        )

        self.assertEqual(result["count"], 3)
        self.assertAlmostEqual(result["mae"], 0.1, places=6)
        self.assertAlmostEqual(result["bias"], 0.1, places=6)


if __name__ == "__main__":
    unittest.main()
