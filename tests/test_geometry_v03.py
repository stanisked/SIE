import unittest

import numpy as np

from sie_core.geometry.bounds import BoundingBox3D, BoxFitter
from sie_core.geometry.deviation import WallDeviation
from sie_core.geometry.error import ErrorPropagation
from sie_core.geometry.plane import PlaneFitter
from sie_core.geometry.transforms import GeometryEngine


class GeometryV03Test(unittest.TestCase):
    def test_box_fitter_measures_object_size(self):
        points = np.array([
            [0.0, 0.0, 1.0],
            [2.0, 0.0, 1.0],
            [0.0, 3.0, 1.0],
            [2.0, 3.0, 5.0],
            [np.nan, 1.0, 1.0],
        ])

        box = BoxFitter().fit(points)

        self.assertEqual(box["point_count"], 4)
        np.testing.assert_allclose(box["size"], (2.0, 3.0, 4.0))
        self.assertAlmostEqual(box["volume"], 24.0)

    def test_bounding_box_returns_none_without_valid_points(self):
        box = BoundingBox3D().fit(np.array([[np.nan, 0.0, 1.0]]))

        self.assertIsNone(box)

    def test_plane_ransac_fits_plane_with_outlier(self):
        xs, ys = np.meshgrid(np.linspace(-1.0, 1.0, 5), np.linspace(-1.0, 1.0, 5))
        zs = np.full_like(xs, 2.0)
        points = np.column_stack([xs.ravel(), ys.ravel(), zs.ravel()])
        points = np.vstack([points, [0.0, 0.0, 5.0]])

        plane = PlaneFitter().fit_ransac(points, threshold=0.01, iterations=50, random_state=1)

        self.assertGreaterEqual(plane["inlier_count"], 25)
        self.assertAlmostEqual(abs(plane["normal"][2]), 1.0, places=5)
        self.assertLess(plane["rmse"], 0.001)

    def test_wall_deviation_reports_millimeters(self):
        points = np.array([
            [0.0, 0.0, 2.000],
            [1.0, 0.0, 2.002],
            [0.0, 1.0, 1.998],
            [1.0, 1.0, 2.001],
        ])
        plane = {"normal": (0.0, 0.0, 1.0), "d": -2.0}

        result = WallDeviation().measure(points, plane=plane)

        self.assertAlmostEqual(result["max_abs_mm"], 2.0, places=4)
        self.assertAlmostEqual(result["mean_abs_mm"], 1.25, places=4)

    def test_error_propagation_depth_uncertainty(self):
        model = ErrorPropagation()

        sigma = model.depth_uncertainty(
            depth=np.array([2.0]),
            disparity=np.array([16.0]),
            disparity_sigma=0.25,
            fx=500,
            baseline=0.064,
        )

        self.assertAlmostEqual(float(sigma[0]), 0.03125, places=6)

    def test_geometry_engine_exposes_v03_measurements(self):
        geometry = GeometryEngine(fx=500, fy=500, cx=320, cy=240)
        points = np.array([
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 1.0],
            [0.0, 1.0, 1.0],
            [1.0, 1.0, 1.0],
        ])

        box = geometry.fit_box(points)
        plane = geometry.fit_plane(points, threshold=0.001, iterations=20)

        np.testing.assert_allclose(box["size"], (1.0, 1.0, 0.0))
        self.assertIsNotNone(plane)


if __name__ == "__main__":
    unittest.main()
