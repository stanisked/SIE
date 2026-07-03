import numpy as np

from sie_core.geometry.bounds import BoundingBox3D, BoxFitter
from sie_core.geometry.deviation import WallDeviation
from sie_core.geometry.error import ErrorPropagation
from sie_core.geometry.plane import PlaneFitter
from sie_core.geometry.safe import SafeGeometry


class GeometryEngine:
    def __init__(self, fx, fy, cx, cy):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.box_fitter = BoxFitter()
        self.plane_fitter = PlaneFitter()
        self.wall_deviation = WallDeviation()
        self.error_model = ErrorPropagation()

    def pixel_to_3d(self, u, v, depth):
        x = (u - self.cx) * depth / self.fx
        y = (v - self.cy) * depth / self.fy
        z = depth
        return SafeGeometry.safe_point(x, y, z)

    def pixels_to_3d(self, pixels, depths):
        pixels = np.asarray(pixels, dtype=np.float64)
        depths = np.asarray(depths, dtype=np.float64)

        if pixels.ndim != 2 or pixels.shape[1] != 2:
            raise ValueError("pixels must have shape (N, 2)")
        if depths.shape[0] != pixels.shape[0]:
            raise ValueError("depths must have the same length as pixels")

        u = pixels[:, 0]
        v = pixels[:, 1]
        x = (u - self.cx) * depths / self.fx
        y = (v - self.cy) * depths / self.fy

        points = np.stack([x, y, depths], axis=1)
        invalid = ~np.isfinite(points).all(axis=1)
        points[invalid] = np.nan
        return points

    def distance(self, p1, p2):
        p1 = np.array(p1)
        p2 = np.array(p2)
        return np.linalg.norm(p1 - p2)

    def fit_box(self, points):
        return self.box_fitter.fit(points)

    def bounding_box_3d(self, points):
        return BoundingBox3D().fit(points)

    def fit_plane(self, points, threshold=0.005, iterations=100):
        return self.plane_fitter.fit_ransac(
            points,
            threshold=threshold,
            iterations=iterations,
        )

    def wall_deviation_mm(self, points, plane=None):
        return self.wall_deviation.measure(points, plane=plane)

    def depth_uncertainty(self, depth, disparity, disparity_sigma, baseline):
        return self.error_model.depth_uncertainty(
            depth,
            disparity,
            disparity_sigma,
            self.fx,
            baseline,
        )
