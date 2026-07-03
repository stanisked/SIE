import numpy as np


class BoundingBox3D:
    def fit(self, points):
        points = self._valid_points(points)
        if len(points) == 0:
            return None

        min_corner = np.min(points, axis=0)
        max_corner = np.max(points, axis=0)
        size = max_corner - min_corner
        center = (min_corner + max_corner) / 2.0

        return {
            "min": tuple(min_corner.astype(float)),
            "max": tuple(max_corner.astype(float)),
            "center": tuple(center.astype(float)),
            "size": tuple(size.astype(float)),
            "width": float(size[0]),
            "height": float(size[1]),
            "depth": float(size[2]),
            "volume": float(np.prod(size)),
            "point_count": int(len(points)),
        }

    def _valid_points(self, points):
        points = np.asarray(points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("points must have shape (N, 3)")
        return points[np.isfinite(points).all(axis=1)]


class BoxFitter(BoundingBox3D):
    pass
