import numpy as np


class PlaneDetector:
    def fit_plane_svd(self, points):
        points = self._valid_points(points)
        if len(points) < 3:
            return None, None

        centroid = np.mean(points, axis=0)
        centered = points - centroid

        _, _, vh = np.linalg.svd(centered)

        normal = vh[-1]
        normal = normal / np.linalg.norm(normal)

        return centroid, normal

    def plane_deviation(self, points, centroid, normal):
        points = self._valid_points(points)
        if len(points) == 0 or centroid is None or normal is None:
            return None

        centroid = np.asarray(centroid, dtype=np.float64)
        normal = np.asarray(normal, dtype=np.float64)
        normal = normal / np.linalg.norm(normal)

        distances = np.dot(points - centroid, normal)

        return float(np.std(distances)), float(np.mean(np.abs(distances)))

    def _valid_points(self, points):
        points = np.asarray(points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("points must have shape (N, 3)")
        return points[np.isfinite(points).all(axis=1)]


class PlaneFitter:
    def fit_least_squares(self, points):
        points = self._valid_points(points)
        if len(points) < 3:
            return None

        centroid = np.mean(points, axis=0)
        _, _, vh = np.linalg.svd(points - centroid)
        normal = vh[-1]
        normal = normal / np.linalg.norm(normal)
        d = -float(np.dot(normal, centroid))

        return {
            "normal": tuple(normal.astype(float)),
            "d": d,
            "centroid": tuple(centroid.astype(float)),
            "inliers": np.arange(len(points)),
            "inlier_count": int(len(points)),
            "rmse": self._rmse(points, normal, d),
        }

    def fit_ransac(self, points, threshold=0.005, iterations=100, random_state=0):
        points = self._valid_points(points)
        if len(points) < 3:
            return None

        rng = np.random.default_rng(random_state)
        best_inliers = np.array([], dtype=int)
        best_model = None

        for _ in range(iterations):
            sample_idx = rng.choice(len(points), size=3, replace=False)
            model = self._plane_from_points(points[sample_idx])
            if model is None:
                continue

            normal, d = model
            distances = np.abs(points @ normal + d)
            inliers = np.where(distances <= threshold)[0]

            if len(inliers) > len(best_inliers):
                best_inliers = inliers
                best_model = model

        if best_model is None or len(best_inliers) < 3:
            return self.fit_least_squares(points)

        refined = self.fit_least_squares(points[best_inliers])
        refined["inliers"] = best_inliers
        refined["inlier_count"] = int(len(best_inliers))
        refined["outlier_count"] = int(len(points) - len(best_inliers))
        return refined

    def distances(self, points, plane):
        points = self._valid_points(points)
        normal = np.asarray(plane["normal"], dtype=np.float64)
        d = float(plane["d"])
        return points @ normal + d

    def _plane_from_points(self, points):
        p1, p2, p3 = points
        normal = np.cross(p2 - p1, p3 - p1)
        norm = np.linalg.norm(normal)
        if norm == 0:
            return None

        normal = normal / norm
        d = -float(np.dot(normal, p1))
        return normal, d

    def _rmse(self, points, normal, d):
        distances = points @ normal + d
        return float(np.sqrt(np.mean(distances ** 2)))

    def _valid_points(self, points):
        points = np.asarray(points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("points must have shape (N, 3)")
        return points[np.isfinite(points).all(axis=1)]
