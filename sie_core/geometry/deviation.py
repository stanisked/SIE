import numpy as np

from sie_core.geometry.plane import PlaneFitter


class WallDeviation:
    def __init__(self):
        self.plane_fitter = PlaneFitter()

    def measure(self, points, plane=None):
        if plane is None:
            plane = self.plane_fitter.fit_ransac(points)
        if plane is None:
            return None

        distances_m = self.plane_fitter.distances(points, plane)
        abs_mm = np.abs(distances_m) * 1000.0

        if len(abs_mm) == 0:
            return None

        return {
            "plane": plane,
            "mean_abs_mm": float(np.mean(abs_mm)),
            "max_abs_mm": float(np.max(abs_mm)),
            "std_mm": float(np.std(distances_m) * 1000.0),
            "point_count": int(len(abs_mm)),
        }
