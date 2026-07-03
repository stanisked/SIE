import numpy as np


class UncertaintyEngine:
    def propagate_depth_error(self, depth, disparity_noise=0.5):
        if depth <= 0:
            return None

        sigma_z = (depth ** 2) * disparity_noise

        return float(sigma_z)

    def confidence_from_geometry(self, point_cloud):
        point_cloud = np.asarray(point_cloud, dtype=np.float64)
        if len(point_cloud) < 50:
            return 0.1

        spread = np.std(point_cloud, axis=0).mean()
        confidence = 1.0 / (1.0 + spread)

        return float(confidence)
