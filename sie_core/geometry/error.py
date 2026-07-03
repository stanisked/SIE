import numpy as np


class ErrorPropagation:
    def depth_uncertainty(self, depth, disparity, disparity_sigma, fx, baseline):
        depth = np.asarray(depth, dtype=np.float64)
        disparity = np.asarray(disparity, dtype=np.float64)

        sigma_z = np.abs((fx * baseline) / (disparity ** 2)) * disparity_sigma
        sigma_z = np.where(np.isfinite(depth) & np.isfinite(sigma_z), sigma_z, np.nan)
        return sigma_z

    def point_uncertainty(self, u, v, depth, sigma_depth, fx, fy, cx, cy):
        z = float(depth)
        sigma_z = float(sigma_depth)

        sigma_x = abs((u - cx) / fx) * sigma_z
        sigma_y = abs((v - cy) / fy) * sigma_z

        return {
            "sigma_x": float(sigma_x),
            "sigma_y": float(sigma_y),
            "sigma_z": float(sigma_z),
            "sigma_norm": float(np.linalg.norm([sigma_x, sigma_y, sigma_z])),
        }
