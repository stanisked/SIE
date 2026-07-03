import cv2
import numpy as np


class SpatialDepthFilter:
    def __init__(self, kernel_size=5):
        if kernel_size % 2 == 0 or kernel_size < 3:
            raise ValueError("kernel_size must be an odd integer >= 3")
        self.kernel_size = kernel_size

    def apply(self, depth_map):
        depth = np.asarray(depth_map, dtype=np.float32)
        valid = np.isfinite(depth)

        if not np.any(valid):
            return depth.copy()

        filled = np.where(valid, depth, 0.0).astype(np.float32)
        filtered = cv2.medianBlur(filled, self.kernel_size)

        return np.where(valid, filtered, np.nan)
