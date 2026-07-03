import numpy as np


class ROIDepthEstimator:
    def __init__(self, kernel_size=5):
        if kernel_size < 1:
            raise ValueError("kernel_size must be >= 1")
        self.k = kernel_size

    def center_roi_depth(self, depth_map):
        depth = np.asarray(depth_map, dtype=np.float32)
        h, w = depth.shape

        x1 = max(0, w // 2 - self.k)
        x2 = min(w, w // 2 + self.k)
        y1 = max(0, h // 2 - self.k)
        y2 = min(h, h // 2 + self.k)

        roi = depth[y1:y2, x1:x2]
        roi = roi[np.isfinite(roi)]

        if len(roi) == 0:
            return None

        return float(np.median(roi))
