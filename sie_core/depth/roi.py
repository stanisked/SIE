import numpy as np


class ROIStableDepth:
    def __init__(self, min_valid_ratio=0.5):
        self.min_valid_ratio = min_valid_ratio

    def measure(self, depth_map, roi):
        x, y, w, h = roi
        patch = depth_map[y:y + h, x:x + w]

        if patch.size == 0:
            return {
                "depth": np.nan,
                "valid_ratio": 0.0,
                "std": np.nan,
                "count": 0,
            }

        valid = np.isfinite(patch)
        valid_ratio = float(np.mean(valid))

        if valid_ratio < self.min_valid_ratio or not np.any(valid):
            depth = np.nan
            std = np.nan
        else:
            depth = float(np.nanmedian(patch))
            std = float(np.nanstd(patch))

        return {
            "depth": depth,
            "valid_ratio": valid_ratio,
            "std": std,
            "count": int(np.sum(valid)),
        }
