import numpy as np

class QualityEngine:

    def evaluate(self, depth_map):
        valid = np.isfinite(depth_map)
        confidence = np.mean(valid)

        return {
            "confidence": float(confidence),
            "valid_ratio": float(confidence)
        }
