import numpy as np


class QualityEngine:
    def _finite_std(self, values):
        values = np.asarray(values, dtype=np.float32)
        finite = values[np.isfinite(values)]
        if finite.size == 0:
            return np.nan
        return float(np.std(finite))

    def _local_consistency(self, disparity):
        disparity = np.asarray(disparity, dtype=np.float32)
        diffs = []

        horizontal_valid = np.isfinite(disparity[:, 1:]) & np.isfinite(disparity[:, :-1])
        if np.any(horizontal_valid):
            horizontal = np.abs(disparity[:, 1:] - disparity[:, :-1])
            diffs.append(horizontal[horizontal_valid])

        vertical_valid = np.isfinite(disparity[1:, :]) & np.isfinite(disparity[:-1, :])
        if np.any(vertical_valid):
            vertical = np.abs(disparity[1:, :] - disparity[:-1, :])
            diffs.append(vertical[vertical_valid])

        if not diffs:
            return 1.0 if np.any(np.isfinite(disparity)) else 0.0

        local_error = float(np.mean(np.concatenate(diffs)))
        return 1.0 / (1.0 + local_error)

    def confidence_map(self, depth_map, previous_depth_map=None, max_depth=None):
        depth = np.asarray(depth_map, dtype=np.float32)
        valid = np.isfinite(depth).astype(np.float32)

        confidence = valid.copy()

        if max_depth is not None:
            depth_score = 1.0 - np.clip(depth / max_depth, 0.0, 1.0)
            confidence *= np.where(np.isfinite(depth_score), depth_score, 0.0)

        if previous_depth_map is not None:
            previous = np.asarray(previous_depth_map, dtype=np.float32)
            delta = np.abs(depth - previous)
            stability = 1.0 / (1.0 + delta)
            confidence *= np.where(np.isfinite(stability), stability, 0.0)

        return np.clip(confidence, 0.0, 1.0)

    def evaluate(self, disparity, depth_sequence=None):
        disparity = np.asarray(disparity, dtype=np.float32)

        valid = np.isfinite(disparity)
        valid_ratio = float(np.mean(valid))

        noise = self._finite_std(disparity)
        texture_score = 0.0 if not np.isfinite(noise) else 1.0 / (1.0 + noise)

        temporal_stability = 1.0
        if depth_sequence is not None and len(depth_sequence) > 1:
            temporal_noise = self._finite_std(np.array(depth_sequence, dtype=np.float32))
            temporal_stability = (
                0.0
                if not np.isfinite(temporal_noise)
                else 1.0 / (1.0 + temporal_noise)
            )

        local_consistency = self._local_consistency(disparity)

        confidence = (
            0.4 * valid_ratio
            + 0.2 * texture_score
            + 0.2 * temporal_stability
            + 0.2 * local_consistency
        )

        return {
            "confidence": float(np.clip(confidence, 0.0, 1.0)),
            "valid_ratio": valid_ratio,
            "noise": float(noise),
            "texture_score": float(texture_score),
            "temporal_stability": float(temporal_stability),
            "local_consistency": float(local_consistency),
        }
