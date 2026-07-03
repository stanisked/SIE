import numpy as np


class RulerBenchmark:
    def evaluate(self, measured_depths, ruler_depths):
        measured = np.asarray(measured_depths, dtype=np.float32)
        expected = np.asarray(ruler_depths, dtype=np.float32)

        if measured.shape != expected.shape:
            raise ValueError("measured_depths and ruler_depths must have the same shape")

        valid = np.isfinite(measured) & np.isfinite(expected)
        if not np.any(valid):
            return {
                "count": 0,
                "mae": np.nan,
                "rmse": np.nan,
                "bias": np.nan,
                "max_abs_error": np.nan,
            }

        errors = measured[valid] - expected[valid]
        abs_errors = np.abs(errors)

        return {
            "count": int(np.sum(valid)),
            "mae": float(np.mean(abs_errors)),
            "rmse": float(np.sqrt(np.mean(errors ** 2))),
            "bias": float(np.mean(errors)),
            "max_abs_error": float(np.max(abs_errors)),
        }
