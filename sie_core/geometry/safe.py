import numpy as np


class SafeGeometry:
    @staticmethod
    def safe_point(x, y, z):
        if not np.isfinite([x, y, z]).all():
            return None
        return (float(x), float(y), float(z))
