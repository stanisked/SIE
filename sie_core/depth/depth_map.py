import numpy as np

class DepthEngine:
    def __init__(self, fx, baseline):
        self.fx = fx
        self.baseline = baseline

    def compute(self, disparity):
        disparity = np.where(disparity <= 0, np.nan, disparity)
        depth = (self.fx * self.baseline) / disparity
        return depth
