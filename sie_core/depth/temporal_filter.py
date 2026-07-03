import numpy as np
import warnings
from collections import deque


class TemporalDepthFilter:
    def __init__(self, window_size=7):
        self.window = deque(maxlen=window_size)

    def update(self, depth_map):
        self.window.append(depth_map)

        stack = np.array([d for d in self.window if d is not None])
        if len(stack) == 0:
            return depth_map

        with warnings.catch_warnings():
            warnings.simplefilter("ignore", category=RuntimeWarning)
            return np.nanmedian(stack, axis=0)
