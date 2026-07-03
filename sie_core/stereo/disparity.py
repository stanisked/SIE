import cv2
import numpy as np

class StereoEngine:
    def __init__(self, matcher=None):
        self.matcher = matcher or cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=96,
            blockSize=5
        )

    def compute(self, left, right):
        gray_l = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

        disp = self.matcher.compute(gray_l, gray_r).astype(np.float32) / 16.0
        return disp
