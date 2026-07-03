import cv2
import numpy as np

from sie_core.stereo.disparity import StereoEngine
from sie_core.depth.depth_map import DepthEngine
from sie_core.depth.temporal_filter import TemporalDepthFilter
from sie_core.geometry.transforms import GeometryEngine
from sie_core.quality.metrics import QualityEngine
from sie_core.api.vision_api import VisionAPI
from sie_core.depth.roi_depth import ROIDepthEstimator


stereo = StereoEngine()
depth_engine = DepthEngine(fx=500, baseline=0.064)
geometry = GeometryEngine(fx=500, fy=500, cx=320, cy=240)
quality = QualityEngine()
temporal = TemporalDepthFilter()

roi = ROIDepthEstimator()

sie = VisionAPI(stereo, depth_engine, geometry, quality, temporal)

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h, w = frame.shape[:2]

    left = frame[:, :w // 2]
    right = frame[:, w // 2:]

    depth, conf = sie.process(left, right)

    center_depth = roi.center_roi_depth(depth)

    print("ROI depth:", center_depth, "confidence:", conf)

    vis = np.nan_to_num(depth)
    vis = (vis - vis.min()) / (vis.max() - vis.min() + 1e-6)

    cv2.imshow("depth", vis)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()
