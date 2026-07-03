from sie_core.api.vision_api import VisionAPI
from sie_core.stereo.disparity import StereoEngine
from sie_core.depth.depth_map import DepthEngine
from sie_core.depth.roi_depth import ROIDepthEstimator
from sie_core.depth.spatial_filter import SpatialDepthFilter
from sie_core.depth.temporal_filter import TemporalDepthFilter
from sie_core.geometry.transforms import GeometryEngine
from sie_core.quality.metrics import QualityEngine

import cv2

stereo = StereoEngine()
depth_engine = DepthEngine(fx=500, baseline=0.064)
geometry = GeometryEngine(fx=500, fy=500, cx=320, cy=240)
quality = QualityEngine()
temporal = TemporalDepthFilter(window_size=7)
spatial = SpatialDepthFilter(kernel_size=5)
center_depth_estimator = ROIDepthEstimator(kernel_size=12)

sie = VisionAPI(stereo, depth_engine, geometry, quality, temporal)

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h, w = frame.shape[:2]
    left = frame[:, :w//2]
    right = frame[:, w//2:]

    depth, q = sie.process(left, right)
    depth = spatial.apply(depth)

    center_depth = center_depth_estimator.center_roi_depth(depth)

    print("center_depth:", center_depth, "confidence:", q)

    disparity = stereo.compute(left, right)
    cv2.imshow("left", left)
    cv2.imshow("disparity", (disparity - disparity.min()) / (disparity.max() - disparity.min() + 1e-6))

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()
