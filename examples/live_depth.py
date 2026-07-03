from sie_core.api.vision_api import VisionAPI
from sie_core.stereo.disparity import StereoEngine
from sie_core.depth.depth_map import DepthEngine
from sie_core.geometry.transforms import GeometryEngine
from sie_core.quality.metrics import QualityEngine

import cv2

stereo = StereoEngine()
depth_engine = DepthEngine(fx=500, baseline=0.064)
geometry = GeometryEngine(fx=500, fy=500, cx=320, cy=240)
quality = QualityEngine()

sie = VisionAPI(stereo, depth_engine, geometry, quality)

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h, w = frame.shape[:2]
    left = frame[:, :w//2]
    right = frame[:, w//2:]

    disparity = stereo.compute(left, right)
    depth = depth_engine.compute(disparity)
    q = quality.evaluate(depth)

    center_depth = depth[h//2, w//4]

    print("center_depth:", center_depth, "confidence:", q)

    cv2.imshow("left", left)
    cv2.imshow("disparity", (disparity - disparity.min()) / (disparity.max() - disparity.min() + 1e-6))

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()
