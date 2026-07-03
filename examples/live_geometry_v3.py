import cv2

from sie_core.stereo.disparity import StereoEngine
from sie_core.depth.depth_map import DepthEngine
from sie_core.geometry.transforms import GeometryEngine
from sie_core.geometry.point_cloud import PointCloudEngine
from sie_core.geometry.plane import PlaneDetector
from sie_core.geometry.measurements import MeasurementEngine
from sie_core.quality.metrics import QualityEngine
from sie_core.quality.uncertainty import UncertaintyEngine
from sie_core.api.vision_api import VisionAPI


stereo = StereoEngine()
depth = DepthEngine(fx=500, baseline=0.064)
geometry = GeometryEngine(fx=500, fy=500, cx=320, cy=240)
quality = QualityEngine()

pc = PointCloudEngine(500, 500, 320, 240)
plane = PlaneDetector()
meas = MeasurementEngine()
unc = UncertaintyEngine()

sie = VisionAPI(stereo, depth, geometry, quality, None, pc, plane, meas, unc)

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h, w = frame.shape[:2]

    left = frame[:, :w // 2]
    right = frame[:, w // 2:]

    result = sie.process_scene(left, right)

    bbox = result["bbox"]
    plane_dev = result["plane"]["deviation"]
    conf = result["confidence"]

    size = None if bbox is None else bbox["size"]
    print("SIZE:", size, "PLANE_DEV:", plane_dev, "CONF:", conf)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()
