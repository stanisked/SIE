import cv2
import numpy as np
from datetime import datetime, timezone

from sie_core.depth.depth_map import DepthEngine
from sie_core.depth.roi_depth import ROIDepthEstimator
from sie_core.depth.temporal_filter import TemporalDepthFilter
from sie_core.quality.metrics import QualityEngine
from sie_core.stereo.disparity import StereoEngine
from vision_core.measurement import Measurement
from vision_core.observation import Observation
from vision_core.quality import QualityReport
from vision_core.result import VisionFrameResult


stereo = StereoEngine()
depth_engine = DepthEngine(fx=500, baseline=0.064)
quality = QualityEngine()
temporal = TemporalDepthFilter()

roi = ROIDepthEstimator()

cap = cv2.VideoCapture(0)
frame_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_count += 1
    timestamp = datetime.now(timezone.utc).isoformat()
    cycle_id = f"vision_cycle_{frame_count}"

    h, w = frame.shape[:2]

    left = frame[:, :w // 2]
    right = frame[:, w // 2:]

    disparity = stereo.compute(left, right)
    depth = depth_engine.compute(disparity)
    depth = temporal.update(depth)
    confidence = quality.evaluate(disparity)

    roi_patch = roi.center_roi(depth)
    valid = roi_patch[np.isfinite(roi_patch) & (roi_patch > 0)]
    quality_report = QualityReport(
        valid=len(valid) > 100,
        confidence=min(1.0, len(valid) / (roi_patch.size + 1e-6)),
        valid_points=len(valid),
        roi_size_px=(roi_patch.shape[1], roi_patch.shape[0]),
        depth_std_mm=float(np.std(valid)) if len(valid) > 0 else None,
    )

    center_depth = roi.center_roi_depth(depth)
    status = "VALID" if center_depth is not None and quality_report.valid else "INVALID"
    observation = Observation.from_depth_pipeline(
        disparity=disparity,
        depth=depth,
        confidence=confidence,
        reference_frame="stereo_camera_01_left_optical_frame",
        cycle_id=cycle_id,
    )
    distance_measurement = Measurement.from_roi_depth(
        value=center_depth,
        observation=observation,
        reference_frame="stereo_camera_01_left_optical_frame",
        confidence=quality_report.confidence,
        quality=quality_report.to_dict(),
        status=status,
        cycle_id=cycle_id,
        estimator=roi.__class__.__name__,
    )
    position_measurement = Measurement.from_roi_position(
        value={
            "u": depth.shape[1] // 2,
            "v": depth.shape[0] // 2,
        },
        observation=observation,
        reference_frame="depth_image",
        confidence=quality_report.confidence,
        quality=quality_report.to_dict(),
        status=status,
        cycle_id=cycle_id,
        estimator=roi.__class__.__name__,
    )

    vision_result = VisionFrameResult(
        cycle_id=cycle_id,
        timestamp=timestamp,
        observations=[observation],
        measurements=[distance_measurement, position_measurement],
        quality=quality_report,
        frame_id=frame_count,
    )

    print("vision_result=", vision_result)

    vis = np.nan_to_num(depth)
    vis = (vis - vis.min()) / (vis.max() - vis.min() + 1e-6)

    cv2.imshow("depth", vis)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()
