from sie_core.depth.roi import ROIStableDepth
from sie_core.contracts import Evidence
from vision_core.measurement import Measurement
from vision_core.observation import Observation


class VisionAPI:
    """
    Public API of Spatial Intelligence Engine (SIE)
    """

    def __init__(
        self,
        stereo_engine,
        depth_engine,
        geometry_engine,
        quality_engine,
        temporal_filter=None,
        pointcloud=None,
        plane=None,
        measurement=None,
        uncertainty=None,
    ):
        self.stereo = stereo_engine
        self.depth = depth_engine
        self.geometry = geometry_engine
        self.quality = quality_engine
        self.temporal = temporal_filter
        self.pc = pointcloud
        self.plane = plane
        self.measurement = measurement
        self.uncertainty = uncertainty

        self.stereo_engine = self.stereo
        self.depth_engine = self.depth
        self.geometry_engine = self.geometry
        self.quality_engine = self.quality
        self.temporal_filter = self.temporal
        self.roi_depth = ROIStableDepth()

    def process(self, left, right):
        disparity = self.stereo.compute(left, right)
        depth = self.depth.compute(disparity)

        if self.temporal:
            depth = self.temporal.update(depth)

        confidence = self.quality.evaluate(disparity)

        return depth, confidence

    def process_depth_measurement(
        self,
        left,
        right,
        roi_estimator,
        reference_frame="camera_left",
    ):
        disparity = self.stereo.compute(left, right)
        depth = self.depth.compute(disparity)

        if self.temporal:
            depth = self.temporal.update(depth)

        confidence = self.quality.evaluate(disparity)
        confidence_value = float(confidence["confidence"])
        roi_depth = roi_estimator.center_roi_depth(depth)
        status = "VALID" if roi_depth is not None and confidence_value > 0.0 else "INVALID"

        evidence = Evidence(
            evidence_id="evidence.stereo_frame.current",
            source="vision_core",
            kind="stereo_depth_pipeline",
            metadata={
                "left_shape": getattr(left, "shape", None),
                "right_shape": getattr(right, "shape", None),
                "confidence_signals": confidence,
            },
        )
        observation = Observation.from_depth_pipeline(
            disparity=disparity,
            depth=depth,
            confidence=confidence,
            reference_frame=reference_frame,
            evidence_ids=(evidence.evidence_id,),
        )
        measurement = Measurement.from_roi_depth(
            value=roi_depth,
            observation=observation,
            reference_frame=reference_frame,
            confidence=confidence_value,
            quality={
                "confidence_signals": confidence,
            },
            status=status,
            estimator=roi_estimator.__class__.__name__,
        )

        return {
            "observation": observation.to_dict(),
            "measurement": measurement.to_dict(),
            "evidence": [evidence.to_dict()],
            "debug": {
                "depth": depth,
                "disparity": disparity,
                "confidence": confidence,
            },
        }

    def process_scene(self, left, right):
        disparity = self.stereo.compute(left, right)
        depth = self.depth.compute(disparity)

        if self.temporal:
            depth = self.temporal.update(depth)

        points = self.pc.depth_to_points(depth) if self.pc is not None else None

        plane_c = None
        plane_n = None
        plane_dev = None
        if self.plane is not None and points is not None and len(points) >= 3:
            plane_c, plane_n = self.plane.fit_plane_svd(points)
            plane_dev = self.plane.plane_deviation(points, plane_c, plane_n)

        bbox = None
        if self.measurement is not None and points is not None:
            bbox = self.measurement.bounding_box_3d(points)

        confidence = self.quality.evaluate(disparity)
        geometry_confidence = None
        if self.uncertainty is not None and points is not None:
            geometry_confidence = self.uncertainty.confidence_from_geometry(points)

        return {
            "depth": depth,
            "points": points,
            "plane": {
                "centroid": plane_c,
                "normal": plane_n,
                "deviation": plane_dev,
            },
            "bbox": bbox,
            "confidence": confidence,
            "geometry_confidence": geometry_confidence,
        }

    def get_depth(self, left_frame, right_frame, filtered=True):
        disparity = self.stereo.compute(left_frame, right_frame)
        depth = self.depth.compute(disparity)

        if filtered and self.temporal is not None:
            depth = self.temporal.update(depth)

        return depth

    def pixel_to_3d(self, u, v, depth_value):
        return self.geometry.pixel_to_3d(u, v, depth_value)

    def pixels_to_3d(self, pixels, depths):
        return self.geometry.pixels_to_3d(pixels, depths)

    def stable_depth_in_roi(self, depth_map, roi):
        return self.roi_depth.measure(depth_map, roi)

    def measure_distance(self, p1, p2):
        return self.geometry.distance(p1, p2)

    def get_confidence(
        self,
        disparity,
        depth_sequence=None,
        previous_depth_map=None,
        max_depth=None,
    ):
        if previous_depth_map is None and max_depth is None:
            return self.quality.evaluate(disparity, depth_sequence=depth_sequence)

        confidence_map = self.quality.confidence_map(
            disparity,
            previous_depth_map=previous_depth_map,
            max_depth=max_depth,
        )
        return {
            "confidence": float(confidence_map.mean()),
            "valid_ratio": float((confidence_map > 0.0).mean()),
            "confidence_map": confidence_map,
        }
