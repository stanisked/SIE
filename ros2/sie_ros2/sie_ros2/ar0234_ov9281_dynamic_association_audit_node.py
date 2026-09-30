"""Diagnostic-only AR0234↔OV9281 association audit for a moving target.

The node deliberately reuses the calibrated live capture and association code,
but it removes the static-scene gate from the *diagnostic* path.  Its output is
evidence about a current RGB/stereo pairing, not a SIE metric Measurement.  In
particular, host capture timestamps are not exposure synchronization, so this
node cannot authorize navigation or an actuator command.
"""

from __future__ import annotations

import math
from datetime import datetime, timezone
from typing import Any

import rclpy
from std_msgs.msg import String

from .ar0234_ov9281_metric_measurement_node import (
    Ar0234Ov9281MetricMeasurementNode,
)
from .contracts import (
    ContractError,
    ar0234_dynamic_target_candidate,
    encode,
    validate_ar0234_observation,
)


class Ar0234Ov9281DynamicAssociationAuditNode(
    Ar0234Ov9281MetricMeasurementNode
):
    """Publish fail-closed diagnostic evidence for dynamic RGB/stereo pairing."""

    def __init__(self) -> None:
        self._diagnostic_only = True
        # AR0234 provides the only semantic person evidence.  The audit must
        # not silently reinstate a second semantic gate on the stereo image.
        self._disable_stereo_person_detector = True
        # The parent initializes only calibrated camera capture, rectification,
        # and the two detectors.  This subclass never calls its Measurement
        # publishing methods and never evaluates its static-scene gate.
        super().__init__()
        self.declare_parameter(
            "audit_topic", "/sie/diagnostics/ar0234_ov9281/dynamic_association"
        )
        self.declare_parameter("frustum_sample_stride_px", 4)
        self.declare_parameter("ar_inner_roi_x_margin_fraction", 0.20)
        self.declare_parameter("ar_inner_roi_top_fraction", 0.30)
        self.declare_parameter("ar_inner_roi_bottom_fraction", 0.15)
        self.declare_parameter("frustum_search_margin_px", 32)
        self.declare_parameter("depth_cluster_gap_m", 0.15)
        self.declare_parameter("depth_cluster_relative_gap", 0.04)
        self.declare_parameter("depth_cluster_min_relative_support", 0.25)
        self.declare_parameter("lr_consistency_max_delta_px", 1.5)
        self.declare_parameter("spatial_component_min_relative_support", 0.25)
        self.declare_parameter("spatial_component_min_footprint_px", 1600)
        self.declare_parameter("spatial_component_min_width_px", 24)
        self.declare_parameter("spatial_component_min_height_px", 24)
        self.audit_topic = str(self.get_parameter("audit_topic").value)
        self.frustum_sample_stride_px = int(
            self.get_parameter("frustum_sample_stride_px").value
        )
        self.ar_inner_roi_x_margin_fraction = float(
            self.get_parameter("ar_inner_roi_x_margin_fraction").value
        )
        self.ar_inner_roi_top_fraction = float(
            self.get_parameter("ar_inner_roi_top_fraction").value
        )
        self.ar_inner_roi_bottom_fraction = float(
            self.get_parameter("ar_inner_roi_bottom_fraction").value
        )
        self.frustum_search_margin_px = int(
            self.get_parameter("frustum_search_margin_px").value
        )
        self.depth_cluster_gap_m = float(
            self.get_parameter("depth_cluster_gap_m").value
        )
        self.depth_cluster_relative_gap = float(
            self.get_parameter("depth_cluster_relative_gap").value
        )
        self.depth_cluster_min_relative_support = float(
            self.get_parameter("depth_cluster_min_relative_support").value
        )
        self.lr_consistency_max_delta_px = float(
            self.get_parameter("lr_consistency_max_delta_px").value
        )
        self.spatial_component_min_relative_support = float(
            self.get_parameter("spatial_component_min_relative_support").value
        )
        self.spatial_component_min_footprint_px = int(
            self.get_parameter("spatial_component_min_footprint_px").value
        )
        self.spatial_component_min_width_px = int(
            self.get_parameter("spatial_component_min_width_px").value
        )
        self.spatial_component_min_height_px = int(
            self.get_parameter("spatial_component_min_height_px").value
        )
        if (
            self.frustum_sample_stride_px < 1
            or not 0.0 <= self.ar_inner_roi_x_margin_fraction < 0.5
            or not 0.0 <= self.ar_inner_roi_top_fraction < 1.0
            or not 0.0 <= self.ar_inner_roi_bottom_fraction < 1.0
            or self.ar_inner_roi_top_fraction + self.ar_inner_roi_bottom_fraction >= 1.0
            or self.frustum_search_margin_px < 0
            or self.depth_cluster_gap_m <= 0.0
            or not 0.0 < self.depth_cluster_relative_gap < 1.0
            or not 0.0 < self.depth_cluster_min_relative_support <= 1.0
            or self.lr_consistency_max_delta_px <= 0.0
            or not 0.0 < self.spatial_component_min_relative_support <= 1.0
            or self.spatial_component_min_footprint_px < 1
            or self.spatial_component_min_width_px < 1
            or self.spatial_component_min_height_px < 1
        ):
            raise ValueError("invalid AR-frustum association parameters")
        # Right-to-left disparity has negative sign.  Comparing it to the
        # ordinary left-to-right map removes one-sided SGBM streaks before any
        # 3D or depth-cluster reasoning.
        self.right_sgbm = self.cv2.StereoSGBM_create(
            minDisparity=-128,
            numDisparities=128,
            blockSize=7,
            P1=8 * 3 * 7 * 7,
            P2=32 * 3 * 7 * 7,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=2,
            disp12MaxDiff=1,
            mode=self.cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )
        self.audit_publisher = self.create_publisher(String, self.audit_topic, 10)
        self.get_logger().info(
            "dynamic association audit enabled; "
            f"publishing diagnostic-only evidence to {self.audit_topic}; "
            f"LR consistency=±{self.lr_consistency_max_delta_px:.1f}px; "
            "connected 3D components required; "
            "AR0234 is the only semantic detector; metric Measurement, navigation, "
            "and actuator access remain disabled"
        )

    @staticmethod
    def _selected_observation(
        raw: dict[str, Any], candidate: dict[str, Any], dynamic: dict[str, Any]
    ) -> dict[str, Any]:
        """Turn one dynamic candidate into a schema-valid association input."""
        bbox = candidate.get("bbox_xyxy_px")
        if type(bbox) is not list or len(bbox) != 4:
            raise ContractError("dynamic candidate has invalid bbox_xyxy_px")
        selected = dict(raw)
        selected.update(
            {
                "target_status": "SINGLE_TARGET",
                "detection_count": 1,
                "eligible_detection_count": 1,
                "detections": [candidate],
                "bbox_xyxy_px": [float(value) for value in bbox],
                "center_x_px": float(candidate["center_x_px"]),
                "confidence": float(candidate["confidence"]),
                "truncated_top": bool(candidate["truncated_top"]),
                "truncated_left": bool(candidate["truncated_left"]),
                "truncated_right": bool(candidate["truncated_right"]),
                "truncated_bottom": bool(candidate["truncated_bottom"]),
                "dynamic_target_selection": {
                    "reason": dynamic["reason"],
                    "source_target_status": dynamic["source_target_status"],
                    "selection": dynamic["selection"],
                },
            }
        )
        return validate_ar0234_observation(selected)

    @staticmethod
    def _projection_diagnostic(
        observation: dict[str, Any], association: dict[str, Any]
    ) -> dict[str, Any]:
        """Describe association geometry without relabelling it calibration error."""
        x1, y1, x2, y2 = (
            float(value) for value in observation["bbox_xyxy_px"]
        )
        projected_x, projected_y = (
            float(value) for value in association["projected_ar0234_point_px"]
        )
        center_x, center_y = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        return {
            "projected_ar0234_point_px": [projected_x, projected_y],
            "ar_bbox_xyxy_px": [x1, y1, x2, y2],
            "projected_point_inside_ar_bbox": (
                x1 <= projected_x <= x2 and y1 <= projected_y <= y2
            ),
            # This is intentionally not called reprojection error: the point
            # comes from a depth ROI while the AR box centre is only a proxy.
            "projected_point_to_ar_bbox_center_px": math.hypot(
                projected_x - center_x, projected_y - center_y
            ),
        }

    def _ar_inner_roi(self, observation: dict[str, Any]) -> tuple[float, float, float, float]:
        """Use the torso-like interior of the RGB box, not its background-heavy edge."""
        x1, y1, x2, y2 = (
            float(value) for value in observation["bbox_xyxy_px"]
        )
        width, height = x2 - x1, y2 - y1
        result = (
            x1 + self.ar_inner_roi_x_margin_fraction * width,
            y1 + self.ar_inner_roi_top_fraction * height,
            x2 - self.ar_inner_roi_x_margin_fraction * width,
            y2 - self.ar_inner_roi_bottom_fraction * height,
        )
        if result[2] <= result[0] or result[3] <= result[1]:
            raise ContractError("AR inner ROI is empty")
        return result

    def _stereo_frustum_search_roi(
        self, ar_inner_roi: tuple[float, float, float, float]
    ) -> tuple[int, int, int, int]:
        """Project an RGB image region through the calibrated 0.5–4.5 m frustum."""
        x1, y1, x2, y2 = ar_inner_roi
        corners = self.np.asarray(
            [[x1, y1], [x2, y1], [x1, y2], [x2, y2]], dtype=self.np.float64
        ).reshape(-1, 1, 2)
        normalized = self.cv2.undistortPoints(corners, self.k_ar, self.d_ar).reshape(-1, 2)
        rays_ar = self.np.column_stack(
            (normalized, self.np.ones((normalized.shape[0],), dtype=self.np.float64))
        )
        projected: list[Any] = []
        for depth_m in (self.range_min_m, self.range_max_m):
            points_ar = rays_ar * depth_m
            points_left = (
                self.r_ar_to_left @ points_ar.T + self.t_ar_to_left_m
            ).T
            points_rectified = (self.r1 @ points_left.T).T
            homogeneous = self.np.column_stack(
                (points_rectified, self.np.ones((points_rectified.shape[0],)))
            )
            image = (self.p1 @ homogeneous.T).T
            valid = image[:, 2] > 0.0
            if valid.any():
                projected.append(image[valid, :2] / image[valid, 2:3])
        if not projected:
            raise RuntimeError("AR frustum projects behind physical_left")
        image_points = self.np.concatenate(projected, axis=0)
        width, height = self.stereo_size
        margin = self.frustum_search_margin_px
        ix1 = max(0, int(math.floor(float(image_points[:, 0].min()))) - margin)
        iy1 = max(0, int(math.floor(float(image_points[:, 1].min()))) - margin)
        ix2 = min(width, int(math.ceil(float(image_points[:, 0].max()))) + margin)
        iy2 = min(height, int(math.ceil(float(image_points[:, 1].max()))) + margin)
        if ix2 <= ix1 or iy2 <= iy1:
            raise RuntimeError("calibrated AR frustum has no physical_left overlap")
        return ix1, iy1, ix2, iy2

    def _depth_clusters(
        self, depths_m: Any
    ) -> list[tuple[Any, dict[str, Any]]]:
        """Find separated, sufficiently supported depth hypotheses in one frustum.

        The returned indices preserve each *entire* cluster.  In particular,
        they prevent a later median-centred crop from making a broad, unreliable
        disparity layer appear precise merely by discarding its tails.
        """
        if int(depths_m.size) == 0:
            return []
        order = self.np.argsort(depths_m)
        ordered = depths_m[order]
        gap_m = max(
            self.depth_cluster_gap_m,
            self.depth_cluster_relative_gap * float(self.np.median(ordered)),
        )
        boundaries = self.np.flatnonzero(self.np.diff(ordered) > gap_m) + 1
        groups = self.np.split(order, boundaries)
        clusters: list[tuple[Any, dict[str, Any]]] = []
        for indices in groups:
            group = depths_m[indices]
            if int(group.size) < self.min_depth_samples:
                continue
            median = float(self.np.median(group))
            clusters.append(
                (indices, {
                    "sample_count": int(group.size),
                    "median_depth_m": median,
                    "mad_m": float(self.np.median(self.np.abs(group - median))),
                })
            )
        return clusters

    def _left_right_disparity(
        self, left_gray: Any, right_gray: Any
    ) -> tuple[Any, Any]:
        """Return left disparity and pixels supported by both matching directions."""
        left = self.sgbm.compute(left_gray, right_gray).astype(self.np.float32) / 16.0
        right = (
            self.right_sgbm.compute(right_gray, left_gray).astype(self.np.float32)
            / 16.0
        )
        height, width = left.shape
        x = self.np.broadcast_to(
            self.np.arange(width, dtype=self.np.int32), (height, width)
        )
        y = self.np.broadcast_to(
            self.np.arange(height, dtype=self.np.int32).reshape(-1, 1),
            (height, width),
        )
        x_right = self.np.rint(x - left).astype(self.np.int32)
        left_valid = self.np.isfinite(left) & (left > 0.5)
        in_right_image = (x_right >= 0) & (x_right < width)
        right_at_match = self.np.full(left.shape, self.np.nan, dtype=self.np.float32)
        lookup = left_valid & in_right_image
        right_at_match[lookup] = right[y[lookup], x_right[lookup]]
        consistent = (
            lookup
            & self.np.isfinite(right_at_match)
            & (right_at_match < -0.5)
            & (
                self.np.abs(left + right_at_match)
                <= self.lr_consistency_max_delta_px
            )
        )
        return left, consistent

    def _spatial_components(
        self,
        u: Any,
        v: Any,
        frustum_roi: tuple[int, int, int, int],
    ) -> list[tuple[Any, dict[str, Any]]]:
        """Find connected supported regions on the sampled physical-left grid."""
        ix1, iy1, ix2, iy2 = frustum_roi
        stride = self.frustum_sample_stride_px
        grid_width = (ix2 - ix1 + stride - 1) // stride
        grid_height = (iy2 - iy1 + stride - 1) // stride
        grid_x = ((u.astype(self.np.int32) - ix1) // stride).clip(0, grid_width - 1)
        grid_y = ((v.astype(self.np.int32) - iy1) // stride).clip(0, grid_height - 1)
        occupancy = self.np.zeros((grid_height, grid_width), dtype=self.np.uint8)
        occupancy[grid_y, grid_x] = 1
        component_count, labels, stats, _ = self.cv2.connectedComponentsWithStats(
            occupancy, connectivity=8
        )
        labels_at_samples = labels[grid_y, grid_x]
        components: list[tuple[Any, dict[str, Any]]] = []
        for label in range(1, component_count):
            indices = self.np.flatnonzero(labels_at_samples == label)
            if not int(indices.size):
                continue
            left, top, width, height, area_cells = (
                int(value) for value in stats[label]
            )
            components.append(
                (
                    indices,
                    {
                        "sample_count": int(indices.size),
                        "coverage_area_px": int(indices.size * stride * stride),
                        "bbox_xyxy_px": [
                            int(ix1 + left * stride),
                            int(iy1 + top * stride),
                            int(min(ix2, ix1 + (left + width) * stride)),
                            int(min(iy2, iy1 + (top + height) * stride)),
                        ],
                        "bbox_width_px": int(width * stride),
                        "bbox_height_px": int(height * stride),
                        "occupied_grid_cells": area_cells,
                    },
                )
            )
        return components

    def _frustum_associate(
        self,
        observation: dict[str, Any],
        left_rectified: Any,
        right_rectified: Any,
    ) -> tuple[dict[str, Any] | None, str, dict[str, Any]]:
        """Associate AR semantics with stereo geometry without a stereo classifier.

        Every candidate starts as a valid disparity point.  It is retained only
        when its calibrated physical-left 3D point reprojects into the central
        AR target region.  A second plausible depth layer is a refusal, not an
        arbitrary choice between foreground and background.
        """
        ar_inner_roi = self._ar_inner_roi(observation)
        frustum_roi = self._stereo_frustum_search_roi(ar_inner_roi)
        left_gray = self.cv2.cvtColor(left_rectified, self.cv2.COLOR_BGR2GRAY)
        right_gray = self.cv2.cvtColor(right_rectified, self.cv2.COLOR_BGR2GRAY)
        disparity, lr_consistent = self._left_right_disparity(left_gray, right_gray)
        self._write_frustum_debug(
            left_rectified, right_rectified, disparity, lr_consistent, frustum_roi
        )

        ix1, iy1, ix2, iy2 = frustum_roi
        stride = self.frustum_sample_stride_px
        grid_y, grid_x = self.np.mgrid[iy1:iy2:stride, ix1:ix2:stride]
        sampled_disparity = disparity[grid_y, grid_x]
        sampled_lr_consistent = lr_consistent[grid_y, grid_x]
        positive = self.np.isfinite(sampled_disparity) & (sampled_disparity > 0.5)
        valid = positive & sampled_lr_consistent
        positive_count = int(positive.sum())
        lr_consistent_count = int(valid.sum())
        if not valid.any():
            return None, "FRUSTUM_LR_CONSISTENT_DISPARITY_UNAVAILABLE", {
                "ar_inner_roi_xyxy_px": list(ar_inner_roi),
                "physical_left_frustum_roi_xyxy_px": list(frustum_roi),
                "sample_stride_px": stride,
                "positive_disparity_sample_count": positive_count,
                "lr_consistent_disparity_sample_count": 0,
            }
        u = grid_x[valid].astype(self.np.float64)
        v = grid_y[valid].astype(self.np.float64)
        disparity_values = sampled_disparity[valid].astype(self.np.float64)
        rectified_depth = abs(float(self.p2[0, 3])) / disparity_values / 1000.0
        in_range = (
            (rectified_depth >= self.range_min_m)
            & (rectified_depth <= self.range_max_m)
        )
        if int(in_range.sum()) < self.min_depth_samples:
            return None, "FRUSTUM_DEPTH_SAMPLES_OUT_OF_RANGE", {
                "ar_inner_roi_xyxy_px": list(ar_inner_roi),
                "physical_left_frustum_roi_xyxy_px": list(frustum_roi),
                "sample_stride_px": stride,
                "positive_disparity_sample_count": positive_count,
                "lr_consistent_disparity_sample_count": lr_consistent_count,
                "in_range_depth_sample_count": int(in_range.sum()),
            }
        u, v, rectified_depth = u[in_range], v[in_range], rectified_depth[in_range]
        x_rect = (u - float(self.p1[0, 2])) * rectified_depth / float(self.p1[0, 0])
        y_rect = (v - float(self.p1[1, 2])) * rectified_depth / float(self.p1[1, 1])
        points_rectified = self.np.column_stack((x_rect, y_rect, rectified_depth))
        points_left = (self.r1.T @ points_rectified.T).T
        points_ar = (
            self.r_ar_to_left.T @ (points_left.T - self.t_ar_to_left_m)
        ).T
        in_front_of_ar = points_ar[:, 2] > 0.0
        if not in_front_of_ar.any():
            return None, "FRUSTUM_POINTS_BEHIND_AR0234", {
                "ar_inner_roi_xyxy_px": list(ar_inner_roi),
                "physical_left_frustum_roi_xyxy_px": list(frustum_roi),
                "in_range_depth_sample_count": int(rectified_depth.size),
            }
        points_ar, points_left, rectified_depth, u, v = (
            points_ar[in_front_of_ar],
            points_left[in_front_of_ar],
            rectified_depth[in_front_of_ar],
            u[in_front_of_ar],
            v[in_front_of_ar],
        )
        projected, _ = self.cv2.projectPoints(
            points_ar.reshape(-1, 1, 3),
            self.np.zeros((3, 1)),
            self.np.zeros((3, 1)),
            self.k_ar,
            self.d_ar,
        )
        projected = projected.reshape(-1, 2)
        ax1, ay1, ax2, ay2 = ar_inner_roi
        inside = (
            (projected[:, 0] >= ax1)
            & (projected[:, 0] <= ax2)
            & (projected[:, 1] >= ay1)
            & (projected[:, 1] <= ay2)
        )
        if int(inside.sum()) < self.min_depth_samples:
            return None, "AR_FRUSTUM_DISPARITY_SAMPLES_TOO_FEW", {
                "ar_inner_roi_xyxy_px": list(ar_inner_roi),
                "physical_left_frustum_roi_xyxy_px": list(frustum_roi),
                "in_range_depth_sample_count": int(rectified_depth.size),
                "reprojected_inside_ar_inner_roi_count": int(inside.sum()),
            }
        points_left, rectified_depth, projected, u, v = (
            points_left[inside],
            rectified_depth[inside],
            projected[inside],
            u[inside],
            v[inside],
        )
        components = self._spatial_components(u, v, frustum_roi)
        diagnostic = {
            "ar_inner_roi_xyxy_px": list(ar_inner_roi),
            "physical_left_frustum_roi_xyxy_px": list(frustum_roi),
            "sample_stride_px": stride,
            "positive_disparity_sample_count": positive_count,
            "lr_consistent_disparity_sample_count": lr_consistent_count,
            "reprojected_inside_ar_inner_roi_count": int(rectified_depth.size),
            "spatial_components": [component for _, component in components],
        }
        if not components:
            return None, "NO_SPATIAL_DISPARITY_COMPONENT", diagnostic
        dominant_count = max(component["sample_count"] for _, component in components)
        support_limit = max(
            self.min_depth_samples,
            int(
                math.ceil(
                    dominant_count * self.spatial_component_min_relative_support
                )
            ),
        )
        plausible = [
            item
            for item in components
            if (
                item[1]["sample_count"] >= support_limit
                and item[1]["coverage_area_px"]
                >= self.spatial_component_min_footprint_px
                and item[1]["bbox_width_px"] >= self.spatial_component_min_width_px
                and item[1]["bbox_height_px"] >= self.spatial_component_min_height_px
            )
        ]
        diagnostic["plausible_spatial_component_count"] = len(plausible)
        diagnostic["spatial_component_min_samples"] = support_limit
        diagnostic["spatial_component_min_footprint_px"] = (
            self.spatial_component_min_footprint_px
        )
        diagnostic["spatial_component_min_width_px"] = self.spatial_component_min_width_px
        diagnostic["spatial_component_min_height_px"] = self.spatial_component_min_height_px
        if not plausible:
            return None, "NO_PLAUSIBLE_SPATIAL_COMPONENT", diagnostic
        if len(plausible) != 1:
            return None, "MULTIPLE_PLAUSIBLE_SPATIAL_COMPONENTS", diagnostic
        component_indices, component = plausible[0]
        component_depths = rectified_depth[component_indices]
        clusters = self._depth_clusters(component_depths)
        diagnostic["selected_spatial_component"] = component
        diagnostic["depth_clusters"] = [cluster for _, cluster in clusters]
        if not clusters:
            return None, "SPATIAL_COMPONENT_DEPTH_CLUSTER_TOO_SMALL", diagnostic
        dominant_depth_count = max(cluster["sample_count"] for _, cluster in clusters)
        depth_support_limit = max(
            self.min_depth_samples,
            int(
                math.ceil(
                    dominant_depth_count * self.depth_cluster_min_relative_support
                )
            ),
        )
        plausible_depth = [
            item for item in clusters if item[1]["sample_count"] >= depth_support_limit
        ]
        diagnostic["plausible_depth_cluster_count"] = len(plausible_depth)
        diagnostic["plausible_depth_cluster_min_samples"] = depth_support_limit
        if not plausible_depth:
            return None, "NO_PLAUSIBLE_DEPTH_CLUSTER", diagnostic
        if len(plausible_depth) != 1:
            return None, "MULTIPLE_PLAUSIBLE_DEPTH_CLUSTERS", diagnostic
        relative_indices, selected = plausible_depth[0]
        selected_indices = component_indices[relative_indices]
        selected_depth = float(selected["median_depth_m"])
        selected_points = points_left[selected_indices]
        selected_projected = projected[selected_indices]
        if int(selected_points.shape[0]) < self.min_depth_samples:
            return None, "SELECTED_DEPTH_CLUSTER_TOO_FEW", diagnostic
        mad_m = float(selected["mad_m"])
        if mad_m > self.max_depth_mad_m:
            diagnostic["selected_depth_mad_m"] = mad_m
            return None, f"DEPTH_MAD_EXCEEDED:{mad_m:.4f}m", diagnostic
        point_left = self.np.median(selected_points, axis=0)
        projected_point = self.np.median(selected_projected, axis=0)
        confidence = min(
            float(observation["confidence"]),
            max(0.0, 1.0 - mad_m / self.max_depth_mad_m),
        )
        association = {
            "range_m": selected_depth,
            "x_m": float(point_left[0]),
            "y_m": float(point_left[1]),
            "z_m": float(point_left[2]),
            "bearing_deg": math.degrees(math.atan2(point_left[0], point_left[2])),
            "confidence": confidence,
            "depth_mad_m": mad_m,
            "depth_sample_count": int(selected_points.shape[0]),
            "projected_ar0234_point_px": [
                float(projected_point[0]), float(projected_point[1])
            ],
        }
        return association, "SUCCESS", diagnostic

    def _write_frustum_debug(
        self, left: Any, right: Any, disparity: Any, lr_consistent: Any,
        frustum_roi: tuple[int, int, int, int],
    ) -> None:
        """Persist raw and bilateral-consistent disparity for headless inspection."""
        left_marked = left.copy()
        x1, y1, x2, y2 = frustum_roi
        self.cv2.rectangle(left_marked, (x1, y1), (x2, y2), (0, 255, 255), 2)
        def colourise(valid: Any) -> Any:
            visual = self.np.zeros(disparity.shape, dtype=self.np.uint8)
            if valid.any():
                low, high = self.np.percentile(disparity[valid], [2, 98])
                if high > low:
                    visual[valid] = self.np.clip(
                        (disparity[valid] - low) * 255.0 / (high - low), 0, 255
                    ).astype(self.np.uint8)
            return self.cv2.applyColorMap(visual, self.cv2.COLORMAP_TURBO)

        raw_valid = self.np.isfinite(disparity) & (disparity > 0.5)
        raw_colour = colourise(raw_valid)
        consistent_colour = colourise(lr_consistent)
        self.cv2.imwrite(str(self.debug_dir / "latest_physical_left_rectified.jpg"), left_marked)
        self.cv2.imwrite(str(self.debug_dir / "latest_physical_right_rectified.jpg"), right)
        self.cv2.imwrite(str(self.debug_dir / "latest_disparity_raw.jpg"), raw_colour)
        self.cv2.imwrite(str(self.debug_dir / "latest_disparity.jpg"), consistent_colour)
        self.cv2.imwrite(
            str(self.debug_dir / "latest_lr_consistency_mask.png"),
            lr_consistent.astype(self.np.uint8) * 255,
        )

    def _audit_base(
        self, cycle_id: str, timestamp: datetime, skew_ms: float
    ) -> dict[str, Any]:
        return {
            "schema_version": "sie.ar0234_ov9281.dynamic_association_audit.v1",
            "audit_id": f"dynamic-association-audit:{cycle_id}",
            "timestamp": timestamp.isoformat(),
            "cycle_id": cycle_id,
            "reference_frame": "ov9281_physical_left_optical_frame",
            "calibration": {
                "ar0234_intrinsic_sha256": self.ar_intrinsic_sha256,
                "ov9281_stereo_sha256": self.stereo_calibration_sha256,
                "ar0234_to_physical_left_extrinsic_sha256": self.extrinsic_sha256,
            },
            "pairing": {
                "capture_order": "ar0234_then_ov9281_combined",
                "capture_pair_skew_ms": skew_ms,
                "max_configured_pair_skew_ms": self.max_pair_skew_ms,
                "host_pairing_diagnostic_only": True,
                "exposure_synchronization_proven": False,
            },
            "temporal_applicability": {
                "status": "NOT_APPROVED",
                "reason": "HOST_CAPTURE_TIMESTAMPS_ARE_NOT_EXPOSURE_SYNCHRONIZATION",
            },
            "metric_measurement_authorized": False,
            "navigation_eligible": False,
            "actuator_access": False,
        }

    def _publish_audit(self, audit: dict[str, Any]) -> None:
        message = String()
        message.data = encode(audit)
        self.audit_publisher.publish(message)

    def _cycle(self) -> None:
        self.sequence += 1
        timestamp = datetime.now(timezone.utc)
        ar_frame, ar_mono = self._read(self.ar_capture)
        combined, stereo_mono = self._read(self.stereo_capture)
        skew_ms = abs(stereo_mono - ar_mono) / 1_000_000.0
        cycle_id = f"ar0234-ov9281-dynamic-audit-{self.run_id}-{self.sequence:08d}"
        audit = self._audit_base(cycle_id, timestamp, skew_ms)

        if ar_frame is None or combined is None:
            self._publish_audit(
                {**audit, "status": "CAMERA_FRAME_UNAVAILABLE", "reason": "CAMERA_FRAME_UNAVAILABLE"}
            )
            return
        if ar_frame.shape[:2] != (1200, 1920) or combined.shape[:2] != (
            self.stereo_size[1],
            self.stereo_size[0] * 2,
        ):
            self._publish_audit(
                {**audit, "status": "CALIBRATION_INVALID", "reason": "UNEXPECTED_FRAME_SIZE"}
            )
            return
        if skew_ms > self.max_pair_skew_ms:
            self._publish_audit(
                {**audit, "status": "PAIR_SKEW_EXCEEDED", "reason": "PAIR_SKEW_EXCEEDED"}
            )
            return

        try:
            raw = validate_ar0234_observation(
                self.observer.observe(ar_frame, captured_at_utc=timestamp, cycle_id=cycle_id)
            )
            # Keep unmodified model evidence visible to the existing tracker.
            self._publish_observation(raw)
            dynamic = ar0234_dynamic_target_candidate(raw)
            audit["ar0234_dynamic_target"] = dynamic
            if dynamic["disposition"] != "OBSERVED":
                self._publish_audit(
                    {
                        **audit,
                        "status": "TARGET_UNAVAILABLE",
                        "reason": dynamic["reason"],
                        "stereo_person": "NOT_USED_AR0234_IS_SEMANTIC_AUTHORITY",
                        "stereo_geometry": None,
                        "association": None,
                    }
                )
                return

            candidate = dynamic["candidate"]
            if type(candidate) is not dict:
                raise ContractError("observed dynamic target has no candidate")
            observation = self._selected_observation(raw, candidate, dynamic)

            # Pi stream identity is fixed by the validated calibration: first
            # half is physical_right, second half is physical_left.
            physical_right = combined[:, : self.stereo_size[0]]
            physical_left = combined[:, self.stereo_size[0] :]
            left_rectified = self.cv2.remap(
                physical_left, self.map_left_1, self.map_left_2, self.cv2.INTER_LINEAR
            )
            right_rectified = self.cv2.remap(
                physical_right, self.map_right_1, self.map_right_2, self.cv2.INTER_LINEAR
            )
            # Do not run a second person classifier on OV9281.  The calibrated
            # AR inner ROI defines a 0.5–4.5 m stereo frustum; disparity points
            # survive only if their 3D back-projection lands inside that ROI.
            association, reason, geometry = self._frustum_associate(
                observation, left_rectified, right_rectified
            )
            audit["stereo_person"] = "NOT_USED_AR0234_IS_SEMANTIC_AUTHORITY"
            audit["stereo_geometry"] = geometry
            if association is None:
                self._publish_audit(
                    {
                        **audit,
                        "status": "ASSOCIATION_UNAVAILABLE",
                        "reason": reason,
                        "association": None,
                    }
                )
                return
            self._publish_audit(
                {
                    **audit,
                    "status": "ASSOCIATED_DIAGNOSTIC_ONLY",
                    "reason": "AR0234_FRUSTUM_STEREO_ASSOCIATION",
                    "association": {
                        **association,
                        "projection_diagnostic": self._projection_diagnostic(
                            observation, association
                        ),
                    },
                }
            )
        except (ContractError, OSError, RuntimeError, ValueError, KeyError) as error:
            self.get_logger().warning(f"dynamic association audit refused: {error}")
            self._publish_audit(
                {
                    **audit,
                    "status": "PROCESSING_REFUSED",
                    "reason": "PROCESSING_REFUSED",
                }
            )

    def _load_profile(self) -> None:
        super()._load_profile()
        # Cache these once for the evidence stream.  They remain SHA-bound to
        # the activation profile checks performed by the parent implementation.
        from .ar0234_ov9281_metric_measurement_node import sha256_file

        self.ar_intrinsic_sha256 = sha256_file(self.ar_intrinsic_path)
        self.stereo_calibration_sha256 = sha256_file(self.stereo_path)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Ar0234Ov9281DynamicAssociationAuditNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
