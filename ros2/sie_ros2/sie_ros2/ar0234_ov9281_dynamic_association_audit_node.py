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
        # The parent initializes only calibrated camera capture, rectification,
        # and the two detectors.  This subclass never calls its Measurement
        # publishing methods and never evaluates its static-scene gate.
        super().__init__()
        self.declare_parameter(
            "audit_topic", "/sie/diagnostics/ar0234_ov9281/dynamic_association"
        )
        self.audit_topic = str(self.get_parameter("audit_topic").value)
        self.audit_publisher = self.create_publisher(String, self.audit_topic, 10)
        self.get_logger().info(
            "dynamic association audit enabled; "
            f"publishing diagnostic-only evidence to {self.audit_topic}; "
            "metric Measurement, navigation, and actuator access remain disabled"
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
                        "stereo_person": None,
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
            left_evidence = self._observe_stereo_person(
                left_rectified, cycle_id=f"{cycle_id}:physical_left"
            )
            audit["stereo_person"] = left_evidence
            association, reason = self._associate(
                observation, left_evidence, left_rectified, right_rectified
            )
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
                    "reason": "CURRENT_FRAME_DYNAMIC_ASSOCIATION",
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
