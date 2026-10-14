"""Foreground RGB/stereo control-input preview, with deferred compact journaling.

Same full-resolution geometry and association algorithm as the audit. RGB and
stereo matching run concurrently. Current-frame mask and local shape diagnostics run in an isolated companion.
This diagnostic topic is NOT a Measurement, cmd_vel or ESP32 command interface.
"""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String

from .ar0234_ov9281_dynamic_association_audit_node import Ar0234Ov9281DynamicAssociationAuditNode
from .contracts import validate_ar0234_observation, ar0234_temporal_target_candidate, encode
from .latest_diagnostic_writer import LatestDiagnosticWriter
from .approach_control_preview import approach_control_preview
from .preview_mask_client import PreviewMaskClient


class Ar0234Ov9281ControlPreviewNode(Ar0234Ov9281DynamicAssociationAuditNode):
    def __init__(self):
        super().__init__()
        if self.evidence_capture_enabled or self.shadow_instance_mask_enabled or self.shadow_local_surfaces_enabled:
            raise ValueError("control preview excludes blocking evidence/mask/surface stages; use the audit node for those")
        self.declare_parameter("preview_mask_surface_enabled", True)
        self._mask_client = None
        if bool(self.get_parameter("preview_mask_surface_enabled").value):
            self._mask_client = PreviewMaskClient(
                self.get_parameter("shadow_segmenter_python").value,
                self.project_root / "vision_core/tools/control_preview_mask_worker.py",
                self.get_parameter("shadow_segmenter_model").value,
                self.shadow_segmenter_timeout_s)
        self.declare_parameter("control_preview_topic", "/sie/diagnostics/ar0234_ov9281/control_preview")
        self.declare_parameter("preview_diagnostic_range_min_m", .5)
        self.declare_parameter("preview_diagnostic_range_max_m", 4.5)
        self._activated_range = [self.range_min_m, self.range_max_m]
        lower = float(self.get_parameter("preview_diagnostic_range_min_m").value)
        upper = float(self.get_parameter("preview_diagnostic_range_max_m").value)
        if not 0 < lower < upper <= 4.5:
            raise ValueError("invalid preview diagnostic range")
        # This subclass emits hypotheses only, outside the original activation
        # when explicitly declared here. No activation/profile file is modified.
        self.range_min_m, self.range_max_m = lower, upper
        self.preview_publisher = self.create_publisher(String, str(self.get_parameter("control_preview_topic").value), 1)
        self._perception_worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="sie-preview-stereo")
        self._cached_stereo = None
        self._writer = LatestDiagnosticWriter(self.evidence_dir / ("control-preview-" + self.run_id))
        self.get_logger().info(
            "control input preview: concurrent RGB/full-resolution stereo; immediate publish; "
            "background compact journal; current mask/local diagnostics; no persistent images; no Measurement or motor command"
        )

    def _compute_stereo(self, combined):
        started = time.monotonic()
        width = self.stereo_size[0]
        right = self.cv2.remap(combined[:, :width], self.map_right_1, self.map_right_2, self.cv2.INTER_LINEAR)
        left = self.cv2.remap(combined[:, width:], self.map_left_1, self.map_left_2, self.cv2.INTER_LINEAR)
        left_gray = self.cv2.cvtColor(left, self.cv2.COLOR_BGR2GRAY)
        right_gray = self.cv2.cvtColor(right, self.cv2.COLOR_BGR2GRAY)
        # Invoke the original full-resolution LR implementation once. Foreground
        # timers are intentionally absent on this worker thread.
        result = Ar0234Ov9281DynamicAssociationAuditNode._left_right_disparity(self, left_gray, right_gray)
        return left, right, result, time.monotonic() - started

    def _left_right_disparity(self, left_gray, right_gray):
        if self._cached_stereo is None:
            raise RuntimeError("no current-cycle cached stereo")
        return self._cached_stereo

    def _persist_disparity_evidence(self, *args):
        pass

    def _write_frustum_debug(self, *args):
        pass

    def _write_evidence_array(self, *args):
        pass

    def _cycle(self):
        if not rclpy.ok():
            return
        self.sequence += 1
        started = time.monotonic()
        self._cached_stereo = None
        self._last_stereo_worker_timings = None
        # Parent LR code calls _timed_call/decorators. Never share a mutable
        # timing collector across its worker and the foreground.
        self._stage_timings = None
        stamp = datetime.now(timezone.utc)
        ar, ar_mono = self._read(self.ar_capture)
        combined, stereo_mono = self._read(self.stereo_capture)
        cycle_id = f"ar0234-ov9281-control-preview-{self.run_id}-{self.sequence:08d}"
        skew_ms = abs(stereo_mono - ar_mono) / 1e6
        audit = self._audit_base(cycle_id, stamp, skew_ms)
        audit.update(schema_version="sie.ar0234_ov9281.control_input_preview.v1",
                     status="PROCESSING_REFUSED", reason="PROCESSING_REFUSED", association=None,
                     diagnostic_depth_range_m=[self.range_min_m, self.range_max_m],
                     activated_depth_range_m=self._activated_range,
                     shape_diagnostic={"status":"NOT_EVALUATED_NO_CURRENT_TARGET"})
        future = None
        try:
            if ar is None or combined is None:
                audit.update(status="CAMERA_FRAME_UNAVAILABLE", reason="CAMERA_FRAME_UNAVAILABLE")
                return
            if ar.shape[:2] != (1200, 1920) or combined.shape[:2] != (self.stereo_size[1], self.stereo_size[0]*2):
                audit.update(status="CALIBRATION_INVALID", reason="UNEXPECTED_FRAME_SIZE")
                return
            if skew_ms > self.max_pair_skew_ms:
                audit.update(status="PAIR_SKEW_EXCEEDED", reason="PAIR_SKEW_EXCEEDED")
                return
            future = self._perception_worker.submit(self._compute_stereo, combined)
            detector_start = time.monotonic()
            raw = validate_ar0234_observation(self.observer.observe(ar, captured_at_utc=stamp, cycle_id=cycle_id))
            audit["rgb_detector_s"] = time.monotonic() - detector_start
            self._publish_observation(raw)
            dynamic = ar0234_temporal_target_candidate(
                raw, prior_track=self._confirmed_target_track,
                max_age_s=self.temporal_target_track_max_age_s,
                minimum_iou=self.temporal_target_minimum_iou,
                maximum_center_displacement_relative=self.temporal_target_maximum_center_displacement_relative,
                minimum_match_margin=self.temporal_target_minimum_match_margin)
            audit["ar0234_dynamic_target"] = dynamic
            if dynamic["disposition"] != "OBSERVED":
                audit.update(status="TARGET_UNAVAILABLE", reason=dynamic["reason"])
                return
            candidate = dynamic["candidate"]
            geometry = self._dynamic_target_geometry(candidate, ar.shape[1])
            if not geometry["accepted"]:
                audit.update(status="TARGET_UNAVAILABLE", reason=geometry["reason"])
                return
            observation = self._selected_observation(raw, candidate, dynamic)
            left, right, self._cached_stereo, stereo_s = future.result()
            audit["stereo_worker_s"] = stereo_s
            audit["stereo_execution"]["worker_timings_s"] = self._last_stereo_worker_timings
            association, reason, geometry = self._frustum_associate(observation, left, right)
            audit.update(stereo_geometry=geometry, association=association,
                         status="ASSOCIATED_DIAGNOSTIC_ONLY" if association else "ASSOCIATION_UNAVAILABLE",
                         reason="AR0234_FRUSTUM_STEREO_ASSOCIATION" if association else reason)
            audit["original_association"] = {"status":audit["status"], "reason":audit["reason"],
                                              "association":association}
            # Independent of the old range/component/global-MAD refusal. Never
            # relabel this shape description as admitted depth or an Action.
            client = getattr(self, "_mask_client", None)
            if client is not None:
                try:
                    disparity, lr = self._cached_stereo
                    current = self.np.where(lr, disparity, self.np.nan)
                    cal = {"P1":self.p1, "P2":self.p2, "R1":self.r1,
                           "R":self.r_ar_to_left, "T":self.t_ar_to_left_m.reshape(3),
                           "K":self.k_ar, "D":self.d_ar, "size":self.stereo_size}
                    audit["shape_diagnostic"] = client.evaluate(cycle_id, ar, current, cal,
                        candidate, self._ar_inner_roi(observation), [self.range_min_m,self.range_max_m])
                except Exception as error:
                    audit["shape_diagnostic"] = {"status":"MASK_SURFACE_UNAVAILABLE",
                        "reason":type(error).__name__, "error_detail":str(error)[:400]}
            else:
                audit["shape_diagnostic"] = {"status":"DISABLED"}
            if association is not None:
                self._confirm_current_target_track(candidate, observation)
        except Exception as error:
            audit.update(status="PROCESSING_REFUSED", reason=type(error).__name__, error_detail=str(error)[:400], association=None)
        except KeyboardInterrupt:
            audit.update(status="CANCELLED_SHUTDOWN", reason="OPERATOR_INTERRUPT", association=None)
            raise
        finally:
            if not rclpy.ok():
                audit.update(status="CANCELLED_SHUTDOWN", reason="ROS_SHUTDOWN", association=None)
            # Drain a started stereo job even when detection failed. No stale
            # frame/map can leak to a later callback or a reused matcher.
            if future is not None:
                try:
                    future.result()
                except Exception:
                    pass
            self._cached_stereo = None
            # A camera-frame diagnostic is never substituted for an admitted
            # robot-front clearance/error/age input to the motion policy.
            audit["decision_preview"] = approach_control_preview()
            elapsed = time.monotonic() - started
            audit["processing_timing"] = {"host_capture_start_to_result_ready_s": elapsed,
                "exposure_age_known": False, "endpoint": "BEFORE_PREVIEW_PUBLISH_AND_BACKGROUND_JOURNAL"}
            audit["working_targets"] = {"base_speed_m_s": .15, "stand_off_from_robot_front_m": 1.,
                "depth_absolute_error_1_to_2m_m": .10, "depth_relative_error_above_2m": .05,
                "exposure_to_command_age_max_s": .25}
            audit["admission_blockers"] = ["DIAGNOSTIC_NOT_METRIC_MEASUREMENT", "EXPOSURE_AGE_NOT_ESTABLISHED",
                "CAMERA_TO_ROBOT_FRONT_CLEARANCE_NOT_ESTABLISHED", "PHYSICAL_ERROR_BOUND_NOT_ESTABLISHED"]
            if elapsed > .25:
                audit["admission_blockers"].append("HOST_PROCESSING_EXCEEDS_250MS_TARGET")
            candidate_depth = (audit.get("association") or {}).get("range_m")
            if candidate_depth is not None and not self._activated_range[0] <= candidate_depth <= self._activated_range[1]:
                audit["admission_blockers"].append("DIAGNOSTIC_DEPTH_OUTSIDE_ACTIVATED_PROFILE")
            audit["journal"] = {"dropped_pending": self._writer.dropped, "write_errors": self._writer.errors}
            message = String()
            message.data = encode(audit)
            if rclpy.ok():
                self.preview_publisher.publish(message)  # FIRST, before any journal disk write.
            self._writer.submit(audit)
            depth = (audit.get("association") or {}).get("range_m")
            self.get_logger().info(f"CONTROL_PREVIEW {self.sequence:08d} status={audit['status']} "
                f"range_m={depth} host_s={elapsed:.3f} host_within_250ms={elapsed <= .25} motors=DISABLED")

    def destroy_node(self):
        client = getattr(self, "_mask_client", None)
        if client is not None:
            client.close()
        worker = getattr(self, "_perception_worker", None)
        if worker is not None:
            worker.shutdown(wait=True, cancel_futures=True)
        writer = getattr(self, "_writer", None)
        if writer is not None:
            writer.close()
            self.get_logger().info(f"CONTROL_PREVIEW_JOURNAL dropped={writer.dropped} errors={writer.errors}")
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Ar0234Ov9281ControlPreviewNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
