"""Live AR0234+OV9281 metric Measurement producer for the activated profile.

This is deliberately a stop-and-measure node.  It reads both cameras in one
cycle, creates a fresh AR0234 Observation, and emits a metric Measurement only
when one rectified OV9281 person candidate has a valid 3D association to that
Observation.  It never accesses actuators.
"""
from __future__ import annotations

import hashlib
import json
import math
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .contracts import (
    ContractError,
    ar0234_target_suitability,
    encode,
    validate_ar0234_observation,
)

DEFAULT_AR_DEVICE = "/dev/v4l/by-id/usb-DECXIN_CAMERA_DECXIN_CAMERA_01.00.00-video-index0"
DEFAULT_STEREO_DEVICE = "/dev/v4l/by-id/usb-TSTC_Web_Camera_TSTC_Web_Camera-video-index0"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def absolute_parameter(node: Node, name: str) -> Path:
    value = str(node.get_parameter(name).value)
    path = Path(value).expanduser()
    if not value or not path.is_absolute():
        raise ValueError(f"{name} must be an absolute path")
    return path


def scalar_text(value: object) -> str:
    import numpy as np

    array = np.asarray(value)
    return str(array.item() if array.shape == () else array.reshape(-1)[0])


class Ar0234Ov9281MetricMeasurementNode(Node):
    """Publish fail-closed, SHA-bound metric measurements from a static scene."""

    def __init__(self) -> None:
        super().__init__("sie_ar0234_ov9281_metric_measurement")
        self.declare_parameter("project_root", "")
        self.declare_parameter("model_path", "")
        self.declare_parameter(
            "activation_profile",
            "/home/elwis/dev_ws/runtime_artifacts/ar0234_ov9281_decision_chain_v1/"
            "ar0234_ov9281_decision_chain_activation.json",
        )
        self.declare_parameter(
            "ar_intrinsic",
            "/home/elwis/dev_ws/runtime_artifacts/ar0234_intrinsic_v5_final_20260910/"
            "calibration_fullres.json",
        )
        self.declare_parameter(
            "stereo_calibration",
            "/home/elwis/dev_ws/runtime_artifacts/ov9281_pi_stereo_v7/"
            "ov9281_pi_stereo_v7.npz",
        )
        self.declare_parameter("ar_device", DEFAULT_AR_DEVICE)
        self.declare_parameter("stereo_device", DEFAULT_STEREO_DEVICE)
        self.declare_parameter("output_topic", "/sie/perception/raw_measurement")
        self.declare_parameter(
            "observation_topic", "/sie/observations/ar0234_person_upper_body"
        )
        self.declare_parameter("frame_rate_hz", 0.5)
        self.declare_parameter("max_pair_skew_ms", 80.0)
        self.declare_parameter("confidence_threshold", 0.40)
        self.declare_parameter("min_depth_samples", 100)
        self.declare_parameter("max_depth_mad_m", 0.05)
        self.declare_parameter("auto_exposure", 3)
        self.declare_parameter(
            "debug_dir", "~/.local/state/sie/debug/ar0234_ov9281_live"
        )

        self.project_root = absolute_parameter(self, "project_root")
        self.model_path = absolute_parameter(self, "model_path")
        self.activation_path = absolute_parameter(self, "activation_profile")
        self.ar_intrinsic_path = absolute_parameter(self, "ar_intrinsic")
        self.stereo_path = absolute_parameter(self, "stereo_calibration")
        self.ar_device = str(self.get_parameter("ar_device").value)
        self.stereo_device = str(self.get_parameter("stereo_device").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.observation_topic = str(self.get_parameter("observation_topic").value)
        self.frame_rate_hz = float(self.get_parameter("frame_rate_hz").value)
        self.max_pair_skew_ms = float(self.get_parameter("max_pair_skew_ms").value)
        self.min_depth_samples = int(self.get_parameter("min_depth_samples").value)
        self.max_depth_mad_m = float(self.get_parameter("max_depth_mad_m").value)
        self.confidence_threshold = float(
            self.get_parameter("confidence_threshold").value
        )
        self.debug_dir = Path(
            str(self.get_parameter("debug_dir").value)
        ).expanduser()
        self.debug_dir.mkdir(parents=True, exist_ok=True)
        if (
            self.frame_rate_hz <= 0
            or self.max_pair_skew_ms <= 0
            or self.min_depth_samples < 1
            or self.max_depth_mad_m <= 0
            or not 0 <= self.confidence_threshold <= 1
        ):
            raise ValueError("invalid live measurement parameters")

        if str(self.project_root) not in sys.path:
            sys.path.insert(0, str(self.project_root))
        import cv2
        import numpy as np
        from vision_core.person_localization.yolo11_person_upper_body_runtime import (
            OnnxRuntimeYolo11PersonUpperBodyObserver,
        )

        self.cv2 = cv2
        self.np = np
        self._load_profile()
        self.observer = OnnxRuntimeYolo11PersonUpperBodyObserver(
            self.model_path, confidence_threshold=self.confidence_threshold
        )
        exposure = int(self.get_parameter("auto_exposure").value)
        self._set_auto_exposure(self.ar_device, exposure)
        self._set_auto_exposure(self.stereo_device, exposure)
        self.ar_capture = self._open_camera(self.ar_device, 1920, 1200, 30.0)
        self.stereo_capture = self._open_camera(
            self.stereo_device, self.stereo_size[0] * 2, self.stereo_size[1], 60.0
        )
        self.measurement_publisher = self.create_publisher(String, self.output_topic, 10)
        self.observation_publisher = self.create_publisher(
            String, self.observation_topic, 10
        )
        self.sequence = 0
        self.timer = self.create_timer(1.0 / self.frame_rate_hz, self._cycle)
        self.get_logger().info(
            "live AR0234+OV9281 metric Measurement -> "
            f"{self.output_topic}; range={self.range_min_m:.1f}..{self.range_max_m:.1f} m; "
            "stop-and-measure only; actuator access disabled"
        )

    def _load_profile(self) -> None:
        if not all(
            path.is_file()
            for path in (
                self.activation_path,
                self.ar_intrinsic_path,
                self.stereo_path,
            )
        ):
            raise FileNotFoundError("activation or calibration artifact is missing")
        activation = json.loads(self.activation_path.read_text(encoding="utf-8"))
        if (
            activation.get("status")
            != "ACTIVE_CONDITIONAL_SUPERVISED_DECISION_RECOMMENDATION_ONLY"
        ):
            raise RuntimeError("decision-chain activation profile is not active")
        authorization = activation.get("authorization")
        if type(authorization) is not dict or authorization.get("execution_authorized") is not False:
            raise RuntimeError("activation must explicitly forbid execution")
        range_value = authorization.get("metric_measurement_range_m")
        if (
            type(range_value) is not list
            or len(range_value) != 2
            or not all(type(item) in (int, float) for item in range_value)
        ):
            raise RuntimeError("activation has no valid metric range")
        self.range_min_m, self.range_max_m = map(float, range_value)
        if self.range_min_m <= 0 or self.range_max_m <= self.range_min_m:
            raise RuntimeError("invalid activated range")

        source = activation.get("calibration_profile", {}).get("source_sha256", {})
        if (
            sha256_file(self.ar_intrinsic_path) != source.get("ar0234_intrinsic")
            or sha256_file(self.stereo_path) != source.get("ov9281_stereo_calibration")
        ):
            raise RuntimeError("runtime calibration SHA-256 does not match activation")
        extrinsic = activation.get("calibration_profile", {}).get(
            "ar0234_to_ov9281_physical_left_extrinsic", {}
        )
        self.extrinsic_path = Path(str(extrinsic.get("path", ""))).expanduser()
        if (
            not self.extrinsic_path.is_file()
            or sha256_file(self.extrinsic_path) != extrinsic.get("sha256")
        ):
            raise RuntimeError("activated extrinsic artifact is absent or changed")
        self.extrinsic_sha256 = str(extrinsic["sha256"])

        ar_intrinsic = json.loads(self.ar_intrinsic_path.read_text(encoding="utf-8"))
        self.k_ar = self.np.asarray(ar_intrinsic["camera_matrix"], dtype=self.np.float64)
        self.d_ar = self.np.asarray(
            ar_intrinsic["distortion_coefficients"], dtype=self.np.float64
        )
        with self.np.load(self.stereo_path, allow_pickle=False) as stereo:
            if scalar_text(stereo["camera_1_semantics"]) != "physical_left":
                raise RuntimeError("stereo K1 must be physical_left")
            if scalar_text(stereo["camera_2_semantics"]) != "physical_right":
                raise RuntimeError("stereo K2 must be physical_right")
            self.k_left, self.d_left = stereo["K1"], stereo["D1"]
            self.k_right, self.d_right = stereo["K2"], stereo["D2"]
            self.r1, self.r2 = stereo["R1"], stereo["R2"]
            self.p1, self.p2 = stereo["P1"], stereo["P2"]
            self.stereo_size = tuple(int(item) for item in stereo["size"])
        with self.np.load(self.extrinsic_path, allow_pickle=False) as candidate:
            candidate_stereo_sha = scalar_text(
                candidate["source_stereo_calibration_sha256"]
            )
            candidate_ar_sha = scalar_text(candidate["source_ar_intrinsic_sha256"])
            if (
                candidate_stereo_sha != sha256_file(self.stereo_path)
                or candidate_ar_sha != sha256_file(self.ar_intrinsic_path)
            ):
                raise RuntimeError("extrinsic candidate source SHA-256 mismatch")
            self.r_ar_to_left = candidate["R_ar0234_to_physical_left"]
            self.t_ar_to_left_m = (
                candidate["T_ar0234_to_physical_left_mm"].reshape(3, 1) / 1000.0
            )

        self.map_left_1, self.map_left_2 = self.cv2.initUndistortRectifyMap(
            self.k_left, self.d_left, self.r1, self.p1, self.stereo_size, self.cv2.CV_32FC1
        )
        self.map_right_1, self.map_right_2 = self.cv2.initUndistortRectifyMap(
            self.k_right, self.d_right, self.r2, self.p2, self.stereo_size, self.cv2.CV_32FC1
        )
        self.sgbm = self.cv2.StereoSGBM_create(
            minDisparity=0,
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

    def _set_auto_exposure(self, device: str, value: int) -> None:
        subprocess.run(
            ["v4l2-ctl", "-d", device, "-c", f"auto_exposure={value}"],
            check=True, text=True, capture_output=True,
        )
        check = subprocess.run(
            ["v4l2-ctl", "-d", device, "-C", "auto_exposure"],
            check=True, text=True, capture_output=True,
        ).stdout.strip()
        if f": {value}" not in check:
            raise RuntimeError(f"auto_exposure verification failed for {device}: {check}")

    def _open_camera(self, device: str, width: int, height: int, fps: float) -> Any:
        capture = self.cv2.VideoCapture(device, self.cv2.CAP_V4L2)
        capture.set(self.cv2.CAP_PROP_FOURCC, self.cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(self.cv2.CAP_PROP_FRAME_WIDTH, width)
        capture.set(self.cv2.CAP_PROP_FRAME_HEIGHT, height)
        capture.set(self.cv2.CAP_PROP_FPS, fps)
        capture.set(self.cv2.CAP_PROP_BUFFERSIZE, 1)
        if not capture.isOpened():
            raise RuntimeError(f"cannot open camera {device}")
        for _ in range(8):
            capture.read()
        return capture

    def _read(self, capture: Any) -> tuple[Any | None, int]:
        ok, frame = capture.read()
        return (frame if ok and frame is not None else None), time.monotonic_ns()

    def _cycle(self) -> None:
        self.sequence += 1
        timestamp = datetime.now(timezone.utc)
        ar_frame, ar_mono = self._read(self.ar_capture)
        combined, stereo_mono = self._read(self.stereo_capture)
        skew_ms = abs(stereo_mono - ar_mono) / 1_000_000.0
        cycle_id = f"ar0234-ov9281-live-{self.sequence:08d}"
        if ar_frame is None or combined is None:
            self._publish_refusal(
                cycle_id, timestamp, "DEPTH_UNAVAILABLE", "CAMERA_FRAME_UNAVAILABLE", skew_ms
            )
            return
        if ar_frame.shape[:2] != (1200, 1920) or combined.shape[:2] != (
            self.stereo_size[1],
            self.stereo_size[0] * 2,
        ):
            self._publish_refusal(
                cycle_id, timestamp, "CALIBRATION_INVALID", "UNEXPECTED_FRAME_SIZE", skew_ms
            )
            return
        if skew_ms > self.max_pair_skew_ms:
            self._publish_refusal(
                cycle_id, timestamp, "DEPTH_UNAVAILABLE", "PAIR_SKEW_EXCEEDED", skew_ms
            )
            return

        try:
            observation = self.observer.observe(
                ar_frame, captured_at_utc=timestamp, cycle_id=cycle_id
            )
            observation = validate_ar0234_observation(observation)
            self._publish_observation(observation)
            suitability = ar0234_target_suitability(observation)
            if not suitability["geometry_eligible"]:
                status = (
                    "MULTIPLE_TARGETS"
                    if observation["target_status"] == "MULTIPLE_TARGETS"
                    else "NO_TARGET"
                )
                self._publish_refusal(cycle_id, timestamp, status, suitability["reason"], skew_ms, observation)
                return

            # Pi stream identity: first combined half is physical_right; second is physical_left.
            physical_right = combined[:, : self.stereo_size[0]]
            physical_left = combined[:, self.stereo_size[0] :]
            left_rectified = self.cv2.remap(
                physical_left, self.map_left_1, self.map_left_2, self.cv2.INTER_LINEAR
            )
            right_rectified = self.cv2.remap(
                physical_right, self.map_right_1, self.map_right_2, self.cv2.INTER_LINEAR
            )
            left_evidence = self.observer.observe(
                left_rectified, captured_at_utc=timestamp, cycle_id=f"{cycle_id}:physical_left"
            )
            association, association_reason = self._associate(
                observation, left_evidence, left_rectified, right_rectified
            )
            if association is None:
                self._publish_refusal(
                    cycle_id, timestamp, "DEPTH_UNAVAILABLE", association_reason,
                    skew_ms, observation
                )
                return
            self._publish_success(
                cycle_id, timestamp, skew_ms, observation, left_evidence, association
            )
        except (ContractError, OSError, RuntimeError, ValueError, KeyError) as error:
            self.get_logger().warning(f"live metric measurement refused: {error}")
            self._publish_refusal(
                cycle_id, timestamp, "DEPTH_UNAVAILABLE", "PROCESSING_REFUSED", skew_ms
            )

    def _associate(
        self, ar_observation: dict[str, Any], left_evidence: dict[str, Any],
        left_rectified: Any, right_rectified: Any,
    ) -> tuple[dict[str, Any] | None, str]:
        left_status = left_evidence.get("target_status")
        if left_status != "SINGLE_TARGET":
            return None, f"STEREO_PERSON_{left_status}"
        bbox = left_evidence.get("bbox_xyxy_px")
        if type(bbox) is not list or len(bbox) != 4:
            return None, "STEREO_PERSON_INVALID_BBOX"
        x1, y1, x2, y2 = (float(item) for item in bbox)
        width, height = self.stereo_size
        ix1 = max(0, int(x1 + 0.30 * (x2 - x1)))
        ix2 = min(width, int(x1 + 0.70 * (x2 - x1)))
        iy1 = max(0, int(y1 + 0.35 * (y2 - y1)))
        iy2 = min(height, int(y1 + 0.85 * (y2 - y1)))
        if ix2 <= ix1 or iy2 <= iy1:
            return None, "STEREO_DEPTH_ROI_EMPTY"
        left_gray = self.cv2.cvtColor(left_rectified, self.cv2.COLOR_BGR2GRAY)
        right_gray = self.cv2.cvtColor(right_rectified, self.cv2.COLOR_BGR2GRAY)
        disparity = self.sgbm.compute(left_gray, right_gray).astype(self.np.float32) / 16.0
        self._write_stereo_debug(left_rectified, right_rectified, bbox, disparity)
        samples = disparity[iy1:iy2, ix1:ix2].reshape(-1)
        samples = samples[self.np.isfinite(samples) & (samples > 0.5)]
        if samples.size < self.min_depth_samples:
            return None, f"DISPARITY_SAMPLES_TOO_FEW:{samples.size}"
        raw_depth_samples = abs(float(self.p2[0, 3])) / samples / 1000.0
        depth_samples = raw_depth_samples[
            (raw_depth_samples >= self.range_min_m)
            & (raw_depth_samples <= self.range_max_m)
        ]
        if depth_samples.size < self.min_depth_samples:
            return None, (
                f"DEPTH_SAMPLES_OUT_OF_RANGE:{depth_samples.size};"
                f"disp_median_px={float(self.np.median(samples)):.3f};"
                f"raw_depth_median_m={float(self.np.median(raw_depth_samples)):.3f}"
            )
        depth_m = float(self.np.median(depth_samples))
        mad_m = float(self.np.median(self.np.abs(depth_samples - depth_m)))
        if mad_m > self.max_depth_mad_m:
            return None, f"DEPTH_MAD_EXCEEDED:{mad_m:.4f}m"
        u, v = (ix1 + ix2) / 2.0, (iy1 + iy2) / 2.0
        x_rect = (u - float(self.p1[0, 2])) * depth_m / float(self.p1[0, 0])
        y_rect = (v - float(self.p1[1, 2])) * depth_m / float(self.p1[1, 1])
        point_left = self.r1.T @ self.np.array([[x_rect], [y_rect], [depth_m]])
        point_ar = self.r_ar_to_left.T @ (point_left - self.t_ar_to_left_m)
        projected, _ = self.cv2.projectPoints(
            point_ar.reshape(1, 1, 3), self.np.zeros((3, 1)), self.np.zeros((3, 1)),
            self.k_ar, self.d_ar,
        )
        ar_x, ar_y = (float(value) for value in projected.reshape(2))
        ax1, ay1, ax2, ay2 = (float(value) for value in ar_observation["bbox_xyxy_px"])
        if not (ax1 <= ar_x <= ax2 and ay1 <= ar_y <= ay2):
            return None, f"AR_REPROJECTION_OUTSIDE_BBOX:{ar_x:.1f},{ar_y:.1f}"
        confidence = min(
            float(ar_observation["confidence"]),
            float(left_evidence["confidence"]),
            max(0.0, 1.0 - mad_m / self.max_depth_mad_m),
        )
        return {
            "range_m": depth_m,
            "x_m": float(point_left[0, 0]),
            "y_m": float(point_left[1, 0]),
            "z_m": float(point_left[2, 0]),
            "bearing_deg": math.degrees(math.atan2(point_left[0, 0], point_left[2, 0])),
            "confidence": confidence,
            "depth_mad_m": mad_m,
            "depth_sample_count": int(depth_samples.size),
            "physical_left_bbox_xyxy_px": [x1, y1, x2, y2],
            "projected_ar0234_point_px": [ar_x, ar_y],
        }, "SUCCESS"

    def _write_stereo_debug(
        self, left: Any, right: Any, bbox: list[float], disparity: Any
    ) -> None:
        """Persist headless evidence; diagnostics do not alter Measurement policy."""
        left_marked = left.copy()
        x1, y1, x2, y2 = (int(round(value)) for value in bbox)
        self.cv2.rectangle(left_marked, (x1, y1), (x2, y2), (0, 255, 0), 2)
        valid = self.np.isfinite(disparity) & (disparity > 0.5)
        visual = self.np.zeros(disparity.shape, dtype=self.np.uint8)
        if valid.any():
            low, high = self.np.percentile(disparity[valid], [2, 98])
            if high > low:
                visual[valid] = self.np.clip(
                    (disparity[valid] - low) * 255.0 / (high - low), 0, 255
                ).astype(self.np.uint8)
        colour = self.cv2.applyColorMap(visual, self.cv2.COLORMAP_TURBO)
        self.cv2.imwrite(str(self.debug_dir / "latest_physical_left_rectified.jpg"), left_marked)
        self.cv2.imwrite(str(self.debug_dir / "latest_physical_right_rectified.jpg"), right)
        self.cv2.imwrite(str(self.debug_dir / "latest_disparity.jpg"), colour)
        roi = disparity[max(0, y1):min(disparity.shape[0], y2), max(0, x1):min(disparity.shape[1], x2)]
        roi_valid = roi[self.np.isfinite(roi) & (roi > 0.5)]
        diagnostic = {
            "roi_bbox_xyxy_px": bbox,
            "roi_positive_disparity_count": int(roi_valid.size),
            "roi_disparity_median_px": None if not roi_valid.size else float(self.np.median(roi_valid)),
            "roi_raw_depth_median_m": None if not roi_valid.size else float(
                self.np.median(abs(float(self.p2[0, 3])) / roi_valid / 1000.0)
            ),
        }
        (self.debug_dir / "latest_stereo_debug.json").write_text(
            json.dumps(diagnostic, indent=2) + chr(10), encoding="utf-8"
        )

    def _publish_observation(self, observation: dict[str, Any]) -> None:
        message = String()
        message.data = encode(observation)
        self.observation_publisher.publish(message)

    def _base_measurement(
        self, cycle_id: str, timestamp: datetime, status: str, reason: str, skew_ms: float,
        observation: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        return {
            "schema_version": "sie.perception.measurement.v1",
            "measurement_id": f"measurement:{cycle_id}",
            "timestamp": timestamp.isoformat(),
            "status": status,
            "reason": reason,
            "reference_frame": "ov9281_physical_left_optical_frame",
            "units": "m",
            "confidence": 0.0,
            "calibration": {
                "calibration_id": "ar0234_ov9281_physical_left_extrinsic_v1",
                "sha256": self.extrinsic_sha256,
            },
            "provenance": {
                "activation_profile": str(self.activation_path),
                "activation_profile_sha256": sha256_file(self.activation_path),
                "ar0234_intrinsic_sha256": sha256_file(self.ar_intrinsic_path),
                "ov9281_stereo_sha256": sha256_file(self.stereo_path),
                "capture_pair_skew_ms": skew_ms,
                "stop_and_measure_only": True,
                "actuator_access": False,
            },
            "source_observation_id": None if observation is None else observation["observation_id"],
            "source_evidence_id": None if observation is None else observation["evidence_id"],
        }

    def _publish_refusal(
        self, cycle_id: str, timestamp: datetime, status: str, reason: str, skew_ms: float,
        observation: dict[str, Any] | None = None,
    ) -> None:
        result = self._base_measurement(
            cycle_id, timestamp, status, reason, skew_ms, observation
        )
        message = String()
        message.data = encode(result)
        self.measurement_publisher.publish(message)

    def _publish_success(
        self, cycle_id: str, timestamp: datetime, skew_ms: float,
        observation: dict[str, Any], left_evidence: dict[str, Any],
        association: dict[str, Any],
    ) -> None:
        result = self._base_measurement(
            cycle_id, timestamp, "SUCCESS", "UNIQUE_STATIC_3D_ASSOCIATION", skew_ms, observation
        )
        result.update(association)
        result["stereo_person_evidence_id"] = left_evidence["evidence_id"]
        message = String()
        message.data = encode(result)
        self.measurement_publisher.publish(message)

    def destroy_node(self) -> bool:
        for name in ("ar_capture", "stereo_capture"):
            capture = getattr(self, name, None)
            if capture is not None:
                capture.release()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Ar0234Ov9281MetricMeasurementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
