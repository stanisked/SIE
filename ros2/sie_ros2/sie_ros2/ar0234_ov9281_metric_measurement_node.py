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
import secrets
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
DEFAULT_STEREO_PERSON_MODEL = (
    "/home/elwis/dev_ws/model_artifacts/opencv_mp_persondet_2023mar/"
    "person_detection_mediapipe_2023mar.onnx"
)
DEFAULT_STEREO_PERSON_REFERENCE = (
    "/home/elwis/dev_ws/model_artifacts/opencv_mp_persondet_2023mar/"
    "mp_persondet.reference.py"
)
EXPECTED_STEREO_PERSON_REFERENCE_SHA256 = (
    "e530a8ebc3c218376d5dd1c13aa8ed39a850fac4dbcd6928144746b33477b9c7"
)


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


def _bbox_xyxy(value: object) -> tuple[float, float, float, float] | None:
    if type(value) is not list or len(value) != 4:
        return None
    try:
        x1, y1, x2, y2 = (float(item) for item in value)
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(item) for item in (x1, y1, x2, y2)) or x2 <= x1 or y2 <= y1:
        return None
    return x1, y1, x2, y2


def _bbox_iou(
    first: tuple[float, float, float, float],
    second: tuple[float, float, float, float],
) -> float:
    left = max(first[0], second[0])
    top = max(first[1], second[1])
    right = min(first[2], second[2])
    bottom = min(first[3], second[3])
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    first_area = (first[2] - first[0]) * (first[3] - first[1])
    second_area = (second[2] - second[0]) * (second[3] - second[1])
    union = first_area + second_area - intersection
    return 0.0 if union <= 0 else intersection / union


def _point_inside_bbox(
    point: tuple[float, float], bbox: tuple[float, float, float, float]
) -> bool:
    return bbox[0] <= point[0] <= bbox[2] and bbox[1] <= point[1] <= bbox[3]



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
        self.declare_parameter("confidence_threshold", 0.70)
        self.declare_parameter("stereo_person_model", DEFAULT_STEREO_PERSON_MODEL)
        self.declare_parameter("stereo_person_reference", DEFAULT_STEREO_PERSON_REFERENCE)
        self.declare_parameter("stereo_person_threshold", 0.40)
        self.declare_parameter("min_depth_samples", 100)
        self.declare_parameter("max_depth_mad_m", 0.05)
        self.declare_parameter("static_confirmation_count", 2)
        self.declare_parameter("static_max_center_delta_px", 12.0)
        self.declare_parameter("static_max_area_relative_change", 0.10)
        self.declare_parameter("static_candidate_max_age_s", 5.0)
        # Resolve an ambiguous AR frame only from a recent, confirmed static
        # 3D association.  It is never an acquisition mechanism.
        self.declare_parameter("target_hold_max_age_s", 2.5)
        self.declare_parameter("target_hold_max_center_delta_px", 80.0)
        self.declare_parameter("target_hold_min_iou", 0.20)
        self.declare_parameter("auto_exposure", 3)
        self.declare_parameter(
            "debug_dir", "~/.local/state/sie/debug/ar0234_ov9281_live"
        )

        self.project_root = absolute_parameter(self, "project_root")
        self.model_path = absolute_parameter(self, "model_path")
        self.stereo_person_model_path = absolute_parameter(self, "stereo_person_model")
        self.stereo_person_reference_path = absolute_parameter(
            self, "stereo_person_reference"
        )
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
        self.stereo_person_threshold = float(
            self.get_parameter("stereo_person_threshold").value
        )
        self.static_confirmation_count = int(
            self.get_parameter("static_confirmation_count").value
        )
        self.static_max_center_delta_px = float(
            self.get_parameter("static_max_center_delta_px").value
        )
        self.static_max_area_relative_change = float(
            self.get_parameter("static_max_area_relative_change").value
        )
        self.static_candidate_max_age_s = float(
            self.get_parameter("static_candidate_max_age_s").value
        )
        self._static_candidate: dict[str, float] | None = None
        self._static_confirmation_observations = 0
        self._confirmed_target: dict[str, Any] | None = None
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
            or not 0 < self.stereo_person_threshold <= 1
            or self.static_confirmation_count < 2
            or self.static_max_center_delta_px <= 0
            or not 0 < self.static_max_area_relative_change < 1
            or self.static_candidate_max_age_s <= 0
            or self.target_hold_max_age_s <= 0
            or self.target_hold_max_center_delta_px <= 0
            or not 0 < self.target_hold_min_iou <= 1
        ):
            raise ValueError("invalid live measurement parameters")

        if str(self.project_root) not in sys.path:
            sys.path.insert(0, str(self.project_root))
        import cv2
        import numpy as np
        from vision_core.person_localization.mp_persondet import MPPersonDetOpenCV
        from vision_core.person_localization.yolo11_person_upper_body_runtime import (
            OnnxRuntimeYolo11PersonUpperBodyObserver,
        )

        self.cv2 = cv2
        self.np = np
        self._load_profile()
        self.observer = OnnxRuntimeYolo11PersonUpperBodyObserver(
            self.model_path, confidence_threshold=self.confidence_threshold
        )
        if not self.stereo_person_model_path.is_file():
            raise FileNotFoundError("stereo person detector model is missing")
        if not self.stereo_person_reference_path.is_file():
            raise FileNotFoundError("stereo person detector reference is missing")
        if (
            sha256_file(self.stereo_person_reference_path)
            != EXPECTED_STEREO_PERSON_REFERENCE_SHA256
        ):
            raise RuntimeError("stereo person detector reference SHA-256 mismatch")
        self.stereo_person_model_sha256 = sha256_file(
            self.stereo_person_model_path
        )
        self.stereo_person_reference_sha256 = sha256_file(
            self.stereo_person_reference_path
        )
        self.stereo_person_detector = MPPersonDetOpenCV(
            self.stereo_person_model_path,
            self.stereo_person_reference_path,
            score_threshold=self.stereo_person_threshold,
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
        self.run_id = secrets.token_hex(8)
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

        # The strict complete-person policy is the default.  A separately
        # activated profile may permit only bottom truncation for a static
        # upper-body target; it never changes execution authorization.
        geometry_policy = activation.get("target_geometry_policy")
        self.allow_static_upper_body_bottom_truncation = False
        if geometry_policy is not None:
            if (
                type(geometry_policy) is not dict
                or geometry_policy.get("schema_version")
                != "sie.ar0234.upper_body_static_gate.v1"
                or geometry_policy.get("allow_bottom_truncation") is not True
                or geometry_policy.get("require_untruncated_edges")
                != ["top", "left", "right"]
                or geometry_policy.get("require_single_target") is not True
                or geometry_policy.get("static_scene_only") is not True
                or geometry_policy.get("dynamic_fusion_permitted") is not False
            ):
                raise RuntimeError("invalid upper-body static geometry policy")
            self.allow_static_upper_body_bottom_truncation = True

        temporal_gate = activation.get("temporal_static_gate")
        if temporal_gate is not None:
            if (
                type(temporal_gate) is not dict
                or temporal_gate.get("schema_version")
                != "sie.temporal.static_gate.v1"
                or temporal_gate.get("apply_before_stereo_fusion") is not True
                or int(temporal_gate.get("minimum_consecutive_observations", 0)) < 2
            ):
                raise RuntimeError("invalid temporal static gate")
            self.static_confirmation_count = int(
                temporal_gate["minimum_consecutive_observations"]
            )
            self.static_max_center_delta_px = float(
                temporal_gate["max_center_delta_px"]
            )
            self.static_max_area_relative_change = float(
                temporal_gate["max_area_relative_change"]
            )
            self.static_candidate_max_age_s = float(
                temporal_gate["candidate_max_age_s"]
            )
            if (
                self.static_max_center_delta_px <= 0
                or not 0 < self.static_max_area_relative_change < 1
                or self.static_candidate_max_age_s <= 0
            ):
                raise RuntimeError("invalid temporal static gate limits")

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
        cycle_id = (
            f"ar0234-ov9281-live-{self.run_id}-{self.sequence:08d}"
        )
        if ar_frame is None or combined is None:
            self._reset_static_gate()
            self._publish_refusal(
                cycle_id, timestamp, "DEPTH_UNAVAILABLE", "CAMERA_FRAME_UNAVAILABLE", skew_ms
            )
            return
        if ar_frame.shape[:2] != (1200, 1920) or combined.shape[:2] != (
            self.stereo_size[1],
            self.stereo_size[0] * 2,
        ):
            self._reset_static_gate()
            self._publish_refusal(
                cycle_id, timestamp, "CALIBRATION_INVALID", "UNEXPECTED_FRAME_SIZE", skew_ms
            )
            return
        if skew_ms > self.max_pair_skew_ms:
            self._reset_static_gate()
            self._publish_refusal(
                cycle_id, timestamp, "DEPTH_UNAVAILABLE", "PAIR_SKEW_EXCEEDED", skew_ms
            )
            return

        try:
            observation = self.observer.observe(
                ar_frame, captured_at_utc=timestamp, cycle_id=cycle_id
            )
            observation = self._resolve_ambiguous_ar_target(observation)
            observation = validate_ar0234_observation(observation)
            self._publish_observation(observation)
            suitability = ar0234_target_suitability(observation)
            geometry_reason = str(suitability["reason"])
            if observation["target_status"] == "NO_TARGET":
                self._reset_target_hold()
            if not suitability["geometry_eligible"]:
                if self._bottom_truncation_allowed(observation):
                    geometry_reason = "UPPER_BODY_BOTTOM_TRUNCATION_STATIC_ALLOWED"
                else:
                    status = (
                        "MULTIPLE_TARGETS"
                        if observation["target_status"] == "MULTIPLE_TARGETS"
                        else "NO_TARGET"
                    )
                    self._reset_static_gate()
                    self._publish_refusal(
                        cycle_id, timestamp, status, geometry_reason, skew_ms, observation
                    )
                    return
            temporal_reason = self._temporal_static_reason(observation)
            if temporal_reason is not None:
                if temporal_reason.startswith("MOTION_DETECTED"):
                    self._reset_target_hold()
                self._publish_refusal(
                    cycle_id, timestamp, "DEPTH_UNAVAILABLE", temporal_reason, skew_ms, observation
                )
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
            left_evidence = self._observe_stereo_person(
                left_rectified, cycle_id=f"{cycle_id}:physical_left"
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
                cycle_id, timestamp, skew_ms, observation, left_evidence, association,
                geometry_reason,
            )
        except (ContractError, OSError, RuntimeError, ValueError, KeyError) as error:
            self._reset_static_gate()
            self.get_logger().warning(f"live metric measurement refused: {error}")
            self._publish_refusal(
                cycle_id, timestamp, "DEPTH_UNAVAILABLE", "PROCESSING_REFUSED", skew_ms
            )

    def _reset_target_hold(self) -> None:
        self._confirmed_target = None

    def _resolve_ambiguous_ar_target(
        self, observation: dict[str, Any]
    ) -> dict[str, Any]:
        """Select one candidate only by continuity with confirmed static 3D evidence."""
        if observation.get("target_status") != "MULTIPLE_TARGETS":
            return observation
        held = self._confirmed_target
        if held is None:
            observation["target_hold"] = {
                "mode": "UNRESOLVED_NO_CONFIRMED_3D_TARGET",
                "original_eligible_detection_count": observation.get(
                    "eligible_detection_count"
                ),
            }
            return observation
        age_s = time.monotonic() - float(held["monotonic_s"])
        if age_s > self.target_hold_max_age_s:
            self._reset_target_hold()
            observation["target_hold"] = {
                "mode": "UNRESOLVED_CONFIRMED_TARGET_STALE",
                "hold_age_s": age_s,
                "original_eligible_detection_count": observation.get(
                    "eligible_detection_count"
                ),
            }
            return observation
        previous_bbox = _bbox_xyxy(held["ar_bbox_xyxy_px"])
        projected = held["projected_ar0234_point_px"]
        if previous_bbox is None or type(projected) is not list or len(projected) != 2:
            self._reset_target_hold()
            observation["target_hold"] = {
                "mode": "UNRESOLVED_CONFIRMED_TARGET_INVALID",
                "original_eligible_detection_count": observation.get(
                    "eligible_detection_count"
                ),
            }
            return observation
        projected_point = float(projected[0]), float(projected[1])
        previous_center = (
            (previous_bbox[0] + previous_bbox[2]) / 2.0,
            (previous_bbox[1] + previous_bbox[3]) / 2.0,
        )
        matches: list[tuple[int, dict[str, Any], float, float]] = []
        for index, candidate in enumerate(observation.get("detections", [])):
            if type(candidate) is not dict:
                continue
            bbox = _bbox_xyxy(candidate.get("bbox_xyxy_px"))
            if bbox is None or not _point_inside_bbox(projected_point, bbox):
                continue
            candidate_center = (bbox[0] + bbox[2]) / 2.0, (bbox[1] + bbox[3]) / 2.0
            center_delta = math.hypot(
                candidate_center[0] - previous_center[0],
                candidate_center[1] - previous_center[1],
            )
            iou = _bbox_iou(previous_bbox, bbox)
            if (
                center_delta <= self.target_hold_max_center_delta_px
                and iou >= self.target_hold_min_iou
            ):
                matches.append((index, candidate, center_delta, iou))
        if len(matches) != 1:
            observation["target_hold"] = {
                "mode": "UNRESOLVED_AMBIGUOUS_CANDIDATES",
                "hold_age_s": age_s,
                "original_eligible_detection_count": observation.get(
                    "eligible_detection_count"
                ),
                "matching_candidate_count": len(matches),
            }
            return observation
        index, selected, center_delta, iou = matches[0]
        selected_bbox = _bbox_xyxy(selected.get("bbox_xyxy_px"))
        if selected_bbox is None:
            raise RuntimeError("selected target-hold candidate has invalid bbox")
        observation = dict(observation)
        observation.update(
            {
                "target_status": "SINGLE_TARGET",
                "eligible_detection_count": 1,
                "detection_count": 1,
                "detections": [selected],
                "bbox_xyxy_px": [float(item) for item in selected_bbox],
                "center_x_px": float(selected["center_x_px"]),
                "confidence": float(selected["confidence"]),
                "truncated_left": bool(selected["truncated_left"]),
                "truncated_right": bool(selected["truncated_right"]),
                "truncated_top": bool(selected["truncated_top"]),
                "truncated_bottom": bool(selected["truncated_bottom"]),
                "target_hold": {
                    "mode": "UNIQUE_PREVIOUS_3D_PROJECTION",
                    "previous_measurement_id": held["measurement_id"],
                    "hold_age_s": age_s,
                    "previous_projected_ar0234_point_px": [
                        float(item) for item in projected_point
                    ],
                    "previous_point_physical_left_m": [
                        float(item) for item in held["point_physical_left_m"]
                    ],
                    "selected_candidate_index": index,
                    "original_eligible_detection_count": held[
                        "original_eligible_detection_count"
                    ],
                    "center_delta_px": center_delta,
                    "iou_with_confirmed_target": iou,
                },
            }
        )
        return observation

    def _remember_confirmed_target(
        self, observation: dict[str, Any], association: dict[str, Any], measurement_id: str
    ) -> None:
        bbox = _bbox_xyxy(observation.get("bbox_xyxy_px"))
        projected = association.get("projected_ar0234_point_px")
        if (
            bbox is None
            or type(projected) is not list
            or len(projected) != 2
            or not all(isinstance(item, (int, float)) and math.isfinite(float(item)) for item in projected)
        ):
            self._reset_target_hold()
            return
        self._confirmed_target = {
            "measurement_id": measurement_id,
            "monotonic_s": time.monotonic(),
            "ar_bbox_xyxy_px": [float(item) for item in bbox],
            "projected_ar0234_point_px": [float(item) for item in projected],
            "point_physical_left_m": [
                float(association["x_m"]),
                float(association["y_m"]),
                float(association["z_m"]),
            ],
            "original_eligible_detection_count": int(
                observation.get("eligible_detection_count", 1)
            ),
        }

    def _reset_static_gate(self) -> None:
        self._static_candidate = None
        self._static_confirmation_observations = 0

    def _temporal_static_reason(self, observation: dict[str, Any]) -> str | None:
        """Require consecutive, fresh, visually stable AR observations before fusion."""
        bbox = observation.get("bbox_xyxy_px")
        if type(bbox) is not list or len(bbox) != 4:
            self._reset_static_gate()
            return "TEMPORAL_STABILITY_PENDING"
        x1, y1, x2, y2 = (float(value) for value in bbox)
        area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        if area <= 0:
            self._reset_static_gate()
            return "TEMPORAL_STABILITY_PENDING"
        now = time.monotonic()
        current = {
            "center_x": (x1 + x2) / 2.0,
            "center_y": (y1 + y2) / 2.0,
            "area": area,
            "monotonic_s": now,
        }
        previous = self._static_candidate
        if (
            previous is None
            or now - previous["monotonic_s"] > self.static_candidate_max_age_s
        ):
            self._static_candidate = current
            self._static_confirmation_observations = 1
            return "TEMPORAL_STABILITY_PENDING"
        center_delta = math.hypot(
            current["center_x"] - previous["center_x"],
            current["center_y"] - previous["center_y"],
        )
        area_relative_change = abs(current["area"] - previous["area"]) / previous["area"]
        self._static_candidate = current
        if (
            center_delta > self.static_max_center_delta_px
            or area_relative_change > self.static_max_area_relative_change
        ):
            self._static_confirmation_observations = 1
            return (
                "MOTION_DETECTED:"
                f"center_delta_px={center_delta:.1f};"
                f"area_relative_change={area_relative_change:.3f}"
            )
        self._static_confirmation_observations += 1
        if self._static_confirmation_observations < self.static_confirmation_count:
            return "TEMPORAL_STABILITY_PENDING"
        return None

    def _bottom_truncation_allowed(self, observation: dict[str, Any]) -> bool:
        """Narrow opt-in exception; strict complete-person gate remains default."""
        return (
            self.allow_static_upper_body_bottom_truncation
            and observation.get("target_status") == "SINGLE_TARGET"
            and observation.get("truncated_bottom") is True
            and observation.get("truncated_top") is False
            and observation.get("truncated_left") is False
            and observation.get("truncated_right") is False
        )

    def _observe_stereo_person(
        self, left_rectified: Any, *, cycle_id: str
    ) -> dict[str, Any]:
        """Create the internal OV9281 evidence used only for 3D association."""
        evidence_id = f"ov9281-mp-persondet:{cycle_id}"
        detections = self.stereo_person_detector.detect(left_rectified)
        if not detections:
            return {
                "target_status": "NO_TARGET",
                "evidence_id": evidence_id,
            }
        if len(detections) != 1:
            return {
                "target_status": "MULTIPLE_TARGETS",
                "evidence_id": evidence_id,
            }
        detection = detections[0]
        return {
            "target_status": "SINGLE_TARGET",
            "bbox_xyxy_px": [
                float(value) for value in detection.bounding_box.to_xyxy()
            ],
            "confidence": float(detection.confidence),
            "evidence_id": evidence_id,
        }

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
                "ar0234_person_detector": "ar0234_person_upper_body_yolo11n_v1",
                "ar0234_confidence_threshold": self.confidence_threshold,
                "ar0234_target_hold": (
                    {"mode": "DIRECT_SINGLE_TARGET"}
                    if observation is None
                    else observation.get(
                        "target_hold", {"mode": "DIRECT_SINGLE_TARGET"}
                    )
                ),
                "stereo_person_detector": "opencv_mp_persondet_2023mar",
                "stereo_person_model_sha256": self.stereo_person_model_sha256,
                "stereo_person_reference_sha256": (
                    self.stereo_person_reference_sha256
                ),
                "stereo_person_threshold": self.stereo_person_threshold,
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
        association: dict[str, Any], geometry_reason: str,
    ) -> None:
        result = self._base_measurement(
            cycle_id, timestamp, "SUCCESS", "UNIQUE_STATIC_3D_ASSOCIATION", skew_ms, observation
        )
        result.update(association)
        result["target_geometry_reason"] = geometry_reason
        result["stereo_person_evidence_id"] = left_evidence["evidence_id"]
        self._remember_confirmed_target(observation, association, result["measurement_id"])
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
