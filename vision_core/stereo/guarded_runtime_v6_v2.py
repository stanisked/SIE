#!/usr/bin/env python3

"""Guarded V6 live stereo-depth processing for SIE Vision Core."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone

import cv2
import numpy as np

from vision_core.stereo.matcher import create_sgbm_matcher
from vision_core.stereo.stereo_calibration_guard_v6 import (
    GuardedCalibration,
    StereoCalibrationGuard,
    TemperatureGateResult,
)


REFERENCE_FRAME = "rectified_left_optical_frame"
REQUIRED_NUM_DISPARITIES = 192
VALIDATED_RUNTIME_PROFILE_ID = "stereo_runtime_v2_sgbm192_roi100"


@dataclass(frozen=True)
class StereoDepthFrame:
    """Internal perception result. This is not an SIE Measurement."""

    sequence: int
    processed_at_utc: str
    reference_frame: str
    calibration_id: str
    calibration_sha256: str
    activation_record_sha256: str
    runtime_profile_id: str
    temperature_gate: TemperatureGateResult
    left_rectified_bgr: np.ndarray
    right_rectified_bgr: np.ndarray
    disparity_q4: np.ndarray
    disparity_px: np.ndarray
    valid_mask: np.ndarray
    depth_m: np.ndarray


class GuardedStereoDepthProcessor:
    """Compute depth only after the conditional calibration gate passes."""

    def __init__(
        self,
        *,
        guard: StereoCalibrationGuard,
        calibration: GuardedCalibration,
        num_disparities: int,
    ) -> None:
        if num_disparities != REQUIRED_NUM_DISPARITIES:
            raise ValueError(
                "unsupported num_disparities; validated V6 runtime requires "
                f"{REQUIRED_NUM_DISPARITIES}, got {num_disparities}"
            )
        self.guard = guard
        self.calibration = calibration
        self.parameters = calibration.parameters
        self.num_disparities = num_disparities
        self.runtime_profile_id = VALIDATED_RUNTIME_PROFILE_ID
        self.sequence = 0

        size = np.asarray(self.parameters["size"], dtype=np.int64).reshape(-1)
        if size.shape != (2,):
            raise ValueError(f"invalid calibration size: {size}")
        self.eye_width = int(size[0])
        self.eye_height = int(size[1])
        if (self.eye_width, self.eye_height) != (1280, 800):
            raise ValueError(
                "validated stereo runtimes require calibration size 1280x800, "
                f"got {self.eye_width}x{self.eye_height}"
            )

        self.fx_rectified_px = float(self.parameters["P1"][0, 0])
        baseline_mm = abs(float(np.asarray(self.parameters["baseline_mm"]).reshape(-1)[0]))
        self.baseline_m = baseline_mm / 1000.0
        if not np.isfinite(self.fx_rectified_px) or self.fx_rectified_px <= 0:
            raise ValueError("invalid rectified focal length")
        if not np.isfinite(self.baseline_m) or self.baseline_m <= 0:
            raise ValueError("invalid stereo baseline")

        map_size = (self.eye_width, self.eye_height)
        self.left_map_x, self.left_map_y = cv2.initUndistortRectifyMap(
            self.parameters["K1"],
            self.parameters["D1"],
            self.parameters["R1"],
            self.parameters["P1"],
            map_size,
            cv2.CV_32FC1,
        )
        self.right_map_x, self.right_map_y = cv2.initUndistortRectifyMap(
            self.parameters["K2"],
            self.parameters["D2"],
            self.parameters["R2"],
            self.parameters["P2"],
            map_size,
            cv2.CV_32FC1,
        )
        self.matcher = create_sgbm_matcher(
            block_size=7,
            uniqueness_ratio=6,
            min_disparity=0,
            num_disparities=num_disparities,
            disp12_max_diff=1,
            speckle_window_size=80,
            speckle_range=4,
            pre_filter_cap=31,
        )

    def process(self, combined_bgr: np.ndarray) -> StereoDepthFrame:
        """Process one combined UVC frame or raise before matcher.compute()."""

        temperature_gate = self.guard.check_before_measurement()
        expected_shape = (self.eye_height, self.eye_width * 2, 3)
        if combined_bgr.shape != expected_shape:
            raise ValueError(
                f"combined frame must have shape {expected_shape}, "
                f"got {combined_bgr.shape}"
            )
        if combined_bgr.dtype != np.uint8:
            raise ValueError(
                f"combined frame must use uint8 pixels, got {combined_bgr.dtype}"
            )

        # Frozen DECXIN UVC semantics used by V5 capture and V6 dataset:
        # combined left half  = physical RIGHT
        # combined right half = physical LEFT
        right_raw = combined_bgr[:, : self.eye_width]
        left_raw = combined_bgr[:, self.eye_width :]

        left_rectified = cv2.remap(
            left_raw,
            self.left_map_x,
            self.left_map_y,
            cv2.INTER_LINEAR,
        )
        right_rectified = cv2.remap(
            right_raw,
            self.right_map_x,
            self.right_map_y,
            cv2.INTER_LINEAR,
        )
        left_gray = cv2.cvtColor(left_rectified, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_rectified, cv2.COLOR_BGR2GRAY)

        disparity_q4 = self.matcher.compute(left_gray, right_gray)
        disparity_px = disparity_q4.astype(np.float32) / 16.0
        valid_mask = np.isfinite(disparity_px) & (disparity_px > 0.0)
        depth_m = np.full(disparity_px.shape, np.nan, dtype=np.float32)
        depth_m[valid_mask] = (
            self.fx_rectified_px * self.baseline_m / disparity_px[valid_mask]
        )

        self.sequence += 1
        return StereoDepthFrame(
            sequence=self.sequence,
            processed_at_utc=datetime.now(timezone.utc).isoformat(),
            reference_frame=REFERENCE_FRAME,
            calibration_id=self.calibration.calibration_id,
            calibration_sha256=self.calibration.calibration_sha256,
            activation_record_sha256=self.calibration.activation_record_sha256,
            runtime_profile_id=self.runtime_profile_id,
            temperature_gate=temperature_gate,
            left_rectified_bgr=left_rectified,
            right_rectified_bgr=right_rectified,
            disparity_q4=disparity_q4,
            disparity_px=disparity_px,
            valid_mask=valid_mask,
            depth_m=depth_m,
        )
