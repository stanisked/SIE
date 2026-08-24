from __future__ import annotations

import inspect
import json
from unittest.mock import Mock, patch

import numpy as np
import pytest

import vision_core.stereo.guarded_runtime_v6 as runtime_v1
import vision_core.stereo.guarded_runtime_v6_v2 as runtime_v2
from vision_core.stereo.stereo_calibration_guard_v6 import (
    GuardedCalibration,
    StereoCalibrationGuard,
    TemperatureGateResult,
)


SGBM192_RUNTIME_PROFILE_ID = "stereo_runtime_v2_sgbm192_roi100"
ROM_MAPPING = {
    "ambient": "289452A555A32FDA",
    "camera_left": "28F952990F510A0C",
    "camera_right": "28FE6BA3299C9AC4",
}


class RecordingMatcher:
    def __init__(self, disparity_q4: int = 32) -> None:
        self.disparity_q4 = disparity_q4
        self.calls: list[tuple[np.ndarray, np.ndarray]] = []

    def compute(self, left_gray: np.ndarray, right_gray: np.ndarray) -> np.ndarray:
        self.calls.append((left_gray.copy(), right_gray.copy()))
        return np.full(left_gray.shape, self.disparity_q4, dtype=np.int16)


@pytest.fixture
def calibration() -> GuardedCalibration:
    intrinsic = np.array(
        [[100.0, 0.0, 640.0], [0.0, 100.0, 400.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    projection = np.array(
        [[100.0, 0.0, 640.0, 0.0], [0.0, 100.0, 400.0, 0.0], [0.0, 0.0, 1.0, 0.0]],
        dtype=np.float64,
    )
    gate = TemperatureGateResult(
        checked_at_utc="2026-08-23T00:00:00+00:00",
        state_age_s=0.0,
        temperatures_c={
            "ambient": 25.0,
            "camera_left": 31.0,
            "camera_right": 32.0,
        },
    )
    return GuardedCalibration(
        calibration_id="synthetic_stereo_calibration_v6",
        calibration_sha256="a" * 64,
        activation_record_sha256="b" * 64,
        parameters={
            "K1": intrinsic.copy(),
            "D1": np.zeros(5, dtype=np.float64),
            "K2": intrinsic.copy(),
            "D2": np.zeros(5, dtype=np.float64),
            "R1": np.eye(3, dtype=np.float64),
            "R2": np.eye(3, dtype=np.float64),
            "P1": projection.copy(),
            "P2": projection.copy(),
            "size": np.array([1280, 800], dtype=np.int64),
            "baseline_mm": np.array([100.0], dtype=np.float64),
        },
        temperature_gate=gate,
    )


@pytest.fixture
def guard(tmp_path) -> StereoCalibrationGuard:
    state_path = tmp_path / "temperature_state.json"
    state_path.write_text(
        json.dumps(
            {
                "status": "OK",
                "reset_detected": False,
                "updated_at_unix_s": 1000.0,
                "rom_mapping": ROM_MAPPING,
                "temperatures_c": {
                    "ambient": 25.0,
                    "camera_left": 31.0,
                    "camera_right": 32.0,
                },
            }
        ),
        encoding="utf-8",
    )
    policy_path = tmp_path / "policy.json"
    policy_path.write_text(
        json.dumps(
            {
                "schema_version": "sie_stereo_runtime_policy_v1",
                "status": "ENABLED",
                "required_activation_status": "ACTIVE_CONDITIONAL",
                "maximum_temperature_state_age_s": 5.0,
                "expected_rom_mapping": ROM_MAPPING,
                "temperature_gating_channels": ["camera_left", "camera_right"],
                "temperature_observational_channels": ["ambient"],
                "activation_record_path": "unused_activation.json",
                "calibration_path": "unused_calibration.npz",
                "temperature_state_file": str(state_path),
            }
        ),
        encoding="utf-8",
    )
    result = StereoCalibrationGuard(
        policy_path,
        project_root=tmp_path,
        now_unix_s=lambda: 1000.0,
    )
    result.activation_record = {}
    result.envelope = {
        "camera_left": {"minimum_c": 30.0, "maximum_c": 32.375},
        "camera_right": {"minimum_c": 31.125, "maximum_c": 33.9375},
    }
    return result


def test_num_disparities_is_required_keyword_only() -> None:
    parameter = inspect.signature(
        runtime_v2.GuardedStereoDepthProcessor.__init__
    ).parameters["num_disparities"]

    assert parameter.kind is inspect.Parameter.KEYWORD_ONLY
    assert parameter.default is inspect.Parameter.empty


def test_missing_num_disparities_is_rejected() -> None:
    with pytest.raises(TypeError, match="num_disparities"):
        runtime_v2.GuardedStereoDepthProcessor(
            guard=object(),
            calibration=object(),
        )


def test_matcher_receives_exactly_192(
    guard: StereoCalibrationGuard,
    calibration: GuardedCalibration,
) -> None:
    matcher_factory = Mock(return_value=RecordingMatcher())

    with patch.object(runtime_v2, "create_sgbm_matcher", matcher_factory):
        processor = runtime_v2.GuardedStereoDepthProcessor(
            guard=guard,
            calibration=calibration,
            num_disparities=192,
        )

    assert matcher_factory.call_count == 1
    assert matcher_factory.call_args.kwargs["num_disparities"] == 192
    assert processor.num_disparities == 192
    assert processor.runtime_profile_id == SGBM192_RUNTIME_PROFILE_ID


@pytest.mark.parametrize("num_disparities", [160, 176, 208, 256])
def test_non_192_values_are_blocked_before_matcher_creation(
    num_disparities: int,
) -> None:
    matcher_factory = Mock()

    with patch.object(runtime_v2, "create_sgbm_matcher", matcher_factory):
        with pytest.raises(ValueError, match="requires 192"):
            runtime_v2.GuardedStereoDepthProcessor(
                guard=object(),
                calibration=object(),
                num_disparities=num_disparities,
            )

    matcher_factory.assert_not_called()


def test_sgbm192_mapping_rectification_and_depth_match_v1(
    guard: StereoCalibrationGuard,
    calibration: GuardedCalibration,
) -> None:
    combined = np.empty((800, 2560, 3), dtype=np.uint8)
    combined[:, :1280] = 11
    combined[:, 1280:] = 22
    matcher_v1 = RecordingMatcher()
    matcher_v2 = RecordingMatcher()

    def identity_remap(image, *_args, **_kwargs):
        return image.copy()

    with (
        patch.object(runtime_v1, "create_sgbm_matcher", return_value=matcher_v1),
        patch.object(runtime_v2, "create_sgbm_matcher", return_value=matcher_v2),
        patch.object(runtime_v1.cv2, "remap", side_effect=identity_remap),
    ):
        processor_v1 = runtime_v1.GuardedStereoDepthProcessor(
            guard=guard,
            calibration=calibration,
            num_disparities=192,
        )
        processor_v2 = runtime_v2.GuardedStereoDepthProcessor(
            guard=guard,
            calibration=calibration,
            num_disparities=192,
        )
        frame_v1 = processor_v1.process(combined)
        frame_v2 = processor_v2.process(combined)

    assert len(matcher_v1.calls) == 1
    assert len(matcher_v2.calls) == 1
    left_gray_v1, right_gray_v1 = matcher_v1.calls[0]
    left_gray_v2, right_gray_v2 = matcher_v2.calls[0]
    assert np.all(left_gray_v1 == 22)
    assert np.all(right_gray_v1 == 11)
    np.testing.assert_array_equal(left_gray_v2, left_gray_v1)
    np.testing.assert_array_equal(right_gray_v2, right_gray_v1)

    assert processor_v2.runtime_profile_id == SGBM192_RUNTIME_PROFILE_ID
    assert processor_v2.fx_rectified_px == processor_v1.fx_rectified_px
    assert processor_v2.baseline_m == processor_v1.baseline_m
    for attribute in (
        "left_rectified_bgr",
        "right_rectified_bgr",
        "disparity_q4",
        "disparity_px",
        "valid_mask",
        "depth_m",
    ):
        np.testing.assert_array_equal(
            getattr(frame_v2, attribute),
            getattr(frame_v1, attribute),
        )
    assert frame_v2.reference_frame == frame_v1.reference_frame
    assert frame_v2.calibration_id == frame_v1.calibration_id
    assert frame_v2.calibration_sha256 == frame_v1.calibration_sha256
    assert (
        frame_v2.activation_record_sha256
        == frame_v1.activation_record_sha256
    )
    assert frame_v2.runtime_profile_id == frame_v1.runtime_profile_id
    assert frame_v2.temperature_gate.temperatures_c == (
        frame_v1.temperature_gate.temperatures_c
    )
