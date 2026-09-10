from __future__ import annotations

import cv2
import numpy as np
import pytest

from vision_core.ar0234_ov9281_extrinsic_forensics import (
    AR_FRAME,
    OV_RAW_LEFT_FRAME,
    OV_RECTIFIED_LEFT_FRAME,
    RigidTransform,
    compose_ov_from_ar,
    cross_reprojection_errors_as_required,
    invert_transform,
    require_raw_frame_contract,
)
from vision_core.ar0234_ov9281_static_extrinsic import (
    CameraIntrinsics,
    StaticExtrinsicError,
    _project_board,
)


def _intrinsics() -> CameraIntrinsics:
    return CameraIntrinsics(
        matrix=np.array([[900.0, 0.0, 640.0], [0.0, 900.0, 400.0], [0.0, 0.0, 1.0]]),
        distortion=np.zeros((1, 5)),
        frame=OV_RAW_LEFT_FRAME,
    )


def _rotation(vector: tuple[float, float, float]) -> np.ndarray:
    return cv2.Rodrigues(np.asarray(vector, dtype=np.float64))[0]


def test_known_camera_board_poses_compose_expected_ov_from_ar():
    ar_rotation = _rotation((0.04, -0.02, 0.01))
    ar_translation = np.array([20.0, -10.0, 900.0])
    expected = RigidTransform(
        "expected", AR_FRAME, OV_RAW_LEFT_FRAME, _rotation((0.01, 0.03, -0.02)), np.array([65.0, -4.0, 3.0])
    )
    ov_rotation = expected.rotation @ ar_rotation
    ov_translation = expected.rotation @ ar_translation + expected.translation_mm

    actual = compose_ov_from_ar(ar_rotation, ar_translation, ov_rotation, ov_translation)

    np.testing.assert_allclose(actual.rotation, expected.rotation, atol=1e-12)
    np.testing.assert_allclose(actual.translation_mm, expected.translation_mm, atol=1e-9)
    assert actual.source_frame == AR_FRAME
    assert actual.target_frame == OV_RAW_LEFT_FRAME


def test_inverse_convention_fails_when_used_as_required_ov_from_ar():
    ar_rotation = _rotation((0.03, 0.01, -0.02))
    ar_translation = np.array([10.0, 5.0, 1000.0])
    expected = RigidTransform(
        "expected", AR_FRAME, OV_RAW_LEFT_FRAME, _rotation((0.0, 0.04, 0.01)), np.array([70.0, -6.0, 4.0])
    )
    ov_corners = _project_board(
        expected.rotation @ ar_rotation,
        expected.rotation @ ar_translation + expected.translation_mm,
        _intrinsics(),
    )
    correct_errors = cross_reprojection_errors_as_required(
        expected,
        ar_rotation_from_board=ar_rotation,
        ar_translation_from_board_mm=ar_translation,
        ov_corners=ov_corners,
        ov_intrinsics=_intrinsics(),
    )
    inverse_errors = cross_reprojection_errors_as_required(
        invert_transform(expected, name="inverse"),
        ar_rotation_from_board=ar_rotation,
        ar_translation_from_board_mm=ar_translation,
        ov_corners=ov_corners,
        ov_intrinsics=_intrinsics(),
    )

    assert float(np.sqrt(np.mean(correct_errors ** 2))) < 1e-9
    assert float(np.sqrt(np.mean(inverse_errors ** 2))) > 1.0


def test_raw_and_rectified_frames_cannot_be_mixed_silently():
    require_raw_frame_contract(source_frame=AR_FRAME, target_frame=OV_RAW_LEFT_FRAME, ov_intrinsics_frame=OV_RAW_LEFT_FRAME)

    with pytest.raises(StaticExtrinsicError, match="raw physical-left K/D"):
        require_raw_frame_contract(
            source_frame=AR_FRAME,
            target_frame=OV_RAW_LEFT_FRAME,
            ov_intrinsics_frame=OV_RECTIFIED_LEFT_FRAME,
        )
    with pytest.raises(StaticExtrinsicError, match="must map"):
        require_raw_frame_contract(
            source_frame=AR_FRAME,
            target_frame=OV_RECTIFIED_LEFT_FRAME,
            ov_intrinsics_frame=OV_RAW_LEFT_FRAME,
        )
