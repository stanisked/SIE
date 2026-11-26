import json
from pathlib import Path

import numpy as np
import pytest

from sie_core.geometry.camera_lidar_mechanical_geometry_v1 import (
    LD19_FRAME,
    OPTICAL_FRAME,
    RIM_FRAME,
    mechanical_transform,
    transform_points,
    validate_rotation,
)

PROFILE_PATH = (
    Path(__file__).resolve().parents[1]
    / "vision_core/config/diagnostics/camera_lidar_mechanical_geometry_v1.json"
)
FORWARD = {"source_frame": LD19_FRAME, "destination_frame": RIM_FRAME}
INVERSE = {"source_frame": RIM_FRAME, "destination_frame": LD19_FRAME}
TOL = 1e-14


@pytest.fixture
def profile():
    return json.loads(PROFILE_PATH.read_text())


def test_rotation_is_right_handed_and_uses_optical_axes(profile):
    rotation, _ = mechanical_transform(profile, **FORWARD)
    np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=TOL, rtol=0)
    assert np.linalg.det(rotation) == pytest.approx(1.0, abs=TOL)
    # LD19 left becomes optical left (negative X).
    np.testing.assert_array_equal(rotation[:, 1], [-1, 0, 0])
    # Horizontal forward is below the upward-pitched axis, positive optical Y.
    np.testing.assert_allclose(rotation[:, 0], [0, 0.3420201433256687, 0.9396926207859084],
                               atol=TOL, rtol=0)
    assert rotation[1, 2] < 0  # Robot up points towards negative optical Y.
    assert np.degrees(np.arctan2(rotation[1, 0], rotation[2, 0])) == pytest.approx(20)


def test_origin_translation_and_known_forward_points_are_in_metres(profile):
    points = [[0, 0, 0], [1, 0, 0], [1, 0.25, 0]]
    transformed = transform_points(points, profile, **FORWARD)
    np.testing.assert_allclose(transformed, [
        [-0.03, -0.1292008784077965, 0.2151653620326789],
        [-0.03, 0.2128192649178722, 1.1548579828185874],
        [-0.28, 0.2128192649178722, 1.1548579828185874],
    ], atol=TOL, rtol=0)


def test_inverse_coefficients_and_both_round_trips(profile):
    rotation, translation = mechanical_transform(profile, **FORWARD)
    inverse_rotation, inverse_translation = mechanical_transform(profile, **INVERSE)
    np.testing.assert_array_equal(inverse_rotation, rotation.T)
    np.testing.assert_allclose(inverse_translation, [-0.158, -0.030, -0.195], atol=TOL, rtol=0)
    np.testing.assert_allclose(inverse_translation, -rotation.T @ translation, atol=TOL, rtol=0)
    # A point on the rim's optical-axis direction has zero optical X and Y.
    point = inverse_rotation @ [0, 0, 2] + inverse_translation
    np.testing.assert_allclose(transform_points(point, profile, **FORWARD), [0, 0, 2], atol=TOL, rtol=0)
    points = np.array([[0, 0, 0], [2, -0.4, 1.2], [-4, 3.5, -3]])
    for first, second in ((FORWARD, INVERSE), (INVERSE, FORWARD)):
        mapped = transform_points(points, profile, **first)
        np.testing.assert_allclose(transform_points(mapped, profile, **second), points, atol=TOL, rtol=0)
    assert transform_points(np.empty((0, 3)), profile, **FORWARD).shape == (0, 3)


@pytest.mark.parametrize("source,destination", [(LD19_FRAME, OPTICAL_FRAME), (OPTICAL_FRAME, LD19_FRAME)])
def test_no_optical_center_transform_without_verified_offset(profile, source, destination):
    with pytest.raises(ValueError, match="optical-center transform unavailable"):
        mechanical_transform(profile, source_frame=source, destination_frame=destination)
    profile["rim_to_optical_center_offset_m"] = [0, 0, 0]
    with pytest.raises(ValueError, match="no verified rim-to-optical-center offset"):
        mechanical_transform(profile, source_frame=source, destination_frame=destination)


@pytest.mark.parametrize(("key", "value", "message"), [
    ("length_unit", "mm", "metre units"),
    ("destination_axes", ["left", "up", "forward"], "declared frames"),
    ("source_frame", "base_link", "declared frames"),
    ("ld19_origin_from_rim_robot_aligned_m", [0.158, 0.030], "shape"),
    ("ld19_origin_from_rim_robot_aligned_m", [np.inf, 0, 0], "finite"),
    ("ld19_origin_from_rim_robot_aligned_m", [True, 0, 0], "real numbers"),
    ("camera_pitch_up_deg", -20, "between 0 and 90"),
    ("camera_pitch_up_deg", np.nan, "finite"),
    ("camera_pitch_up_deg", "20", "real numbers"),
    ("ld19_scan_plane", "UNKNOWN", "horizontal"),
    ("alignment_assumptions", None, "explicit alignment"),
    ("extrinsic_independently_validated", True, "diagnostic only"),
    ("motor_control_authorized", True, "diagnostic only"),
    ("measurement_uncertainty", 0, "unknown uncertainty"),
])
def test_rejects_invalid_profile_geometry(profile, key, value, message):
    profile[key] = value
    with pytest.raises(ValueError, match=message):
        mechanical_transform(profile, **FORWARD)


def test_v1_keeps_assumptions_explicit_and_rejects_unsupported_alignment(profile):
    assert profile["operator_provenance"]["reported_date"] == "2026-11-26"
    assert profile["operator_provenance"]["superseded_pitch_deg_for_this_profile"] == [10.0, 15.0]
    assert profile["confidence"] is profile["measurement_uncertainty"] is None
    assert all(profile[key] is False for key in (
        "measurement_authorized", "world_state_update_authorized", "motor_control_authorized"))
    assert profile["alignment_assumptions"] == {
        "ld19_yaw": {"degrees": 0.0, "status": "PROVISIONAL"},
        "camera_yaw": {"degrees": 0.0, "status": "UNVERIFIED_ASSUMPTION"},
        "camera_roll": {"degrees": 0.0, "status": "UNVERIFIED_ASSUMPTION"},
    }
    profile["alignment_assumptions"]["ld19_yaw"]["degrees"] = 1.0
    with pytest.raises(ValueError, match="requires zero ld19_yaw"):
        mechanical_transform(profile, **FORWARD)
    profile["alignment_assumptions"]["ld19_yaw"]["degrees"] = 0.0
    profile["alignment_assumptions"]["camera_roll"]["status"] = "VALIDATED"
    with pytest.raises(ValueError, match="explicit camera_roll assumption"):
        mechanical_transform(profile, **FORWARD)


@pytest.mark.parametrize("points", [[np.inf, 0, 0], [0, np.nan, 0], [1, 0], [1j, 0, 0], 1])
def test_invalid_points_refuse(profile, points):
    with pytest.raises(ValueError):
        transform_points(points, profile, **FORWARD)


@pytest.mark.parametrize(("rotation", "message"), [
    (np.diag([1, 1, -1]), "determinant"),
    (np.diag([1, 1, 2]), "orthogonal"),
    ([[0, 1], [1, 0]], "shape"),
    (np.diag([np.nan, 1, 1]), "finite"),
])
def test_invalid_rotation_refuses(rotation, message):
    with pytest.raises(ValueError, match=message):
        validate_rotation(rotation)


def test_unsupported_frame_pair_refuses(profile):
    with pytest.raises(ValueError, match="unsupported source and destination"):
        mechanical_transform(profile, source_frame="base_link", destination_frame=RIM_FRAME)
