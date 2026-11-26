"""Pure diagnostic LD19 ↔ physical-left OV9281 lens-rim geometry."""

from collections.abc import Mapping
import math
from numbers import Real

import numpy as np

LD19_FRAME = "ld19_sensor"
RIM_FRAME = "physical_left_lens_front_rim_frame"
OPTICAL_FRAME = "physical_left_optical_frame"
SCHEMA = "sie.camera_lidar_mechanical_geometry.v1"
PROFILE_STATUS = "OPERATOR_MECHANICAL_GEOMETRY_DIAGNOSTIC_ONLY"

_ALIGNMENT_STATUS = {
    "ld19_yaw": "PROVISIONAL",
    "camera_yaw": "UNVERIFIED_ASSUMPTION",
    "camera_roll": "UNVERIFIED_ASSUMPTION",
}


def _finite_array(value, name):
    array = np.asarray(value, dtype=object)
    if any(isinstance(item, (bool, np.bool_)) or not isinstance(item, Real)
           for item in array.flat):
        raise ValueError(f"{name} must contain real numbers")
    try:
        with np.errstate(over="ignore", invalid="ignore"):
            array = array.astype(np.float64)
    except OverflowError as error:
        raise ValueError(f"{name} must be finite in float64") from error
    if not np.isfinite(array).all():
        raise ValueError(f"{name} must be finite in float64")
    return array


def validate_rotation(rotation):
    """Require a finite, orthogonal 3×3 rotation with determinant +1."""
    matrix = _finite_array(rotation, "rotation")
    if matrix.shape != (3, 3):
        raise ValueError("rotation must have shape (3, 3)")
    # Numerical tolerance for float64 arithmetic, independent of mount uncertainty.
    with np.errstate(over="ignore", invalid="ignore"):
        if not np.allclose(matrix.T @ matrix, np.eye(3), rtol=0, atol=1e-12):
            raise ValueError("rotation must be orthogonal")
    if not math.isclose(np.linalg.det(matrix), 1.0, rel_tol=0, abs_tol=1e-12):
        raise ValueError("rotation determinant must be +1")


def mechanical_transform(profile, *, source_frame, destination_frame):
    """Return (R, t_m) for p_destination = R @ p_source + t_m."""
    if not isinstance(profile, Mapping) or (
        profile.get("schema_version") != SCHEMA
        or profile.get("profile_id") != "camera_lidar_mechanical_geometry_v1"
        or profile.get("status") != PROFILE_STATUS
    ):
        raise ValueError("expected a diagnostic mechanical geometry v1 profile")
    if any(profile.get(flag) is not False for flag in (
        "extrinsic_independently_validated", "measurement_authorized",
        "world_state_update_authorized", "motor_control_authorized",
    )):
        raise ValueError("mechanical geometry v1 must remain diagnostic only")
    if (
        profile.get("source_frame") != LD19_FRAME
        or profile.get("destination_frame") != RIM_FRAME
        or profile.get("source_axes") != ["forward", "left", "up"]
        or profile.get("destination_axes") != ["right", "down", "forward"]
        or profile.get("length_unit") != "m"
    ):
        raise ValueError("mechanical geometry v1 requires its declared frames, axes and metre units")
    if profile.get("rim_to_optical_center_offset_m") is not None:
        raise ValueError("mechanical geometry v1 has no verified rim-to-optical-center offset")
    if profile.get("measurement_uncertainty") is not None or profile.get("confidence") is not None:
        raise ValueError("mechanical geometry v1 has unknown uncertainty and confidence")
    if OPTICAL_FRAME in (source_frame, destination_frame):
        raise ValueError("optical-center transform unavailable: verified rim-to-optical-center offset required")

    frames = (source_frame, destination_frame)
    if frames not in ((LD19_FRAME, RIM_FRAME), (RIM_FRAME, LD19_FRAME)):
        raise ValueError("unsupported source and destination frame pair")

    if profile.get("ld19_scan_plane") != "OPERATOR_CHECKED_HORIZONTAL":
        raise ValueError("expected an operator-checked horizontal LD19 scan plane")
    alignment = profile.get("alignment_assumptions")
    if not isinstance(alignment, Mapping):
        raise ValueError("explicit alignment assumptions required")
    for name, status in _ALIGNMENT_STATUS.items():
        assumption = alignment.get(name)
        if not isinstance(assumption, Mapping) or assumption.get("status") != status:
            raise ValueError(f"expected explicit {name} assumption with status {status}")
        angle = _finite_array(assumption.get("degrees"), name)
        if angle.shape != () or angle.item() != 0.0:
            raise ValueError(f"mechanical geometry v1 requires zero {name}")

    pitch = _finite_array(profile.get("camera_pitch_up_deg"), "camera pitch")
    if pitch.shape != () or not 0 <= pitch.item() <= 90:
        raise ValueError("upward camera pitch must be between 0 and 90 degrees")
    displacement = _finite_array(profile.get("ld19_origin_from_rim_robot_aligned_m"), "LD19 displacement")
    if displacement.shape != (3,):
        raise ValueError("LD19 displacement must have shape (3,)")

    angle = math.radians(pitch.item())
    sine, cosine = math.sin(angle), math.cos(angle)
    rotation = np.array([[0.0, -1.0, 0.0], [sine, 0.0, -cosine], [cosine, 0.0, sine]])
    validate_rotation(rotation)
    with np.errstate(over="ignore", invalid="ignore"):
        translation = rotation @ displacement
        if source_frame == RIM_FRAME:
            rotation = rotation.T
            translation = -(rotation @ translation)
    if not np.isfinite(translation).all():
        raise ValueError("transformed LD19 displacement must be finite")
    return rotation, translation


def transform_points(points_m, profile, *, source_frame, destination_frame):
    """Transform a point (3,) or an array of points (N, 3) in metres."""
    points = _finite_array(points_m, "points")
    if points.ndim not in (1, 2) or points.shape[-1] != 3:
        raise ValueError("points must have shape (3,) or (N, 3)")
    rotation, translation = mechanical_transform(
        profile, source_frame=source_frame, destination_frame=destination_frame,
    )
    with np.errstate(over="ignore", invalid="ignore"):
        transformed = points @ rotation.T + translation
    if not np.isfinite(transformed).all():
        raise ValueError("transformed points must be finite")
    return transformed
