"""Opt-in diagnostic origin estimate composed with the unchanged rim transform."""

from collections.abc import Mapping
import math

import numpy as np

from sie_core.geometry.camera_lidar_mechanical_geometry_v1 import LD19_FRAME, RIM_FRAME, mechanical_transform

ESTIMATED_OPTICAL_FRAME = "estimated_physical_left_optical_frame"


def estimated_ld19_transform(mechanical, datum):
    """Return LD19 → estimated optical-origin rotation and translation in metres."""
    rotation, rim_translation = mechanical_transform(
        mechanical, source_frame=LD19_FRAME, destination_frame=RIM_FRAME,
    )
    required = {
        "schema_version": "sie.camera_lidar_optical_datum.v1",
        "status": "ESTIMATED_DIAGNOSTIC_ONLY",
        "source_frame": RIM_FRAME,
        "destination_frame": ESTIMATED_OPTICAL_FRAME,
        "reference_mechanical_profile": mechanical["profile_id"],
        "offset_direction_assumption": "ALONG_PHYSICAL_LEFT_OPTICAL_AXIS",
        "pitch_up_deg": 20.0,
        "optical_center_to_rim_robot_forward_component_m": 0.0125,
    }
    if (not isinstance(datum, Mapping) or any(datum.get(key) != value for key, value in required.items())
            or mechanical["camera_pitch_up_deg"] != 20.0):
        raise ValueError("optical datum v1 requires the declared Step 66 component, pitch and assumptions")
    if any(datum.get(flag) is not False for flag in (
        "extrinsic_independently_validated", "measurement_authorized",
        "world_state_update_authorized", "motor_control_authorized",
    )) or datum.get("uncertainty_m") is not None or datum.get("confidence") is not None:
        raise ValueError("optical datum must remain an unqualified diagnostic estimate")
    axial_m = datum["optical_center_to_rim_robot_forward_component_m"] / math.cos(
        math.radians(datum["pitch_up_deg"])
    )
    return rotation, rim_translation + np.array([0.0, 0.0, axial_m]), axial_m
