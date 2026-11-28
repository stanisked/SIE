"""Regression checks for the diagnostic axle-centred geometry and TF recipe."""
from decimal import Decimal
import json
from pathlib import Path
import shlex


PACKAGE = Path(__file__).resolve().parents[1]
PROFILE = PACKAGE / "config/diagnostics/robot_geometry_footprint_v1.json"
GUIDE = PACKAGE.parents[1] / "docs/diagnostics/robot_geometry_footprint_v1.md"


def test_ld19_scan_centre_is_behind_axle():
    profile = json.loads(PROFILE.read_text(), parse_float=Decimal)
    mount = profile["ld19_mount"]
    transform = mount["candidate_static_tf"]

    assert profile["body_datum"]["axes_xyz"] == ["forward", "left", "up"]
    assert transform["parent_frame_id"] == "base_link"
    assert transform["child_frame_id"] == "ld19_sensor"
    assert transform["translation_xyz_m"] == [Decimal("-0.025"), Decimal("0.000"), Decimal("0.187")]
    assert transform["rpy_rad"] == [0, 0, 0]
    assert (
        mount["scan_plane_height_above_floor_m"] - profile["body_datum"]["axle_height_above_floor_m"]
        == transform["translation_xyz_m"][2]
    )
    assert transform["runtime_enabled"] is False


def test_physical_footprint_includes_drive_wheels():
    profile = json.loads(PROFILE.read_text(), parse_float=Decimal)
    provenance = profile["provenance"]
    footprint = profile["physical_footprint"]
    front, rear, left, right = map(footprint.get, ("front_x_m", "rear_x_m", "left_y_m", "right_y_m"))

    assert provenance["original_chassis_front_x_m"] == Decimal("0.0255")
    assert provenance["width_measured_between"] == "outermost_drive_wheel_edges"
    assert front == provenance["physical_drive_wheel_diameter_m"] / 2 == Decimal("0.034")
    assert front - rear == footprint["reported_overall_length_m"] == Decimal("0.2985")
    assert left - right == footprint["reported_overall_width_m"] == Decimal("0.320")
    assert footprint["vertices_xy_m"] == [[front, left], [rear, left], [rear, right], [front, right]]
    assert footprint["vertex_order"] == "counterclockwise"
    assert footprint["coverage_status"] == "OPERATOR_VISUAL_ASSESSMENT"
    assert "no independent measurement" in footprint["coverage_limit"]


def test_motor_off_qualification_has_explicit_evidence_limits():
    profile = json.loads(PROFILE.read_text())

    assert profile["status"] == "ROBOT_GEOMETRY_AND_FOOTPRINT_MOTOR_OFF_QUALIFIED"
    assert profile["qualification"]["runtime_parameter_service_verified"] is False
    assert profile["qualification"]["live_odom_to_ld19_chain_qualified"] is False
    assert profile["navigation_admitted"] is False
    assert profile["powered_motion_authorized"] is False


def test_documented_static_publisher_matches_profile():
    profile = json.loads(PROFILE.read_text(), parse_float=Decimal)
    transform = profile["ld19_mount"]["candidate_static_tf"]
    command = next(
        block.strip() for block in GUIDE.read_text().split("```")
        if block.startswith("bash\n   ros2 run tf2_ros static_transform_publisher")
    )
    argv = shlex.split(command.removeprefix("bash\n").replace("\\\n", " "))
    assert argv[:4] == ["ros2", "run", "tf2_ros", "static_transform_publisher"]
    assert [Decimal(argv[argv.index(flag) + 1]) for flag in ("--x", "--y", "--z")] == transform["translation_xyz_m"]
    assert [Decimal(argv[argv.index(flag) + 1]) for flag in ("--roll", "--pitch", "--yaw")] == transform["rpy_rad"]
    assert argv[argv.index("--frame-id") + 1] == transform["parent_frame_id"]
    assert argv[argv.index("--child-frame-id") + 1] == transform["child_frame_id"]
    assert argv[argv.index("-r") + 1] == f"__node:={transform['proposed_diagnostic_authority_node']}"
