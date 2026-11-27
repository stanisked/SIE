from copy import deepcopy
import json
import subprocess
import threading
import time
from types import SimpleNamespace

import pytest

from sie_ros2.live_entity_centric_node import DiagnosticSession
from sie_ros2.live_entity_centric_v1 import (
    CAMERA_SCHEMA, FRONT_FRAME, RECTIFIED_FRAME, RGB_FRAME, MASK_STATUS, TORSO_STATUS,
    camera_event, camera_view, obstacle_event,
)
from vision_core.person_localization.torso_core_geometry import TORSO_CORE_POLICY


def camera_audit(now_ns, *, cycle="cycle-1", track="track-1", offset_ms=-8, torso=True):
    now = now_ns / 1e9
    rgb_stamp = now - .060
    stereo_stamp = rgb_stamp + offset_ms / 1000
    frames = {
        key: {"sequence": number, "flags": 0x12000, "driver_timestamp_s": stamp,
              "host_packet_received_monotonic_s": stamp + .020, "pair_selected_monotonic_s": now - .025}
        for key, number, stamp in (("ar0234", 10, rgb_stamp), ("ov9281", 20, stereo_stamp))
    }
    instance = {"instance_index": 0, "bbox_xyxy_px": [400, 200, 800, 1000], "confidence": .9}
    provenance = {"source_cycle_id": cycle, "mask_sha256": "a" * 64, "model_sha256": "b" * 64,
                  "instance_index": 0, "rgb_arrays_sha256": "c" * 64, "fusion_arrays_sha256": "d" * 64}
    shape = {
        "cycle_id": cycle, "status": "MASKED_DEPTH_SHADOW_DIAGNOSTIC_ONLY",
        "reference_frame": RECTIFIED_FRAME, "unit": "m",
        "mask_depth": {"status": MASK_STATUS, "plausible_layer_count": 1, "candidate_depth_m": 2.0},
        "mask": {"status": "CURRENT_FRAME_MASK_DIAGNOSTIC_ONLY", "instance_index": 0,
                 "source_rgb_arrays_sha256": provenance["rgb_arrays_sha256"]},
        "mask_sha256": provenance["mask_sha256"], "arrays_sha256": provenance["fusion_arrays_sha256"],
        "model_sha256": provenance["model_sha256"],
    }
    if torso:
        shape["stable_torso_core_front_geometry"] = {
            "status": TORSO_STATUS, "provenance": provenance, "source_depth_status": MASK_STATUS,
            "torso_core_policy": TORSO_CORE_POLICY, "point_count": 150, "reference_frame": FRONT_FRAME,
            "unit": "m", "distance_from_front_m": None, "selected_known_term_m": 1.75,
            "known_term_m": {"median": 1.75}, "profile_sha256": "e" * 64,
        }
    return {
        "cycle_id": cycle, "schema_version": CAMERA_SCHEMA, "metric_measurement_authorized": False,
        "navigation_eligible": False, "actuator_access": False, "status": shape["status"],
        "calibration": {"ar0234_intrinsic_sha256": "1" * 64, "ov9281_stereo_sha256": "2" * 64,
                        "ar0234_to_physical_left_extrinsic_sha256": "3" * 64},
        "stereo_resolution": {"mode": "half_640x400", "scale": .5},
        "frame_freshness": {"frames": frames, "capture_backend": "latest_v4l2"},
        "segmentation_observation": {
            "cycle_id": cycle, "status": "CURRENT_INSTANCES_DIAGNOSTIC_ONLY",
            "canonical_observation_published": False, "reference_frame": RGB_FRAME, "units": "px",
            "class_name": "person", "instances": [instance], "instance_mask_sha256": ["a" * 64],
            "model_sha256": "b" * 64, "arrays_sha256": "c" * 64,
        },
        "ar0234_dynamic_target": {"disposition": "OBSERVED", "current_eligible_count": 1, "candidate": instance},
        "shadow_track_confirmation": {"track_id": track, "state": "CONFIRMED_BY_CURRENT_RGB_STEREO_ASSOCIATION",
                                      "confirmed_observation_id": f"segmentation-preview:{cycle}:0"},
        "target_geometry": {"accepted": True}, "shape_diagnostic": shape,
    }


def laser(index, *, count=3):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="ld19_sensor", stamp=SimpleNamespace(sec=1_800_000_000, nanosec=index)),
        angle_min=0., angle_max=.005 * (count - 1), angle_increment=.005, time_increment=.001,
        scan_time=.1, range_min=.1, range_max=10., ranges=[float("inf"), .03] + [2.] * (count - 2),
        intensities=[],
    )


def view(audit, now, stamps=None):
    cycle, semantic, spatial, timing, producer, _ = camera_view(audit, lambda: now, {} if stamps is None else stamps)
    return camera_event(cycle, semantic, spatial) | {"timing": timing, "producer": producer}


def closed(row):
    assert all(row[key] is False for key in ("fact_admitted", "measurement_authorized", "world_state_update_authorized",
                                            "motor_control_authorized", "decision_authorized", "navigation_authorized"))


@pytest.mark.parametrize("torso", [True, False])
def test_current_camera_builds_entity_without_lidar_or_surface_evidence(torso):
    now = 10_000_000_000
    audit = camera_audit(now, torso=torso)
    row = view(audit, now)
    obs = row["semantic"]["observation"]
    estimate = row["spatial"]["component"]
    assert row["provisional_entity"]["track_id"] == obs["track_id"] == "track-1"
    assert estimate["semantic_observation_id"] == obs["observation_id"]
    assert estimate["calibration"] == audit["calibration"]
    assert estimate["stereo_resolution"] == {"mode": "half_640x400", "scale": .5}
    assert estimate["source_sequence"] == 20 and estimate["source_timestamp_ns"] == 9_932_000_000
    assert obs["mask_evidence"]["sha256"] == "a" * 64 and obs["source_sequence"] == 10
    assert estimate["unit"] == "m"
    if torso:
        assert estimate["reference_frame"] == FRONT_FRAME and estimate["method"] == TORSO_STATUS
        assert estimate["quantity"] == "partial_front_known_term_median" and estimate["value_m"] == 1.75
        assert estimate["quality"]["torso_core"]["distance_from_front_m"] is None
    else:
        assert estimate["reference_frame"] == RECTIFIED_FRAME and estimate["quantity"] == "rectified_optical_z_median"
        assert estimate["value_m"] == 2.
    assert row["ld19"]["status"] == row["surface_level"]["status"] == "UNAVAILABLE"
    assert row["ld19_entity_association"]["links"] == []
    closed(row)


@pytest.mark.parametrize("failure,expected", [
    ("missing", "UNAVAILABLE"), ("ambiguous", "AMBIGUOUS"), ("stereo_packet", "UNAVAILABLE"),
    ("mask", "REJECTED"), ("previous_geometry", "REJECTED"), ("delta", "REJECTED"),
    ("geometry_override", "REJECTED"), ("foreign_native_clock", "REJECTED"),
])
def test_partial_camera_evidence(failure, expected):
    now = 10_000_000_000
    audit = camera_audit(now, offset_ms=-26 if failure == "delta" else -8)
    if failure == "missing":
        audit["shape_diagnostic"] = {}
    elif failure == "ambiguous":
        audit["shape_diagnostic"]["mask_depth"]["plausible_layer_count"] = 2
    elif failure == "stereo_packet":
        audit["frame_freshness"]["frames"].pop("ov9281")
    elif failure == "mask":
        audit["shape_diagnostic"]["mask_sha256"] = "f" * 64
    elif failure == "previous_geometry":
        audit["shape_diagnostic"]["stable_torso_core_front_geometry"]["provenance"]["source_cycle_id"] = "old"
    elif failure == "geometry_override":
        audit["dynamic_shadow_geometry_override"] = {"reason": "diagnostic fallback"}
    elif failure == "foreign_native_clock":
        audit["frame_freshness"]["frames"]["ov9281"]["flags"] = 0
    row = view(audit, now)
    assert row["semantic"]["status"] == "AVAILABLE"
    assert row["spatial"]["status"] == expected and row["provisional_entity"] is None
    closed(row)


def test_target_loss_hold_and_reacquisition_have_no_cached_geometry():
    now = 10_000_000_000
    stamps = {}
    first = view(camera_audit(now), now, stamps)
    assert first["provisional_entity"]
    now += 100_000_000
    lost = camera_audit(now, cycle="cycle-2")
    lost["ar0234_dynamic_target"] = {"disposition": "HELD", "reason": "CURRENT_TARGET_UNAVAILABLE"}
    lost["shadow_rgb_continuity"] = {"status": "HELD", "track_id": "track-1"}
    held = view(lost, now, stamps)
    assert held["semantic"]["status"] == "UNAVAILABLE"
    assert held["semantic"]["rgb_track_continuity"]["track_id"] == "track-1"
    assert "component" not in held["spatial"]
    assert held["provisional_entity"] is None
    now += 100_000_000
    reacquired = camera_audit(now, cycle="cycle-3", track="track-2")
    reacquired["shadow_rgb_reacquisition"] = {"status": "HELD_RGB_ID_REACQUIRED", "identity_only": True}
    current = view(reacquired, now, stamps)
    assert current["provisional_entity"]["track_id"] == "track-2"
    assert current["provisional_entity"]["candidate_id"] != first["provisional_entity"]["candidate_id"]


def test_stale_duplicate_and_later_camera_packets():
    now = 10_000_000_000
    old = camera_audit(now)
    stale = view(old, now + 250_000_000)
    assert stale["semantic"]["status"] == "STALE"
    assert stale["semantic"]["observation"]["observation_state"] == "STALE"
    assert stale["provisional_entity"] is None and stale["producer"]["spatial_status"] == "AVAILABLE"
    stamps = {}
    assert view(old, now, stamps)["provisional_entity"]
    reused = view(deepcopy(old), now + 5_000_000, stamps)
    assert reused["semantic"]["status"] == "REUSED" and reused["provisional_entity"] is None
    newer = camera_audit(now + 70_000_000, cycle="cycle-2")
    assert view(newer, now + 70_000_000, stamps)["provisional_entity"]


def test_ld19_remains_independent_during_slow_camera_and_replaces_pending(tmp_path):
    camera_started, camera_release = threading.Event(), threading.Event()
    lidar_started, lidar_release = threading.Event(), threading.Event()
    obstacle_received = threading.Event()
    camera_output, lidar_output = [], []

    def slow_camera(value):
        camera_started.set()
        assert camera_release.wait(3)
        return value

    def slow_scan(value):
        if value.header.stamp.nanosec == 1:
            lidar_started.set()
            assert lidar_release.wait(3)
        return value

    def publish_lidar(encoded):
        lidar_output.append(json.loads(encoded))
        obstacle_received.set()

    session = DiagnosticSession(tmp_path / "run", "enabled", slow_camera, slow_scan,
                                publish_camera=camera_output.append, publish_ld19=publish_lidar)
    try:
        session.camera.submit(camera_audit(time.monotonic_ns()))
        assert camera_started.wait(2)
        session.ld19.submit(laser(1))
        assert lidar_started.wait(2)
        for index in range(2, 81):
            session.ld19.submit(laser(index))
        assert session.ld19.writer.pending.qsize() == 1
        assert session.ld19.writer.dropped == 78
        lidar_release.set()
        assert obstacle_received.wait(2)
        assert camera_output == []
    finally:
        lidar_release.set()
        camera_release.set()
        session.close()

    assert [row["ld19"]["observation"]["header"]["stamp_nanosec"] for row in lidar_output] == [1, 80]
    scan = lidar_output[-1]["ld19"]["observation"]
    assert scan["reference_frame"] == "ld19_sensor" and scan["units"]["bearing"] == "rad"
    assert scan["timing"]["receipt_sequence"] == 80 and scan["physical_acquisition_time"] == "UNKNOWN"
    assert scan["beams"][0]["status"] == "INVALID_RANGE" and "xy_m" not in scan["beams"][0]
    assert scan["beams"][1]["status"] == "RANGE_OUT_OF_BOUNDS" and "xy_m" not in scan["beams"][1]
    assert scan["beams"][2]["status"] == "VALID_RETURN"
    assert lidar_output[-1]["ld19_entity_association"]["links"] == []
    closed(lidar_output[-1])
    summary = json.loads((tmp_path / "run/summary.json").read_text())
    assert summary["ld19"]["input"]["count"] == 80
    assert summary["ld19"]["rates"]["obstacle_updates"]["count"] == 2
    assert summary["ld19"]["pending"] == 0 and summary["ld19"]["max_pending_observed"] == 1
    assert not session.ld19.writer.worker.is_alive() and not session.camera.writer.worker.is_alive()


def test_ld19_only_duplicates_and_stale_host_receipt(tmp_path):
    now = 10_000_000_000
    session = DiagnosticSession(tmp_path / "ld", "enabled", lambda data: data, lambda data: data, clock=lambda: now)
    try:
        first, *_ = session.ld19_adapter(laser(5), now - 300_000_000, 1, 1_800_000_000_000_000_000)
        assert first["ld19"]["status"] == "STALE"
        assert first["ld19"]["observation"]["timing"]["header_age_ns"] is None
        repeated, *_ = session.ld19_adapter(laser(5), now, 2, 1_800_000_000_000_000_000)
        assert repeated["ld19"]["status"] == "REUSED" and "observation" not in repeated["ld19"]
        bad = laser(6)
        bad.header.frame_id = "wrong"
        refused, *_ = session.ld19_adapter(bad, now, 3, 1_800_000_000_000_000_000)
        assert refused["ld19"]["status"] == "REJECTED"
        assert obstacle_event(first["ld19"])["camera_ld19_timing"]["status"] == "NOT_EVALUATED_INDEPENDENT_STREAMS"
    finally:
        session.close()


def test_baseline_and_enabled_receive_same_sources_with_no_baseline_publications(tmp_path):
    now = 10_000_000_000
    baseline_out, enabled_out = [], []
    for mode, out in (("baseline", baseline_out), ("enabled", enabled_out)):
        session = DiagnosticSession(tmp_path / mode, mode, lambda data: data, lambda data: data,
                                    publish_camera=out.append, publish_ld19=out.append, clock=lambda: now)
        try:
            source = camera_audit(now)
            previous = deepcopy(source)
            session.camera.submit(source)
            session.ld19.submit(laser(1))
        finally:
            session.close()
        assert source == previous
    assert baseline_out == [] and len(enabled_out) == 2
    base, enabled = [json.loads((tmp_path / mode / "summary.json").read_text()) for mode in ("baseline", "enabled")]
    for lane in ("camera", "ld19"):
        assert base[lane]["input"] == enabled[lane]["input"]
        assert base[lane]["rates"]["ros_output"]["count"] == 0
        assert enabled[lane]["rates"]["ros_output"]["count"] == 1
        assert base[lane]["processing_or_journal_errors"] == enabled[lane]["processing_or_journal_errors"] == 0
    assert base["camera"]["rates"]["rgb_producer"] == enabled["camera"]["rates"]["rgb_producer"]
    assert enabled["camera"]["rates"]["entity_preview"]["count"] == 1
    assert enabled["ld19"]["rates"]["obstacle_updates"]["count"] == 1
    closed(enabled)


def test_launcher_requires_an_explicit_mode_without_initializing_ros():
    path = "vision_core/tools/run_live_entity_centric_v1.sh"
    for arguments, expected in (([], 2), (["--help"], 0), (["enabled", "unexpected"], 2)):
        result = subprocess.run(["bash", path, *arguments], check=False, capture_output=True)
        assert result.returncode == expected
