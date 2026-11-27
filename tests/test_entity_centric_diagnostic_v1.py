import json
from copy import deepcopy
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from sie_ros2.ld19_diagnostic_evidence import make_ld19_diagnostic_record
from sie_ros2.live_temporal_shadow_monitor import LiveTemporalShadowMonitor
from vision_core.person_localization.shadow_source_snapshot import LatestSourceSnapshotWriter, pack_mask
from vision_core.stereo.entity_centric_diagnostic_v1 import evidence, integrate_cycle, lidar_observation
from vision_core.stereo.three_sensor_semantic_spatial_v1 import MATCH, RGB_SHAPE
from vision_core.tools import replay_entity_centric_diagnostic_v1 as tool
from vision_core.tools import replay_three_sensor_semantic_v1 as step71
from vision_core.tools import stereo_lidar_joint_static_v1 as joint
from test_stereo_lidar_joint_static_v1 import archive_synthetic_pair
from test_three_sensor_semantic_spatial_v1 import (
    CYCLE, MODEL, MS, ROT, ZERO, make_manifest, make_timing, policies, shape_scene,
)


def raw_scan(scan, *, angle=0., ranges=(2., float("inf"), .01)):
    stamp = SimpleNamespace(sec=scan.header_ns // 10**9, nanosec=scan.header_ns % 10**9)
    return SimpleNamespace(
        header=SimpleNamespace(stamp=stamp, frame_id="ld19_sensor"),
        angle_min=angle, angle_max=angle + .005 * (len(ranges) - 1),
        angle_increment=.005, time_increment=.001, scan_time=.1,
        range_min=.1, range_max=10., ranges=ranges, intensities=[],
    )


def with_scan(row, scan, **geometry):
    row["ld19_payload_archive"] = LiveTemporalShadowMonitor._archive_scan(
        row, ((scan.receipt_sequence, scan.header_ns, raw_scan(scan, **geometry)),),
    )
    return row


def assert_closed(result):
    assert all(result[key] is False for key in (
        "fact_admitted", "measurement_authorized", "world_state_update_authorized",
        "decision_authorized", "navigation_authorized", "motor_control_authorized",
    ))


@pytest.fixture
def camera(policies, monkeypatch):
    row, frames, scan = make_timing()
    mask = np.ones(RGB_SHAPE, bool)
    manifest = make_manifest(mask)
    manifest["step71_semantic"]["target"]["candidate"]["confidence"] = .91
    manifest["frame_freshness"] = frames
    manifest["npz_sha256"] = "e" * 64
    disparity, lr, grid, calibration = shape_scene(2.)
    policy = policies[1]
    monkeypatch.setattr(step71, "reconstruct_snapshot", lambda *args: (disparity, lr, grid))
    return row, scan, (manifest, b"stereo", mask, "d" * 64), calibration, policy


def integrate(camera, *, surface="NO_CANDIDATES"):
    row, _, snapshot, calibration, policy = camera
    semantic, spatial = tool.independent_camera_evidence(row, snapshot, calibration, None, policy)
    link = row["zero_wait"]["rgb_ld19_ros_header"]
    lidar = tool.independent_ld19_evidence(row, link)
    return integrate_cycle(CYCLE, semantic, spatial, lidar, rgb_ld19=link,
                           surface_level=evidence(surface, "TEST_SURFACE_CONTEXT"))


def test_candidate_with_zero_surface_matches_preserves_independent_observations(camera):
    row, scan, snapshot, _, _ = camera
    with_scan(row, scan)
    result = integrate(camera)
    target = result["semantic"]["observation"]
    component = result["spatial"]["component"]
    lidar = result["ld19"]["observation"]
    assert result["provisional_entity"]["semantic_observation_id"] == target["observation_id"]
    assert result["provisional_entity"]["spatial_component_id"] == component["observation_id"]
    assert target["observation_id"] == f"segmentation-preview:{CYCLE}:0"
    assert target["semantic_class"] == "person" and target["track_id"] == "person-1"
    assert target["reference_frame"] == "ar0234_image_frame" and target["unit"] == "px"
    assert target["observation_state"] == "CURRENT" and target["confidence"] == .91
    assert target["source_timestamp_ns"] == row["rgb"]["timestamp_ns"]
    assert target["mask_evidence"]["sha256"] == snapshot[0]["instance_mask_sha256"]
    assert component["reference_frame"] == "physical_left_optical_frame" and component["unit"] == "m"
    assert component["component_grid_reference_frame"] == "rectified_left_optical_frame"
    assert component["component_grid_unit"] == "px"
    assert component["depth_m"] == pytest.approx(2.)
    assert component["calibration"] == step71.ARTIFACT_SHA256
    assert component["source_timestamp_ns"] == row["actual_camera_pair"]["stereo"]["timestamp_ns"]
    assert lidar["header_timestamp_ns"] == scan.header_ns
    assert lidar["reference_frame"] == "ld19_sensor" and lidar["units"]["position"] == "m"
    assert lidar["physical_acquisition_time"] == "UNKNOWN"
    assert [beam["status"] for beam in lidar["beams"]] == ["VALID_RETURN", "INVALID_RANGE", "RANGE_OUT_OF_BOUNDS"]
    assert lidar["beams"][0]["xy_m"] == [2.0, 0.]
    assert "xy_m" not in lidar["beams"][1]
    assert result["camera_ld19_timing"]["status"] == MATCH
    assert result["ld19_entity_association"] == {"status": "ENTITY_MEMBERSHIP_UNRESOLVED", "links": []}
    assert_closed(result)


@pytest.mark.parametrize("missing", ["rgb", "target", "stereo", "ld19"])
def test_independent_evidence_when_source_is_missing(camera, missing):
    row, scan, snapshot, _, _ = camera
    if missing == "rgb":
        row["rgb"] = None
    elif missing == "target":
        snapshot[0]["step71_semantic"]["target"] = {"current_eligible_count": 0}
    elif missing == "stereo":
        row["actual_camera_pair"]["stereo"] = None
    if missing != "ld19":
        with_scan(row, scan)
    result = integrate(camera)
    assert (result["provisional_entity"] is not None) == (missing == "ld19")
    assert result["ld19"]["status"] == ("UNAVAILABLE" if missing == "ld19" else "AVAILABLE")
    assert result["semantic"]["status"] == ("UNAVAILABLE" if missing in ("rgb", "target") else "AVAILABLE")
    assert_closed(result)


def test_ambiguous_camera_components_and_lidar_correspondences(camera, monkeypatch):
    row, scan, _, _, _ = camera
    with_scan(row, scan)
    association = integrate(camera, surface="AMBIGUOUS")
    assert association["ld19_entity_association"] == {"status": "AMBIGUOUS", "links": []}
    assert association["provisional_entity"]
    _, columns = np.indices((80, 100))
    disparity, lr, grid, _ = shape_scene(np.where(columns < 50, 1.3, 3.1))
    monkeypatch.setattr(step71, "reconstruct_snapshot", lambda *args: (disparity, lr, grid))
    result = integrate(camera)
    assert result["semantic"]["status"] == "AVAILABLE"
    assert result["spatial"]["status"] == "AMBIGUOUS"
    assert result["provisional_entity"] is None
    assert result["ld19"]["status"] == "AVAILABLE"


@pytest.mark.parametrize("failure", ["stale_rgb", "stale_stereo", "stale_ld19", "incompatible_stereo",
                                     "incompatible_ld19", "future_stereo", "ambiguous_rgb"])
def test_freshness_and_pairing_refusals_keep_usable_sources(camera, failure):
    row, scan, snapshot, _, _ = camera
    manifest = snapshot[0]
    with_scan(row, scan)
    if failure in ("stale_rgb", "stale_stereo", "future_stereo"):
        rgb = failure == "stale_rgb"
        packet = row["rgb"] if rgb else row["actual_camera_pair"]["stereo"]
        offset = 450 * MS if failure == "future_stereo" else -450 * MS
        packet["timestamp_ns"] += offset
        packet_key = "ar0234" if rgb else "ov9281"
        manifest["frame_freshness"]["frames"][packet_key]["driver_timestamp_s"] = packet["timestamp_ns"] / 1e9
    elif failure == "stale_ld19":
        row["zero_wait"]["rgb_ld19_ros_header"]["candidate_freshness"]["status"] = "SOURCE_STALE"
    elif failure == "incompatible_stereo":
        row["zero_wait"]["rgb_stereo"]["status"] = "DELTA_EXCEEDED"
    elif failure == "incompatible_ld19":
        row["zero_wait"]["rgb_ld19_ros_header"]["status"] = "DELTA_EXCEEDED"
    else:
        manifest["step71_semantic"]["target"]["current_eligible_count"] = 2
    result = integrate(camera)
    assert result["ld19_entity_association"]["links"] == []
    assert (result["ld19"]["status"] == "STALE") == (failure == "stale_ld19")
    if failure in ("stale_ld19", "incompatible_ld19"):
        assert result["provisional_entity"]
        assert result["ld19_entity_association"]["status"] != "ENTITY_MEMBERSHIP_UNRESOLVED"
    else:
        assert result["provisional_entity"] is None
        assert result["ld19"]["status"] == "AVAILABLE"
    if failure == "stale_rgb":
        assert result["semantic"]["status"] == "STALE"
    elif failure == "ambiguous_rgb":
        assert result["semantic"]["status"] == "AMBIGUOUS"
    elif failure == "stale_stereo":
        assert result["spatial"]["status"] == "STALE"
    elif failure in ("future_stereo", "incompatible_stereo"):
        assert result["spatial"]["status"] == "REJECTED"
        assert result["semantic"]["status"] == "AVAILABLE"


def test_held_tracking_metadata_is_not_a_current_candidate(camera):
    _, _, snapshot, _, _ = camera
    meta = snapshot[0]["step71_semantic"]
    meta["rgb_track_continuity"] = {"status": "HELD_RGB_ID_DIAGNOSTIC_ONLY", "track_id": "person-1"}
    meta["target"] = {"disposition": "HELD", "current_eligible_count": 0}
    result = integrate(camera)
    assert result["semantic"]["target_selection"]["disposition"] == "HELD"
    assert result["semantic"]["rgb_track_continuity"]["track_id"] == "person-1"
    assert result["provisional_entity"] is None


def test_spatial_binding_mismatch_cannot_create_entity(camera):
    result = integrate(camera)
    broken = deepcopy(result["spatial"])
    broken["component"]["semantic_observation_id"] = "unrelated"
    result = integrate_cycle(CYCLE, result["semantic"], broken, result["ld19"], rgb_ld19={},
                             surface_level=evidence("UNAVAILABLE", "UNSET"))
    assert result["spatial"]["status"] == "REJECTED"
    assert result["provisional_entity"] is None


def test_independent_ld19_cli_replay_does_not_require_camera_calibration(tmp_path, monkeypatch):
    _, _, scan = make_timing()
    record = make_ld19_diagnostic_record(raw_scan(scan), received_at_utc="2026-11-27T00:00:00+00:00")
    tampered = deepcopy(record)
    tampered["payload"]["scan"]["ranges_m"][0] = 3.
    with pytest.raises(ValueError, match="IDENTITY_MISMATCH"):
        lidar_observation(tampered, timing={})
    journal = tmp_path / "ld19.jsonl"
    journal.write_text("\n".join(map(joint.encoded, (record, tampered))) + "\n")
    output = tmp_path / "preview"
    monkeypatch.setattr("sys.argv", ["replay", "--ld19-journal", str(journal), "--output", str(output)])
    tool.main()
    summary = json.loads((output / "summary.json").read_text())
    assert summary["source_cycles"] == summary["standalone_ld19_records"] == 2
    assert summary["independent_ld19_obstacle_observation_count"] == 1
    assert summary["source_dispositions"]["ld19"] == {"AVAILABLE": 1, "REJECTED": 1}
    valid, bad = joint.read_rows(output / "cycles.jsonl")
    assert valid["ld19"]["observation"]["observation_id"] == record["observation_id"]
    assert valid["ld19"]["observation"]["timing"]["status"] == "UNASSESSED_INDEPENDENT_CLOCK"
    assert valid["provisional_entity"] is None and bad["ld19"]["status"] == "REJECTED"
    assert_closed(summary)


def test_replay_uses_archives_preserves_audit_only_cycles_and_is_deterministic(tmp_path, monkeypatch, policies):
    fixture = tmp_path / "fixture"
    archive_synthetic_pair(fixture, monkeypatch)
    stereo_jpeg = (fixture / "frame.mjpg").read_bytes()
    cal = joint.load_calibration((fixture / "v7.npz").read_bytes())
    cal.update({"K": np.array([[640., 0, 960.], [0, 640., 600.], [0, 0, 1.]]),
                "R": np.eye(3), "T": ZERO, "D": np.zeros(5), "size": np.array(joint.SIZE)})
    local_policy, policy = policies
    rgb = np.full((*RGB_SHAPE, 3), 50, np.uint8)
    ok, encoded = cv2.imencode(".jpg", rgb)
    assert ok
    mask = np.zeros(RGB_SHAPE, bool)
    mask[480:760, 760:1160] = True
    manifest = make_manifest(mask)
    manifest["segmentation_rgb_payload_sha256"] = joint.digest(rgb.tobytes())
    row, frames, scan = make_timing()
    manifest["frame_freshness"] = frames
    with_scan(row, scan, angle=1.4, ranges=(2.0, 4.0))
    audit_id = "another-cycle"
    audit_row = deepcopy(row) | {"cycle_id": audit_id, "actual_camera_pair": {"stereo": None}}
    run = tmp_path / "run"
    journal = run / "control-preview-test/temporal-association-shadow.jsonl"
    journal.parent.mkdir(parents=True)
    journal.write_text(joint.encoded(row) + "\n" + joint.encoded(audit_row) + "\n")
    metadata = manifest["step71_semantic"]
    audit = {
        "schema_version": "sie.ar0234_ov9281.control_input_preview.v1",
        "cycle_id": audit_id, "frame_freshness": frames,
        "segmentation_observation": metadata["segmentation"] | {"cycle_id": audit_id},
        "ar0234_dynamic_target": metadata["target"],
    }
    joint.write_json(journal.parent / f"{audit_id}.json", audit)
    writer = LatestSourceSnapshotWriter(run / "shadow-source-snapshots")
    try:
        assert writer.submit(CYCLE, encoded.tobytes(), stereo_jpeg, np.zeros((400, 640)),
                             np.zeros((400, 640), bool), cal, pack_mask(mask), manifest)
    finally:
        writer.close()
    prior = tmp_path / "prior"
    step71.replay(run, prior, (cal, ROT, ZERO, 0., local_policy, policy), MODEL)
    first, second = tmp_path / "preview", tmp_path / "repeat"
    summary_path = tool.replay(run, first, (cal, policy), MODEL, surface_replay=prior)
    tool.replay(run, second, (cal, policy), MODEL, surface_replay=prior)
    for filename in ("cycles.jsonl", "summary.json"):
        assert (first / filename).read_bytes() == (second / filename).read_bytes()
    result, audit_only = joint.read_rows(first / "cycles.jsonl")
    assert result["surface_level"]["status"] == "NO_CANDIDATES"
    assert result["provisional_entity"]
    assert audit_only["semantic"]["status"] == "AVAILABLE"
    assert audit_only["semantic"]["observation"]["mask_evidence"]["payload_verified"] is False
    assert audit_only["spatial"]["status"] == "UNAVAILABLE" and audit_only["ld19"]["status"] == "AVAILABLE"
    assert audit_only["camera_pair_timing"] == audit_row["zero_wait"]["rgb_stereo"]
    summary = json.loads(summary_path.read_text())
    assert summary["source_cycles"] == 2 and summary["semantic_candidate_count"] == 2
    assert summary["provisional_entity_candidate_count"] == 1
    assert summary["independent_ld19_obstacle_observation_count"] == 2
    assert summary["independent_ld19_unique_scan_count"] == 1
    assert summary["ld19_entity_associations"] == 0
    assert json.loads((first / "processing_cost.json").read_text())["wall_time_ns"] > 0
    assert_closed(summary)

    # A refused optional surface replay cannot veto the independently bound sources.
    (prior / "policy.json").write_text("{}")
    third = tmp_path / "refused-surface"
    tool.replay(run, third, (cal, policy), MODEL, surface_replay=prior)
    conflict, _ = joint.read_rows(third / "cycles.jsonl")
    assert conflict["surface_level"]["status"] == "REJECTED"
    assert conflict["provisional_entity"] == result["provisional_entity"]
    assert conflict["ld19"] == result["ld19"]
