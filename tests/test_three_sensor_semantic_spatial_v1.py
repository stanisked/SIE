import hashlib
import json
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from sie_ros2.latest_v4l2_capture import CameraTiming
from sie_ros2.live_temporal_association_v1 import CycleTiming, ScanTiming, evaluate_cycle
from sie_ros2.live_temporal_shadow_monitor import LiveTemporalShadowMonitor
from vision_core.person_localization.shadow_source_snapshot import LatestSourceSnapshotWriter, pack_mask
from vision_core.person_localization.torso_core_geometry import TORSO_CORE_POLICY
from vision_core.stereo.local_stereo_lidar_correspondence_v1 import stereo_grid
from vision_core.stereo.three_sensor_semantic_spatial_v1 import (
    RGB_SHAPE, associate_cycle, bind_semantic_target, mask_coordinates, project_ar,
    semantic_snapshot_metadata, semantic_stereo_component, temporal_eligibility,
)
from vision_core.tools import replay_stereo_lidar_local_v1 as step69
from vision_core.tools import replay_three_sensor_semantic_v1 as tool
from vision_core.tools import stereo_lidar_joint_static_v1 as joint
from test_stereo_lidar_joint_static_v1 import archive_synthetic_pair, matrices

MS = 1_000_000
T = 10_000 * MS
OFFSET = 1_800_000_000_000_000_000
CYCLE = "preview-cycle-0003"
MODEL = "a" * 64
ROT = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]])
ZERO = np.zeros(3)


@pytest.fixture
def policies():
    return step69.frozen_policy()[1], json.loads((joint.CONFIG / tool.POLICY_FILE).read_text())


def make_timing(stereo_ms=-7, ld_ms=-15):
    from sie_ros2.host_timebase_bridge_v1 import make_host_timebase_bridge
    from sie_ros2.live_temporal_association_v1 import BRIDGE_POLICY
    boot = "test"
    cutoff = T + 32 * MS
    brackets = [{"monotonic_before_ns": t - 80, "realtime_ns": t + OFFSET,
                 "monotonic_after_ns": t + 80} for t in (T - MS, T, T + MS)]
    bridge = make_host_timebase_bridge(brackets, boot_id=boot, policy=BRIDGE_POLICY)
    rgb = CameraTiming(10, T / 1e9, (T + 5 * MS) / 1e9, cutoff / 1e9)
    stereo = CameraTiming(11, (T + stereo_ms * MS) / 1e9, (T + 27 * MS) / 1e9, (T + 30 * MS) / 1e9)
    scan = ScanTiming(4, T + OFFSET + ld_ms * MS, T + 2 * MS, "ld19_sensor",
                      "OPERATOR_ATTESTED_SAME_HOST_REALTIME")
    cycle = CycleTiming(CYCLE, "PREVIEW", rgb, stereo, "NEW_RGB_ARRIVAL", T, T, cutoff, cutoff, cutoff, {})
    timing = evaluate_cycle(cycle, {"ov9281": ((stereo,), None), "ar0234": ((rgb,), None)}, (scan,), None,
                            ((T + 2 * MS, bridge),), boot_id=boot, session="run", ld19_clock_evidence="e")
    frames = {"frames": {key: value._asdict() for key, value in (("ar0234", rgb), ("ov9281", stereo))}}
    return timing, frames, scan


def make_manifest(mask, *, torso=False):
    mask_hash = hashlib.sha256(mask.tobytes()).hexdigest()
    instance = {"instance_index": 0, "bbox_xyxy_px": [600., 300., 1320., 950.], "truncated_top": torso}
    seg = {"status": "CURRENT_INSTANCES_DIAGNOSTIC_ONLY", "cycle_id": CYCLE, "instances": [instance],
           "instance_mask_sha256": [mask_hash], "model_sha256": MODEL, "arrays_sha256": "rgb-binding",
           "input_transport": "jpeg"}
    shape = {"cycle_id": CYCLE, "mask_sha256": mask_hash, "model_sha256": MODEL, "arrays_sha256": "fusion",
             "mask": {"status": "CURRENT_FRAME_MASK_DIAGNOSTIC_ONLY", "instance_index": 0,
                      "source_rgb_arrays_sha256": "rgb-binding"}}
    audit = {"segmentation_observation": seg, "shape_diagnostic": shape,
             "ar0234_dynamic_target": {"disposition": "OBSERVED", "current_eligible_count": 1, "candidate": instance},
             "target_geometry": {"accepted": True},
             "shadow_track_confirmation": {"track_id": "person-1",
                 "state": "CONFIRMED_BY_CURRENT_RGB_STEREO_ASSOCIATION",
                 "confirmed_observation_id": f"segmentation-preview:{CYCLE}:0"}}
    manifest = {"cycle_id": CYCLE, "track_id": "person-1", "selected_instance_index": 0,
                "instance_mask_sha256": mask_hash, "segmentation_model_sha256": MODEL,
                "fusion_arrays_sha256": "fusion", "calibration": tool.ARTIFACT_SHA256,
                "step71_semantic": semantic_snapshot_metadata(audit, {"cycle_id": CYCLE, "mask_sha256": mask_hash})}
    if torso:
        manifest["stable_torso_core_front_geometry"] = {
            "status": "STABLE_TORSO_CORE_FRONT_DIAGNOSTIC_ONLY", "torso_core_policy": TORSO_CORE_POLICY,
            "point_count": 140, "provenance": {"source_cycle_id": CYCLE, "mask_sha256": mask_hash,
                "instance_index": 0, "model_sha256": MODEL, "rgb_arrays_sha256": "rgb-binding",
                "fusion_arrays_sha256": "fusion"},
        }
    return manifest


def shape_scene(z_m):
    shape = (80, 100)
    p, q = matrices()
    cal = {"P1": p, "P2": p, "Q": q, "R1": np.eye(3), "R": np.eye(3), "T": ZERO,
           "K": np.array([[100., 0, 960.], [0, 100., 600.], [0, 0, 1.]]), "D": np.zeros(5)}
    disparity = 10 / np.broadcast_to(z_m, shape)
    lr = np.ones(shape, bool)
    return disparity, lr, stereo_grid(disparity, lr, q, cal["R1"], lr), cal


def test_extrinsic_inversion_and_distorted_projection():
    r, _ = cv2.Rodrigues(np.array([.01, -.15, .04]))
    t = np.array([.05, -.1, .03])
    k = np.array([[1000., 0, 960.], [0, 900., 600.], [0, 0, 1.]])
    d = np.array([.10, -.03, .001, .002, 0.])
    ar = np.array([[.2, .05, 2.], [-1, .5, 4.]])
    actual, depth = project_ar(ar @ r.T + t, {"R": r, "T": t, "K": k, "D": d})
    want, _ = cv2.projectPoints(ar, np.zeros(3), np.zeros(3), k, d)
    np.testing.assert_allclose(actual, want[:, 0, :])
    np.testing.assert_allclose(depth, ar[:, 2])


def test_resized_letterbox_uses_explicit_pixel_center_mapping():
    pixels = np.array([[0., 0.], [959.5, 599.5], [1919., 1199.]])
    np.testing.assert_array_equal(mask_coordinates(pixels, {"mode": "ORIGINAL_DISTORTED_RGB"}, RGB_SHAPE), pixels)
    mapping = {"mode": "AFFINE_RESIZED_OR_LETTERBOXED_RGB", "rgb_shape": [1200, 1920],
               "mask_shape": [640, 640], "scale_xy": [1/3, 1/3], "pad_xy_px": [0, 120]}
    np.testing.assert_allclose(mask_coordinates(pixels, mapping, (640, 640)),
                               (pixels + .5) / 3 - .5 + [0, 120])
    with pytest.raises(ValueError, match="UNDECLARED"):
        mask_coordinates(pixels, {"mode": "ORIGINAL_DISTORTED_RGB"}, (640, 640))


@pytest.mark.parametrize("failure,reason", [
    ("track", "CURRENT_RGB_TRACK_BINDING_MISMATCH"),
    ("mask", "CURRENT_RGB_MASK_PROVENANCE_MISMATCH"),
    ("multiple", "UNVERIFIED_OR_MULTIPLE_ELIGIBLE_RGB_TARGETS"),
    ("torso", "TRUNCATED_TORSO_WITHOUT_CURRENT_STABLE_CORE"),
])
def test_current_semantic_binding_refusals(failure, reason):
    mask = np.ones(RGB_SHAPE, bool)
    manifest = make_manifest(mask, torso=failure == "torso")
    if failure == "track":
        manifest["track_id"] = "other-person"
    elif failure == "mask":
        mask[0, 0] = False
    elif failure == "multiple":
        manifest["step71_semantic"]["target"]["current_eligible_count"] = 2
    else:
        manifest.pop("stable_torso_core_front_geometry")
    with pytest.raises(ValueError, match=reason):
        bind_semantic_target(manifest, mask)


def test_full_and_truncated_torso_semantic_support(policies):
    _, policy = policies
    _, _, grid, cal = shape_scene(2.)
    mask = np.ones(RGB_SHAPE, bool)
    full, report = semantic_stereo_component(grid, mask, bind_semantic_target(make_manifest(mask), mask), cal, policy)
    assert full.sum() == 8000 and report["torso_core_roi_px"] is None
    torso_binding = bind_semantic_target(make_manifest(mask, torso=True), mask)
    torso_binding["bbox_xyxy_px"] = [880., 540., 1040., 720.]
    torso, report = semantic_stereo_component(grid, mask, torso_binding, cal, policy)
    assert 100 < torso.sum() < full.sum()
    assert report["torso_core_roi_px"] == pytest.approx([920, 590.4, 1000, 630.0])


def test_unrelated_background_inside_same_semantic_mask_stays_ambiguous(policies):
    _, policy = policies
    _, x = np.indices((80, 100))
    _, _, grid, cal = shape_scene(np.where(x < 50, 1.3, 3.1))
    mask = np.ones(RGB_SHAPE, bool)
    component, report = semantic_stereo_component(grid, mask, bind_semantic_target(make_manifest(mask), mask), cal, policy)
    assert not component.any()
    assert report["status"] == "AMBIGUOUS_RGB_STEREO_DEPTH_LAYERS"


def test_rgb_projection_collision_does_not_assign_occluded_stereo_point(policies):
    _, policy = policies
    _, _, grid, cal = shape_scene(2.)
    xyz, support = grid
    xyz[10, 10] = np.array([0., 0., 4.])  # Same RGB ray as central 2 m sample.
    mask = np.ones(RGB_SHAPE, bool)
    component, report = semantic_stereo_component((xyz, support), mask,
                                                   bind_semantic_target(make_manifest(mask), mask), cal, policy)
    assert report["rgb_occluded_projection_samples"] == 1
    assert component[40, 50] and not component[10, 10]


def test_global_plane_refusal_and_large_residual_without_changing_support(policies):
    local, policy = policies
    y, x = np.indices((80, 100))
    z = 2.0 + .00035 * (x - 50)**2 + .00027 * (y - 40)**2
    disparity, lr, grid, cal = shape_scene(z)
    mask = np.ones(RGB_SHAPE, bool)
    raw = {"ranges_m": [2.0, 6.0, "+Infinity"], "range_min_m": .1, "range_max_m": 10.,
           "angle_min_rad": 0., "angle_increment_rad": 0.}
    result = associate_cycle(disparity, lr, grid, mask, bind_semantic_target(make_manifest(mask), mask),
                             raw, cal, ROT, ZERO, 0., local, policy)
    assert result["independent_global_plane"]["status"] == "GLOBAL_PLANE_REFUSED"
    close, far, missing = result["beams"]
    assert close["semantic_status"] == far["semantic_status"] == tool.CANDIDATE
    assert close["local_stereo"] == far["local_stereo"]
    assert far["ld19_minus_stereo_optical_z_m"] > 3.9
    assert missing["semantic_status"] == "INVALID_OR_MISSING_LD19_RANGE"


def test_no_depth_and_laser_projection_outside_semantic_surface(policies):
    local, policy = policies
    disparity, lr, grid, cal = shape_scene(2.)
    mask = np.zeros(RGB_SHAPE, bool)
    mask[550:650, 915:950] = True
    binding = bind_semantic_target(make_manifest(mask), mask)
    raw = {"ranges_m": [2.0], "range_min_m": .1, "range_max_m": 10.,
           "angle_min_rad": 0., "angle_increment_rad": 0.}
    result = associate_cycle(disparity, lr, grid, mask, binding, raw, cal, ROT, ZERO, 0., local, policy)
    assert result["beams"][0]["semantic_status"] == "LD19_PROJECTION_OUTSIDE_SELECTED_RGB_STEREO_COMPONENT"
    lr.fill(False)
    empty = stereo_grid(disparity, lr, cal["Q"], cal["R1"], np.ones_like(lr))
    _, report = semantic_stereo_component(empty, mask, binding, cal, policy)
    assert report["status"] == "NO_SUPPORTED_RGB_OWNED_STEREO_LAYER"


def test_temporal_independent_edges_pass_and_direct_third_edge_refuses():
    timing, frames, _ = make_timing(24, -39)
    result = temporal_eligibility(timing, frames)
    assert result["rgb_stereo"]["status"] == result["rgb_ld19"]["status"] == "MATCH_DIAGNOSTIC_ONLY"
    assert result["stereo_ld19"]["ld19_header_minus_stereo_ns"] == -63 * MS
    assert result["status"] == "STEREO_LD19_HEADER_DELTA_EXCEEDED"


def test_temporal_binding_and_staleness_refuse():
    timing, frames, _ = make_timing()
    assert temporal_eligibility(timing, frames)["status"] == "THREE_SOURCE_TIMING_DIAGNOSTIC_ONLY"
    frames["frames"]["ov9281"]["sequence"] += 1
    assert temporal_eligibility(timing, frames)["status"] == "CAMERA_PACKET_BINDING_MISMATCH"
    stale, frames, _ = make_timing(-260, -15)
    assert temporal_eligibility(stale, frames)["status"] == "INDEPENDENT_RGB_TEMPORAL_LINK_REFUSED"


def test_payload_identity_hash_and_admission(tmp_path):
    row, _, scan = make_timing()
    stamp = SimpleNamespace(sec=scan.header_ns // 10**9, nanosec=scan.header_ns % 10**9)
    message = SimpleNamespace(header=SimpleNamespace(stamp=stamp, frame_id="ld19_sensor"),
                              angle_min=0., angle_increment=.01, angle_max=.01,
                              time_increment=.001, scan_time=.1, range_min=.1, range_max=10.,
                              ranges=[2., np.inf], intensities=[])
    row["ld19_payload_archive"] = LiveTemporalShadowMonitor._archive_scan(
        row, ((scan.receipt_sequence, scan.header_ns + 1, message),),
    )
    raw, _ = tool.scan_payload(row, row["zero_wait"]["rgb_ld19_ros_header"]["candidate"])
    assert raw["ranges_m"] == [2., "+Infinity"]
    row["ld19_payload_archive"]["record"]["payload"]["scan"]["ranges_m"][0] = 1.
    with pytest.raises(ValueError, match="LD19_PAYLOAD_HEADER_OR_SHA256_MISMATCH"):
        tool.scan_payload(row, row["zero_wait"]["rgb_ld19_ros_header"]["candidate"])
    candidate = tmp_path / "changed.npz"
    candidate.write_bytes(b"changed")
    with pytest.raises(ValueError, match="ARTIFACT_SHA256_MISMATCH"):
        tool.verify_sha(candidate, "0" * 64)


def test_bounded_source_writer_through_unchanged_matcher_to_replay(tmp_path, monkeypatch, policies):
    fixtures = tmp_path / "fixtures"
    archive_synthetic_pair(fixtures, monkeypatch)
    stereo_jpeg = (fixtures / "frame.mjpg").read_bytes()
    cal = joint.load_calibration((fixtures / "v7.npz").read_bytes())
    cal.update({"K": np.array([[640., 0, 960.], [0, 640., 600.], [0, 0, 1.]]),
                "R": np.eye(3), "T": ZERO, "D": np.zeros(5), "size": np.array(joint.SIZE)})
    mode, policy = policies
    rgb = np.full((*RGB_SHAPE, 3), 50, np.uint8)
    ok, encoded = cv2.imencode(".jpg", rgb)
    assert ok
    mask = np.zeros(RGB_SHAPE, bool)
    mask[480:760, 760:1160] = True
    manifest = make_manifest(mask)
    manifest["segmentation_rgb_payload_sha256"] = joint.digest(rgb.tobytes())
    temporal, frames, scan = make_timing()
    manifest["frame_freshness"] = frames
    stamp = SimpleNamespace(sec=scan.header_ns // 10**9, nanosec=scan.header_ns % 10**9)
    message = SimpleNamespace(header=SimpleNamespace(stamp=stamp, frame_id="ld19_sensor"),
        angle_min=0., angle_max=.005, angle_increment=.005, time_increment=.001,
        scan_time=.1, range_min=.1, range_max=10., ranges=[2.0, 4.0], intensities=[])
    temporal["ld19_payload_archive"] = LiveTemporalShadowMonitor._archive_scan(
        temporal, ((scan.receipt_sequence, scan.header_ns, message),),
    )
    run = tmp_path / "run"
    journal = run / "control-preview-test/temporal-association-shadow.jsonl"
    journal.parent.mkdir(parents=True)
    journal.write_text(joint.encoded(temporal) + "\n")
    writer = LatestSourceSnapshotWriter(run / "shadow-source-snapshots")
    try:
        assert writer.submit(CYCLE, encoded.tobytes(), stereo_jpeg, np.zeros((400, 640)),
                             np.zeros((400, 640), bool), cal, pack_mask(mask), manifest)
    finally:
        writer.close()
    assert writer.completed == 1
    summary = json.loads(tool.replay(run, tmp_path / "report", (cal, ROT, ZERO, 0., mode, policy), MODEL).read_text())
    assert summary["coherent_semantic_candidates"] == 2
    assert summary["optical_z_residual"]["p95_absolute_mm"] > 1700
    assert not any(summary[key] for key in ("measurement_authorized", "world_state_update_authorized",
                                             "decision_authorized", "motor_control_authorized"))
    cycle, = joint.read_rows(tmp_path / "report/cycles.jsonl")
    assert cycle["beams"][0]["stereo_anchor_rgb_mask_membership"] is True
    assert cycle["beams"][0]["rgb_owned_stereo_component_at_projection"] is True
    try:
        from jsonschema import Draft202012Validator
    except ModuleNotFoundError:
        pass  # The schema is an interchange reference, not a runtime dependency.
    else:
        schema = json.loads((joint.ROOT / "docs/diagnostics/three_sensor_semantic_spatial_v1.schema.json").read_text())
        Draft202012Validator(schema).validate(cycle)
    archive = run / "shadow-source-snapshots" / f"{CYCLE}.npz"
    archive.write_bytes(b"damaged")
    with pytest.raises(ValueError, match="evidence digest mismatch"):
        tool.read_snapshot(archive.parent, CYCLE, MODEL)


def test_temporal_writer_archives_matched_callback_only_when_opted_in(tmp_path, monkeypatch):
    from sie_ros2 import live_temporal_shadow_monitor as runtime
    temporal, _, scan = make_timing()
    candidate = temporal["zero_wait"]["rgb_ld19_ros_header"]["candidate"]
    candidate["receipt_sequence"] = 1
    stamp = SimpleNamespace(sec=scan.header_ns // 10**9, nanosec=scan.header_ns % 10**9)
    message = SimpleNamespace(header=SimpleNamespace(stamp=stamp, frame_id="ld19_sensor"),
        angle_min=0., angle_max=.005, angle_increment=.005, time_increment=.001,
        scan_time=.1, range_min=.1, range_max=10., ranges=[2.0, 4.0], intensities=[])
    monkeypatch.setattr(runtime.LiveTemporalShadowMonitor, "_spin_subscriber", lambda self: None)
    monkeypatch.setattr(runtime, "evaluate_cycle", lambda *args, **kw: dict(temporal))
    monkeypatch.setattr(runtime, "time", SimpleNamespace(
        monotonic_ns=lambda: scan.received_monotonic_ns, time_ns=lambda: scan.header_ns + 1))
    cameras = {name: SimpleNamespace(timing_history=lambda: ((), None)) for name in ("ar0234", "ov9281")}
    node = SimpleNamespace(count_publishers=lambda topic: 1)
    for enabled in (False, True):
        path = tmp_path / f"journal-{enabled}.jsonl"
        monitor = runtime.LiveTemporalShadowMonitor(path, "run", cameras, None, "test", archive_scans=enabled)
        monitor._on_scan(message, node)
        monitor.submit(CYCLE, "PREVIEW", {}, {}, T, None)
        assert monitor.close()
        row, summary = joint.read_rows(path)
        assert summary["journal"]["write_errors"] == 0
        if enabled:
            raw, _ = tool.scan_payload(row, candidate)
            assert raw["ranges_m"] == [2.0, 4.0]
        else:
            assert "ld19_payload_archive" not in row
