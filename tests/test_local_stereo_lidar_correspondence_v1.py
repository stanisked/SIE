import io
import json
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from test_stereo_lidar_joint_static_v1 import archive_synthetic_pair, matrices
from sie_core.geometry.camera_lidar_optical_datum_v1 import estimated_ld19_transform
from vision_core.stereo.local_stereo_lidar_correspondence_v1 import (
    AMBIGUOUS, CANDIDATE, compare_local_scan, local_stereo_support, stereo_grid,
)
from vision_core.stereo.stereo_lidar_static_v1 import fit_stereo_surface, target_mask
from vision_core.tools import replay_stereo_lidar_local_v1 as tool
from vision_core.tools import stereo_lidar_joint_static_v1 as joint

SHAPE = (80, 100)
ROTATION = np.array([[0., -1., 0.], [0., 0., -1.], [1., 0., 0.]])
AXIS = ROTATION[:, 0]
EXAMPLE_TARGET = [[10, 10], [90, 10], [90, 70], [10, 70]]


@pytest.fixture
def policy():
    return tool.frozen_policy()[1]


def scan(values, start=0., step=.01):
    return {"ranges_m": values, "range_min_m": .1, "range_max_m": 10.,
            "angle_min_rad": start, "angle_increment_rad": step}


def scene(z_m, *, lr=None, region=None, r1=None):
    p, q = matrices()
    if lr is None:
        lr = np.ones(SHAPE, bool)
    if region is None:
        region = target_mask(SHAPE, EXAMPLE_TARGET)
    if r1 is None:
        r1 = np.eye(3)
    disparity = 10 / np.broadcast_to(z_m, SHAPE)
    return stereo_grid(disparity, lr, q, r1, region), region, r1, p, q, disparity, lr


def compare(policy, values, z=2., **kwargs):
    grid, region, r1, p, _, _, _ = scene(z, **kwargs)
    return compare_local_scan(scan(values, step=0), grid, region, r1, p,
                              ROTATION, np.zeros(3), 0, (-5, 5), policy)


def test_projection_and_r1_transpose_with_real_mechanical_assumptions(policy):
    profiles = [json.loads((joint.CONFIG / name).read_text()) for name in (
        "camera_lidar_mechanical_geometry_v1.json", "camera_lidar_optical_datum_v1.json")]
    rotation, translation, axial = estimated_ld19_transform(*profiles)
    r1, _ = cv2.Rodrigues(np.array([.12, .05, 0.]))
    grid, region, _, p, _, _, _ = scene(2., r1=r1)
    rows, _ = compare_local_scan(scan([2.1]), grid, region, r1, p, rotation,
                                 translation, axial, (-10, 10), policy)
    row, = rows
    expected_optical = rotation @ [2.1, 0, 0] + translation
    expected_rectified = r1 @ expected_optical
    np.testing.assert_allclose(row["hit_estimated_optical_m"], expected_optical)
    np.testing.assert_allclose(row["hit_rectified_left_m"], expected_rectified)
    np.testing.assert_allclose(row["projected_rectified_left_px"],
                               expected_rectified[:2] / expected_rectified[2] * 100 + [50, 40])
    assert row["status"] == CANDIDATE
    assert row["hit_longitudinal_from_rim_m"] == pytest.approx(2.1 + .158)
    # Every populated stereo point lies on the 2 m rectified plane.
    points = grid[0][np.isfinite(grid[0]).all(axis=2)]
    np.testing.assert_allclose((points @ r1.T)[:, 2], 2, atol=1e-12)


def test_planar_candidate_and_large_cross_sensor_residual_untrimmed(policy):
    rows, counts = compare(policy, [2.0, 6.0])
    near, far = rows
    assert near["status"] == far["status"] == CANDIDATE
    assert near["local_stereo"] == far["local_stereo"]  # Same pixel, different LD19 depth.
    assert near["local_stereo"]["selected_count"] == 225
    assert near["local_stereo"]["selected_coverage"] == 1
    assert near["local_stereo"]["optical_z"]["median_m"] == pytest.approx(2.0)
    assert near["local_stereo"]["optional_local_plane"]["stereo_point_to_plane"]["p95_absolute_mm"] < 1e-8
    assert far["ld19_minus_stereo_optical_z_m"] == pytest.approx(4)
    assert far["ld19_minus_stereo_longitudinal_m"] == pytest.approx(4)
    assert far["physical_target_membership"] == "UNVERIFIED"
    assert counts["coherent_local_surface_candidates"] == 2


def test_nonplanar_scene_can_be_local_candidate_after_global_refusal(policy):
    y, x = np.indices(SHAPE)
    z = 2.0 + .00035 * (x - 50) ** 2 + .00027 * (y - 40) ** 2
    grid, region, r1, p, q, disparity, lr = scene(z)
    with pytest.raises(ValueError, match="STEREO_TARGET_NOT_PLANAR_UNDER_TRIAL_LIMIT"):
        fit_stereo_surface(disparity, lr, q, r1, region)
    rows, _ = compare_local_scan(scan([2.1]), grid, region, r1, p, ROTATION,
                                 np.zeros(3), 0, (-5, 5), policy)
    row, = rows
    assert row["status"] == CANDIDATE
    assert row["local_stereo"]["coherent"] is True
    assert row["local_stereo"]["optional_local_plane"]["stereo_point_to_plane"]["p95_absolute_mm"] > 1


@pytest.mark.parametrize("supply,expected", [
    ("none", "NO_LOCAL_LR_SUPPORT"),
    ("invalid", "NO_VALID_STEREO_XYZ"),
    ("sparse", "INSUFFICIENT_LOCAL_COMPONENT_SUPPORT"),
    ("chain", "INCOHERENT_LOCAL_STEREO_DEPTH"),
])
def test_support_and_coherence_refusals(policy, supply, expected):
    lr = np.ones(SHAPE, bool)
    z = 2.
    if supply == "none":
        lr[:] = False
    elif supply == "invalid":
        z = np.nan
    elif supply == "sparse":
        lr[:] = False
        lr[39:42, 49:52] = True
    else:
        z = 2.0 + .025 * (np.indices(SHAPE)[1] - 50)
    rows, counts = compare(policy, [2.], z=z, lr=lr)
    row, = rows
    assert row["status"] == expected
    assert "ld19_minus_stereo_optical_z_m" not in row
    assert counts["coherent_local_surface_candidates"] == 0


def test_foreground_background_edge_stays_ambiguous_at_any_laser_depth(policy):
    _, x = np.indices(SHAPE)
    rows, counts = compare(policy, [1.5, 3.5, 6.0], z=np.where(x < 50, 1.5, 3.5))
    for row in rows:
        assert row["status"] == AMBIGUOUS
        assert row["local_stereo"]["depth_discontinuity_edges"] > 0
        assert row["local_stereo"]["plausible_component_indices"] == [0, 1]
        assert "ld19_minus_stereo_optical_z_m" not in row
    assert rows[0]["local_stereo"] == rows[1]["local_stereo"] == rows[2]["local_stereo"]
    assert counts["ambiguous_beams"] == 3


def test_disconnected_stereo_surfaces_are_never_joined_across_lr_holes(policy):
    lr = np.ones(SHAPE, bool)
    lr[:, 49:52] = False
    rows, _ = compare(policy, [2.0], lr=lr)
    assert rows[0]["status"] == AMBIGUOUS
    assert rows[0]["local_stereo"]["depth_discontinuity_edges"] == 0


def test_foreground_edge_away_from_anchor_is_retained_as_local_diagnostic(policy):
    _, x = np.indices(SHAPE)
    rows, _ = compare(policy, [2.0], z=np.where(x < 55, 2., 3.))
    assert rows[0]["status"] == CANDIDATE
    report = rows[0]["local_stereo"]
    assert report["depth_discontinuity_edges"] > 0
    assert len(report["components"]) == 2
    assert all(x < 55 for x, _ in report["selected_pixels_xy_px"])


@pytest.mark.parametrize("distance,theta,sector,expected", [
    ("+Infinity", 0, (-90, 90), "INVALID_OR_MISSING_LD19_RANGE"),
    (0, 0, (-90, 90), "INVALID_OR_MISSING_LD19_RANGE"),
    (11, 0, (-90, 90), "INVALID_OR_MISSING_LD19_RANGE"),
    (2, np.pi, (-180, 180), "NONPOSITIVE_OPTICAL_DEPTH"),
    (2, .8, (-90, 90), "PROJECTED_OUTSIDE_IMAGE"),
    (2, .43, (-90, 90), "PROJECTED_OUTSIDE_TARGET_POLYGON"),
    (2, .8, (-5, 5), "OUTSIDE_DECLARED_SECTOR"),
])
def test_range_projection_and_region_failures(policy, distance, theta, sector, expected):
    grid, region, r1, p, _, _, _ = scene(2.)
    rows, _ = compare_local_scan(scan([distance], start=theta), grid, region, r1, p,
                                 ROTATION, np.zeros(3), 0, sector, policy)
    assert rows[0]["status"] == expected
    assert rows[0]["inside_target_polygon"] is False


def test_negative_reconstructed_depth_does_not_become_stereo_support(policy):
    _, region, r1, _, q, disparity, lr = scene(2.)
    q[3, 2] *= -1
    xyz, support = stereo_grid(disparity, lr, q, r1, region)
    local = local_stereo_support([50, 40], xyz, support, region, AXIS, 0, policy)
    assert local["status"] == "NO_VALID_STEREO_XYZ"
    assert local["lr_pixels_without_valid_xyz"] == 225


@pytest.fixture
def archive(tmp_path, monkeypatch):
    capture = tmp_path / "capture"
    archive_synthetic_pair(capture, monkeypatch)
    # The Step 66 test fixture serializes parsed profiles. Rebind this one to
    # the byte-identical production copies as real Step 66 capture does.
    session = json.loads((capture / "session.json").read_text())
    for key, file in (("mechanical", "camera_lidar_mechanical_geometry_v1.json"),
                      ("optical_datum", "camera_lidar_optical_datum_v1.json")):
        item = session["inputs"][key]
        data = (joint.CONFIG / file).read_bytes()
        (capture / item["path"]).write_bytes(data)
        item["sha256"] = joint.digest(data)
    (capture / "session.json").write_text(joint.encoded(session))
    output = tmp_path / "step66"
    joint.analyze(SimpleNamespace(capture=capture, output=output,
                                 polygon=[500, 510, 750, 510, 750, 690, 500, 690],
                                 sector_deg=(-3, 3), target_label="synthetic panel"))
    return capture, output


def test_offline_replay_binds_original_arrays_and_global_behavior(archive, tmp_path):
    capture, original = archive
    initial = joint.sha256_file(original / "comparisons.jsonl")
    summary = json.loads(tool.replay(capture, original, tmp_path / "local").read_text())
    local, = joint.read_rows(tmp_path / "local/comparisons.jsonl")
    assert summary["temporal_matches"] == 1 and summary["temporal_refusals"] == 0
    assert summary["local_beams"]["coherent_local_surface_candidates"] == 2
    assert summary["distinct_compared_ld19_scans"] == 1
    assert summary["stereo_array_source_counts"] == {"VERIFIED_STEP66_ARRAYS": 1}
    assert local["global_disposition_reproduced"] is True
    assert summary["global_plane_p95_limit_m"] == .04
    assert not summary["motor_control_authorized"] and not summary["world_state_update_authorized"]
    assert initial == joint.sha256_file(original / "comparisons.jsonl")


def _update_comparison(original, operation):
    journal = original / "comparisons.jsonl"
    row, = joint.read_rows(journal)
    operation(row)
    journal.write_text(joint.encoded(row) + "\n")


def test_missing_archive_arrays_reconstruct_unchanged_matcher(archive, tmp_path):
    capture, original = archive
    _update_comparison(original, lambda row: row.pop("stereo_arrays"))
    summary = json.loads(tool.replay(capture, original, tmp_path / "local").read_text())
    assert summary["stereo_array_source_counts"] == {"RECONSTRUCTED_FROM_VERIFIED_MJPEG": 1}
    assert summary["local_beams"]["coherent_local_surface_candidates"] == 2


@pytest.mark.parametrize("what,reason", [
    ("calibration", "active OV9281 v7 calibration SHA-256 mismatch"),
    ("profile", "profile SHA-256 mismatch"),
    ("archive", "evidence digest mismatch"),
    ("binding", "archived stereo array provenance mismatch"),
    ("journal", "Step 66 analysis provenance mismatch"),
])
def test_mismatched_evidence_refuses(archive, monkeypatch, tmp_path, what, reason):
    capture, original = archive
    if what == "calibration":
        monkeypatch.setattr(joint, "ACTIVE_V7_SHA256", "0" * 64)
        path = original / "analysis-policy.json"
        old = json.loads(path.read_text())
        old["calibration_sha256"] = joint.ACTIVE_V7_SHA256
        path.write_text(joint.encoded(old))
    elif what == "profile":
        monkeypatch.setattr(tool, "PROFILE_DIGESTS", tool.PROFILE_DIGESTS | {"mechanical": "0" * 64})
    elif what == "archive":
        row, = joint.read_rows(original / "comparisons.jsonl")
        (original / row["stereo_arrays"]["path"]).write_bytes(b"changed")
    elif what == "binding":
        _update_comparison(original, lambda row: row.update(scan_header_ns=10))
    else:
        with (capture / "frames.jsonl").open("a") as stream:
            stream.write("\n")
    with pytest.raises(ValueError, match=reason):
        tool.replay(capture, original, tmp_path / "local")


def test_replay_keeps_global_refusal_and_local_candidates_in_same_frame(archive, tmp_path):
    capture, original = archive

    def curved_evidence(row):
        h, w = joint.SIZE[::-1]
        y, x = np.indices((h, w))
        z = 2.0 + .000055 * (x - 625) ** 2 + .00004 * (y - 600) ** 2
        disparity = (64 / z).astype(np.float32)
        arrays = io.BytesIO()
        np.savez_compressed(arrays, disparity_px=disparity, lr_consistent=np.ones((h, w), bool))
        row["stereo_arrays"] = joint.save_bytes(original, "curved-test-only.npz", arrays.getvalue())

    _update_comparison(original, curved_evidence)
    summary = json.loads(tool.replay(capture, original, tmp_path / "local").read_text())
    assert summary["global_plane_status_counts"] == {"STEREO_TARGET_NOT_PLANAR_UNDER_TRIAL_LIMIT": 1}
    assert summary["local_beams"]["coherent_local_surface_candidates"] == 2
    assert summary["frames_with_local_candidates_and_global_plane_refusal"] == 1


def test_zero_candidates_is_successful_diagnostic_with_separate_refusals(archive, tmp_path):
    capture, original = archive
    frames = capture / "frames.jsonl"
    frame, = joint.read_rows(frames)
    frame["clock"] = {"clock_valid": False, "status": "BRIDGE_EXPIRED"}
    frames.write_text(joint.encoded(frame) + "\n")
    policy = original / "analysis-policy.json"
    updated = json.loads(policy.read_text())
    updated["frame_journal_sha256"] = joint.sha256_file(frames)
    policy.write_text(joint.encoded(updated))
    summary = json.loads(tool.replay(capture, original, tmp_path / "local").read_text())
    assert summary["result"] == "NO_LOCAL_CANDIDATES"
    assert summary["temporal_refusals"] == 1 and summary["temporal_status_counts"] == {"BRIDGE_EXPIRED": 1}
    assert summary["local_beams"]["coherent_local_surface_candidates"] == 0
    assert summary["spatial_frame_status_counts"] == {}
    assert summary["ld19_minus_stereo_optical_z"]["median_mm"] is None


def test_matched_frame_with_no_stereo_support_remains_a_diagnostic(archive, tmp_path):
    capture, original = archive

    def no_support(row):
        shape = joint.SIZE[::-1]
        arrays = io.BytesIO()
        np.savez_compressed(arrays, disparity_px=np.full(shape, 32, np.float32),
                            lr_consistent=np.zeros(shape, bool))
        row["stereo_arrays"] = joint.save_bytes(original, "unsupported-test-only.npz", arrays.getvalue())

    _update_comparison(original, no_support)
    summary = json.loads(tool.replay(capture, original, tmp_path / "local").read_text())
    assert summary["result"] == "NO_LOCAL_CANDIDATES"
    assert summary["temporal_matches"] == 1 and summary["temporal_refusals"] == 0
    assert summary["spatial_frame_status_counts"] == {"NO_LOCAL_CANDIDATES": 1}
    assert summary["beam_status_counts"]["NO_LOCAL_LR_SUPPORT"] == 2
    assert summary["distinct_temporally_paired_ld19_scans"] == 1
    assert summary["distinct_compared_ld19_scans"] == 0


def test_policy_bytes_are_frozen(monkeypatch):
    monkeypatch.setattr(tool, "POLICY_SHA256", "0" * 64)
    with pytest.raises(ValueError, match="frozen Step 69 local policy SHA-256 mismatch"):
        tool.frozen_policy()
