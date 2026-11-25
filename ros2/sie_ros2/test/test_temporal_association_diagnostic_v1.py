"""Synthetic regression tests for RGB-anchor matching; no live sensor access."""
from copy import deepcopy
import json

import pytest

from sie_ros2.temporal_association_diagnostic_v1 import (
    SCHEMA, TemporalAssociationTrialPolicy, associate_warmed_metadata,
)

BASE = 10_000_000_000
OFFSET = 1_795_619_749_801_904_074
SHA = "a" * 64


@pytest.fixture
def policy():
    # Explicit TEST ONLY limits, not approved runtime admission.
    return TemporalAssociationTrialPolicy(
        policy_id="synthetic-25ms-stereo-50ms-ros-header-EXPERIMENT",
        max_rgb_stereo_delta_ns=25_000_000,
        max_rgb_ld19_header_delta_ns=50_000_000,
    )


def frame(milliseconds, sequence):
    stamp = BASE + round(milliseconds * 1_000_000)
    return {
        "sequence": sequence,
        "driver_monotonic_ns": stamp,
        "received_monotonic_ns": stamp + 20_000_000,
    }


def scan(milliseconds, index):
    stamp = BASE + round(milliseconds * 1_000_000)
    return {
        "header_ns": stamp + OFFSET,
        "receive_monotonic_ns": stamp + 500_000,
        "frame_id": "ld19_sensor",
    }


def evidence(*, rgb=None, stereo=None, lidar=None):
    if rgb is None:
        rgb = [frame(100, 1)]
    if stereo is None:
        stereo = [frame(116.185, 2)]
    if lidar is None:
        lidar = [scan(60.236, 1), scan(250, 2)]
    return {
        "schema_version": "sie.three_sensor_warmed.v1.diagnostic",
        "boot_id": "pi-boot-1",
        "bridge": {
            "boot_id": "pi-boot-1",
            "status": "DIAGNOSTIC_BRIDGE_READY",
            "monotonic_clock": "CLOCK_MONOTONIC",
            "realtime_clock": "CLOCK_REALTIME",
            "first_sample_monotonic_ns": BASE,
            "last_sample_monotonic_ns": BASE + 300_000_000,
            "offset_ns": OFFSET,
        },
        "camera_frames": {"ar0234": rgb, "ov9281": stereo},
        "ld19": lidar,
    }


def match(policy, report=None):
    return associate_warmed_metadata(
        evidence() if report is None else report, source_sha256=SHA, policy=policy
    )


def test_one_false_triple_rejection_does_not_block_both_independent_links(policy):
    result = match(policy)
    assert result["schema_version"] == SCHEMA
    assert result["summary"]["rgb_anchors"] == 1
    assert result["summary"]["both_diagnostic_links"] == 1
    row = result["associations"][0]
    assert row["status"] == "RGB_STEREO_MATCH_DIAGNOSTIC_ONLY"
    assert row["lidar_status"] == "LD19_ROS_HEADER_NEAR_RGB_DIAGNOSTIC_ONLY"
    assert row["rgb_stereo_delta_ns"] == 16_185_000
    assert row["rgb_ld19_header_delta_ns"] == -39_764_000
    assert row["stereo_ld19_header_delta_ns"] == -55_949_000
    assert row["confidence"] is None
    assert row["stereo"]["reference_frame"] == "UNVERIFIED_IN_CAPTURE"
    assert row["ld19"]["reference_frame"] == "ld19_sensor"
    assert row["ld19"]["timestamp_basis"].endswith("NOT_LASER_ACQUISITION")
    assert not result["measurement_authorized"]
    assert not result["world_state_update_authorized"]
    assert not result["motor_control_authorized"]
    assert json.loads(json.dumps(result)) == result


def test_stereo_refusal_preserves_separate_lidar_link(policy):
    report = evidence(stereo=[frame(160, 2)])
    row = match(policy, report)["associations"][0]
    assert row["status"] == "RGB_STEREO_DELTA_EXCEEDED"
    assert row["stereo"] is None
    assert row["lidar_status"] == "LD19_ROS_HEADER_NEAR_RGB_DIAGNOSTIC_ONLY"
    assert row["ld19"] is not None


def test_ld19_header_refusal_preserves_stereo_link(policy):
    report = evidence(lidar=[scan(1, 1), scan(250, 2)])
    row = match(policy, report)["associations"][0]
    assert row["status"] == "RGB_STEREO_MATCH_DIAGNOSTIC_ONLY"
    assert row["lidar_status"] == "LD19_ROS_HEADER_DELTA_EXCEEDED"
    assert row["ld19"] is None


def test_camera_pair_still_works_without_lidar(policy):
    report = evidence(lidar=[])
    result = match(policy, report)
    assert result["summary"]["rgb_anchors"] == 1
    row = result["associations"][0]
    assert row["stereo"] is not None
    assert row["lidar_status"] == "LD19_UNAVAILABLE"
    assert row["ld19"] is None


def test_reused_lidar_header_does_not_create_independent_measurements(policy):
    report = evidence(
        rgb=[frame(180, 10), frame(190, 12)],
        stereo=[frame(180.2, 11), frame(190.2, 13)],
        lidar=[scan(80, 1), scan(150, 2), scan(260, 3)],
    )
    result = match(policy, report)
    assert result["summary"]["rgb_anchors"] == 2
    assert result["summary"]["rgb_stereo_matches"] == 2
    assert result["summary"]["rgb_stereo_unique_stereo"] == 2
    assert result["summary"]["rgb_ld19_header_links"] == 2
    assert result["summary"]["rgb_ld19_unique_headers"] == 1
    assert not result["measurement_authorized"]


@pytest.mark.parametrize("mutation", [
    lambda x: x.update(schema_version="wrong"),
    lambda x: x["bridge"].update(boot_id="different-boot"),
    lambda x: x["bridge"].update(status="BRIDGE_EXPIRED"),
    lambda x: x["bridge"].update(monotonic_clock="CLOCK_BOOTTIME"),
    lambda x: x["bridge"].update(first_sample_monotonic_ns=BASE + 400_000_000),
    lambda x: x["camera_frames"]["ar0234"][0].update(
        received_monotonic_ns=BASE + 80_000_000),
    lambda x: x["ld19"][0].update(frame_id="base_link"),
])
def test_untrusted_metadata_refused(policy, mutation):
    report = deepcopy(evidence())
    mutation(report)
    with pytest.raises(ValueError):
        match(policy, report)


def test_provenance_is_bound_to_private_source_sha(policy):
    row = match(policy)["associations"][0]
    assert row["rgb"]["evidence_ids"] == [
        f"private-capture-sha256:{SHA}#camera_frames.ar0234[0]"
    ]
    assert row["stereo"]["evidence_ids"] == [
        f"private-capture-sha256:{SHA}#camera_frames.ov9281[0]"
    ]
    assert row["ld19"]["evidence_ids"] == [
        f"private-capture-sha256:{SHA}#ld19[0]"
    ]


def test_trial_policy_rejects_implicit_or_invalid_values():
    with pytest.raises(ValueError):
        TemporalAssociationTrialPolicy("", 25_000_000, 50_000_000)
    with pytest.raises(ValueError):
        TemporalAssociationTrialPolicy("test", -1, 50_000_000)
    with pytest.raises(ValueError):
        TemporalAssociationTrialPolicy("test", True, 50_000_000)


def test_bad_evidence_sha_is_not_accepted(policy):
    with pytest.raises(ValueError):
        associate_warmed_metadata(evidence(), source_sha256="not-a-sha", policy=policy)
