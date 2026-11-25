"""Pure synthetic freshness and refusal tests, no ROS, sensor or clock calls."""
from copy import deepcopy
import json

import pytest

from sie_ros2.temporal_association_diagnostic_v1 import (
    SCHEMA as ASSOCIATION_SCHEMA, RGB_STAMP_BASIS, LD19_STAMP_BASIS,
)
from sie_ros2.temporal_association_freshness_v1 import (
    SCHEMA, FreshnessTrialPolicy, evaluate_association_freshness,
)

START = 10_000_000_000
LAST = START + 300_000_000
OFFSET = 1_795_619_749_801_904_074
SHA = "a" * 64


@pytest.fixture
def policy():
    return FreshnessTrialPolicy(
        policy_id="synthetic-100ms-source-age-500ms-bridge-TRIAL",
        max_rgb_age_ns=100_000_000,
        max_stereo_age_ns=100_000_000,
        max_ld19_ros_header_age_ns=100_000_000,
        max_bridge_holdover_ns=500_000_000,
        max_future_skew_ns=1_000_000,
    )


def reference(source_id, ms_from_last, path, timestamp_basis, reference_frame):
    return {
        "source_id": source_id,
        "sequence": 10 if source_id != "ld19_sensor" else None,
        "timestamp_ns": LAST + OFFSET + round(ms_from_last * 1e6),
        "timestamp_basis": timestamp_basis,
        "reference_frame": reference_frame,
        "evidence_ids": [f"private-capture-sha256:{SHA}#{path}[0]"],
    }


def association():
    return {
        "schema_version": ASSOCIATION_SCHEMA,
        "status": "DIAGNOSTIC_ONLY_NOT_ADMITTED",
        "boot_id": "boot-a",
        "timebase": "HOST_REALTIME_ESTIMATE_DIAGNOSTIC_ONLY",
        "ros_clock_domain_independently_verified": False,
        "source_evidence_sha256": SHA,
        "measurement_authorized": False,
        "world_state_update_authorized": False,
        "motor_control_authorized": False,
        "associations": [{
            "status": "RGB_STEREO_MATCH_DIAGNOSTIC_ONLY",
            "lidar_status": "LD19_ROS_HEADER_NEAR_RGB_DIAGNOSTIC_ONLY",
            "rgb": reference("ar0234", -5, "camera_frames.ar0234", RGB_STAMP_BASIS, "UNVERIFIED_IN_CAPTURE"),
            "stereo": reference("ov9281", -4, "camera_frames.ov9281", RGB_STAMP_BASIS, "UNVERIFIED_IN_CAPTURE"),
            "ld19": reference("ld19_sensor", -6, "ld19", LD19_STAMP_BASIS, "ld19_sensor"),
            "physical_laser_acquisition_time": "UNKNOWN",
            "physical_camera_exposure_time_independently_verified": False,
            "confidence": None,
            "measurement_authorized": False,
            "world_state_update_authorized": False,
            "motor_control_authorized": False,
        }],
    }


def bridge():
    return {
        "boot_id": "boot-a",
        "status": "DIAGNOSTIC_BRIDGE_READY",
        "monotonic_clock": "CLOCK_MONOTONIC",
        "realtime_clock": "CLOCK_REALTIME",
        "first_sample_monotonic_ns": START,
        "last_sample_monotonic_ns": LAST,
        "offset_ns": OFFSET,
        "policy_limits": {"max_age_ns": 1_000_000_000},
    }


def evaluate(policy, source=None, time_ns=LAST+10_000_000, prior=None, clock_bridge=None):
    return evaluate_association_freshness(
        association() if source is None else source,
        bridge() if clock_bridge is None else clock_bridge,
        evaluation_monotonic_ns=time_ns,
        previous_evaluation_monotonic_ns=prior,
        boot_id="boot-a",
        policy=policy,
    )


def test_recent_host_metadata_all_three_independent_links(policy):
    out = evaluate(policy)
    assert out["schema_version"] == SCHEMA
    assert out["status"] == "EVALUATED_DIAGNOSTIC_ONLY"
    assert out["summary"] == {
        "evaluated": 1, "rgb_stereo_recent": 1,
        "ld19_header_recent": 1, "both_independent_links_recent": 1,
    }
    row = out["rows"][0]
    assert row["age_ns"] == {
        "rgb": 15_000_000,
        "stereo": 14_000_000,
        "ld19_ros_header": 16_000_000,
    }
    assert row["rgb_stereo_freshness_status"] == "RGB_STEREO_RECENT_DIAGNOSTIC_ONLY"
    assert row["rgb_ld19_header_freshness_status"] == "LD19_ROS_HEADER_RECENT_DIAGNOSTIC_ONLY"
    assert row["ld19_laser_acquisition_recency"] == "UNKNOWN"
    assert out["ros_clock_domain_independently_verified"] is False
    assert all(row[name] is False for name in (
        "measurement_authorized", "world_state_update_authorized",
        "motor_control_authorized",
    ))
    assert not out["motor_control_authorized"]
    assert json.loads(json.dumps(out, allow_nan=False)) == out


def test_stale_camera_timestamps_are_refused_independently(policy):
    out = evaluate(policy, time_ns=LAST+200_000_000)
    row = out["rows"][0]
    assert row["age_ns"]["rgb"] == 205_000_000
    assert row["source_status"]["rgb"] == "SOURCE_STALE"
    assert row["rgb_stereo_freshness_status"] == "RGB_SOURCE_STALE"
    assert row["rgb_ld19_header_freshness_status"] == "RGB_SOURCE_STALE"
    assert out["summary"]["both_independent_links_recent"] == 0


def test_stale_stereo_does_not_block_lidar_header_link(policy):
    src = association()
    src["associations"][0]["stereo"]["timestamp_ns"] -= 150_000_000
    out = evaluate(policy, src)
    row = out["rows"][0]
    assert row["rgb_stereo_freshness_status"] == "STEREO_SOURCE_STALE"
    assert row["rgb_ld19_header_freshness_status"] == "LD19_ROS_HEADER_RECENT_DIAGNOSTIC_ONLY"


def test_stale_lidar_header_does_not_block_camera_pair(policy):
    src = association()
    src["associations"][0]["ld19"]["timestamp_ns"] -= 150_000_000
    out = evaluate(policy, src)
    row = out["rows"][0]
    assert row["rgb_stereo_recent"]
    assert row["rgb_ld19_header_freshness_status"] == "LD19_SOURCE_STALE"


def test_future_source_stamp_refused(policy):
    src = association()
    src["associations"][0]["rgb"]["timestamp_ns"] += 30_000_000
    row = evaluate(policy, src)["rows"][0]
    assert row["age_ns"]["rgb"] == -15_000_000
    assert row["source_status"]["rgb"] == "STAMP_FROM_FUTURE"
    assert row["rgb_stereo_recent"] is False


def test_missing_ld19_does_not_block_rgb_stereo(policy):
    src = association()
    src["associations"][0].update(
        ld19=None, lidar_status="LD19_UNAVAILABLE",
    )
    row = evaluate(policy, src)["rows"][0]
    assert row["rgb_stereo_recent"]
    assert row["source_status"]["ld19_ros_header"] == "SOURCE_UNAVAILABLE"
    assert not row["rgb_ld19_header_recent"]
    assert row["age_ns"]["ld19_ros_header"] is None


def test_missing_stereo_preserves_lidar_header_trial(policy):
    src = association()
    src["associations"][0].update(stereo=None, status="STEREO_UNAVAILABLE")
    row = evaluate(policy, src)["rows"][0]
    assert row["rgb_stereo_freshness_status"] == "RGB_STEREO_ASSOCIATION_UNAVAILABLE"
    assert row["rgb_ld19_header_recent"]
    assert row["age_ns"]["stereo"] is None


def test_expired_bridge_refuses_all_ages(policy):
    out = evaluate(policy, time_ns=LAST+500_000_001)
    assert out["status"] == "BRIDGE_EXPIRED"
    assert out["rows"] == []
    assert out["evaluation_realtime_estimate_ns"] is None


def test_original_bridge_age_limit_overrides_trial_extension(policy):
    b = bridge()
    b["policy_limits"]["max_age_ns"] = 5_000_000
    out = evaluate(policy, time_ns=LAST+10_000_000, clock_bridge=b)
    assert out["status"] == "BRIDGE_EXPIRED"


def test_future_bridge_refuses_all_ages(policy):
    assert evaluate(policy, time_ns=LAST-1)["status"] == "BRIDGE_FROM_FUTURE"


def test_monotonic_evaluation_rollback_refuses_all_ages(policy):
    out = evaluate(policy, time_ns=LAST+10_000_000, prior=LAST+20_000_000)
    assert out["status"] == "EVALUATION_MONOTONIC_ROLLBACK"
    assert not out["rows"]


@pytest.mark.parametrize("mutation", [
    lambda x: x.update(schema_version="wrong"),
    lambda x: x.update(status="ACCEPTED"),
    lambda x: x.update(world_state_update_authorized=True),
    lambda x: x.update(ros_clock_domain_independently_verified=True),
    lambda x: x.update(boot_id="boot-b"),
    lambda x: x.update(timebase="CLOCK_UNKNOWN"),
    lambda x: x["associations"][0].update(motor_control_authorized=True),
    lambda x: x["associations"][0].update(confidence=1.0),
    lambda x: x["associations"][0]["rgb"].update(source_id="wrong"),
    lambda x: x["associations"][0]["rgb"].update(evidence_ids=["forged"]),
])
def test_source_tampering_refused(policy, mutation):
    src = association()
    mutation(src)
    with pytest.raises(ValueError):
        evaluate(policy, src)


@pytest.mark.parametrize("mutation", [
    lambda x: x.update(status="CLOCK_STEP_SUSPECTED"),
    lambda x: x.update(boot_id="boot-b"),
    lambda x: x.update(realtime_clock="FAKE"),
    lambda x: x.update(first_sample_monotonic_ns=LAST+1),
    lambda x: x.update(policy_limits={}),
])
def test_bridge_tampering_refused(policy, mutation):
    b = bridge()
    mutation(b)
    with pytest.raises(ValueError):
        evaluate(policy, clock_bridge=b)


def test_trial_policy_requires_explicit_limits():
    with pytest.raises(ValueError):
        FreshnessTrialPolicy("x", -1, 1, 1, 1, 1)
    with pytest.raises(ValueError):
        FreshnessTrialPolicy("", 1, 1, 1, 1, 1)
    with pytest.raises(ValueError):
        FreshnessTrialPolicy("x", 1, 1, True, 1, 1)
