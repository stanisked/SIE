"""Pure metadata-only assessment for the live Temporal Association shadow."""

from dataclasses import asdict, dataclass

from .host_timebase_bridge_v1 import HostBridgeTestPolicy, convert_monotonic_stamp
from .latest_v4l2_capture import CameraTiming
from .temporal_association_diagnostic_v1 import (
    LD19_STAMP_BASIS, TemporalAssociationTrialPolicy,
)
from .temporal_association_freshness_v1 import FreshnessTrialPolicy, classify_source_age

SCHEMA = "sie.live_temporal_association.v1.diagnostic"
BOUNDARY = {
    "classification": "FACT",
    "status": "DIAGNOSTIC_ONLY_NOT_ADMITTED",
    "measurement_authorized": False,
    "world_state_update_authorized": False,
    "motor_control_authorized": False,
    "physical_laser_acquisition_time": "UNKNOWN",
    "physical_camera_exposure_time_independently_verified": False,
}
MS = 1_000_000
BRIDGE_POLICY = HostBridgeTestPolicy(
    "live-shadow-host-bridge-v1-TRIAL", 100_000, 100 * MS, 100_000,
    50_000, 100_000, 1_000 * MS, 250 * MS, MS,
)
FRESHNESS = FreshnessTrialPolicy(
    "live-shadow-source-freshness-v1-TRIAL", 250 * MS, 250 * MS, 250 * MS,
    1_000 * MS, MS,
)
PAIRING = TemporalAssociationTrialPolicy("live-shadow-pairing-v1-TRIAL", 25 * MS, 50 * MS)
RECENT = "RECENT_HOST_TIMESTAMP_DIAGNOSTIC_ONLY"


def nanoseconds(seconds):
    return None if seconds is None else round(seconds * 1_000_000_000)


@dataclass(frozen=True)
class ScanTiming:
    receipt_sequence: int
    header_ns: int | None
    received_monotonic_ns: int
    reference_frame: str
    clock_status: str
    source_status: str = "HEADER_PRESENT"


@dataclass(frozen=True)
class CycleTiming:
    cycle_id: str
    preview_status: str
    rgb: CameraTiming | None
    stereo: CameraTiming | None
    selection_policy: str | None
    selection_requested_ns: int | None
    rgb_frozen_ns: int | None
    pair_selected_ns: int | None
    result_ready_ns: int
    publication_ns: int | None
    capture_status: dict


def bridge_at(windows, when, boot_id):
    """Use the latest bridge observation already available at the endpoint."""
    if when is None:
        return {"clock_valid": False, "status": "ENDPOINT_UNAVAILABLE", "bridge": None}
    bridge = next((bridge for received, bridge in reversed(windows) if received <= when), None)
    if bridge is None:
        status = "BRIDGE_UNAVAILABLE"
    elif bridge["status"] != "DIAGNOSTIC_BRIDGE_READY":
        status = bridge["status"]
    elif when - bridge["last_sample_monotonic_ns"] > FRESHNESS.max_bridge_holdover_ns:
        status = "BRIDGE_EXPIRED"
    else:
        status = convert_monotonic_stamp(
            bridge, source_monotonic_ns=when, evaluation_monotonic_ns=when,
            boot_id=boot_id, policy=BRIDGE_POLICY,
        )["status"]
    return {
        "clock_valid": status == "HOST_TIMEBASE_CONVERTED_DIAGNOSTIC_ONLY",
        "status": status,
        "bridge": bridge,
    }


def _camera(source, packet, session):
    if packet is None:
        return None
    return {
        "source_id": source,
        "sequence": packet.sequence,
        "evidence_ref": f"live-shadow:{session}:{source}:driver:{packet.sequence}",
        "timestamp_ns": nanoseconds(packet.driver_timestamp_s),
        "timestamp_basis": "V4L2_DRIVER_DECLARED_MONOTONIC_START_OF_EXPOSURE",
        "host_dequeue_monotonic_ns": nanoseconds(packet.host_dequeue_monotonic_s),
        "received_monotonic_ns": nanoseconds(packet.host_packet_received_monotonic_s),
        "reference_frame": "UNVERIFIED_IN_CAPTURE",
    }


def _scan(timing, session):
    return {
        "source_id": "ld19_sensor",
        "sequence": None,  # ROS 2 LaserScan has no publisher sequence in its header.
        "receipt_sequence": timing.receipt_sequence,
        "evidence_ref": f"live-shadow:{session}:ld19:receipt:{timing.receipt_sequence}",
        "timestamp_ns": timing.header_ns,
        "timestamp_basis": LD19_STAMP_BASIS,
        "received_monotonic_ns": timing.received_monotonic_ns,
        "reference_frame": timing.reference_frame,
        "ros_clock_status": timing.clock_status,
        "source_status": timing.source_status,
    }


def _assess(source, when, clock, max_age):
    if source is None:
        return {"status": "SOURCE_UNAVAILABLE", "age_ns": None}
    if when is None:
        return {"status": "ENDPOINT_UNAVAILABLE", "age_ns": None}
    if source["received_monotonic_ns"] > when:
        return {"status": "NOT_RECEIVED_BY_ENDPOINT", "age_ns": None}
    if source.get("source_status", "HEADER_PRESENT") != "HEADER_PRESENT":
        return {"status": source["source_status"], "age_ns": None}
    if not clock["clock_valid"]:
        return {"status": clock["status"], "age_ns": None}
    offset = 0
    if source["source_id"] == "ld19_sensor":
        if source["ros_clock_status"] != "OPERATOR_ATTESTED_SAME_HOST_REALTIME":
            return {"status": source["ros_clock_status"], "age_ns": None}
        offset = clock["bridge"]["offset_ns"]
    age = when + offset - source["timestamp_ns"]
    status = classify_source_age(age, max_age, FRESHNESS.max_future_skew_ns)
    if status == RECENT and source["timestamp_ns"] > source["received_monotonic_ns"] + offset:
        status = "STAMP_AHEAD_OF_RECEIPT"
    return {"status": status, "age_ns": age}


def _link(rgb, candidate, rgb_age, candidate_age, delta, limit):
    if rgb_age["status"] != RECENT:
        reason = "RGB_" + rgb_age["status"]
    elif candidate_age["status"] != RECENT:
        reason = candidate_age["status"]
    elif abs(delta) > limit:
        reason = "DELTA_EXCEEDED"
    else:
        reason = "MATCH_DIAGNOSTIC_ONLY"
    return {
        "status": reason,
        "candidate": candidate,
        "rgb_freshness": rgb_age,
        "candidate_freshness": candidate_age,
        "candidate_minus_rgb_ns": delta,
        "delta_limit_ns": limit,
    }


def _nearest_causal(rgb, candidates, evicted_through, when, clock, max_age, limit):
    rgb_age = _assess(rgb, when, clock, FRESHNESS.max_rgb_age_ns)
    available = [sample for sample in candidates if when is not None
                 and sample["received_monotonic_ns"] <= when]
    pairs = [(sample, _assess(sample, when, clock, max_age)) for sample in available]
    # A discarded packet received inside the freshness horizon could affect the choice.
    truncated = (when is not None and evicted_through is not None
                 and evicted_through >= when - max_age)
    if truncated:
        return _link(rgb, None, rgb_age, {"status": "CAUSAL_HISTORY_TRUNCATED", "age_ns": None},
                     None, limit) | {"available_count": len(available)}
    if not pairs:
        return _link(rgb, None, rgb_age, {"status": "SOURCE_UNAVAILABLE", "age_ns": None},
                     None, limit) | {"available_count": 0}

    def delta(sample):
        if rgb is None or sample["timestamp_ns"] is None:
            return None
        if sample["source_id"] == "ld19_sensor":
            if (not clock["clock_valid"]
                    or sample["ros_clock_status"] != "OPERATOR_ATTESTED_SAME_HOST_REALTIME"
                    or sample["source_status"] != "HEADER_PRESENT"):
                return None
            return sample["timestamp_ns"] - (rgb["timestamp_ns"] + clock["bridge"]["offset_ns"])
        return sample["timestamp_ns"] - rgb["timestamp_ns"]

    def rank(pair):
        sample, freshness = pair
        difference = delta(sample)
        return (freshness["status"] != RECENT, difference is None,
                abs(difference) if difference is not None else 0,
                sample["timestamp_ns"] if difference is not None else -sample["received_monotonic_ns"])

    candidate, candidate_age = min(pairs, key=rank)
    return _link(rgb, candidate, rgb_age, candidate_age, delta(candidate), limit) | {
        "available_count": len(available)
    }


def evaluate_cycle(cycle, camera_histories, scans, scan_evicted_through, windows, *,
                   boot_id, session, ld19_clock_evidence):
    """Compare what was received at RGB arrival with the untouched preview pair."""
    rgb = _camera("ar0234", cycle.rgb, session)
    actual_stereo = _camera("ov9281", cycle.stereo, session)
    stereo_frames, stereo_evicted_through_s = camera_histories["ov9281"]
    rgb_received = rgb["received_monotonic_ns"] if rgb else None
    endpoints = {
        "zero_wait": rgb_received,
        "selection": cycle.pair_selected_ns,
        "result_ready": cycle.result_ready_ns,
        "publication": cycle.publication_ns,
    }
    clocks = {name: bridge_at(windows, instant, boot_id) for name, instant in endpoints.items()}
    camera_link = _nearest_causal(
        rgb, [_camera("ov9281", frame, session) for frame in stereo_frames],
        nanoseconds(stereo_evicted_through_s), rgb_received, clocks["zero_wait"],
        FRESHNESS.max_stereo_age_ns, PAIRING.max_rgb_stereo_delta_ns,
    )
    ld19_link = _nearest_causal(
        rgb, [_scan(scan, session) for scan in scans], scan_evicted_through,
        rgb_received, clocks["zero_wait"], FRESHNESS.max_ld19_ros_header_age_ns,
        PAIRING.max_rgb_ld19_header_delta_ns,
    )
    actual_delta = (actual_stereo["timestamp_ns"] - rgb["timestamp_ns"]
                    if rgb and actual_stereo else None)
    matched_stereo = camera_link["candidate"] if camera_link["status"] == "MATCH_DIAGNOSTIC_ONLY" else None
    return {
        **BOUNDARY,
        "schema_version": SCHEMA,
        "boot_id": boot_id,
        "session_id": session,
        "cycle_id": cycle.cycle_id,
        "preview_status": cycle.preview_status,
        "capture_status": cycle.capture_status,
        "rgb": rgb,
        "selection_policy": cycle.selection_policy,
        "selection_requested_monotonic_ns": cycle.selection_requested_ns,
        "rgb_frozen_monotonic_ns": cycle.rgb_frozen_ns,
        "endpoint_monotonic_ns": endpoints,
        "clock": clocks,
        "association_policy": asdict(PAIRING),
        "freshness_policy": asdict(FRESHNESS),
        "ld19_clock_evidence": ld19_clock_evidence,
        "history": {
            **{source: {"retained": len(rows), "evicted_through_monotonic_ns": nanoseconds(evicted)}
               for source, (rows, evicted) in camera_histories.items()},
            "ld19": {"retained": len(scans), "evicted_through_monotonic_ns": scan_evicted_through},
        },
        "zero_wait": {"rgb_stereo": camera_link, "rgb_ld19_ros_header": ld19_link},
        "actual_camera_pair": {
            "stereo": actual_stereo,
            "stereo_minus_rgb_ns": actual_delta,
            "stereo_received_by_rgb_receipt": (
                actual_stereo["received_monotonic_ns"] <= rgb_received
                if actual_stereo is not None and rgb_received is not None else None),
            "same_stereo_as_zero_wait": (
                actual_stereo["sequence"] == matched_stereo["sequence"]
                if actual_stereo is not None and matched_stereo is not None else None),
        },
        "freshness_by_endpoint": {
            endpoint: {
                "rgb": _assess(rgb, instant, clocks[endpoint], FRESHNESS.max_rgb_age_ns),
                "actual_stereo": _assess(actual_stereo, instant, clocks[endpoint], FRESHNESS.max_stereo_age_ns),
                "zero_wait_stereo": _assess(matched_stereo, instant, clocks[endpoint], FRESHNESS.max_stereo_age_ns),
                "zero_wait_ld19_header": _assess(
                    ld19_link["candidate"] if ld19_link["status"] == "MATCH_DIAGNOSTIC_ONLY" else None,
                    instant, clocks[endpoint], FRESHNESS.max_ld19_ros_header_age_ns),
            }
            for endpoint, instant in endpoints.items() if endpoint != "zero_wait"
        },
    }
