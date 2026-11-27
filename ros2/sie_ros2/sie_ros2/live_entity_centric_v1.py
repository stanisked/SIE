"""Source-bound adapters for Step 74. No ROS, capture, or control execution."""

import math
import re

from .ld19_diagnostic_evidence import make_ld19_diagnostic_record
from .live_temporal_association_v1 import FRESHNESS, PAIRING, RECENT, nanoseconds
from .temporal_association_freshness_v1 import classify_source_age
from vision_core.person_localization.diagnostic_instance_selection import has_unique_eligible_instance_binding
from vision_core.person_localization.torso_core_geometry import TORSO_CORE_POLICY
from vision_core.stereo.entity_centric_diagnostic_v1 import BOUNDARY, evidence, integrate_cycle, lidar_observation

SCHEMA = "sie.live_entity_centric.v1.diagnostic"
POLICY = "live-entity-centric-v1-TRIAL"
CAMERA_INPUT = "/sie/diagnostics/ar0234_ov9281/temporal_shadow_preview"
CAMERA_SCHEMA = "sie.ar0234_ov9281.control_input_preview.v1"
CAMERA_OUTPUT = "/sie/diagnostics/entity_centric/camera"
LD19_OUTPUT = "/sie/diagnostics/entity_centric/obstacles"
NO_SURFACE = evidence("UNAVAILABLE", "NOT_EVALUATED_IN_LIVE_PREVIEW")
NO_JOIN = {"status": "NOT_EVALUATED_INDEPENDENT_STREAMS"}
HASH = re.compile(r"[0-9a-f]{64}")
CALIBRATION_KEYS = ("ar0234_intrinsic_sha256", "ov9281_stereo_sha256",
                    "ar0234_to_physical_left_extrinsic_sha256")
FRONT_FRAME = "robot_longitudinal_axis_from_front_plane"
RECTIFIED_FRAME = "rectified_left_optical_frame"
RGB_FRAME = "ar0234_image_frame"
MASK_STATUS = "COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY"
TORSO_STATUS = "STABLE_TORSO_CORE_FRONT_DIAGNOSTIC_ONLY"


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def packet_timing(frame, source, now_ns, previous_stamp):
    source_data = (frame.get("frames") or {}).get(source) or {}
    if not source_data:
        return evidence("UNAVAILABLE", "V4L2_PACKET_UNAVAILABLE")
    provenance = {"source_id": source, "sequence": source_data.get("sequence"),
                  "timestamp_basis": "V4L2_DRIVER_DECLARED_MONOTONIC_START_OF_EXPOSURE",
                  "source_packet": source_data}
    stamp, receipt, selection = (source_data.get(key) for key in (
        "driver_timestamp_s", "host_packet_received_monotonic_s", "pair_selected_monotonic_s"))
    flags, sequence = source_data.get("flags"), source_data.get("sequence")
    if (frame.get("capture_backend") != "latest_v4l2"
            or type(sequence) is not int or not 0 <= sequence <= 0xffffffff
            or type(flags) is not int or flags & 0xe000 != 0x2000
            or flags & 0x70000 != 0x10000 or flags & 0x40
            or not all(finite(value) for value in (stamp, receipt, selection))
            or not 0 < stamp <= receipt <= selection):
        return evidence("REJECTED", "V4L2_CLOCK_OR_PACKET_UNVERIFIED", **provenance)
    stamp_ns, receipt_ns, selection_ns = map(nanoseconds, (stamp, receipt, selection))
    provenance.update(source_timestamp_ns=stamp_ns, packet_received_monotonic_ns=receipt_ns,
                      pair_selected_monotonic_ns=selection_ns, age_ns=now_ns - stamp_ns)
    if selection_ns > now_ns:
        return evidence("REJECTED", "PAIR_SELECTION_AHEAD_OF_HOST", **provenance)
    if previous_stamp is not None and stamp_ns <= previous_stamp:
        return evidence("REUSED", "DUPLICATE_OR_REVERSED_SOURCE_STAMP", **provenance)
    limit = FRESHNESS.max_rgb_age_ns if source == "ar0234" else FRESHNESS.max_stereo_age_ns
    age = classify_source_age(now_ns - stamp_ns, limit, FRESHNESS.max_future_skew_ns)
    status = "AVAILABLE" if age == RECENT else ("STALE" if age == "SOURCE_STALE" else "REJECTED")
    return evidence(status, age, **provenance)


def semantic_from_camera(audit):
    cycle = audit["cycle_id"]
    seg = audit.get("segmentation_observation") or {}
    target = audit.get("ar0234_dynamic_target") or {}
    confirmation = audit.get("shadow_track_confirmation") or {}
    continuity = audit.get("shadow_rgb_continuity") or {}
    details = {"target_disposition": target.get("disposition"), "producer_status": seg.get("status"),
               "track_confirmation": confirmation, "rgb_track_continuity": continuity,
               "rgb_reacquisition": audit.get("shadow_rgb_reacquisition")}
    if (seg.get("status") != "CURRENT_INSTANCES_DIAGNOSTIC_ONLY" or seg.get("cycle_id") != cycle
            or seg.get("canonical_observation_published") is not False
            or seg.get("reference_frame") != RGB_FRAME or seg.get("units") != "px"):
        return evidence("UNAVAILABLE", "CURRENT_SEGMENTATION_UNAVAILABLE", **details)
    if not has_unique_eligible_instance_binding(seg.get("instances"), target):
        count = target.get("current_eligible_count")
        status = "AMBIGUOUS" if type(count) is int and count > 1 else "UNAVAILABLE"
        return evidence(status, target.get("reason") or "CURRENT_UNIQUE_TARGET_UNAVAILABLE", **details)
    instance = target["candidate"]
    index = instance["instance_index"]
    obs_id = f"segmentation-preview:{cycle}:{index}"
    track, basis = None, None
    if (confirmation.get("state") == "CONFIRMED_BY_CURRENT_RGB_STEREO_ASSOCIATION"
            and confirmation.get("confirmed_observation_id") == obs_id):
        track, basis = confirmation.get("track_id"), "CURRENT_CONFIRMATION"
    elif not confirmation and continuity.get("status") == "MATCHED_CURRENT_RGB_DIAGNOSTIC_ONLY":
        track, basis = continuity.get("track_id"), "CURRENT_RGB_CONTINUITY"
    if continuity and continuity.get("track_id") != track:
        track, basis = None, None
    hashes = seg.get("instance_mask_sha256")
    mask = hashes[index] if isinstance(hashes, list) and index < len(hashes) else None
    return evidence("AVAILABLE", "CURRENT_PRODUCER_SEMANTIC_OBSERVATION",
                    observation={
                        "observation_id": obs_id, "instance_index": index, "track_id": track,
                        "track_basis": basis, "observation_state": "CURRENT",
                        "semantic_class": seg.get("class_name"), "instance": instance,
                        "confidence": instance.get("confidence"), "reference_frame": RGB_FRAME, "unit": "px",
                        "captured_at_utc": seg.get("captured_at_utc"),
                        "mask_evidence": {"sha256": mask, "producer_arrays_sha256": seg.get("arrays_sha256"),
                                          "model_sha256": seg.get("model_sha256")},
                    }, **details)


def spatial_from_camera(audit, semantic):
    if semantic["status"] != "AVAILABLE":
        return evidence("UNAVAILABLE", "CURRENT_SEMANTIC_TARGET_UNAVAILABLE")
    observation = semantic["observation"]
    shape = audit.get("shape_diagnostic") or {}
    depth = shape.get("mask_depth") or {}
    depth_status = depth.get("status")
    count = depth.get("plausible_layer_count")
    if type(count) is int and count > 1:
        return evidence("AMBIGUOUS", "COMPETING_MASK_DEPTH_LAYERS", depth_quality=depth)
    if not depth:
        return evidence("UNAVAILABLE", shape.get("status") or "LIVE_MASK_DEPTH_UNAVAILABLE")
    cal = audit.get("calibration") or {}
    mask = shape.get("mask") or {}
    source = observation["mask_evidence"]
    expected = {"source_cycle_id": audit["cycle_id"], "mask_sha256": source["sha256"],
                "instance_index": observation["instance_index"], "model_sha256": source["model_sha256"],
                "rgb_arrays_sha256": source["producer_arrays_sha256"],
                "fusion_arrays_sha256": shape.get("arrays_sha256")}
    if (shape.get("cycle_id") != audit["cycle_id"] or count != 1
            or mask.get("status") != "CURRENT_FRAME_MASK_DIAGNOSTIC_ONLY"
            or mask.get("instance_index") != observation["instance_index"]
            or not (audit.get("target_geometry") or {}).get("accepted")
            or any(key in audit for key in ("dynamic_shadow_edge_override", "dynamic_shadow_geometry_override"))
            or observation["instance"].get("truncated_left") or observation["instance"].get("truncated_right")
            or shape.get("mask_sha256") != source["sha256"]
            or shape.get("model_sha256") != source["model_sha256"]
            or mask.get("source_rgb_arrays_sha256") != source["producer_arrays_sha256"]
            or any(not isinstance(expected[key], str) or not HASH.fullmatch(expected[key]) for key in
                   ("mask_sha256", "model_sha256", "rgb_arrays_sha256", "fusion_arrays_sha256"))
            or any(not isinstance(cal.get(key), str) or not HASH.fullmatch(cal[key]) for key in CALIBRATION_KEYS)):
        return evidence("REJECTED", "LIVE_RGB_STEREO_PROVENANCE_OR_QUALITY_REFUSED", depth_quality=depth)

    torso = shape.get("stable_torso_core_front_geometry") or {}
    if torso.get("status") == TORSO_STATUS:
        value = torso.get("selected_known_term_m")
        points = torso.get("point_count")
        if (torso.get("provenance") != expected or torso.get("source_depth_status") != depth_status
                or torso.get("torso_core_policy") != TORSO_CORE_POLICY
                or type(points) is not int or points < TORSO_CORE_POLICY["minimum_stable_points"]
                or torso.get("reference_frame") != FRONT_FRAME or torso.get("unit") != "m"
                or torso.get("distance_from_front_m") is not None
                or not finite(value) or value != (torso.get("known_term_m") or {}).get("median")
                or not HASH.fullmatch(torso.get("profile_sha256") or "")):
            return evidence("REJECTED", "TORSO_CORE_SOURCE_OR_DATUM_MISMATCH", depth_quality=depth)
        method, frame, quantity = TORSO_STATUS, FRONT_FRAME, "partial_front_known_term_median"
        quality = {"torso_core": torso, "mask_depth": depth}
    else:
        value = depth.get("candidate_depth_m")
        if (depth_status != MASK_STATUS or not finite(value) or value <= 0
                or shape.get("reference_frame") != RECTIFIED_FRAME or shape.get("unit") != "m"
                or observation["instance"].get("truncated_top") or observation["instance"].get("truncated_bottom")):
            return evidence("REJECTED", depth_status or "COHERENT_LIVE_DEPTH_UNAVAILABLE", depth_quality=depth)
        method, frame, quantity = MASK_STATUS, RECTIFIED_FRAME, "rectified_optical_z_median"
        quality = {"mask_depth": depth}
    return evidence("AVAILABLE", "CURRENT_PRODUCER_SPATIAL_ESTIMATE",
                    component={"observation_id": f"live-spatial:{audit['cycle_id']}:{method}",
                               "semantic_observation_id": observation["observation_id"],
                               "track_id": observation["track_id"], "method": method, "quantity": quantity,
                               "value_m": value, "reference_frame": frame, "unit": "m",
                               "calibration": cal, "stereo_resolution": audit.get("stereo_resolution"),
                               "provenance": expected, "quality": quality})


def camera_view(audit, clock, previous_stamps):
    """Adapt producer outputs with the original source stamps and trial thresholds."""
    cycle = audit.get("cycle_id")
    if (audit.get("schema_version") != CAMERA_SCHEMA or not isinstance(cycle, str) or not cycle
            or any(audit.get(key) is not False for key in
                   ("metric_measurement_authorized", "navigation_eligible", "actuator_access"))):
        raise ValueError("CAMERA_DIAGNOSTIC_SCHEMA_OR_BOUNDARY_MISMATCH")

    semantic = semantic_from_camera(audit)
    spatial = spatial_from_camera(audit, semantic)
    publication_ns = clock()
    frame = audit.get("frame_freshness") or {}
    rgb, stereo = (packet_timing(frame, source, publication_ns, previous_stamps.get(source))
                   for source in ("ar0234", "ov9281"))
    for info in (rgb, stereo):
        if info["status"] in ("AVAILABLE", "STALE"):
            previous_stamps[info["source_id"]] = info["source_timestamp_ns"]
    if "observation" in semantic:
        semantic["observation"].update(source_timestamp_ns=rgb.get("source_timestamp_ns"),
                                       source_sequence=rgb.get("sequence"))
    producer = {"camera_status": audit.get("status"),
                "rgb_inference_completed": (semantic.get("producer_status") == "CURRENT_INSTANCES_DIAGNOSTIC_ONLY"
                                            and (audit.get("segmentation_observation") or {}).get("cycle_id") == cycle),
                "semantic_status": semantic["status"], "spatial_status": spatial["status"],
                "track_id": (semantic.get("observation") or {}).get("track_id"),
                "rgb_source_timestamp_ns": rgb.get("source_timestamp_ns"),
                "stereo_source_timestamp_ns": stereo.get("source_timestamp_ns"),
                "host_processing": audit.get("processing_timing")}
    if audit.get("status") in ("PROCESSING_REFUSED", "CANCELLED_SHUTDOWN"):
        semantic = semantic | {"status": "REJECTED", "reason": "PRODUCER_CYCLE_REFUSED"}
        spatial = spatial | {"status": "REJECTED", "reason": "PRODUCER_CYCLE_REFUSED"}
    if semantic["status"] == "AVAILABLE" and rgb["status"] != "AVAILABLE":
        semantic = semantic | {"status": rgb["status"], "reason": rgb["reason"]}
    if "observation" in semantic and semantic["status"] != "AVAILABLE":
        semantic["observation"].update(observation_state=semantic["status"],
                                       producer_observation_state="CURRENT")
    if "component" in spatial:
        spatial["component"].update(source_timestamp_ns=stereo.get("source_timestamp_ns"),
                                    source_sequence=stereo.get("sequence"))
    pair = {"status": "UNAVAILABLE", "limit_ns": PAIRING.max_rgb_stereo_delta_ns}
    if rgb["status"] == stereo["status"] == "AVAILABLE":
        delta = stereo["source_timestamp_ns"] - rgb["source_timestamp_ns"]
        same_selection = rgb["pair_selected_monotonic_ns"] == stereo["pair_selected_monotonic_ns"]
        status = "ACTUAL_PAIR_COMPATIBLE_DIAGNOSTIC_ONLY" if (
            same_selection and abs(delta) <= PAIRING.max_rgb_stereo_delta_ns) else "ACTUAL_PAIR_TEMPORALLY_INCOMPATIBLE"
        pair.update(status=status, stereo_minus_rgb_ns=delta)
    if spatial["status"] == "AVAILABLE":
        if semantic["status"] != "AVAILABLE" or stereo["status"] != "AVAILABLE":
            blocked = semantic if semantic["status"] != "AVAILABLE" else stereo
            spatial = spatial | {"status": blocked["status"], "reason": blocked["reason"]}
        elif pair["status"] != "ACTUAL_PAIR_COMPATIBLE_DIAGNOSTIC_ONLY":
            spatial = spatial | {"status": "REJECTED", "reason": pair["status"]}
    timing = {"ar0234": rgb, "ov9281": stereo, "actual_camera_pair": pair}
    return cycle, semantic, spatial, timing, producer, publication_ns


def camera_event(cycle, semantic, spatial):
    return integrate_cycle(cycle, semantic, spatial, evidence("UNAVAILABLE", "INDEPENDENT_LD19_STREAM"),
                           rgb_ld19=NO_JOIN, surface_level=NO_SURFACE)


def scan_view(message, received_at_utc, receipt_sequence, receipt_ns, last_header):
    record = make_ld19_diagnostic_record(message, received_at_utc=received_at_utc)
    header = record["payload"]["scan"]["header"]
    stamp = header["stamp_sec"] * 10**9 + header["stamp_nanosec"]
    timing = {"receipt_sequence": receipt_sequence, "receipt_monotonic_ns": receipt_ns,
              "timestamp_basis": "ROS_SCAN_CONVERSION_TIME_NOT_ACQUISITION",
              "header_age_ns": None, "header_clock_status": "UNASSESSED_INDEPENDENT_CLOCK"}
    if last_header is not None and stamp <= last_header:
        return evidence("REUSED", "DUPLICATE_OR_REVERSED_LD19_HEADER",
                        observation_id=record["observation_id"], header_timestamp_ns=stamp,
                        reference_frame=record["reference_frame"], timing=timing), last_header
    scan = lidar_observation(record, timing=timing)
    status = "AVAILABLE" if scan["valid_return_count"] else "REJECTED"
    reason = "LD19_SOURCE_LOCAL_GEOMETRY" if scan["valid_return_count"] else "NO_VALID_LD19_RETURNS"
    return evidence(status, reason, observation=scan), stamp


def obstacle_event(ld19):
    return {**BOUNDARY, "ld19": ld19, "camera_ld19_timing": NO_JOIN,
            "surface_level": NO_SURFACE, "ld19_entity_association": {"status": "UNAVAILABLE", "links": []}}
