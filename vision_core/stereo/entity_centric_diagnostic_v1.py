"""Source-local evidence and provisional entity views for offline diagnostics."""

import hashlib
import json
import math

from sie_ros2.ld19_diagnostic_evidence import DIAGNOSTIC_SCHEMA, EXPECTED_FRAME
from sie_ros2.live_temporal_association_v1 import BOUNDARY as TEMPORAL_BOUNDARY
from vision_core.stereo.three_sensor_semantic_spatial_v1 import MATCH


SCHEMA = "sie.entity_centric_preview.v1.diagnostic"
POLICY_ID = "entity-centric-preview-v1-TRIAL"
BOUNDARY = {
    **TEMPORAL_BOUNDARY,
    "fact_admitted": False,
    "decision_authorized": False,
    "navigation_authorized": False,
}


def evidence(status, reason, **details):
    return {"status": status, "reason": reason, **details}


def lidar_observation(record, *, timing):
    """Keep occupied returns in the original scan frame, including invalid dispositions."""
    if (record.get("schema_version") != DIAGNOSTIC_SCHEMA
            or record.get("status") != "DIAGNOSTIC_ONLY_NOT_ADMITTED"
            or record.get("source_id") != EXPECTED_FRAME
            or record.get("reference_frame") != EXPECTED_FRAME
            or any(record.get("admission", {}).get(key) is not False for key in
                   ("measurement_authorized", "world_state_update_authorized", "motor_control_authorized"))):
        raise ValueError("LD19_DIAGNOSTIC_RECORD_INVALID")
    raw = record["payload"]["scan"]
    digest = hashlib.sha256(json.dumps(raw, allow_nan=False, sort_keys=True,
                                       separators=(",", ":")).encode()).hexdigest()
    header = raw["header"]
    sec, nsec = header["stamp_sec"], header["stamp_nanosec"]
    if (type(sec) is not int or sec <= 0 or type(nsec) is not int or not 0 <= nsec < 10**9
            or header["frame_id"] != EXPECTED_FRAME
            or record["observation_id"] != f"ld19-diagnostic:{sec}:{nsec:09d}:{digest[:12]}"
            or record["evidence_ids"] != [f"ld19-raw:{digest}"]
            or record["evidence"]["raw_scan_sha256"] != digest):
        raise ValueError("LD19_SCAN_IDENTITY_MISMATCH")
    start, end, step = (raw[key] for key in ("angle_min_rad", "angle_max_rad", "angle_increment_rad"))
    minimum, maximum = raw["range_min_m"], raw["range_max_m"]
    ranges = raw["ranges_m"]
    if (not all(type(value) in (int, float) and math.isfinite(value)
                for value in (start, end, step, minimum, maximum))
            or start > end or step <= 0 or not 0 <= minimum < maximum
            or not isinstance(ranges, list) or not 0 < len(ranges) <= 5000):
        raise ValueError("LD19_SCAN_GEOMETRY_INVALID")

    beams = []
    for index, raw_range in enumerate(ranges):
        bearing = start + index * step
        if not math.isfinite(bearing):
            raise ValueError("LD19_BEARING_INVALID")
        beam = {"index": index, "bearing_rad": bearing, "raw_range": raw_range}
        if raw_range is None:
            beam["status"] = "MISSING_RANGE"
        elif type(raw_range) not in (int, float) or not math.isfinite(raw_range):
            beam["status"] = "INVALID_RANGE"
        elif not (0 < raw_range and minimum <= raw_range <= maximum):
            beam["status"] = "RANGE_OUT_OF_BOUNDS"
        else:
            beam.update(status="VALID_RETURN", range_m=raw_range,
                        xy_m=[raw_range * math.cos(bearing), raw_range * math.sin(bearing)])
        beams.append(beam)
    count = sum(beam["status"] == "VALID_RETURN" for beam in beams)
    return {
        "observation_id": record["observation_id"], "source_id": record["source_id"],
        "source_topic": record.get("source_topic"),
        "evidence_ids": record["evidence_ids"], "raw_scan_sha256": digest,
        "reference_frame": EXPECTED_FRAME,
        "units": {"range": "m", "position": "m", "bearing": "rad", "header_stamp": "ns", "scan_interval": "s"},
        "header": header, "header_timestamp_ns": sec * 10**9 + nsec,
        "source_timestamp": record["timestamp"], "received_at_utc": record["received_at_utc"], "timing": timing,
        "angle_min_rad": start, "angle_max_rad": end, "angle_increment_rad": step,
        "range_min_m": minimum, "range_max_m": maximum,
        "scan_time_s": raw["scan_time_s"], "time_increment_s": raw["time_increment_s"],
        "physical_acquisition_time": "UNKNOWN",
        "quality": record.get("quality"), "confidence": record.get("confidence"),
        "valid_return_count": count, "beams": beams,
    }


def integrate_cycle(cycle_id, semantic, spatial, ld19, *, rgb_ld19, surface_level):
    """Combine verified camera evidence; retain LiDAR even without camera evidence."""
    candidate = None
    if semantic["status"] == spatial["status"] == "AVAILABLE":
        observation, component = semantic["observation"], spatial["component"]
        if (observation["observation_id"] == component["semantic_observation_id"]
                and observation["track_id"] and observation["track_id"] == component["track_id"]
                and observation["observation_state"] == "CURRENT"):
            candidate = {
                "candidate_id": f"entity-preview:{observation['observation_id']}",
                "status": "PROVISIONAL_DIAGNOSTIC_ONLY",
                "semantic_class": observation["semantic_class"], "track_id": observation["track_id"],
                "semantic_observation_id": observation["observation_id"],
                "spatial_component_id": component["observation_id"],
            }
        else:
            spatial = spatial | {"status": "REJECTED", "reason": "SEMANTIC_SPATIAL_BINDING_MISMATCH"}

    # No LD19 membership policy is approved for this preview. Even a
    # Step 71 Surface-Level candidate is only supporting diagnostic context.
    if candidate is None or ld19["status"] == "UNAVAILABLE":
        relation = "UNAVAILABLE"
    elif ld19["status"] in ("REJECTED", "STALE"):
        relation = "LD19_EVIDENCE_REFUSED"
    elif rgb_ld19.get("status") != MATCH:
        relation = "TEMPORAL_LINK_UNRESOLVED"
    elif surface_level["status"] == "AMBIGUOUS":
        relation = "AMBIGUOUS"
    else:
        relation = "ENTITY_MEMBERSHIP_UNRESOLVED"
    return {
        **BOUNDARY, "schema_version": SCHEMA, "cycle_id": cycle_id,
        "semantic": semantic, "spatial": spatial, "ld19": ld19,
        "provisional_entity": candidate, "camera_ld19_timing": rgb_ld19,
        "surface_level": surface_level, "ld19_entity_association": {"status": relation, "links": []},
    }
