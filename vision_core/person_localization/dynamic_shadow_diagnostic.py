"""Current-frame RGB plus OV9281 diagnostic for moving Shadow targets.

The output is never a Shadow range or motor admission. A stable person ID,
physical front clearance, exposure-to-command age, and physical error bound
need separate validation before the motion runner may consume an observation.
"""
from __future__ import annotations

import math


STEREO_SHA256 = "39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52"


def select_edge_candidate(segmentation: dict, legacy: dict, *, min_confidence: float):
    """One current clipped person for diagnostics, never an identity or action."""
    if legacy.get("disposition") == "OBSERVED":
        return legacy, False
    instances = segmentation.get("instances")
    if not isinstance(instances, list) or len(instances) != 1:
        return legacy, False
    person = instances[0]
    if not isinstance(person, dict) or not any(
            person.get(flag) is True for flag in
            ("truncated_left", "truncated_right", "truncated_top")):
        return legacy, False
    box = person.get("bbox_xyxy_px")
    confidence = person.get("confidence")
    if (type(confidence) not in (float, int) or not math.isfinite(confidence) or
            confidence < min_confidence or not isinstance(box, list) or len(box) != 4 or
            not all(type(x) in (float, int) and math.isfinite(x) for x in box) or
            not (0 <= box[0] < box[2] <= 1920 and 0 <= box[1] < box[3] <= 1200) or
            box[2] - box[0] < 30 or box[3] - box[1] < 60 or
            type(person.get("instance_index")) is not int):
        return legacy, False
    return {"disposition": "OBSERVED", "candidate": person,
            "current_eligible_count": 1, "metric_measurement_authorized": False,
            "selection_source": "CURRENT_FRAME_SINGLE_EDGE_DIAGNOSTIC_ONLY",
            "reason": "SINGLE_CURRENT_EDGE_CLIPPED_PERSON_DIAGNOSTIC_ONLY"}, True


def allow_near_edge_geometry(segmentation: dict, candidate: dict, geometry: dict,
                             *, min_confidence: float):
    """Permit a selected single person past a near-edge refusal for diagnostics."""
    if geometry.get("accepted") is not False or "BBOX_NEAR_" not in str(geometry.get("reason")):
        return False
    instances = segmentation.get("instances")
    if not isinstance(instances, list) or len(instances) != 1 or instances[0] != candidate:
        return False
    box = candidate.get("bbox_xyxy_px")
    conf = candidate.get("confidence")
    return (type(conf) in (int, float) and math.isfinite(conf) and conf >= min_confidence
            and type(candidate.get("instance_index")) is int
            and isinstance(box, list) and len(box) == 4
            and all(type(x) in (int, float) and math.isfinite(x) for x in box)
            and 0 <= box[0] < box[2] <= 1920 and 0 <= box[1] < box[3] <= 1200
            and box[2]-box[0] >= 150 and box[3]-box[1] >= 60)


class DiagnosticContinuity:
    """Adjacent box association; a gap or ambiguous frame starts a new segment."""
    def __init__(self):
        self._prior = None
        self._last_s = None
        self._segment = 0

    def observe(self, candidate, *, now_s):
        box = (candidate or {}).get("bbox_xyxy_px")
        valid = (isinstance(box, list) and len(box) == 4 and
                 all(type(x) in (int, float) and math.isfinite(x) for x in box))
        if not valid:
            self._prior = None
            self._last_s = None
            return {"status": "NO_SINGLE_CURRENT_PERSON", "segment": self._segment,
                    "same_person_proven": False}
        contiguous = False
        if self._prior is not None and self._last_s is not None and 0 <= now_s - self._last_s <= .5:
            previous = self._prior
            width = max(previous[2]-previous[0], box[2]-box[0])
            height = max(previous[3]-previous[1], box[3]-box[1])
            contiguous = (abs((previous[0]+previous[2]-box[0]-box[2])/2) <= .35*width
                          and abs((previous[1]+previous[3]-box[1]-box[3])/2) <= .35*height)
        if not contiguous:
            self._segment += 1
        self._prior = box[:]
        self._last_s = now_s
        return {"status": ("ADJACENT_BOX_MATCH_DIAGNOSTIC_ONLY" if contiguous else
                           "NEW_SEGMENT_IDENTITY_UNVERIFIED"),
                "segment": self._segment, "same_person_proven": False}


def continue_after_static_gate(gate: dict, *, dynamic_diagnostic_enabled: bool):
    """Legacy refusal stays effective unless the separate diagnostic is on."""
    return gate.get("status") == "PASS" or dynamic_diagnostic_enabled is True


def dynamic_shadow_diagnostic(audit: dict, *, now_s: float, max_driver_age_s=.5):
    result = {
        "schema_version": "sie.dynamic_shadow_rgb_depth_diagnostic.v1",
        "status": "NO_CURRENT_RGB_TARGET", "cycle_id": audit.get("cycle_id"),
        "static_gate_bypassed_for_diagnostic": False,
        "legacy_static_gate_status": (audit.get("existing_profile_static_gate") or {}).get("status"),
        "legacy_static_gate_reason": (audit.get("existing_profile_static_gate") or {}).get("reason"),
        "motor_command_authorized": False, "metric_measurement_authorized": False,
        "stable_person_identity_established": False,
        "front_clearance_established": False,
        "exposure_to_command_age_established": False,
        "physical_error_bound_established": False,
        "candidate_optical_z_m": None,
        "reference_frame": "rectified_left_optical_frame",
    }
    target = audit.get("ar0234_dynamic_target") or {}
    segmentation = audit.get("segmentation_observation") or {}
    candidate = target.get("candidate") or {}
    frame = audit.get("frame_freshness") or {}
    frames = frame.get("frames") or {}
    rgb = frames.get("ar0234") or {}
    stereo = frames.get("ov9281") or {}
    shape = audit.get("shape_diagnostic") or {}
    mask = shape.get("mask") or {}
    depth = shape.get("mask_depth") or {}
    result["mask_status"] = depth.get("status")
    result["source_instance_index"] = candidate.get("instance_index")
    result["rgb_arrays_sha256"] = segmentation.get("arrays_sha256")
    result["stereo_calibration_sha256"] = (audit.get("calibration") or {}).get("ov9281_stereo_sha256")
    result["refusal_reasons"] = []

    if (target.get("disposition") != "OBSERVED" or
            not (audit.get("target_geometry") or {}).get("accepted") or
            segmentation.get("cycle_id") != audit.get("cycle_id") or
            type(candidate.get("instance_index")) is not int):
        result["refusal_reasons"].append("NO_CURRENT_RGB_TARGET")
        return result
    result["status"] = "RGB_CURRENT_DEPTH_UNAVAILABLE"
    if result["stereo_calibration_sha256"] != STEREO_SHA256:
        result["refusal_reasons"].append("STEREO_CALIBRATION_MISMATCH")
    if frame.get("capture_backend") != "latest_v4l2":
        result["refusal_reasons"].append("DRIVER_TIMESTAMPS_UNAVAILABLE")
    stamps = [rgb.get("driver_timestamp_s"), stereo.get("driver_timestamp_s")]
    if (not all(type(t) in (int, float) and math.isfinite(t) and
                type(now_s) in (int, float) and math.isfinite(now_s) and
                0 <= now_s - t <= max_driver_age_s for t in stamps)):
        result["refusal_reasons"].append("DRIVER_FRAME_STALE_OR_UNAVAILABLE")
    if (shape.get("cycle_id") != audit.get("cycle_id") or
            mask.get("status") != "CURRENT_FRAME_MASK_DIAGNOSTIC_ONLY" or
            mask.get("instance_index") != candidate["instance_index"] or
            mask.get("source_rgb_arrays_sha256") != segmentation.get("arrays_sha256") or
            not segmentation.get("arrays_sha256")):
        result["refusal_reasons"].append("CURRENT_FRAME_MASK_BINDING_INVALID")
    if (depth.get("status") != "COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY" or
            type(depth.get("candidate_depth_m")) not in (int, float) or
            not math.isfinite(depth["candidate_depth_m"]) or
            depth["candidate_depth_m"] <= 0):
        result["refusal_reasons"].append("COHERENT_MASKED_DEPTH_UNAVAILABLE")
    if result["refusal_reasons"]:
        return result
    result["status"] = "CURRENT_RGB_MASKED_DEPTH_DIAGNOSTIC_ONLY"
    result["candidate_optical_z_m"] = depth["candidate_depth_m"]
    result["refusal_reasons"] = [
        "STABLE_PERSON_IDENTITY_NOT_ESTABLISHED",
        "ROBOT_FRONT_CLEARANCE_NOT_ESTABLISHED",
        "EXPOSURE_TO_COMMAND_AGE_NOT_ESTABLISHED",
        "PHYSICAL_DEPTH_ERROR_BOUND_NOT_ESTABLISHED",
    ]
    return result
