"""Bind one fast control-preview cycle to an experimental Shadow observation.

The selected RGB instance, its exact mask, stereo fusion, track confirmation,
front profile and driver timestamps must agree. A Shadow v3 box alone cannot
stand in for this instance mask. This module does not emit SIE Measurements.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

from vision_core.person_localization.fast_local_depth_consistency import STABLE_LOCAL_PATCH_POLICY
from vision_core.tools.run_shadow_uart import ShadowObservation
from vision_core.tools.shadow_range_gate import (
    ACTIVE_STEREO_SHA256, FRONT_FRAME, RgbTarget, TargetRange,
)


AR_INTRINSIC_SHA256 = "e9454ed3d93ab36a6b159aa8c8e2356353cca8a6b0480e73c371199937ae7096"
SHADOW_V3_SHA256 = "fccd45fbcf490134eb786ab3b38cf115dd03466edd85801ad44c8353f38549f5"
FRONT_PROFILE = (Path(__file__).parents[1] / "config/diagnostics/robot_front_mount_operator_v1.json")


def front_profile_sha256(path: Path = FRONT_PROFILE) -> str:
    profile = json.loads(path.read_text())
    return hashlib.sha256(json.dumps(profile, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def match_v3_to_mask_instance(v3: dict, segmentation: dict, candidate: dict,
                              *, rgb_payload_sha256: str) -> dict:
    """Match a current v3 upper-body box to one COCO instance, never to pixels.

    The actual pixels remain owned by the independent, exact-cycle COCO mask.
    Ambiguous overlaps refuse; this comparison does not relabel a box as a mask.
    """
    result = {"status": "V3_MASK_MATCH_UNAVAILABLE", "source_cycle_id": segmentation.get("cycle_id"),
              "v3_model_sha256": v3.get("model_sha256"),
              "v3_target_status": v3.get("target_status"),
              "v3_detection_count": v3.get("detection_count"),
              "source_rgb_arrays_sha256": segmentation.get("arrays_sha256"),
              "rgb_payload_sha256": rgb_payload_sha256,
              "mask_instance_index": candidate.get("instance_index")}
    if (v3.get("source_cycle_id") != segmentation.get("cycle_id") or
            v3.get("model_sha256") != SHADOW_V3_SHA256 or
            v3.get("target_status") != "SINGLE_TARGET" or
            v3.get("detection_count") != 1 or
            not rgb_payload_sha256 or
            segmentation.get("rgb_payload_sha256") != rgb_payload_sha256):
        return result
    box = v3.get("bbox_xyxy_px")
    instances = segmentation.get("instances")
    if (not isinstance(box, list) or len(box) != 4 or
            not isinstance(instances, list) or
            not all(type(x) in (int, float) and math.isfinite(x) for x in box) or
            box[2] <= box[0] or box[3] <= box[1]):
        return result
    area = (box[2] - box[0]) * (box[3] - box[1])
    scores = []
    for instance in instances:
        other = instance.get("bbox_xyxy_px")
        if (not isinstance(other, list) or len(other) != 4 or
                not all(type(x) in (int, float) and math.isfinite(x) for x in other)):
            return result
        overlap = max(0, min(box[2], other[2]) - max(box[0], other[0])) * \
                  max(0, min(box[3], other[3]) - max(box[1], other[1]))
        scores.append((overlap / area, instance.get("instance_index")))
    scores.sort(reverse=True)
    if (not scores or scores[0][1] != candidate.get("instance_index") or
            scores[0][0] < .70 or
            (len(scores) > 1 and scores[0][0] - scores[1][0] < .20)):
        return result
    return {**result, "status": "BOUND_TO_CURRENT_MASK_INSTANCE",
            "v3_box_xyxy_px": box, "overlap_fraction": scores[0][0],
            "next_overlap_fraction": scores[1][0] if len(scores) > 1 else 0.}


def from_control_preview(audit: dict, *, now_s: float,
                         rim_forward_of_optical_center_m: float,
                         expected_front_profile_sha256: str,
                         expected_segmentation_model_sha256: str,
                         semantic_source: str = "v3_mask_binding",
                         local_preview: bool = False,
                         max_age_s: float = .5) -> tuple[ShadowObservation, str]:
    """Return current evidence or a precise PAUSE reason; no cached fallback."""
    empty = ShadowObservation(None, ())
    if semantic_source not in ("v3_mask_binding", "segmentation_mask"):
        raise ValueError("unsupported Shadow semantic source")
    if type(local_preview) is not bool:
        raise ValueError("local_preview must be boolean")
    if not isinstance(audit, dict):
        return empty, "AUDIT_UNAVAILABLE"
    if (type(rim_forward_of_optical_center_m) not in (int, float) or
            not math.isfinite(rim_forward_of_optical_center_m) or
            not 0 <= rim_forward_of_optical_center_m <= .10):
        raise ValueError("FRONT_OPTICAL_DATUM_REQUIRED")
    cid = audit.get("cycle_id")
    if (not isinstance(cid, str) or not cid or
            audit.get("schema_version") != "sie.ar0234_ov9281.control_input_preview.v1" or
            audit.get("status") not in ("MASKED_DEPTH_SHADOW_DIAGNOSTIC_ONLY",
                                        "SHAPE_AWARE_SHADOW_DIAGNOSTIC_ONLY") or
            audit.get("calibration", {}).get("ar0234_intrinsic_sha256") != AR_INTRINSIC_SHA256 or
            audit.get("calibration", {}).get("ov9281_stereo_sha256") != ACTIVE_STEREO_SHA256 or
            audit.get("stereo_resolution", {}).get("mode") != "half_640x400"):
        return empty, "CYCLE_OR_CALIBRATION_INVALID"
    target = audit.get("ar0234_dynamic_target") or {}
    person = target.get("candidate") or {}
    seg = audit.get("segmentation_observation") or {}
    instances = seg.get("instances")
    mask_hashes = seg.get("instance_mask_sha256")
    shape = audit.get("shape_diagnostic") or {}
    mask = shape.get("mask") or {}
    depth = shape.get("mask_depth") or {}
    all_front = shape.get("front_geometry") or {}
    front = shape.get("candidate_layer_front_geometry") or {}
    stable_front = shape.get("stable_candidate_front_geometry") or {}
    confirmation = audit.get("shadow_track_confirmation") or {}
    continuity = audit.get("shadow_rgb_continuity") or {}
    v3 = audit.get("shadow_v3_mask_match") or {}
    confirmed=(confirmation.get("state") == "CONFIRMED_BY_CURRENT_RGB_STEREO_ASSOCIATION"
        and confirmation.get("confirmed_observation_id") ==
            f"segmentation-preview:{cid}:{person.get('instance_index')}"
        and bool(confirmation.get("track_id")))
    local_path=(local_preview and semantic_source == "segmentation_mask"
        and depth.get("status") == "DEPTH_MAD_EXCEEDED" and not confirmation
        and continuity.get("status") == "MATCHED_CURRENT_RGB_DIAGNOSTIC_ONLY"
        and isinstance(continuity.get("track_id"),str) and bool(continuity["track_id"]))
    if (target.get("disposition") != "OBSERVED" or
            target.get("current_eligible_count") != 1 or
            not (audit.get("target_geometry") or {}).get("accepted") or
            audit.get("dynamic_shadow_edge_override") or
            audit.get("dynamic_shadow_geometry_override") or
            seg.get("cycle_id") != cid or
            seg.get("status") != "CURRENT_INSTANCES_DIAGNOSTIC_ONLY" or
            not expected_segmentation_model_sha256 or
            seg.get("model_sha256") != expected_segmentation_model_sha256 or
            not isinstance(instances, list) or
            len(instances) != 1 or
            not isinstance(mask_hashes, list) or
            type(person.get("instance_index")) is not int or
            not 0 <= person["instance_index"] < len(instances) or
            len(mask_hashes) != len(instances) or
            instances[person["instance_index"]] != person or
            mask.get("status") != "CURRENT_FRAME_MASK_DIAGNOSTIC_ONLY" or
            mask.get("instance_index") != person["instance_index"] or
            mask.get("source_rgb_arrays_sha256") != seg.get("arrays_sha256") or
            not seg.get("arrays_sha256") or
            shape.get("cycle_id") != cid or
            shape.get("model_sha256") != seg.get("model_sha256") or
            not shape.get("mask_sha256") or
            shape.get("mask_sha256") != mask_hashes[person["instance_index"]] or
            not shape.get("arrays_sha256") or
            (semantic_source == "v3_mask_binding" and (
                audit.get("shadow_semantic_source") is not None or
                v3.get("status") != "BOUND_TO_CURRENT_MASK_INSTANCE" or
                v3.get("v3_model_sha256") != SHADOW_V3_SHA256 or
                v3.get("source_cycle_id") != cid or
                v3.get("source_rgb_arrays_sha256") != seg.get("arrays_sha256") or
                not v3.get("rgb_payload_sha256") or
                v3.get("rgb_payload_sha256") != seg.get("rgb_payload_sha256") or
                v3.get("mask_instance_index") != person["instance_index"])) or
            (semantic_source == "segmentation_mask" and (
                audit.get("shadow_semantic_source") != "COCO_CURRENT_INSTANCE_MASK" or
                audit.get("shadow_v3_mask_match") is not None)) or
            not (confirmed or local_path)):
        return empty, "CURRENT_RGB_MASK_TRACK_BINDING_INVALID"
    expected = {"source_cycle_id": cid, "mask_sha256": shape["mask_sha256"],
                "instance_index": person["instance_index"],
                "model_sha256": shape["model_sha256"],
                "rgb_arrays_sha256": seg["arrays_sha256"],
                "fusion_arrays_sha256": shape.get("arrays_sha256")}
    if (depth.get("status") != ("DEPTH_MAD_EXCEEDED" if local_path else
                                  "COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY") or
            depth.get("plausible_layer_count") != 1 or
            type(depth.get("candidate_sample_count")) is not int or
            depth["candidate_sample_count"] < 100 or
            all_front.get("provenance") != expected or
            all_front.get("profile_sha256") != expected_front_profile_sha256 or
            all_front.get("source_depth_status") != depth["status"] or
            all_front.get("status") != "PARTIAL_FRONT_GEOMETRY_DIAGNOSTIC_ONLY" or
            all_front.get("point_set") != "ALL_CURRENT_MASK_OWNED_STEREO_POINTS" or
            type(all_front.get("point_count")) is not int or
            all_front["point_count"] < depth["candidate_sample_count"]):
        return empty, "CURRENT_STEREO_FRONT_BINDING_INVALID"
    # This is the exact numerical group checked by the unchanged depth MAD.
    # It is still a diagnostic estimate, not verified physical ownership.
    if (front.get("provenance") != expected or
            front.get("profile_sha256") != expected_front_profile_sha256 or
            front.get("source_depth_status") != depth["status"] or
            front.get("status") != "PARTIAL_FRONT_GEOMETRY_DIAGNOSTIC_ONLY" or
            front.get("point_set") != "NUMERIC_CANDIDATE_DEPTH_LAYER_DIAGNOSTIC_ONLY" or
            front.get("semantic_ownership_verified") is not False or
            type(front.get("point_count")) is not int or
            front["point_count"] != depth["candidate_sample_count"]):
        return empty, "FRONT_LAYER_BINDING_UNVERIFIED"
    selected_front=front
    if local_path:
        patches=stable_front.get("stable_local_patches") or {}
        residual=patches.get("selected_held_out_residual") or {}
        count=stable_front.get("point_count")
        if (stable_front.get("provenance") != expected or
                stable_front.get("profile_sha256") != expected_front_profile_sha256 or
                stable_front.get("source_depth_status") != depth["status"] or
                stable_front.get("status") != "STABLE_CANDIDATE_FRONT_DIAGNOSTIC_ONLY" or
                stable_front.get("point_set") != "STABLE_LOCAL_PATCHES_NUMERIC_CANDIDATE_LAYER_DIAGNOSTIC_ONLY" or
                stable_front.get("semantic_ownership_verified") is not False or
                stable_front.get("actuator_access") is not False or
                stable_front.get("distance_from_front_m") is not None or
                stable_front.get("candidate_sample_count") != front["point_count"] or
                type(count) is not int or
                not STABLE_LOCAL_PATCH_POLICY["minimum_selected_points"] <= count <= front["point_count"] or
                patches.get("status") != "STABLE_LOCAL_PATCHES_DIAGNOSTIC_ONLY" or
                patches.get("policy") != STABLE_LOCAL_PATCH_POLICY or
                patches.get("selected_point_count") != count or
                type(patches.get("selected_patch_count")) is not int or
                patches["selected_patch_count"] < 1 or
                residual.get("sample_count") != count):
            return empty, "STABLE_LOCAL_FRONT_UNAVAILABLE"
        selected_front=stable_front
    frame = audit.get("frame_freshness") or {}
    stamps = frame.get("frames") or {}
    ages = frame.get("driver_declared_soe_to_result_s") or {}
    try:
        pair = [float(stamps[k]["driver_timestamp_s"]) for k in ("ar0234", "ov9281")]
        received = [float(stamps[k]["host_packet_received_monotonic_s"])
                    for k in ("ar0234", "ov9281")]
        flags = [stamps[k]["flags"] for k in ("ar0234", "ov9281")]
        measured = [float(ages[k]) for k in ("ar0234", "ov9281")]
        distance = float(selected_front["known_term_m"]["p05"]) - rim_forward_of_optical_center_m
        confidence = float(person["confidence"])
        bearing = float(audit["shadow_target_bearing_deg"])
        age = [now_s - t for t in pair]
    except (KeyError, TypeError, ValueError, OverflowError):
        return empty, "FAST_EVIDENCE_FIELD_MISSING"
    if (frame.get("capture_backend") != "latest_v4l2" or
            frame.get("endpoint") != "RESULT_READY_BEFORE_PUBLICATION; not a motor command" or
            not all(math.isfinite(v) for v in pair + received + measured + age +
                    [distance, confidence, bearing]) or
            any(type(v) is not int or v & 0xe000 != 0x2000 or
                v & 0x70000 != 0x10000 or v & 0x40 for v in flags) or
            any(not pair[i] <= received[i] <= now_s for i in range(2)) or
            any(not 0 <= v <= max_age_s for v in age) or
            any(not 0 <= measured[i] <= age[i] + .01 for i in range(2)) or
            abs(pair[0] - pair[1]) > .2 or
            distance <= 0 or not 0 <= confidence <= 1):
        return empty, "DRIVER_AGE_OR_GEOMETRY_INVALID"
    observed = min(pair)
    track_id = continuity["track_id"] if local_path else confirmation["track_id"]
    rgb = RgbTarget(track_id, observed, bearing, confidence, AR_INTRINSIC_SHA256)
    rng = TargetRange("OV9281", track_id, observed, distance, confidence,
                      FRONT_FRAME, ACTIVE_STEREO_SHA256,
                      f"{'fast-local-preview' if local_path else 'fast-mask'}:{cid}:{shape['mask_sha256']}")
    return ShadowObservation(rgb, (rng,)), (
        "CURRENT_STABLE_LOCAL_PREVIEW" if local_path else "CURRENT_FAST_MASK_OBSERVATION")
