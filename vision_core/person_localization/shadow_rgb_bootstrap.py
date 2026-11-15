"""Motor-off RGB identity bootstrap after a long target absence."""
from __future__ import annotations

import math

from vision_core.person_localization.shadow_rgb_continuity import continued_track
from vision_core.person_localization.torso_core_geometry import TORSO_CORE_POLICY


def advance_bootstrap(prior: dict | None, candidate: dict, stamp: str,
                      segmentation: dict, shape: dict,
                      *, minimum_iou: float,
                      maximum_center_displacement_relative: float) -> dict | None:
    """Count matching current masks with a fitted torso; never carry depth.

    A caller may seed a diagnostic identity after two consecutive observations.
    This does not confirm stereo association or emit a range on either cycle.
    """
    index = candidate.get("instance_index")
    masks = segmentation.get("instance_mask_sha256")
    cycle_id = segmentation.get("cycle_id")
    model_sha = segmentation.get("model_sha256")
    depth = shape.get("mask_depth") or {}
    torso = shape.get("stable_torso_core_front_geometry") or {}
    front = torso.get("known_term_m") or {}
    value = front.get("median")
    if (type(index) is not int or not isinstance(masks, list) or
            not 0 <= index < len(masks) or not masks[index] or
            not isinstance(cycle_id, str) or not cycle_id or
            not isinstance(model_sha, str) or not model_sha or
            shape.get("cycle_id") != cycle_id or
            shape.get("model_sha256") != model_sha or
            shape.get("mask_sha256") != masks[index] or
            depth.get("status") != "DEPTH_MAD_EXCEEDED" or
            depth.get("plausible_layer_count") != 1 or
            torso.get("status") != "STABLE_TORSO_CORE_FRONT_DIAGNOSTIC_ONLY" or
            torso.get("point_set") !=
                "STABLE_LOCAL_PATCHES_WITHIN_CURRENT_RGB_TORSO_CORE_DIAGNOSTIC_ONLY" or
            torso.get("source_depth_status") != depth["status"] or
            torso.get("torso_core_policy") != TORSO_CORE_POLICY or
            type(torso.get("point_count")) is not int or
            torso["point_count"] < TORSO_CORE_POLICY["minimum_stable_points"] or
            torso.get("semantic_ownership_verified") is not False or
            torso.get("actuator_access") is not False or
            type(value) not in (float, int) or not math.isfinite(value) or
            not .5 <= value <= 4.5):
        return None
    matched = continued_track(prior, candidate, stamp,
        minimum_iou=minimum_iou,
        maximum_center_displacement_relative=maximum_center_displacement_relative)
    return {"track_id": "DIAGNOSTIC_BOOTSTRAP_ONLY",
            "bbox_xyxy_px": list(candidate["bbox_xyxy_px"]),
            "last_rgb_at_utc": stamp,
            "count": min(2, (prior.get("count", 0) + 1) if matched else 1)}
