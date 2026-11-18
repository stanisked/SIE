"""Short current-frame RGB continuity for the single-person Shadow preview."""
from __future__ import annotations

from datetime import datetime
import math


MAX_RGB_FRAME_GAP_S = .75
MAX_RGB_REACQUIRE_GAP_S = 1.50

def retain_current_rgb_on_depth_refusal(*, rgb_matched_current: bool,
                                        audit_status: str,
                                        depth_status: str) -> bool:
    """Keep a *previously confirmed* RGB identity across a depth-only failure.

    The caller must establish a unique, fresh and geometrically matched RGB
    candidate via continued_track. This does not publish depth, confirm a
    new track, or authorize motion. A future usable depth must be fresh.
    """
    return (rgb_matched_current is True
            and audit_status == "MASKED_DEPTH_SHADOW_DIAGNOSTIC_ONLY"
            and depth_status in (
                "DEPTH_MAD_EXCEEDED",
                "MULTIPLE_OR_NO_PLAUSIBLE_DEPTH_LAYERS",
            ))



def retain_short_gap(status: str, reason: str, prior: dict | None) -> bool:
    """Hold identity during a missed frame, without publishing a target.

    The next detected frame must still pass continued_track's age and box
    match before the held ID can be used again.
    """
    return (isinstance(prior, dict) and bool(prior.get("track_id")) and
            status == "TARGET_UNAVAILABLE" and
            reason in ("NO_CURRENT_NON_EDGE_TARGET",
                       "BBOX_TOO_NARROW_FOR_UPPER_BODY",
                       "BBOX_NEAR_RIGHT_IMAGE_EDGE",
                       "BBOX_NEAR_LEFT_IMAGE_EDGE"))


def continued_track(prior: dict | None, candidate: dict, captured_at_utc: str,
                    *, minimum_iou: float, maximum_center_displacement_relative: float,
                    maximum_gap_s: float = MAX_RGB_FRAME_GAP_S
                    ) -> dict | None:
    """Carry a confirmed ID only across a fresh, geometrically matching RGB frame.

    The caller must separately require exactly one eligible current instance.
    This result never confirms depth or authorizes an action.
    """
    if not isinstance(prior, dict) or not prior.get("track_id"):
        return None
    try:
        age = (datetime.fromisoformat(captured_at_utc) -
               datetime.fromisoformat(prior["last_rgb_at_utc"])).total_seconds()
        previous = [float(v) for v in prior["bbox_xyxy_px"]]
        current = [float(v) for v in candidate["bbox_xyxy_px"]]
        if (not 0 <= age <= maximum_gap_s or len(previous) != 4 or
                len(current) != 4 or
                not all(math.isfinite(v) for v in previous + current) or
                not 0 <= minimum_iou <= 1 or
                maximum_center_displacement_relative <= 0):
            return None
        def area(box):
            return (box[2] - box[0]) * (box[3] - box[1])
        if area(previous) <= 0 or area(current) <= 0:
            return None
        intersection = (max(0., min(previous[2], current[2]) - max(previous[0], current[0])) *
                        max(0., min(previous[3], current[3]) - max(previous[1], current[1])))
        iou = intersection / (area(previous) + area(current) - intersection)
        diagonal = math.hypot(previous[2] - previous[0], previous[3] - previous[1])
        shift = math.hypot((current[0] + current[2] - previous[0] - previous[2]) / 2,
                           (current[1] + current[3] - previous[1] - previous[3]) / 2)
        if iou < minimum_iou or shift / diagonal > maximum_center_displacement_relative:
            return None
        return {"track_id": prior["track_id"], "bbox_xyxy_px": current,
                "last_rgb_at_utc": captured_at_utc}
    except (KeyError, TypeError, ValueError, OverflowError):
        return None



def reacquire_held_rgb_track(prior: dict | None, candidate: dict,
                             captured_at_utc: str, *,
                             minimum_confidence: float) -> dict | None:
    """Identity-only reacquisition after a short missed-detection gap.

    Caller must verify exactly one eligible current RGB instance and valid
    non-edge geometry. This function never authorizes motion or reuses depth.
    """
    if not isinstance(prior, dict) or not isinstance(candidate, dict):
        return None
    confidence = candidate.get("confidence")
    if (type(confidence) not in (int, float)
            or not math.isfinite(confidence)
            or not 0 < minimum_confidence <= 1
            or confidence < minimum_confidence):
        return None
    try:
        age = (datetime.fromisoformat(captured_at_utc) -
               datetime.fromisoformat(prior["last_rgb_at_utc"])).total_seconds()
    except (KeyError, TypeError, ValueError, OverflowError):
        return None
    if not MAX_RGB_FRAME_GAP_S < age <= MAX_RGB_REACQUIRE_GAP_S:
        return None
    return continued_track(
        prior, candidate, captured_at_utc,
        minimum_iou=.60,
        maximum_center_displacement_relative=.20,
        maximum_gap_s=MAX_RGB_REACQUIRE_GAP_S,
    )


def remember_track(track_id: str, candidate: dict, captured_at_utc: str) -> dict:
    return {"track_id": track_id,
            "bbox_xyxy_px": [float(v) for v in candidate["bbox_xyxy_px"]],
            "last_rgb_at_utc": captured_at_utc}


def match_current_edge_track(prior: dict | None, instances: list,
                             captured_at_utc: str, *, refusal_reason: str,
                             minimum_confidence: float,
                             minimum_iou: float,
                             maximum_center_displacement_relative: float
                             ) -> tuple[dict, dict] | None:
    """Match the same, uniquely visible person at a horizontal image edge.

    A validated RGB-only result cannot publish a stereo range or authorize
    motion. Never accept another person, an ambiguous frame or a stale box.
    """
    if (refusal_reason not in ("NO_CURRENT_NON_EDGE_TARGET",
                               "BBOX_NEAR_RIGHT_IMAGE_EDGE",
                               "BBOX_NEAR_LEFT_IMAGE_EDGE")
            or not isinstance(instances, list) or len(instances) != 1
            or not isinstance(instances[0], dict)):
        return None
    person = instances[0]
    box = person.get("bbox_xyxy_px")
    confidence = person.get("confidence")
    if (not isinstance(box, list) or len(box) != 4
            or not all(type(x) in (float, int) and math.isfinite(x) for x in box)
            or not 0 <= box[0] < box[2] <= 1920
            or not 0 <= box[1] < box[3] <= 1200
            or not type(confidence) in (int, float)
            or not math.isfinite(confidence)
            or confidence < minimum_confidence
            or type(person.get("instance_index")) is not int
            or box[2] - box[0] < 30 or box[3] - box[1] < 60):
        return None
    if refusal_reason == "BBOX_NEAR_RIGHT_IMAGE_EDGE" and box[2] < 1900:
        return None
    if refusal_reason == "BBOX_NEAR_LEFT_IMAGE_EDGE" and box[0] > 20:
        return None
    if refusal_reason == "NO_CURRENT_NON_EDGE_TARGET" and not (
            (person.get("truncated_right") is True and box[2] >= 1900) or
            (person.get("truncated_left") is True and box[0] <= 20)):
        return None
    matched = continued_track(prior, person, captured_at_utc,
        minimum_iou=minimum_iou,
        maximum_center_displacement_relative=maximum_center_displacement_relative)
    return (matched, person) if matched is not None else None
