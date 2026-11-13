"""Short current-frame RGB continuity for the single-person Shadow preview."""
from __future__ import annotations

from datetime import datetime
import math


MAX_RGB_FRAME_GAP_S = .5


def continued_track(prior: dict | None, candidate: dict, captured_at_utc: str,
                    *, minimum_iou: float, maximum_center_displacement_relative: float
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
        if (not 0 <= age <= MAX_RGB_FRAME_GAP_S or len(previous) != 4 or
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


def remember_track(track_id: str, candidate: dict, captured_at_utc: str) -> dict:
    return {"track_id": track_id,
            "bbox_xyxy_px": [float(v) for v in candidate["bbox_xyxy_px"]],
            "last_rgb_at_utc": captured_at_utc}
