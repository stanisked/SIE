"""Shadow's sensor-independent target range gate (diagnostic, no motor I/O).

The producer must associate each range to the RGB person's stable track ID.
This module never infers identity from the nearest LiDAR return or from a
bounding box after track loss. Distances use the cart's front edge as origin.
"""
from __future__ import annotations

from dataclasses import dataclass
import math


ACTIVE_STEREO_SHA256 = "39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52"
SOURCES = frozenset({"OV9281", "LIDAR"})
FRONT_FRAME = "cart_front_to_person_surface"


@dataclass(frozen=True)
class RgbTarget:
    track_id: str
    observed_s: float
    bearing_deg: float
    confidence: float
    calibration_id: str


@dataclass(frozen=True)
class TargetRange:
    source: str
    target_track_id: str
    observed_s: float
    distance_m: float
    confidence: float
    reference_frame: str
    calibration_id: str
    association_evidence_id: str


@dataclass(frozen=True)
class RangeDecision:
    action: str                 # PAUSE, HOLD, or FOLLOW_ELIGIBLE
    reason: str
    selected: TargetRange | None = None


def select_target_range(target: RgbTarget | None, ranges: list[TargetRange],
                        now_s: float, *, max_age_s: float = .5,
                        max_pair_skew_s: float = .2,
                        minimum_confidence: float = .8,
                        stop_distance_m: float = .8,
                        resume_margin_m: float = .15,
                        expected_rgb_calibration_id: str,
                        expected_lidar_calibration_id: str | None = None) -> RangeDecision:
    """Admit a fresh range for the same current RGB target, regardless of sensor.

    FOLLOW_ELIGIBLE is a perception decision, never a motion command. The
    caller must separately prove obstacle clearance, motor state, and protocol
    support before either MOVE or RESUME. A source change is logged in selected.
    """
    numbers = (now_s, max_age_s, max_pair_skew_s, minimum_confidence,
               stop_distance_m, resume_margin_m)
    if not all(type(x) in (int, float) and math.isfinite(x) for x in numbers):
        raise ValueError("INVALID_RANGE_POLICY")
    if (max_age_s <= 0 or max_pair_skew_s < 0 or not 0 < minimum_confidence <= 1
            or stop_distance_m <= 0 or resume_margin_m < 0):
        raise ValueError("INVALID_RANGE_POLICY")
    if target is None:
        return RangeDecision("PAUSE", "RGB_TARGET_UNAVAILABLE")
    if (not target.track_id or target.calibration_id != expected_rgb_calibration_id
            or not math.isfinite(target.observed_s)
            or not math.isfinite(target.bearing_deg)
            or not math.isfinite(target.confidence)
            or not 0 <= now_s - target.observed_s <= max_age_s
            or target.confidence < minimum_confidence):
        return RangeDecision("PAUSE", "RGB_TARGET_INVALID_OR_STALE")
    eligible = []
    for item in ranges:
        expected_calibration = (ACTIVE_STEREO_SHA256 if item.source == "OV9281"
                                else expected_lidar_calibration_id)
        if (item.source not in SOURCES or not expected_calibration
                or item.calibration_id != expected_calibration
                or item.target_track_id != target.track_id
                or not item.association_evidence_id
                or item.reference_frame != FRONT_FRAME
                or not math.isfinite(item.observed_s)
                or not math.isfinite(item.distance_m)
                or not math.isfinite(item.confidence)
                or not 0 <= now_s - item.observed_s <= max_age_s
                or abs(item.observed_s - target.observed_s) > max_pair_skew_s
                or item.distance_m <= 0
                or item.confidence < minimum_confidence):
            continue
        eligible.append(item)
    if not eligible:
        return RangeDecision("PAUSE", "NO_FRESH_ASSOCIATED_RANGE")
    # Prefer the newest observation. On equal timestamps use the nearer range;
    # this is conservative at the stopping boundary and deterministic.
    selected = max(eligible, key=lambda r: (r.observed_s, -r.distance_m,
                                             r.source, r.association_evidence_id))
    # Conflicting fresh sensors need a new observation instead of silent
    # failover to whichever happens to be newer.
    if any(abs(item.distance_m - selected.distance_m) > resume_margin_m
           for item in eligible):
        return RangeDecision("PAUSE", "RANGE_SOURCES_DISAGREE")
    if selected.distance_m <= stop_distance_m:
        return RangeDecision("HOLD", "TARGET_WITHIN_STOP_DISTANCE", selected)
    if selected.distance_m <= stop_distance_m + resume_margin_m:
        return RangeDecision("HOLD", "WITHIN_RESUME_MARGIN", selected)
    return RangeDecision("FOLLOW_ELIGIBLE", "FRESH_ASSOCIATED_RANGE", selected)
