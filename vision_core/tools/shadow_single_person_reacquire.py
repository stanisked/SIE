"""Conservative, experimental SINGLE-PERSON scene reacquisition gate.

This is NOT person re-identification. It assumes one human is present in a
supervised, bounded test. A new camera track is only a candidate after loss.
No range or identity is inherited from the missing track; every candidate
pair must be independently fresh and valid before the motion runner calls us.
"""
from __future__ import annotations

import math


MAX_LOSS_S = 3.0
MAX_BEARING_FROM_LAST_DEG = 15.0
MAX_DISTANCE_FROM_LAST_M = 0.75
MAX_PAIR_GAP_S = 0.75
MAX_BEARING_BETWEEN_PAIRS_DEG = 12.0
MAX_DISTANCE_BETWEEN_PAIRS_M = 0.45


class SinglePersonReacquireGate:
    """Require two distinct, current, single-instance RGB/stereo pairs."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.track_id = None
        self.rgb_s = None
        self.range_s = None
        self.bearing_deg = None
        self.distance_m = None

    def consider(self, *, target, selected, single_person_rgb: bool,
                 old_track_id: str, last_verified: tuple | None,
                 last_bearing_deg: float | None, now_s: float):
        """Return (WAIT|MATCH|REFUSE, reason), never a motion command."""
        if (last_verified is None or last_bearing_deg is None or
                last_verified[0] != old_track_id or
                not all(math.isfinite(v) for v in
                        (last_verified[1], last_verified[2],
                         last_bearing_deg, now_s)) or
                not 0 <= now_s - last_verified[1] <= MAX_LOSS_S):
            self.reset()
            return "REFUSE", "LAST_CONFIRMED_PAIR_EXPIRED"
        if not single_person_rgb:
            self.reset()
            return "WAIT", "MULTIPLE_OR_UNVERIFIED_RGB_INSTANCES"
        if (target is None or selected is None or
                not target.track_id or target.track_id == old_track_id or
                selected.target_track_id != target.track_id or
                not all(math.isfinite(v) for v in
                        (target.observed_s, target.bearing_deg,
                         selected.observed_s, selected.distance_m)) or
                not 0 <= now_s - target.observed_s <= .5 or
                not 0 <= now_s - selected.observed_s <= .5 or
                abs(target.observed_s - selected.observed_s) > .2):
            self.reset()
            return "WAIT", "CURRENT_RGB_DEPTH_PAIR_UNAVAILABLE"
        if (abs(target.bearing_deg - last_bearing_deg) >
                MAX_BEARING_FROM_LAST_DEG or
                abs(selected.distance_m - last_verified[2]) >
                MAX_DISTANCE_FROM_LAST_M):
            self.reset()
            return "REFUSE", "POSITION_INCOMPATIBLE_WITH_LOST_TARGET"
        if (self.track_id != target.track_id or self.rgb_s is None or
                target.observed_s - self.rgb_s > MAX_PAIR_GAP_S):
            self.track_id = target.track_id
            self.rgb_s = target.observed_s
            self.range_s = selected.observed_s
            self.bearing_deg = target.bearing_deg
            self.distance_m = selected.distance_m
            return "WAIT", "FIRST_FRESH_CANDIDATE_PAIR"
        if (target.observed_s <= self.rgb_s or
                selected.observed_s <= self.range_s):
            return "WAIT", "DISTINCT_NEW_PAIR_REQUIRED"
        if (abs(target.bearing_deg - self.bearing_deg) >
                MAX_BEARING_BETWEEN_PAIRS_DEG or
                abs(selected.distance_m - self.distance_m) >
                MAX_DISTANCE_BETWEEN_PAIRS_M):
            self.reset()
            return "REFUSE", "CANDIDATE_UNSTABLE_BETWEEN_FRAMES"
        self.reset()
        return "MATCH", "TWO_CONSISTENT_FRESH_SINGLE_PERSON_PAIRS"
