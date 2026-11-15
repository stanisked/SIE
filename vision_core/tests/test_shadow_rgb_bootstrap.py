"""Reacquisition uses two exact current masks before assigning a diagnostic ID."""
import unittest

from vision_core.person_localization.shadow_rgb_bootstrap import advance_bootstrap
from vision_core.person_localization.shadow_rgb_continuity import (
    continued_track, remember_track,
)
from vision_core.person_localization.torso_core_geometry import TORSO_CORE_POLICY


def frame(cycle, box):
    candidate = {"instance_index": 0, "bbox_xyxy_px": box, "confidence": .9}
    seg = {"cycle_id": cycle, "model_sha256": "model", "instance_mask_sha256": [cycle]}
    shape = {"cycle_id": cycle, "model_sha256": "model", "mask_sha256": cycle,
             "mask_depth": {"status": "DEPTH_MAD_EXCEEDED", "plausible_layer_count": 1},
             "stable_torso_core_front_geometry": {
                 "status": "STABLE_TORSO_CORE_FRONT_DIAGNOSTIC_ONLY",
                 "point_set": "STABLE_LOCAL_PATCHES_WITHIN_CURRENT_RGB_TORSO_CORE_DIAGNOSTIC_ONLY",
                 "source_depth_status": "DEPTH_MAD_EXCEEDED",
                 "torso_core_policy": TORSO_CORE_POLICY,
                 "point_count": 870, "known_term_m": {"median": 2.1},
                 "semantic_ownership_verified": False, "actuator_access": False}}
    return candidate, seg, shape


class ShadowRgbBootstrapTests(unittest.TestCase):
    def advance(self, prior, cycle, box, stamp, *, tamper=None):
        candidate, seg, shape = frame(cycle, box)
        if tamper:
            tamper(seg, shape)
        return advance_bootstrap(prior, candidate, stamp, seg, shape,
            minimum_iou=.2, maximum_center_displacement_relative=.75)

    def test_two_current_masks_seed_only_next_frame_identity(self):
        first = self.advance(None, "one", [400, 30, 800, 1190],
                             "2026-11-15T10:00:00+00:00")
        self.assertEqual(first["count"], 1)
        second = self.advance(first, "two", [420, 25, 820, 1190],
                              "2026-11-15T10:00:00.3+00:00")
        self.assertEqual(second["count"], 2)
        prior = remember_track("person-2", {"bbox_xyxy_px": second["bbox_xyxy_px"]},
                               second["last_rgb_at_utc"])
        current = continued_track(prior, {"bbox_xyxy_px": [440, 20, 840, 1190]},
            "2026-11-15T10:00:00.6+00:00", minimum_iou=.2,
            maximum_center_displacement_relative=.75)
        self.assertEqual(current["track_id"], "person-2")

    def test_stale_unrelated_or_wrong_source_resets_bootstrap(self):
        first = self.advance(None, "one", [400, 30, 800, 1190],
                             "2026-11-15T10:00:00+00:00")
        self.assertEqual(self.advance(first, "two", [1500, 30, 1800, 1190],
                         "2026-11-15T10:00:00.3+00:00")["count"], 1)
        self.assertEqual(self.advance(first, "three", [420, 30, 820, 1190],
                         "2026-11-15T10:00:00.9+00:00")["count"], 1)
        self.assertIsNone(self.advance(first, "four", [420, 30, 820, 1190],
            "2026-11-15T10:00:00.3+00:00",
            tamper=lambda seg, shape: shape.update(mask_sha256="different")))
        self.assertIsNone(self.advance(first, "five", [420, 30, 820, 1190],
            "2026-11-15T10:00:00.3+00:00",
            tamper=lambda seg, shape: shape["stable_torso_core_front_geometry"].update(point_count=12)))


if __name__ == "__main__":
    unittest.main()
