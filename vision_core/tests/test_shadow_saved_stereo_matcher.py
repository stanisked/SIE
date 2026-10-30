"""Checks for paired offline matching and fail-closed mask projection."""

import sys
from pathlib import Path
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from compare_shadow_saved_stereo_matcher import evaluate, lr_pair, matcher


class SavedStereoMatcherTest(unittest.TestCase):
    def test_three_profiles_use_same_rectified_pair_and_lr_sign(self):
        rng = np.random.default_rng(3)
        left = rng.integers(0, 256, (128, 256), dtype=np.uint8)
        right = np.zeros_like(left)
        right[:, :-16] = left[:, 16:]
        original_left, original_right = left.copy(), right.copy()
        for profile in ("sgbm7", "sgbm11", "bm15"):
            with self.subTest(profile=profile):
                disparity, lr, _ = lr_pair(left, right, profile)
                self.assertGreater(int(lr.sum()), 1000)
                self.assertAlmostEqual(float(np.median(disparity[lr])), 16.0, delta=.2)
        np.testing.assert_array_equal(left, original_left)
        np.testing.assert_array_equal(right, original_right)
        with self.assertRaises(ValueError):
            matcher("unreviewed_profile", 0)

    def test_projection_rejects_incompatible_calibration_shape(self):
        disparity = np.full((128, 256), 16.0, dtype=np.float32)
        mask = np.full((1200, 1920), 255, dtype=np.uint8)
        with self.assertRaisesRegex(ValueError, "shape/calibration mismatch"):
            evaluate(disparity, disparity > 0, mask, [0, 0, 1920, 1200], 4,
                     {"size": (1280, 800)}, .5, 4.5)


if __name__ == "__main__":
    unittest.main()
