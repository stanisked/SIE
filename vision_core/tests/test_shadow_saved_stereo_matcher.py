"""Checks for paired offline matching and fail-closed mask projection."""

import os
import subprocess
import sys
from pathlib import Path
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from compare_shadow_saved_stereo_matcher import evaluate, lr_pair, matcher


class SavedStereoMatcherTest(unittest.TestCase):
    def test_missing_scipy_keeps_mask_depth_available(self):
        script = """
import builtins
original_import = builtins.__import__
def reject_scipy(name, *args, **kwargs):
    if name == 'scipy' or name.startswith('scipy.'):
        raise ModuleNotFoundError("No module named 'scipy'", name='scipy')
    return original_import(name, *args, **kwargs)
builtins.__import__ = reject_scipy
from compare_shadow_saved_stereo_matcher import evaluate
import numpy as np
cal = {'size': (256, 128), 'P2': np.array([[0., 0., 0., -75000.]])}
result, _ = evaluate(np.zeros((128, 256), dtype=np.float32),
                     np.zeros((128, 256), dtype=bool),
                     np.zeros((1200, 1920), dtype=np.uint8),
                     [0, 0, 1920, 1200], 4, cal, .5, 4.5)
assert result['mask_depth']['status'] == 'INSUFFICIENT_MASKED_DEPTH'
assert result['local_surface']['status'] == 'UNAVAILABLE_SCIPY_NOT_INSTALLED'
"""
        env = os.environ.copy()
        env["PYTHONPATH"] = os.pathsep.join((str(Path(__file__).resolve().parents[2]),
                                              str(Path(__file__).resolve().parents[1] / "tools"),
                                              env.get("PYTHONPATH", "")))
        run = subprocess.run([sys.executable, "-c", script], env=env,
                             capture_output=True, text=True, check=False)
        self.assertEqual(run.returncode, 0, run.stdout + run.stderr)

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
