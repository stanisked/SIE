"""Check geometry, candidate membership and signed color semantics for replay plots."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "vision_core/tools"))
from visualize_shadow_depth_regions import (band_statistics, candidate_indices, extract_points,
                                            render, residual_colors)
from compare_shadow_mask_depth_median import project
from vision_core.person_localization.instance_mask_stereo_fusion import masked_depth_hypothesis


def fixture(base: Path) -> tuple[Path, dict, dict]:
    cid = "synthetic-00000003"
    disparity = np.full((64, 64), 75.95418816496264 / 2.24, dtype=np.float32)
    disparity[32:] = 75.95418816496264 / 2.36
    np.save(base / "disparity_lr_consistent_px.npy", disparity)
    mask = np.zeros((1200, 1920), dtype=np.uint8)
    mask[560:640, 920:1000] = 255
    cv2.imwrite(str(base / f"{cid}.png"), mask)
    rgb = np.full((1200, 1920, 3), 180, dtype=np.uint8)
    rgb[560:600, 920:1000] = (100, 120, 90)
    rgb[600:640, 920:1000] = (100, 100, 110)
    cv2.imwrite(str(base / "ar0234_source.jpg"), rgb)
    p1 = np.array([[100., 0, 32, 0], [0, 100, 32, 0], [0, 0, 1, 0]])
    p2 = p1.copy()
    p2[0, 3] = -75954.18816496264
    cal = {"P1": p1, "P2": p2, "R1": np.eye(3), "R": np.eye(3), "T": np.zeros(3),
           "K": np.array([[100., 0, 960], [0, 100, 600], [0, 0, 1]]), "D": np.zeros(5)}
    v, u = np.mgrid[0:64:4, 0:64:4]
    z = abs(float(p2[0, 3])) / disparity[::4, ::4] / 1000.
    xy = project(u.flatten().astype(float), v.flatten().astype(float), z.flatten(), cal)
    hypothesis = masked_depth_hypothesis(xy, z.flatten(), mask == 255)
    raw = {"cycle_id": cid, "mask_depth": hypothesis}
    path = base / "audit.json"
    path.write_text(json.dumps({"cycle_id": cid, "stereo_geometry": {
        "sample_stride_px": 4, "ar_inner_roi_xyxy_px": [900, 550, 1030, 650]}}))
    return path, raw, cal


class DepthRegionTests(unittest.TestCase):
    def test_signed_shared_color_scale(self):
        actual = residual_colors(np.array([-.3, -.2, 0, .2, .3]))
        np.testing.assert_array_equal(actual, [[255, 0, 0], [255, 0, 0], [255, 255, 255],
                                               [0, 0, 255], [0, 0, 255]])

    def test_only_supported_candidate_layer_is_colored(self):
        z = np.r_[np.full(200, 2.3), np.full(10, 4.0)]
        indices = candidate_indices(z, {"candidate_sample_count": 200,
                                       "candidate_depth_m": 2.3, "candidate_mad_m": 0.0})
        self.assertEqual(len(indices), 200)
        self.assertTrue((indices < 200).all())
        with self.assertRaises(ValueError):
            candidate_indices(np.r_[np.full(200, 2.3), np.full(200, 4.)], {})

    def test_band_boundaries_have_no_duplicate_points(self):
        xy = np.array([[10, 0], [10, 1], [10, 2], [10, 3]], dtype=float)
        bands = band_statistics(xy, np.array([2., 2.1, 2.2, 2.3]), [0, 0, 20, 3], 2.15)
        self.assertEqual([b["sample_count"] for b in bands], [1, 1, 2])
        self.assertEqual(sum(b["sample_count"] for b in bands), 4)

    def test_extract_render_and_source_preservation(self):
        with tempfile.TemporaryDirectory() as name:
            base = Path(name)
            path, raw, cal = fixture(base)
            originals = {p: p.read_bytes() for p in base.iterdir()}
            xy, z, indices, _ = extract_points(path, cal, .5, 4.5, raw)
            self.assertEqual(len(z), 256)
            self.assertEqual(len(indices), raw["mask_depth"]["candidate_sample_count"])
            self.assertTrue((xy[:128, 1] < 600).all())
            self.assertTrue((xy[128:, 1] >= 600).all())
            image, report = render(path, raw, cal, .5, 4.5)
            self.assertEqual(image.shape, (810, 1040, 3))
            self.assertEqual(sum(b["sample_count"] for b in report["image_height_bands"]), 256)
            self.assertLess(report["image_height_bands"][0]["median_minus_global_m"], 0)
            self.assertGreater(report["image_height_bands"][-1]["median_minus_global_m"], 0)
            self.assertEqual(originals, {p: p.read_bytes() for p in base.iterdir()})


if __name__ == "__main__":
    unittest.main()
