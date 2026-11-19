"""Synthetic-only tests; no RGB samples of real people committed."""
import unittest

import numpy as np

from vision_core.person_localization.shadow_appearance_signature import (
    clothing_signature, clothing_distance)


class AppearanceColorTests(unittest.TestCase):
    def setUp(self):
        self.bbox = (20, 20, 180, 180)
        self.mask = np.zeros((200, 200), dtype=bool)
        self.mask[20:180, 20:180] = True

    def frame(self, shirt=(40, 20, 20), background=(80, 90, 100)):
        image = np.zeros((200, 200, 3), dtype=np.uint8)
        image[:] = background
        image[20:180, 20:180] = shirt
        return image

    def test_background_and_face_do_not_change_torso_color(self):
        a = self.frame(background=(255, 255, 0))
        b = self.frame(background=(0, 255, 255))
        a[20:48, 20:180] = (210, 180, 140)
        b[20:48, 20:180] = (100, 200, 220)
        self.assertAlmostEqual(clothing_distance(
            clothing_signature(a,self.mask,self.bbox),
            clothing_signature(b,self.mask,self.bbox)),0.0,places=7)

    def test_different_shirt_colors_have_higher_distance(self):
        a = clothing_signature(self.frame((50,30,20)),self.mask,self.bbox)
        b = clothing_signature(self.frame((50,30,20)),self.mask,self.bbox)
        c = clothing_signature(self.frame((30,20,220)),self.mask,self.bbox)
        self.assertLess(clothing_distance(a,b), clothing_distance(a,c))

    def test_refuse_missing_current_mask(self):
        with self.assertRaisesRegex(ValueError,"INSUFFICIENT_CURRENT"):
            clothing_signature(self.frame(),np.zeros_like(self.mask),self.bbox)

    def test_refuse_wrong_shapes(self):
        with self.assertRaisesRegex(ValueError,"INVALID_CURRENT"):
            clothing_signature(self.frame(),self.mask[:10],self.bbox)


if __name__ == "__main__":
    unittest.main()
