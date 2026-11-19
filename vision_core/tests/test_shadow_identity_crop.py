"""Synthetic source-frame and privacy tests: no personal images or hardware."""
import copy
import unittest

import numpy as np

from vision_core.person_localization.shadow_identity_crop import (
    pack_selected_identity_source,unpack_selected_identity_source)


class IdentityCropTests(unittest.TestCase):
    def setUp(self):
        self.rgb=np.zeros((1200,1920,3),dtype=np.uint8)
        self.rgb[200:1000,300:700]=(40,80,180)
        self.mask=np.zeros((1200,1920),dtype=bool)
        self.mask[240:960,340:660]=True
        self.bbox=[300.,200.,700.,1000.]
        self.kw=dict(cycle_id="ar0234-ov9281-control-preview-a"*1+"-00000001",
                     instance_index=0,
                     rgb_arrays_sha256="b"*64,
                     segmentation_model_sha256="c"*64)

    def make(self):
        return pack_selected_identity_source(self.rgb,self.mask,self.bbox,
                                             **self.kw)

    def test_roundtrip_has_exact_mask_and_crop_not_full_frame(self):
        payload=self.make()
        self.assertEqual(payload["instance_index"],0)
        self.assertEqual(payload["crop_shape_hw"],[800,400])
        self.assertEqual(payload["crop_origin_xy_px"],[300,200])
        self.assertEqual(payload["scope"],
                         "MOTOR_OFF_SOURCE_CROP_NOT_IDENTITY_PROOF")
        out,mask=unpack_selected_identity_source(payload)
        self.assertEqual(out.shape,(800,400,3))
        self.assertTrue(np.array_equal(mask,self.mask[200:1000,300:700]))
        self.assertNotIn("raw_face_embedding",payload)

    def test_mutated_jpeg_is_rejected(self):
        payload=self.make()
        payload["crop_jpeg_sha256"]="d"*64
        with self.assertRaisesRegex(ValueError,"HASH_OR_SIZE"):
            unpack_selected_identity_source(payload)

    def test_mutated_mask_is_rejected(self):
        payload=self.make()
        payload["crop_mask_bits_sha256"]="0"*64
        with self.assertRaisesRegex(ValueError,"HASH_OR_SIZE"):
            unpack_selected_identity_source(payload)

    def test_invalid_bbox_or_missing_pixels_rejected(self):
        with self.assertRaisesRegex(ValueError,"BBOX"):
            pack_selected_identity_source(
                self.rgb,self.mask,[400,200,300,1000],**self.kw)
        with self.assertRaisesRegex(ValueError,"INSUFFICIENT_PIXELS"):
            pack_selected_identity_source(
                self.rgb,np.zeros_like(self.mask),self.bbox,**self.kw)

    def test_full_resolution_binding_is_reported(self):
        payload=self.make()
        import hashlib
        self.assertEqual(payload["full_instance_mask_sha256"],
                         hashlib.sha256(self.mask.tobytes()).hexdigest())
        self.assertEqual(payload["rgb_arrays_sha256"],"b"*64)


if __name__=="__main__":
    unittest.main()
