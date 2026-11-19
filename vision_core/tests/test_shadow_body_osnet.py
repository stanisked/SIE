"""Pure synthetic unit tests for motor-off Body ReID. No person imagery."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from vision_core.person_localization.shadow_body_osnet import (
    FixedBatchOsnet, OSNET_SHA256, SUPPORTED_CROPS, body_input_tensor,
    cosine_pairs, normalize_embeddings, similarity_summary,
)


class ModelTensorTests(unittest.TestCase):
    def setUp(self):
        self.rgb=np.zeros((240,160,3),dtype=np.uint8)
        self.rgb[:]=(10,30,200)
        self.rgb[:70,:]=(220,90,20)
        self.mask=np.zeros((240,160),dtype=bool)
        self.mask[15:225,20:140]=True
        self.bbox=(20,15,140,225)

    def test_valid_rgb_and_mask_variants(self):
        tensors=[body_input_tensor(self.rgb,self.mask,self.bbox,
                                   variant=variant)
                 for variant in SUPPORTED_CROPS]
        for array in tensors:
            self.assertEqual(array.shape,(3,256,128))
            self.assertEqual(array.dtype,np.float32)
            self.assertTrue(np.isfinite(array).all())
        self.assertFalse(np.allclose(tensors[0],tensors[2]))

    def test_neutral_background_changes_only_masked_out_pixels(self):
        mask=self.mask.copy()
        mask[:,20:43]=False
        clean=body_input_tensor(self.rgb,mask,self.bbox,variant="bbox_rgb")
        neutral=body_input_tensor(self.rgb,mask,self.bbox,
                                  variant="mask_neutral_background")
        self.assertFalse(np.allclose(clean,neutral))

    def test_reject_wrong_mask_and_bounding_box(self):
        with self.assertRaisesRegex(ValueError,"INVALID_CURRENT_RGB_BODY_MASK"):
            body_input_tensor(self.rgb,self.mask[:20],self.bbox)
        with self.assertRaisesRegex(ValueError,"BODY_MASK_INSUFFICIENT_PIXELS"):
            body_input_tensor(self.rgb,np.zeros_like(self.mask),self.bbox)
        with self.assertRaisesRegex(ValueError,"BODY_CROP_TOO_SMALL"):
            body_input_tensor(self.rgb,self.mask,(0,0,5,5))
        with self.assertRaisesRegex(ValueError,"INVALID_CURRENT_RGB_BODY_BBOX"):
            body_input_tensor(self.rgb,self.mask,(10,20,10,100))

    def test_normalization_rejects_zero_or_nan(self):
        with self.assertRaisesRegex(ValueError,"ZERO_NORM"):
            normalize_embeddings(np.zeros((16,512),dtype=np.float32))
        data=np.ones((16,512),dtype=np.float32)
        data[0,0]=np.nan
        with self.assertRaisesRegex(ValueError,"FINI"):
            normalize_embeddings(data)


class FakeSpec:
    def __init__(self,name,shape):
        self.name=name
        self.shape=shape
        self.type="tensor(float)"


class FakeOrt:
    def __init__(self):
        self.calls=[]
    def get_inputs(self):
        return [FakeSpec("input",[16,3,256,128])]
    def get_outputs(self):
        return [FakeSpec("output",[16,512])]
    def run(self,names,feeds):
        batch=feeds["input"]
        assert batch.shape==(16,3,256,128)
        self.calls.append(batch.copy())
        embeddings=np.ones((16,512),dtype=np.float32)
        embeddings[:,0]=batch[:,0,0,0]+2.
        return [embeddings]


class Fixed16BatchTests(unittest.TestCase):
    def prepare(self,fake):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/"mock.onnx"
            path.write_bytes(b"fake-onnx-only-for-unit-tests")
            with patch(
                "vision_core.person_localization.shadow_body_osnet.file_sha256",
                return_value=OSNET_SHA256):
                model=FixedBatchOsnet(path,session_factory=lambda _:fake)
            return model

    def test_seven_samples_use_16_input_not_16_evidence(self):
        fake=FakeOrt()
        model=self.prepare(fake)
        inputs=np.ones((7,3,256,128),dtype=np.float32)
        embeddings=model.encode(inputs)
        self.assertEqual(len(fake.calls),1)
        self.assertEqual(embeddings.shape,(7,512))
        self.assertEqual(model.inference_calls,1)
        self.assertTrue(np.allclose(np.linalg.norm(embeddings,axis=1),1.))

    def test_multiple_batches_trim_padding(self):
        fake=FakeOrt()
        model=self.prepare(fake)
        inputs=np.ones((17,3,256,128),dtype=np.float32)
        embeddings=model.encode(inputs)
        self.assertEqual(len(fake.calls),2)
        self.assertEqual(embeddings.shape,(17,512))
        self.assertEqual(len(model.inference_times_ms),2)

    def test_reject_wrong_hash(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/"mock.onnx"
            path.write_bytes(b"wrong-checksum")
            with self.assertRaisesRegex(ValueError,"SHA256_MISMATCH"):
                FixedBatchOsnet(path,session_factory=lambda _:FakeOrt())

    def test_same_cross_and_untracked_groups_not_person_labels(self):
        data=np.zeros((4,512),dtype=np.float32)
        data[0,0]=data[1,0]=data[2,0]=1.
        data[3,1]=1.
        pairs=cosine_pairs(data,[1,10,60,70],
                          [":1",":1",None,":3"])
        groups=similarity_summary(pairs)
        self.assertEqual(groups["SAME_RGB_TRACK"]["pairs"],1)
        self.assertEqual(groups["DIFFERENT_RGB_TRACK"]["pairs"],2)
        self.assertEqual(groups["UNTRACKED_INVOLVED"]["pairs"],3)
        self.assertEqual(groups["DIFFERENT_RGB_TRACK"]["max"],0.)


if __name__=="__main__":
    unittest.main()
