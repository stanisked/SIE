"""Synthetic-only tests for same-cycle face/mask provenance and geometry."""
import hashlib
import io
import json
import tarfile
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from vision_core.tools.audit_shadow_face_body import (
    checked_mask_and_binding, mask_face_geometry)


class FaceMaskGeometryTests(unittest.TestCase):
    def test_face_mask_overlap_and_center_with_same_frame(self):
        mask=np.zeros((120,160),dtype=bool)
        mask[20:100,30:100]=True
        result=mask_face_geometry(mask,(40.,30.,40.,40.))
        self.assertEqual(result["face_box_mask_overlap_fraction"],1.)
        self.assertTrue(result["face_center_inside_mask"])
        self.assertTrue(result["face_box_inside_image"])

    def test_outside_target_mask_is_not_association(self):
        mask=np.zeros((120,160),dtype=bool)
        mask[20:100,30:100]=True
        result=mask_face_geometry(mask,(105.,30.,30.,40.))
        self.assertEqual(result["face_box_mask_overlap_fraction"],0.)
        self.assertFalse(result["face_center_inside_mask"])

    def test_partial_edge_and_invalid_geometry(self):
        mask=np.ones((120,160),dtype=bool)
        result=mask_face_geometry(mask,(-5.,20.,20.,20.))
        self.assertEqual(result["face_box_mask_overlap_fraction"],1.)
        self.assertFalse(result["face_box_inside_image"])
        with self.assertRaises(ValueError):
            mask_face_geometry(mask,(1,2,0,5))
        with self.assertRaises(ValueError):
            mask_face_geometry(mask.astype(np.uint8),(1,2,3,5))


class FaceBodyEvidenceBindingTests(unittest.TestCase):
    @staticmethod
    def make_archive(path, *, bad_sha=False, ambiguous=False):
        cycle="ar0234-ov9281-control-preview-aabbccdd-00000001"
        mask=np.zeros((1200,1920),dtype=bool)
        mask[100:500,200:650]=True
        stream=io.BytesIO()
        np.savez_compressed(stream,instance_mask=mask)
        payload=stream.getvalue()
        mask_sha=hashlib.sha256(mask.tobytes()).hexdigest()
        meta={
            "cycle_id":cycle,
            "instance_mask_sha256":mask_sha,
            "npz_sha256":("0"*64 if bad_sha else
                           hashlib.sha256(payload).hexdigest()),
            "segmentation_model_sha256":"model-sha",
            "selected_instance_index":0,
        }
        audit={
            "cycle_id":cycle,
            "ar0234_dynamic_target":{
                "disposition":"OBSERVED",
                "current_eligible_count":(2 if ambiguous else 1),
                "candidate":{"instance_index":0,
                             "bbox_xyxy_px":[100,100,650,1000]}
            },
            "shape_diagnostic":{"mask_sha256":mask_sha},
            "segmentation_observation":{"model_sha256":"model-sha"}
        }
        with tarfile.open(path,"w:gz") as tar:
            for name,data in (
                (f"shadow-source-snapshots/{cycle}.json",
                 json.dumps(meta).encode()),
                (f"shadow-source-snapshots/{cycle}.npz",payload),
                (f"control-preview-aabbccdd/{cycle}.json",
                 json.dumps(audit).encode())
            ):
                header=tarfile.TarInfo(name)
                header.size=len(data)
                tar.addfile(header,io.BytesIO(data))
        return cycle,mask_sha

    def test_verified_cycle_mask_and_instance_binding(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/"evidence.tar.gz"
            cycle,expected=self.make_archive(path)
            with tarfile.open(path,"r:gz") as tar:
                files={m.name:m for m in tar.getmembers() if m.isfile()}
                mask,candidate,meta=checked_mask_and_binding(tar,files,cycle)
            self.assertEqual(mask.shape,(1200,1920))
            self.assertEqual(hashlib.sha256(mask.tobytes()).hexdigest(),expected)
            self.assertEqual(candidate["instance_index"],0)

    def test_refuse_archive_without_source_integrity(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/"evidence.tar.gz"
            cycle,_=self.make_archive(path,bad_sha=True)
            with tarfile.open(path,"r:gz") as tar:
                files={m.name:m for m in tar.getmembers() if m.isfile()}
                with self.assertRaisesRegex(ValueError,"SHA256_MISMATCH"):
                    checked_mask_and_binding(tar,files,cycle)

    def test_refuse_multi_person_scene_binding(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/"evidence.tar.gz"
            cycle,_=self.make_archive(path,ambiguous=True)
            with tarfile.open(path,"r:gz") as tar:
                files={m.name:m for m in tar.getmembers() if m.isfile()}
                with self.assertRaisesRegex(ValueError,"BINDING_INVALID"):
                    checked_mask_and_binding(tar,files,cycle)


if __name__=="__main__":
    unittest.main()
