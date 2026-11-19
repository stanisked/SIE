"""Synthetic tests for SIEE private person_001 profile enrollment."""
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import numpy as np

from vision_core.tools.enroll_shadow_face_dataset import (
    load_reviewed_approval, collect_enrollment, save_private_enrollment)
from vision_core.tools.shadow_face_offline import load_private_profile


class FakeEncoder:
    yunet_sha256 = "yu-net-test-model"
    sface_sha256 = "s-face-test-model"


def manifest_row(index):
    return {"sample_index":index,
            "file":f"rgb/rgb_{index:05d}.jpg",
            "jpeg_sha256":f"{index:064x}"}


def write_approval(dataset, *, tamper=False):
    curation = dataset / "curation_v1"
    curation.mkdir()
    indexes = list(range(1,24)) + [132]
    original = [manifest_row(n) for n in indexes]
    chosen = [dict(row) for row in original if row["sample_index"] != 132]
    if tamper:
        chosen[0]["jpeg_sha256"] = "f"*64
    common = {
        "dataset_dir":str(dataset),
        "yu_net_model_sha256":FakeEncoder.yunet_sha256,
        "sface_model_sha256":FakeEncoder.sface_sha256,
    }
    (curation/"candidates.json").write_text(json.dumps({
        **common,
        "scope":"PRIVATE_CURATION_CANDIDATES_ONLY_NOT_ENROLLMENT",
        "candidate_count":24,"selected":original}))
    (curation/"candidates_approved.json").write_text(json.dumps({
        **common,
        "scope":"OPERATOR_REVIEWED_ENROLLMENT_CANDIDATES",
        "candidate_count":23,"selected":chosen,
        "excluded_by_operator":{"132":"PARTIAL_FACE"}}))
    return chosen


def vectors():
    result = np.zeros((23,128),dtype=np.float32)
    for i in range(23):
        result[i,i]=1.
    return result


class EnrollmentTests(unittest.TestCase):
    def test_approval_exactly_excludes_132_and_binds_models(self):
        with tempfile.TemporaryDirectory() as d:
            dataset=Path(d)/"enrollment"
            dataset.mkdir()
            approved=write_approval(dataset)
            result,digest=load_reviewed_approval(dataset,FakeEncoder())
            self.assertEqual(result,approved)
            self.assertEqual(len(digest),64)
            self.assertEqual(len(result),23)
            self.assertNotIn(132,[x["sample_index"] for x in result])

    def test_refuse_mutation_of_curated_metadata(self):
        with tempfile.TemporaryDirectory() as d:
            dataset=Path(d)/"enrollment"
            dataset.mkdir()
            write_approval(dataset,tamper=True)
            with self.assertRaisesRegex(ValueError,"UNAPPROVED_OR_MODIFIED"):
                load_reviewed_approval(dataset,FakeEncoder())

    def test_private_profile_can_be_read_by_existing_loader(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            dataset=root/"enrollment"
            dataset.mkdir()
            approved=write_approval(dataset)
            profile=root/"private"/"person_001_face_enrollment_v1.npz"
            meta=save_private_enrollment(profile,features=vectors(),
                approved=approved,encoder=FakeEncoder(),
                approval_sha256="a"*64,dataset=dataset)
            self.assertEqual(meta["target_entity_id"],"person_001")
            self.assertFalse(meta["face_threshold_calibrated"])
            self.assertFalse(meta["motor_admission"])
            self.assertEqual(profile.stat().st_mode & 0o777,0o600)
            self.assertEqual(profile.parent.stat().st_mode & 0o777,0o700)
            loaded,restored=load_private_profile(profile,FakeEncoder())
            self.assertEqual(loaded.shape,(23,128))
            self.assertEqual(restored["sample_indices"],
                             [r["sample_index"] for r in approved])
            with self.assertRaises(FileExistsError):
                save_private_enrollment(profile,features=vectors(),
                    approved=approved,encoder=FakeEncoder(),
                    approval_sha256="a"*64,dataset=dataset)

    def test_collect_enrollment_requires_all_23_reextracted(self):
        with tempfile.TemporaryDirectory() as d:
            dataset=Path(d)
            approved=[manifest_row(n) for n in range(1,24)]
            rows=[(dict(manifest_row(n),
                        faces=[{"xywh_px":[0,0,90,90]}]),"placeholder")
                  for n in range(1,24)]
            candidates=[{"sample_index":n,
                "feature":vectors()[i]} for i,n in enumerate(range(1,24))]
            with mock.patch("vision_core.tools.enroll_shadow_face_dataset.read_source_rows",
                            return_value=iter(rows)), mock.patch(
                  "vision_core.tools.enroll_shadow_face_dataset.extract_candidates",
                  return_value=(candidates,{
                      "no_single_face":0,"not_detected_again":0,
                      "low_quality":0,"encoding_error":0},23)):
                actual=collect_enrollment(dataset,approved,FakeEncoder())
                self.assertEqual(actual.shape,(23,128))
            with mock.patch("vision_core.tools.enroll_shadow_face_dataset.read_source_rows",
                            return_value=iter(rows)), mock.patch(
                  "vision_core.tools.enroll_shadow_face_dataset.extract_candidates",
                  return_value=(candidates[:-1],{
                      "no_single_face":0,"not_detected_again":1,
                      "low_quality":0,"encoding_error":0},23)):
                with self.assertRaisesRegex(ValueError,"REDETECTION"):
                    collect_enrollment(dataset,approved,FakeEncoder())


if __name__=="__main__":
    unittest.main()
