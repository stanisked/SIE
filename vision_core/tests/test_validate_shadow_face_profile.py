"""Offline face validation contract tests using synthetic images only."""
from pathlib import Path
import unittest
from unittest import mock

import numpy as np

from vision_core.tools.validate_shadow_face_profile import evaluate


class FakeDetector:
    def __init__(self, counts):
        self.counts=list(counts)
        self.calls=0
    def setInputSize(self,size):
        self.size=size
    def detect(self,image):
        n=self.counts[self.calls]
        self.calls+=1
        if n==0:
            return 1,None
        face=np.zeros((n,15),dtype=np.float32)
        face[:, :4]=[60,50,50,55]
        face[:,14]=.98
        return 1,face


class FakeRecognizer:
    def alignCrop(self,image,face):
        return np.zeros((112,112,3),dtype=np.uint8)
    def feature(self,aligned):
        vector=np.zeros((1,128),dtype=np.float32)
        vector[0,0]=1.
        return vector


class FakeEncoder:
    yunet_sha256="yunet-test"
    sface_sha256="sface-test"
    recognizer=FakeRecognizer()
    def __init__(self,counts):
        self.detector=FakeDetector(counts)


def profile_data(source="enrollment_synthetic"):
    vectors=np.zeros((23,128),dtype=np.float32)
    vectors[:,0]=1.
    return vectors,{
        "target_entity_id":"person_001",
        "face_profile_id":"person_001_face_enrollment_v1",
        "sample_indices":[1,13,18,25,35,44,48,51,64,70,81,83,
                          86,88,106,111,116,119,122,146,150,154,169],
        "source_dataset":source
    }


def source(counts):
    rgb=np.zeros((1200,1920,3),dtype=np.uint8)
    return iter(({
        "sample_index":i+1,"host_monotonic_s":100.+i*.5,
        "faces":[{}] if original else []},rgb)
        for i,original in enumerate(counts))


class ValidationTests(unittest.TestCase):
    def run_evaluation(self,original,redetected,dataset="/tmp/validation_synthetic",
                       profile=None):
        enc=FakeEncoder(redetected)
        with mock.patch(
                "vision_core.tools.validate_shadow_face_profile.load_private_profile",
                return_value=profile if profile is not None else profile_data()), mock.patch(
                "vision_core.tools.validate_shadow_face_profile.read_source_rows",
                return_value=source(original)):
            return evaluate(Path(dataset),Path("/tmp/profile.npz"),enc)

    def test_independent_positive_sample_sface_is_scored_not_admitted(self):
        result=self.run_evaluation([1,1,0],[1,1,0])
        self.assertEqual(result["frames"],3)
        self.assertEqual(result["original_faces"],2)
        self.assertEqual(result["scored_faces"],2)
        self.assertEqual(result["cosine_max"]["median"],1.)
        self.assertEqual(result["redetection"]["both_empty"],1)
        self.assertEqual(result["scope"],
            "PRIVATE_POSITIVE_ONLY_SFACE_EVALUATION_NO_MOTOR_ADMISSION")
        self.assertNotIn("motor_authorized",result)
        self.assertEqual(result["by_time_window"][0]["scored_frames"],2)

    def test_disagreement_and_multiple_face_not_misclassified_as_identity(self):
        result=self.run_evaluation([1,0,1,0],[0,1,2,0])
        self.assertEqual(result["scored_faces"],0)
        self.assertEqual(result["redetection"]["original_only"],1)
        self.assertEqual(result["redetection"]["redetection_only"],1)
        self.assertEqual(result["redetection"]["ambiguous_redetection"],1)
        self.assertIsNone(result["cosine_max"])

    def test_reject_enrollment_reused_as_validation(self):
        with self.assertRaisesRegex(ValueError,"ENROLLMENT_DATASET_IS_NOT_VALIDATION"):
            self.run_evaluation([1],[1],dataset="/tmp/enrollment_synthetic")

    def test_reject_wrong_target_profile(self):
        feature,metadata=profile_data()
        metadata["target_entity_id"]="someone_else"
        with self.assertRaisesRegex(ValueError,"WRONG_OR_UNVERIFIED"):
            self.run_evaluation([1],[1],profile=(feature,metadata))


if __name__=="__main__":
    unittest.main()
