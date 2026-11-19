"""Face verification unit tests. Synthetic arrays only, no biometric images."""
import os
from pathlib import Path
import tempfile
import unittest

import numpy as np

from vision_core.person_localization.shadow_face_verify import (
    normalized_feature, face_cosine, selected_face)
from vision_core.tools.shadow_face_offline import (
    load_private_profile, write_private_profile)


def fake_face(x=35, y=35, w=70, h=70, score=.99):
    row = np.zeros(15, dtype=np.float32)
    row[:4] = [x, y, w, h]
    row[14] = score
    return row


class FaceAssociationTests(unittest.TestCase):
    def test_one_good_face_inside_selected_person(self):
        f = fake_face()
        self.assertIsNotNone(selected_face(
            np.asarray([f]), (10,10,190,240)))

    def test_refuse_two_faces_even_if_one_matches(self):
        a = fake_face()
        b = fake_face(230, 30)
        self.assertIsNone(selected_face(
            np.asarray([a,b]), (10,10,190,240)))

    def test_refuse_outside_person_or_lower_body(self):
        self.assertIsNone(selected_face(
            np.asarray([fake_face(240, 20)]), (10,10,190,240)))
        self.assertIsNone(selected_face(
            np.asarray([fake_face(30, 190)]), (10,10,190,240)))

    def test_refuse_low_confidence_and_tiny_face(self):
        self.assertIsNone(selected_face(
            np.asarray([fake_face(score=.45)]), (10,10,190,240)))
        self.assertIsNone(selected_face(
            np.asarray([fake_face(w=10)]), (10,10,190,240)))

    def test_embedding_is_normalized_and_cosine_is_bounded(self):
        a = np.zeros(128, dtype=np.float32)
        b = np.zeros(128, dtype=np.float32)
        a[0] = 2
        b[1] = 2
        self.assertAlmostEqual(float(np.linalg.norm(normalized_feature(a))),1.)
        self.assertAlmostEqual(face_cosine(a,a),1.)
        self.assertAlmostEqual(face_cosine(a,b),0.)

    def test_invalid_embedding_fails(self):
        with self.assertRaisesRegex(ValueError,"INVALID_SFACE"):
            normalized_feature(np.zeros(50))
        with self.assertRaisesRegex(ValueError,"EMPTY_SFACE"):
            normalized_feature(np.zeros(128))


class FacePrivateProfileTests(unittest.TestCase):
    def test_atomic_profile_mode_and_binding(self):
        class FakeEncoder:
            yunet_sha256 = "detector-hash"
            sface_sha256 = "recognizer-hash"
        encoder = FakeEncoder()
        arrays = []
        for n in range(3):
            a = np.zeros(128, dtype=np.float32)
            a[n] = 1
            arrays.append(a)
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "private" / "target.npz"
            write_private_profile(path, arrays, ["a","b","c"], encoder)
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)
            self.assertEqual(path.parent.stat().st_mode & 0o777, 0o700)
            features, meta = load_private_profile(path, encoder)
            self.assertEqual(features.shape, (3,128))
            self.assertEqual(meta["enrolled_cycle_ids"],["a","b","c"])
            self.assertFalse(meta["person_identity_proven"])
            with self.assertRaises(FileExistsError):
                write_private_profile(path, arrays, ["a","b","c"], encoder)
            os.chmod(path, 0o644)
            with self.assertRaises(PermissionError):
                load_private_profile(path, encoder)


if __name__ == "__main__":
    unittest.main()
