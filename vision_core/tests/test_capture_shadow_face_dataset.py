"""Synthetic privacy and frame-provenance tests. No camera or real face."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from vision_core.tools.capture_shadow_face_dataset import (
    capture_session, face_detections)


class FakeCamera:
    def __init__(self, *, valid=True):
        self.frame = np.zeros((1200, 1920, 3), dtype=np.uint8) if valid else (
            np.zeros((200, 320, 3), dtype=np.uint8))
    def read(self):
        return True, self.frame


class FakeDetector:
    def setInputSize(self, size):
        self.size = size
    def detect(self, image):
        face = np.zeros((1, 15), dtype=np.float32)
        face[0, :4] = [80, 90, 40, 50]
        face[0, 14] = .98
        return 1, face


class FakeClock:
    def __init__(self):
        self.now = 1.
    def __call__(self):
        self.now += .11
        return self.now


class PrivateCaptureTests(unittest.TestCase):
    def test_face_detection_scaled_to_full_frame(self):
        detections = face_detections(
            FakeDetector(), np.zeros((1200,1920,3), dtype=np.uint8))
        self.assertEqual(len(detections),1)
        self.assertEqual(detections[0]["xywh_px"], [160., 180., 80., 100.])

    def test_private_images_and_manifest_are_bound(self):
        with tempfile.TemporaryDirectory() as d:
            destination = Path(d) / "pilot"
            summary = capture_session(
                camera=FakeCamera(), detector=FakeDetector(),
                directory=destination, duration_s=1.1,
                sample_interval_s=.25, monotonic=FakeClock())
            self.assertGreaterEqual(summary["frames_saved"], 2)
            self.assertEqual(summary["frames_without_face"], 0)
            self.assertEqual(destination.stat().st_mode & 0o777, 0o700)
            self.assertEqual((destination / "rgb").stat().st_mode & 0o777, 0o700)
            self.assertEqual((destination / "manifest.jsonl").stat().st_mode & 0o777, 0o600)
            rows = [json.loads(line) for line in
                    (destination / "manifest.jsonl").read_text().splitlines()]
            self.assertEqual(len(rows), summary["frames_saved"])
            for row in rows:
                jpeg = destination / row["file"]
                self.assertEqual(jpeg.stat().st_mode & 0o777, 0o600)
                self.assertEqual(hashlib.sha256(jpeg.read_bytes()).hexdigest(),
                                 row["jpeg_sha256"])
                self.assertFalse(row["identity_verified"])
                self.assertFalse(row["face_body_associated"])
            with self.assertRaises(FileExistsError):
                capture_session(camera=FakeCamera(),detector=FakeDetector(),
                    directory=destination,duration_s=.3,
                    sample_interval_s=.3,monotonic=FakeClock())

    def test_wrong_resolution_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaisesRegex(RuntimeError, "UNEXPECTED_FRAME_SHAPE"):
                capture_session(camera=FakeCamera(valid=False),
                    detector=FakeDetector(), directory=Path(d)/"pilot",
                    duration_s=.4,sample_interval_s=.25,monotonic=FakeClock())

    def test_zero_or_excessive_capture_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            for duration, interval in ((0., .5), (400., .5), (10., .1)):
                with self.subTest(duration=duration, interval=interval):
                    with self.assertRaises(ValueError):
                        capture_session(camera=FakeCamera(),
                            detector=FakeDetector(), directory=Path(d)/"pilot",
                            duration_s=duration,sample_interval_s=interval,
                            monotonic=FakeClock())


if __name__ == "__main__":
    unittest.main()
