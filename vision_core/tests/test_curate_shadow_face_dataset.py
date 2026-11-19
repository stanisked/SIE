"""Tests for private curation selection, no real biometric images."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from vision_core.tools.curate_shadow_face_dataset import (
    read_source_rows, select_candidates)


def example_row(index, time_s):
    vector=np.zeros(128,dtype=np.float32)
    vector[index%128] = 1.
    return {"sample_index":index,"time_s":time_s,
            "quality":.8, "yaw_proxy":index/100.,
            "feature":vector}


class FaceCurationTests(unittest.TestCase):
    def test_select_24_distinct_diverse_temporal_bins(self):
        # Six 15-second windows, six candidates each, spaced by 2 seconds.
        pool=[example_row(b*10+i+1,b*15+i*2.)
              for b in range(6) for i in range(6)]
        chosen=select_candidates(pool)
        self.assertEqual(len(chosen),24)
        self.assertEqual(len(set(r["sample_index"] for r in chosen)),24)
        for b in range(6):
            self.assertEqual(sum(
                b*15 <= r["time_s"] < (b+1)*15
                for r in chosen),4)
        self.assertEqual([r["sample_index"] for r in chosen],
                         [r["sample_index"] for r in select_candidates(pool)])

    def test_refuse_undersampled_bin(self):
        pool=[example_row(b*10+i+1,b*15+i*2.)
              for b in range(6) for i in range(3)]
        with self.assertRaisesRegex(ValueError,"NOT_ENOUGH_QUALITY"):
            select_candidates(pool)

    def test_sha256_and_path_binding(self):
        with tempfile.TemporaryDirectory() as home:
            root=Path(home)
            (root/"rgb").mkdir()
            image=np.full((1200,1920,3),20,dtype=np.uint8)
            ok,encoded=cv2.imencode(".jpg",image)
            self.assertTrue(ok)
            payload=encoded.tobytes()
            (root/"rgb/rgb_00001.jpg").write_bytes(payload)
            entry={"sample_index":1,"file":"rgb/rgb_00001.jpg",
                   "host_monotonic_s":200.0,
                   "frame_width":1920,"frame_height":1200,
                   "jpeg_sha256":hashlib.sha256(payload).hexdigest(),
                   "faces":[]}
            (root/"manifest.jsonl").write_text(json.dumps(entry)+"\n")
            rows=list(read_source_rows(root))
            self.assertEqual(len(rows),1)
            self.assertEqual(rows[0][1].shape,(1200,1920,3))
            entry["file"]="../rgb/rgb_00001.jpg"
            (root/"manifest.jsonl").write_text(json.dumps(entry)+"\n")
            with self.assertRaisesRegex(ValueError,"UNSAFE_IMAGE"):
                list(read_source_rows(root))
            entry["file"]="rgb/rgb_00001.jpg"
            entry["jpeg_sha256"]="0"*64
            (root/"manifest.jsonl").write_text(json.dumps(entry)+"\n")
            with self.assertRaisesRegex(ValueError,"SHA256_MISMATCH"):
                list(read_source_rows(root))


if __name__=="__main__":
    unittest.main()
