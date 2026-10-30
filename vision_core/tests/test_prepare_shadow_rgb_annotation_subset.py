"""Frozen source validation and time-coverage selection for Shadow RGB."""

import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from vision_core.tools.prepare_shadow_rgb_annotation_subset import (
    choose_frames, verify_session)


class ShadowRgbSubsetTest(unittest.TestCase):
    def test_one_view_per_window_and_change_priority(self):
        rows = [{"file": f"images/ar0234_{i+1:06d}.jpg", "sha256": "x", "slot": i}
                for i in range(1200)]
        scores = [0.0] * 1200
        scores[2] = 20.0
        chosen = choose_frames(rows, scores)
        self.assertEqual(len(chosen), 120)
        self.assertEqual(chosen[0]["source_slot"], 2)
        self.assertEqual(chosen[0]["selection_reason"], "HIGH_FRAME_CHANGE")
        self.assertEqual(chosen[1]["source_slot"], 14)
        self.assertEqual(chosen[1]["selection_reason"], "WINDOW_MIDPOINT")
        self.assertTrue(all(row["person_visible"] is None for row in chosen))

    def test_full_source_hash_validation_catches_changed_image(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / "images").mkdir()
            rows = []
            for i in range(1200):
                name = f"images/ar0234_{i+1:06d}.jpg"
                payload = b"\xff\xd8" + i.to_bytes(2, "little") + b"\xff\xd9"
                (root / name).write_bytes(payload)
                rows.append({"file": name, "sha256": hashlib.sha256(payload).hexdigest(),
                             "bytes": len(payload), "slot": i, "driver_sequence": i+10})
            journal = root / "frames.jsonl"
            journal.write_text("".join(json.dumps(row) + "\n" for row in rows))
            session = {"schema_version": "sie.shadow_rgb_presence_capture.v1",
                       "status": "COMPLETE", "frames_saved": 1200, "duration_requested_s": 600,
                       "sample_hz": 2, "slots_skipped": 0, "actuator_access": False,
                       "frames_jsonl_sha256": hashlib.sha256(journal.read_bytes()).hexdigest(),
                       "bytes_saved": 1200 * 6}
            (root / "session.json").write_text(json.dumps(session))
            self.assertEqual(len(verify_session(root)[1]), 1200)
            (root / rows[4]["file"]).write_bytes(b"\xff\xd8changed\xff\xd9")
            with self.assertRaisesRegex(ValueError, "SOURCE_IMAGE_HASH_MISMATCH:5"):
                verify_session(root)


if __name__ == "__main__":
    unittest.main()
