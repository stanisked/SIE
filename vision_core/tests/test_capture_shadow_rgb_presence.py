"""Check frozen MJPEG provenance and the scheduled capture timeline."""

import hashlib
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

from vision_core.tools.capture_shadow_rgb_presence import record_session, save_packet, slot_after


class ShadowRgbPresenceCaptureTest(unittest.TestCase):
    def test_saved_bytes_are_original_and_exclusive(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / "images").mkdir()
            packet = b"\xff\xd8untouched compressed packet\xff\xd9"
            meta = {"sequence": 12, "driver_timestamp_s": 31.1,
                    "host_packet_received_monotonic_s": 31.2,
                    "received_count": 12, "replaced_latest_count": 10}
            with patch("vision_core.tools.capture_shadow_rgb_presence.shutil.disk_usage") as disk:
                disk.return_value.free = 10_000
                row = save_packet(root, 1, packet, meta, 31.0, min_free_bytes=100)
                self.assertEqual((root / row["file"]).read_bytes(), packet)
                self.assertEqual(row["sha256"], hashlib.sha256(packet).hexdigest())
                with self.assertRaises(FileExistsError):
                    save_packet(root, 1, packet, meta, 31.0, min_free_bytes=100)
                disk.return_value.free = 100
                with self.assertRaisesRegex(RuntimeError, "DISK_RESERVE"):
                    save_packet(root, 2, packet, meta, 31.0, min_free_bytes=100)

    def test_late_capture_skips_old_slots(self):
        self.assertEqual(slot_after(100.001, 100.0, .5, 0), 0)
        self.assertEqual(slot_after(101.51, 100.0, .5, 1), 3)
        self.assertEqual(slot_after(100.51, 100.0, .5, 3), 3)

    def test_camera_failure_still_writes_partial_session(self):
        class BrokenCapture:
            released = False

            def __init__(self, *_args, **_kwargs):
                self.calls = 0

            def snapshot(self, *_args, **_kwargs):
                self.calls += 1
                if self.calls > 1:
                    raise RuntimeError("camera stopped")
                return b"\xff\xd8", {}

            def release(self):
                self.released = True

        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp) / "capture"
            binary = Path(temp) / "fake_native"
            binary.write_bytes(b"test")
            fake = BrokenCapture()
            with (patch("vision_core.tools.capture_shadow_rgb_presence.shutil.disk_usage") as disk,
                  patch("vision_core.tools.capture_shadow_rgb_presence.configure_camera",
                        return_value={"device": "fake"}),
                  patch("vision_core.tools.capture_shadow_rgb_presence.compile_capture",
                        return_value=(binary, "source-sha")),
                  patch("vision_core.tools.capture_shadow_rgb_presence.LatestV4L2Capture",
                        return_value=fake)):
                disk.return_value.free = 4 * 1024**3
                report = record_session(folder, duration_s=1, sample_hz=2,
                                        projector_state="on", scenario="test")
            self.assertEqual(report["status"], "FAILED")
            self.assertIn("camera stopped", report["error"])
            self.assertTrue(fake.released)
            self.assertEqual(json.loads((folder / "session.json").read_text())["status"], "FAILED")

    def test_short_success_writes_fresh_packets_and_manifest(self):
        class Camera:
            def __init__(self):
                self.sequence = 0

            def snapshot(self, *_args, **kwargs):
                self.sequence += 1
                now = time.monotonic()
                return (b"\xff\xd8native" + bytes([self.sequence]) + b"\xff\xd9",
                        {"sequence": self.sequence, "driver_timestamp_s": now - .01,
                         "host_packet_received_monotonic_s": max(
                             now, kwargs.get("not_before_received_s", now)),
                         "received_count": self.sequence, "replaced_latest_count": 0})

            def release(self):
                pass

        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp) / "capture"
            binary = Path(temp) / "fake_native"
            binary.write_bytes(b"test")
            with (patch("vision_core.tools.capture_shadow_rgb_presence.shutil.disk_usage") as disk,
                  patch("vision_core.tools.capture_shadow_rgb_presence.configure_camera",
                        return_value={"device": "fake"}),
                  patch("vision_core.tools.capture_shadow_rgb_presence.compile_capture",
                        return_value=(binary, "source-sha")),
                  patch("vision_core.tools.capture_shadow_rgb_presence.LatestV4L2Capture",
                        return_value=Camera())):
                disk.return_value.free = 4 * 1024**3
                report = record_session(folder, duration_s=1, sample_hz=2,
                                        projector_state="on", scenario="short test")
            rows = [json.loads(line) for line in (folder / "frames.jsonl").read_text().splitlines()]
            self.assertEqual(report["status"], "COMPLETE")
            self.assertEqual(report["frames_saved"], 2)
            self.assertEqual([r["slot"] for r in rows], [0, 1])
            for row in rows:
                self.assertEqual(hashlib.sha256((folder / row["file"]).read_bytes()).hexdigest(),
                                 row["sha256"])


if __name__ == "__main__":
    unittest.main()
