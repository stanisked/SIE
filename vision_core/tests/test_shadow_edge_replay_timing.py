"""Offline edge replay must respect recorded result-ready time.

No ESP32 connection or motor commands are available in these tests.
"""
import unittest

from vision_core.tools.replay_shadow_rgb_edge_recovery import (
    latest_ready_edge, result_ready_monotonic_s,
)


class ShadowEdgeReplayTimingTests(unittest.TestCase):
    def test_uses_recorded_ready_age_not_assumed_100ms(self):
        audit = {"frame_freshness": {
            "frames": {
                "ar0234": {
                    "driver_timestamp_s": 19910.782331,
                    "host_packet_received_monotonic_s": 19910.815,
                },
                "ov9281": {
                    "driver_timestamp_s": 19910.7865,
                    "host_packet_received_monotonic_s": 19910.803,
                },
            },
            "driver_declared_soe_to_result_s": {
                "ar0234": .33157, "ov9281": .32732,
            },
        }}
        ready = result_ready_monotonic_s(audit)
        self.assertGreater(ready, 19911.11)
        self.assertLess(ready, 19911.13)
        self.assertGreater(ready - 19910.782331, .30)

    def test_edge_cannot_be_used_before_frame_ready(self):
        edges = [
            {"stamp": 19924.047, "ready_s": 19924.235, "edge_track": "person-1"},
            {"stamp": 19924.247, "ready_s": 19924.432, "edge_track": "person-1"},
        ]
        self.assertIsNone(latest_ready_edge(edges, 19924.113))
        first = latest_ready_edge(edges, 19924.300)
        self.assertEqual(first["stamp"], 19924.047)
        self.assertEqual(latest_ready_edge(edges, 19924.480)["stamp"], 19924.247)
        self.assertIsNone(latest_ready_edge(edges, 19925.000))

    def test_rejects_corrupt_result_ready_metadata(self):
        audit = {"frame_freshness": {
            "frames": {
                "ar0234": {"driver_timestamp_s": 1.,
                           "host_packet_received_monotonic_s": 1.03},
                "ov9281": {"driver_timestamp_s": 1.01,
                           "host_packet_received_monotonic_s": 1.04},
            },
            "driver_declared_soe_to_result_s": {
                "ar0234": -0.3, "ov9281": 0.2,
            },
        }}
        with self.assertRaisesRegex(ValueError, "INVALID_RECORDED_RESULT_READY_TIMING"):
            result_ready_monotonic_s(audit)


if __name__ == "__main__":
    unittest.main()
