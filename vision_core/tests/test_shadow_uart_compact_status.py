"""Motor-off/serial-off contract tests. No hardware access or motion calls."""
import json
import unittest

from vision_core.tools.shadow_uart_compact_status import (
    WIRE_FIELDS, compress_status, expand_status,
)


class CompactStatusContractTests(unittest.TestCase):
    def setUp(self):
        self.full = {
            "firmware_build_id": "v4_2_uart_shadow_v8_short_forward_completion",
            "shadow_api_version": "v2_paused_edge_turn",
            "boot_session_id": "TESTBOOT12345678",
            "state": "READY",
            "active_command_id": None,
            "last_command_id": "test-command-1",
            "last_command_state": "STOPPED",
            "shadow_transition_seq": 4,
            "shadow_pause_reason": None,
            "shadow_turn_result": "NONE",
            "shadow_segments_started": 0,
            "shadow_turns_started": 0,
            "bounded_fault_latched": False,
            "steered_safety_latched": False,
            "remote_stop_pending": False,
            "fault": None,
            "bounded_fault_reason": None,
            "right_pwm": 0,
            "left_pwm": 0,
            "heading_rad": -0.025,
            "right_encoder_raw_count": -121,
            "left_encoder_raw_count": 128,
            "right_distance_m": 0.120,
            "left_distance_m": 0.129,
            "average_distance_m": 0.124,
            "last_steer_update_seq": 3,
            "active_steer_update_seq": None,
        }

    def test_roundtrip_states_and_errors(self):
        for state in ("READY", "STARTING", "DRIVING", "BRAKING", "COASTING", "PAUSED", "FAULT", "SUCCESS", "CORRECTING"):
            with self.subTest(state=state):
                full = dict(self.full, state=state)
                if state == "DRIVING":
                    full.update(active_command_id="test-command-2", right_pwm=190, left_pwm=180)
                if state == "PAUSED":
                    full.update(active_command_id="test-command-2", shadow_pause_reason="FEEDBACK_TIMEOUT")
                if state == "FAULT":
                    full.update(fault="BOUNDED_DISTANCE_LIMIT", bounded_fault_latched=True,
                                bounded_fault_reason="BOUNDED_DISTANCE_LIMIT")
                self.assertEqual(expand_status(compress_status(full)), full)

    def test_preserves_control_and_error_fields(self):
        full = dict(self.full, state="PAUSED", active_command_id="test-command-2",
                    shadow_pause_reason="FEEDBACK_TIMEOUT", remote_stop_pending=True,
                    steered_safety_latched=True, fault="STEERED_FAULT")
        restored = expand_status(compress_status(full))
        self.assertEqual(restored, full)
        self.assertEqual(set(restored), {name for _, name in WIRE_FIELDS})

    def test_incomplete_and_bad_types_refuse(self):
        wire = compress_status(self.full)
        for key, value in (("b", None), ("r", "0"), ("f", 0), ("i", ""), ("s", "MOVING"), ("s", "UNKNOWN")):
            with self.subTest(key=key):
                bad = dict(wire, **{key: value})
                with self.assertRaises(ValueError):
                    expand_status(bad)
        for mutation in (dict((k, v) for k, v in wire.items() if k != "z"),
                         dict(wire, extra=True),
                         dict(wire, _=2),
                         dict(wire, _=True)):
            with self.assertRaises(ValueError):
                expand_status(mutation)

    def test_size_reduction(self):
        original = json.dumps(self.full, separators=(",", ":"))
        compact = json.dumps(compress_status(self.full), separators=(",", ":"))
        self.assertLess(len(compact), len(original) * .6)


if __name__ == "__main__":
    unittest.main()
