"""Motor-off integration coverage for version-negotiated STATUS_COMPACT.

FakeController is fully in-memory: no serial device, no motor commands and no
firmware flashing. Compile the v9 sketch and validate the actual UART bytes
separately before any physical deployment.
"""
import json
import re
import tempfile
import unittest
from pathlib import Path
from urllib.parse import urlsplit

from vision_core.tests.test_run_shadow_uart import FakeClock, FakeController
from vision_core.tools.run_shadow_uart import (
    EDGE_TURN_BUILD_V9, ShadowObservation, run_session,
)
from vision_core.tools.shadow_range_gate import (
    FRONT_FRAME, RgbTarget, TargetRange,
)
from vision_core.tools.shadow_uart_compact_status import (
    WIRE_FIELDS, compress_status,
)
from vision_core.tools.uart_http_transport import UartHttp


def sample(stamp):
    target = RgbTarget("person-7", stamp, 2., .95, "rgb")
    measured = TargetRange("LIDAR", "person-7", stamp, 1.5, .95,
                           FRONT_FRAME, "lidar", f"pair-{stamp}")
    return ShadowObservation(target, (measured,))


class CompactV9Controller(FakeController):
    """Simulated v9 controller with full STATUS and strict compact reply."""
    def __init__(self, *, corrupt=False, reboot=False, fault=False):
        super().__init__()
        self.paths = []
        self.corrupt = corrupt
        self.reboot = reboot
        self.fault = fault

    def __call__(self, method, url, timeout):
        path = urlsplit(url).path
        self.paths.append((method, path))
        if method == "GET" and path not in ("/status", "/status-compact"):
            raise AssertionError("INVALID_GET:" + path)
        code, body = super().__call__(method, url, timeout)
        if method != "GET":
            return code, body
        body = dict(body)
        body.update(
            firmware_build_id=EDGE_TURN_BUILD_V9,
            shadow_api_version="v2_paused_edge_turn",
            last_command_state="STOPPED" if self.last else None,
            shadow_pause_reason="HOST_REQUEST" if self.state == "PAUSED" else None,
            shadow_turn_result="NONE",
            shadow_segments_started=1 if self.active else 0,
            shadow_turns_started=0,
            fault=None,
            bounded_fault_reason=None,
            heading_rad=0.,
            right_encoder_raw_count=-12,
            left_encoder_raw_count=12,
            right_distance_m=.012,
            left_distance_m=.012,
            average_distance_m=.012,
            last_steer_update_seq=self.steer_seq,
            active_steer_update_seq=self.steer_seq if self.active else None,
        )
        if self.fault and self.active:
            body.update(state="FAULT", fault="BOUNDED_DISTANCE_LIMIT",
                        bounded_fault_latched=True,
                        bounded_fault_reason="BOUNDED_DISTANCE_LIMIT",
                        right_pwm=0, left_pwm=0)
        if path == "/status-compact":
            if self.reboot and self.active:
                body["boot_session_id"] = "boot-2"
            compact = compress_status(body)
            if self.corrupt and self.active:
                del compact["r"]
            return code, compact
        return code, body


class CompactV9MotorOffIntegrationTests(unittest.TestCase):
    def run_case(self, controller, stream):
        clock = FakeClock()
        with tempfile.TemporaryDirectory() as tmp:
            journal = Path(tmp) / "bridge.jsonl"
            report = run_session(
                request=controller,
                observations=stream(clock),
                journal_path=journal,
                clock=clock,
                rgb_calibration_id="rgb",
                lidar_calibration_id="lidar",
                minimum_confidence=.4,
                lock_path=Path(tmp) / "motor-off.lock",
            )
            rows = [json.loads(line) for line in journal.read_text().splitlines()]
        return report, rows

    def test_status_opcode_no_query_and_no_motion(self):
        self.assertEqual(UartHttp._fields(
            "GET", "http://uart.local/status-compact"), ["STATUS_COMPACT"])
        with self.assertRaises(ValueError):
            UartHttp._fields("GET", "http://uart.local/status-compact?x=1")

    def test_v9_negotiates_and_resumes_then_full_stop(self):
        controller = CompactV9Controller()

        def stream(clock):
            yield sample(clock.now)
            clock.now += .1
            yield ShadowObservation(None, ())
            clock.now += .1
            yield sample(clock.now)

        report, rows = self.run_case(controller, stream)
        self.assertEqual(report["result"], "SESSION_ENDED")
        self.assertEqual(report["operations"], ["START", "PAUSE", "RESUME"])
        gets = [p for m, p in controller.paths if m == "GET"]
        self.assertEqual(gets[0], "/status")
        self.assertIn("/status-compact", gets)
        self.assertEqual(gets[-1], "/status")
        self.assertEqual(controller.posts[-1][0], "/stop")
        self.assertEqual(report["final_status"]["state"], "READY")
        self.assertEqual(report["final_status"]["right_pwm"], 0)
        self.assertEqual(report["final_status"]["left_pwm"], 0)
        self.assertTrue(any(r.get("endpoint") == "/status-compact" for r in rows))

    def test_corrupted_compact_refuses_and_stops(self):
        controller = CompactV9Controller(corrupt=True)
        report, _ = self.run_case(controller, lambda clock: iter((sample(clock.now),)))
        self.assertEqual(report["result"], "STOPPED_ON_REFUSAL")
        self.assertIn("COMPACT_STATUS_WIRE_KEYS_INVALID", report["error"])
        self.assertEqual(controller.posts[-1][0], "/stop")
        self.assertEqual(report["final_status"]["right_pwm"], 0)
        self.assertEqual(report["final_status"]["left_pwm"], 0)

    def test_boot_session_change_refuses_and_stops(self):
        controller = CompactV9Controller(reboot=True)
        report, _ = self.run_case(controller, lambda clock: iter((sample(clock.now),)))
        self.assertEqual(report["result"], "STOPPED_ON_REFUSAL")
        self.assertIn("BOOT_SESSION_CHANGED", report["error"])
        self.assertEqual(controller.posts[-1][0], "/stop")

    def test_compact_fault_refuses_and_stops(self):
        controller = CompactV9Controller(fault=True)
        report, _ = self.run_case(controller, lambda clock: iter((sample(clock.now),)))
        self.assertEqual(report["result"], "STOPPED_ON_REFUSAL")
        self.assertEqual(controller.posts[-1][0], "/stop")

    def test_v9_firmware_serializer_has_exact_wire_keys(self):
        root = Path(__file__).resolve().parents[2]
        sketch = (root / "firmware" / "esp32"
                  / "traction_control_v4_2_uart_shadow_v9_compact_status"
                  / "traction_control_v4_2_uart_shadow_v9_compact_status.ino")
        code = sketch.read_text()
        section = code.split("String buildUartStatusCompact()", 1)[1].split(
            "\nvoid handleUartFrame(", 1)[0]
        keys = re.findall(r'\\"([_A-Za-z])\\":', section)
        self.assertEqual(keys, ["_"] + [alias for alias, _ in WIRE_FIELDS])
        self.assertIn('fields[2] == "STATUS_COMPACT" && count == 3', code)
        self.assertIn('sendJson(200, buildUartStatusCompact())', code)
        self.assertIn('constexpr char FIRMWARE_BUILD_ID[] = "v4_2_uart_shadow_v9_compact_status"', code)


if __name__ == "__main__":
    unittest.main()
