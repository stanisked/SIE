"""Pseudo-terminal UART round trip for STATUS and STATUS_COMPACT.

Only in-memory fake controller replies; no ESP32, ROS, cameras, or actuators.
"""
import json
import os
import pty
import threading
import unittest

from vision_core.tests.test_shadow_uart_compact_v9_integration import CompactV9Controller
from vision_core.tools.shadow_uart_compact_status import expand_status
from vision_core.tools.uart_http_transport import UartHttp, frame_crc


class MotorOffUartPeer:
    def __init__(self, *, bad_crc=False, corrupt_version=False):
        self.master_fd, self.slave_fd = pty.openpty()
        self.slave_path = os.ttyname(self.slave_fd)
        self.bad_crc = bad_crc
        self.corrupt_version = corrupt_version
        self.controller = CompactV9Controller()
        self.seen_operations = []
        self.thread = threading.Thread(target=self._serve, daemon=True)

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *_):
        os.close(self.slave_fd)
        os.close(self.master_fd)
        self.thread.join(timeout=1.0)

    def _serve(self):
        buffered = b""
        while True:
            try:
                block = os.read(self.master_fd, 512)
            except OSError:
                return
            if not block:
                return
            buffered += block
            while b"\n" in buffered:
                line, buffered = buffered.split(b"\n", 1)
                core, separator, crc = line.rstrip(b"\r").rpartition(b"|")
                if not separator or frame_crc(core) != crc:
                    continue
                parts = core.decode("ascii").split("|")
                if len(parts) != 3 or parts[0] != "@1":
                    continue
                request_id, operation = parts[1:]
                self.seen_operations.append(operation)
                if operation not in ("STATUS", "STATUS_COMPACT"):
                    continue  # never permit motor opcodes from this fixture
                endpoint = "/status" if operation == "STATUS" else "/status-compact"
                code, body = self.controller(
                    "GET", "http://uart.local" + endpoint, 1.0)
                if self.corrupt_version and operation == "STATUS_COMPACT":
                    body["_"] = 2
                body_text = json.dumps(body, separators=(",", ":"), allow_nan=False)
                response_core = (
                    "@1|" + request_id + "|" + str(code) + "|" + body_text
                ).encode("ascii")
                response_crc = b"0000" if self.bad_crc else frame_crc(response_core)
                try:
                    os.write(self.master_fd, response_core + b"|" + response_crc + b"\n")
                except OSError:
                    return


class CompactUartPtyTests(unittest.TestCase):
    def test_full_then_compact_roundtrip_same_controller(self):
        with MotorOffUartPeer() as peer:
            with UartHttp(peer.slave_path) as uart:
                first_code, full = uart("GET", "http://uart.local/status", 1.0)
                second_code, compact = uart("GET", "http://uart.local/status-compact", 1.0)
        self.assertEqual(first_code, 200)
        self.assertEqual(second_code, 200)
        self.assertEqual(peer.seen_operations, ["STATUS", "STATUS_COMPACT"])
        expanded = expand_status(compact)
        for key, value in expanded.items():
            self.assertEqual(value, full[key], key)
        self.assertEqual(expanded["state"], "READY")
        self.assertEqual((expanded["right_pwm"], expanded["left_pwm"]), (0, 0))

    def test_crc_corruption_times_out_without_retries(self):
        with MotorOffUartPeer(bad_crc=True) as peer:
            with UartHttp(peer.slave_path) as uart:
                with self.assertRaises(TimeoutError):
                    uart("GET", "http://uart.local/status-compact", .12)
        self.assertEqual(peer.seen_operations, ["STATUS_COMPACT"])

    def test_unknown_version_refuses_without_motor_command(self):
        with MotorOffUartPeer(corrupt_version=True) as peer:
            with UartHttp(peer.slave_path) as uart:
                code, response = uart("GET", "http://uart.local/status-compact", 1.0)
        self.assertEqual(code, 200)
        with self.assertRaisesRegex(ValueError, "COMPACT_STATUS_UNSUPPORTED_VERSION"):
            expand_status(response)
        self.assertEqual(peer.seen_operations, ["STATUS_COMPACT"])


if __name__ == "__main__":
    unittest.main()
