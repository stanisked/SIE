"""Test framing and exclusivity with stream sockets and a simulated PTY."""
from contextlib import ExitStack
import errno
import json
import os
import pty
import socket
from pathlib import Path

import pytest

from sie_ros2 import wheel_odometry_uart as uart


def frame(request_id, body, *, corrupt=False):
    prefix = f"@1|{request_id}|200|".encode() + json.dumps(body).encode()
    return prefix + b"|" + (b"XXXX" if corrupt else uart.frame_crc(prefix)) + b"\r\n"


def status():
    return dict(firmware_build_id=uart.FIRMWARE_BUILD, boot_session_id="0123456789ABCDEF",
                state="READY", remote_stop_pending=False, fault=None,
                bounded_fault_latched=False, steered_safety_latched=False)


def test_status_identity_and_framing():
    assert uart.decode_status(frame("00000001", status()), "00000001") == status()
    with pytest.raises(ValueError, match="CRC"):
        uart.decode_status(frame("00000001", status(), corrupt=True), "00000001")
    with pytest.raises(ValueError, match="FIRMWARE_BUILD_MISMATCH"):
        uart.decode_status(frame("00000001", {**status(), "firmware_build_id": "old"}), "00000001")
    with pytest.raises(ValueError, match="STATUS_FAULT"):
        uart.decode_status(frame("00000001", {**status(), "bounded_fault_latched": True}), "00000001")
    with pytest.raises(ValueError, match="INVALID_RESPONSE"):
        uart.decode_status(frame("00000002", status()), "00000001")


def test_read_only_timeout_retains_written_bytes_and_does_not_retry():
    with ExitStack() as stack:
        source, target = socket.socketpair()
        stack.enter_context(source)
        stack.enter_context(target)
        session = uart.ReadOnlyUart(source.fileno(), 0.01, lambda: 1)
        attempt = session.make_attempt(status=False)
        assert b"|WHEEL_ENCODERS_V1|" in bytes.fromhex(attempt.request_hex)
        with pytest.raises(TimeoutError, match="REPLY_TIMEOUT"):
            session.exchange(attempt)
        assert target.recv(4096) == bytes.fromhex(attempt.written_hex)
        assert not attempt.response_hex
        with pytest.raises(ValueError, match="INVALID_REQUEST_ID"):
            session.exchange(attempt)
        following = session.make_attempt(status=True)
        assert following.request_id != attempt.request_id
        assert b"|STATUS|" in bytes.fromhex(following.request_hex)
        following.operation = "STOP"
        with pytest.raises(ValueError, match="INVALID_READ_ONLY_OPERATION"):
            session.exchange(following)


def test_serial_and_authority_locks_with_simulated_port(tmp_path, monkeypatch):
    master, slave = pty.openpty()
    port = Path(os.ttyname(slave))
    os.close(slave)
    monkeypatch.setattr(uart, "owners", lambda _: set())
    try:
        with uart.lease(tmp_path / "v7_operator_one_step.lock"):
            with pytest.raises(RuntimeError, match="AUTHORITY_ALREADY_OWNED"):
                with uart.open_uart(port, tmp_path):
                    pytest.fail("port reached despite existing runner lease")
        observed = iter((set(), {os.getpid()}, set(), {os.getpid()}))
        monkeypatch.setattr(uart, "owners", lambda _: next(observed))
        with uart.open_uart(port, tmp_path):
            with pytest.raises(OSError) as exc:
                os.open(port, os.O_RDWR | os.O_NOCTTY)
            assert exc.value.errno == errno.EBUSY
        with uart.open_uart(port, tmp_path):
            pass
        monkeypatch.setattr(uart, "owners", lambda _: {os.getpid()})
        with pytest.raises(RuntimeError, match="UART_ALREADY_OWNED"):
            with uart.open_uart(port, tmp_path):
                pytest.fail("existing reader not refused")
    finally:
        os.close(master)
