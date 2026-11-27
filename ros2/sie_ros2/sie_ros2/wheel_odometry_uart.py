"""Exclusive, read-only UART session for the two odometry inputs."""
from contextlib import contextmanager, suppress
from dataclasses import dataclass
import fcntl
import json
import os
from pathlib import Path
import select
import stat
import subprocess
import termios
import time

from firmware.esp32.tools.wheel_encoder_telemetry_v1 import (
    BOOT_ID, CONTROLLER_STATES, MAX_FRAME_BYTES, decode_reply, encode_request,
)
from vision_core.tools.uart_http_transport import frame_crc

FIRMWARE_BUILD = "v4_2_uart_shadow_v15_cumulative_encoder_telemetry"


@contextmanager
def lease(path):
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    with path.open("a") as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError(f"AUTHORITY_ALREADY_OWNED: {path}") from exc
        yield


def owners(path):
    check = subprocess.run(["fuser", str(path)], capture_output=True, text=True, timeout=2)
    pids = check.stdout.split()
    if check.returncode == 0 and pids and all(pid.isdecimal() for pid in pids):
        return {int(pid) for pid in pids}
    if check.returncode == 1 and not pids and not check.stderr.strip():
        return set()
    raise RuntimeError("UART_OWNERSHIP_CHECK_FAILED")


@contextmanager
def open_uart(port, state_dir):
    # Match the existing run_shadow_uart/default bounded-runner lease.
    with lease(state_dir / "v7_operator_one_step.lock"):
        path = Path(port).resolve(strict=True)
        device = path.stat()
        if not stat.S_ISCHR(device.st_mode):
            raise ValueError("UART_NOT_CHARACTER_DEVICE")
        if owners(path):
            raise RuntimeError("UART_ALREADY_OWNED")
        fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        original = None
        exclusive = False
        try:
            if os.fstat(fd).st_rdev != device.st_rdev:
                raise RuntimeError("UART_DEVICE_CHANGED")
            fcntl.ioctl(fd, termios.TIOCEXCL)
            exclusive = True
            if owners(path) != {os.getpid()}:
                raise RuntimeError("UART_EXCLUSIVE_OWNERSHIP_UNVERIFIED")
            original = termios.tcgetattr(fd)
            attrs = termios.tcgetattr(fd)
            attrs[:6] = [
                0, 0, termios.CLOCAL | termios.CREAD | termios.CS8, 0,
                termios.B115200, termios.B115200,
            ]
            attrs[6][termios.VMIN] = 1
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            yield fd
        finally:
            try:
                if original is not None:
                    termios.tcsetattr(fd, termios.TCSANOW, original)
            finally:
                if exclusive:
                    with suppress(OSError):
                        fcntl.ioctl(fd, termios.TIOCNXCL)
                os.close(fd)


@dataclass
class Attempt:
    operation: str
    request_id: str
    request_hex: str
    written_hex: str = ""
    response_hex: str = ""
    started_ns: int | None = None
    received_ns: int | None = None
    ros_receipt_ns: int | None = None


def unique_object(pairs):
    body = dict(pairs)
    if len(body) != len(pairs):
        raise ValueError("STATUS_DUPLICATE_FIELD")
    return body


def decode_status(frame, request_id):
    line = frame[:-1].removesuffix(b"\r")
    core, separator, crc = line.rpartition(b"|")
    if (not frame.endswith(b"\n") or b"\n" in line or b"\r" in line
            or len(line) > MAX_FRAME_BYTES or not separator):
        raise ValueError("STATUS_INVALID_FRAMING")
    if frame_crc(core) != crc:
        raise ValueError("STATUS_CRC_MISMATCH")
    parts = core.split(b"|", 3)
    if len(parts) != 4 or parts[:3] != [b"@1", request_id.encode("ascii"), b"200"]:
        raise ValueError("STATUS_INVALID_RESPONSE")
    try:
        body = json.loads(parts[3].decode("ascii"), object_pairs_hook=unique_object)
    except (ValueError, RecursionError) as exc:
        raise ValueError("STATUS_INVALID_JSON") from exc
    if not isinstance(body, dict) or body.get("firmware_build_id") != FIRMWARE_BUILD:
        raise ValueError("FIRMWARE_BUILD_MISMATCH")
    identity = body.get("boot_session_id")
    if not isinstance(identity, str) or not BOOT_ID.fullmatch(identity):
        raise ValueError("STATUS_INVALID_BOOT_ID")
    state = body.get("state")
    if (not isinstance(state, str) or state not in CONTROLLER_STATES or state == "FAULT"
            or "fault" not in body or body["fault"] is not None
            or any(body.get(key) is not False for key in (
                "bounded_fault_latched", "steered_safety_latched", "remote_stop_pending",
            ))):
        raise ValueError("CONTROLLER_STATUS_FAULT_OR_PENDING")
    return body


def wait_io(fd, deadline, *, read):
    wait = deadline - time.monotonic()
    if wait > 0:
        readers, writers, _ = select.select([fd] if read else [], [] if read else [fd], [], wait)
        if readers or writers:
            return
    raise TimeoutError("UART_REPLY_TIMEOUT" if read else "UART_SEND_TIMEOUT")


class ReadOnlyUart:
    def __init__(self, fd, timeout_s, ros_now_ns):
        self.fd = fd
        self.timeout_s = timeout_s
        self.ros_now_ns = ros_now_ns
        self.index = 0

    @staticmethod
    def request_bytes(operation, request_id):
        if operation == "STATUS":
            core = f"@1|{request_id}|STATUS".encode("ascii")
            return core + b"|" + frame_crc(core) + b"\n"
        if operation == "WHEEL_ENCODERS_V1":
            return encode_request(request_id)
        raise ValueError("INVALID_READ_ONLY_OPERATION")

    def make_attempt(self, *, status):
        if self.index == 0xFFFFFFFF:
            raise ValueError("UART_REQUEST_IDS_EXHAUSTED")
        self.index += 1
        request_id = f"{self.index:08X}"
        operation = "STATUS" if status else "WHEEL_ENCODERS_V1"
        frame = self.request_bytes(operation, request_id)
        return Attempt(operation, request_id, frame.hex())

    def exchange(self, attempt):
        fd = self.fd
        deadline = time.monotonic() + self.timeout_s
        if attempt.request_id != f"{self.index:08X}" or attempt.started_ns is not None:
            raise ValueError("INVALID_REQUEST_ID")
        request = self.request_bytes(attempt.operation, attempt.request_id)
        if request.hex() != attempt.request_hex:
            raise ValueError("INVALID_REQUEST_BYTES")
        sent = 0
        response = bytearray()
        try:
            if select.select([fd], [], [], 0)[0]:
                response.extend(os.read(fd, MAX_FRAME_BYTES + 3))
                raise ValueError("UNSOLICITED_UART_DATA")
            while sent < len(request):
                wait_io(fd, deadline, read=False)
                if sent == 0:
                    attempt.started_ns = time.monotonic_ns()
                sent += os.write(fd, request[sent:])
            while len(response) < MAX_FRAME_BYTES + 3:
                wait_io(fd, deadline, read=True)
                chunk = os.read(fd, MAX_FRAME_BYTES + 3 - len(response))
                if not chunk:
                    raise ValueError("UART_EOF")
                response.extend(chunk)
                if b"\n" in response:
                    attempt.received_ns = time.monotonic_ns()
                    attempt.ros_receipt_ns = self.ros_now_ns()
                    frame = bytes(response)
                    if attempt.operation == "STATUS":
                        return decode_status(frame, attempt.request_id)
                    return decode_reply(frame, attempt.request_id)
            raise ValueError("UART_RESPONSE_TOO_LONG")
        finally:
            attempt.written_hex = request[:sent].hex()
            attempt.response_hex = response.hex()
