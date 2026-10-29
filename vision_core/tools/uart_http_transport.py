"""Bounded UART request facade for the SIEE supervised live steering runner."""
from __future__ import annotations

import binascii
import json
import os
import re
import select
import termios
import time
import tty
from urllib.parse import parse_qs, urlsplit


HEX8 = re.compile(r'[0-9A-F]{8}\Z')
SAFE = re.compile(r'[A-Za-z0-9_.-]{1,64}\Z')


def frame_crc(core: bytes) -> bytes:
    return f'{binascii.crc_hqx(core, 0xffff):04X}'.encode('ascii')


class UartHttp:
    """Serial transport. Never retries; caller owns motion ambiguity policy."""

    def __init__(self, port: str):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.old = termios.tcgetattr(self.fd)
        try:
            tty.setraw(self.fd)
            attrs = termios.tcgetattr(self.fd)
            attrs[4] = attrs[5] = termios.B115200
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            termios.tcflush(self.fd, termios.TCIFLUSH)
        except BaseException:
            termios.tcsetattr(self.fd, termios.TCSANOW, self.old)
            os.close(self.fd)
            raise
        self.next_id = 0
        self.buffer = b''

    def close(self):
        if self.fd is not None:
            termios.tcsetattr(self.fd, termios.TCSANOW, self.old)
            os.close(self.fd)
            self.fd = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    @staticmethod
    def _fields(method: str, url: str):
        parsed = urlsplit(url)
        query = parse_qs(parsed.query, strict_parsing=True)
        if parsed.path == '/status' and method == 'GET' and not query:
            return ['STATUS']
        if parsed.path == '/stop' and method == 'POST' and not query:
            return ['STOP']
        if parsed.path == '/move-forward-steered' and method == 'POST':
            keys = ('boot_session_id', 'command_id', 'distance_m')
            op = 'MOVE'
        elif parsed.path == '/steer-update' and method == 'POST':
            keys = ('boot_session_id', 'command_id', 'seq', 'bias_pwm')
            op = 'STEER'
        else:
            raise ValueError('UNSUPPORTED_UART_ENDPOINT')
        if set(query) != set(keys) or any(len(query[k]) != 1 for k in keys):
            raise ValueError('INVALID_UART_ARGUMENTS')
        values = [query[k][0] for k in keys]
        if not all(SAFE.fullmatch(v) for v in values):
            raise ValueError('INVALID_UART_FIELD')
        return [op, *values]

    def __call__(self, method: str, url: str, timeout: float):
        if self.fd is None:
            raise RuntimeError('UART_CLOSED')
        if not 0 < timeout <= 3.0:
            raise ValueError('INVALID_UART_TIMEOUT')
        fields = self._fields(method, url)
        self.next_id = (self.next_id + 1) & 0xffffffff
        request_id = f'{self.next_id:08X}'
        core = ('@1|' + request_id + '|' + '|'.join(fields)).encode('ascii')
        frame = core + b'|' + frame_crc(core) + b'\n'
        deadline = time.monotonic() + timeout
        sent = 0
        while sent < len(frame):
            if time.monotonic() >= deadline:
                raise TimeoutError('UART_SEND_TIMEOUT')
            _, writable, _ = select.select([], [self.fd], [],
                                            max(0, deadline - time.monotonic()))
            if not writable:
                raise TimeoutError('UART_SEND_TIMEOUT')
            sent += os.write(self.fd, frame[sent:])
        while time.monotonic() < deadline:
            while b'\n' in self.buffer:
                line, self.buffer = self.buffer.split(b'\n', 1)
                line = line.rstrip(b'\r')
                if len(line) > 2048 or not line.startswith(b'@1|'):
                    continue
                core, sep, crc = line.rpartition(b'|')
                if not sep or frame_crc(core) != crc:
                    continue
                pieces = core.split(b'|', 3)
                if len(pieces) != 4 or pieces[1] != request_id.encode('ascii'):
                    continue
                try:
                    code = int(pieces[2])
                    data = json.loads(pieces[3])
                except (ValueError, UnicodeDecodeError):
                    continue
                if not isinstance(data, dict):
                    continue
                return code, data
            ready, _, _ = select.select([self.fd], [], [],
                                         max(0, deadline - time.monotonic()))
            if ready:
                self.buffer += os.read(self.fd, 512)
                if len(self.buffer) > 4096:
                    self.buffer = self.buffer[-2048:]
        raise TimeoutError('UART_REPLY_TIMEOUT')
