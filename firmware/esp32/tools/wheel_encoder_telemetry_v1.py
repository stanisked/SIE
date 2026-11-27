"""Offline codec and continuity checks for the diagnostic UART encoder sample."""

import json
import re
from dataclasses import dataclass

from vision_core.tools.uart_http_transport import HEX8, frame_crc


OPERATION = "WHEEL_ENCODERS_V1"
MAX_FRAME_BYTES = 2048  # Existing host limit, excluding CR/LF.
U64_MAX = (1 << 64) - 1
I64_MIN = -(1 << 63)
I64_MAX = (1 << 63) - 1
BOOT_ID = re.compile(r"[0-9A-F]{16}")
DECIMAL = re.compile(r"(?:0|-?[1-9][0-9]{0,19})")
CONTROLLER_STATES = frozenset(
    {"READY", "PAUSED", "STARTING", "DRIVING", "BRAKING", "CORRECTING",
     "COASTING", "SUCCESS", "FAULT"}
)
BOOL_FIELDS = (
    "fault_latched", "steered_safety_latched", "right_counter_overflow",
    "left_counter_overflow", "sequence_exhausted",
)
NUMERIC_FIELDS = {
    "sample_sequence": (1, U64_MAX),
    "sample_monotonic_us": (0, I64_MAX),
    "right_total_signed_ticks": (I64_MIN, I64_MAX),
    "left_total_signed_ticks": (I64_MIN, I64_MAX),
}
V1_FIELDS = frozenset(("version", "boot_session_id", "controller_state",
                       *BOOL_FIELDS, *NUMERIC_FIELDS))


@dataclass(frozen=True)
class EncoderSample:
    boot_session_id: str
    sample_sequence: int
    sample_monotonic_us: int
    right_total_signed_ticks: int
    left_total_signed_ticks: int
    controller_state: str
    fault_latched: bool
    steered_safety_latched: bool
    right_counter_overflow: bool
    left_counter_overflow: bool
    sequence_exhausted: bool


def _request_id_bytes(request_id: str) -> bytes:
    if not isinstance(request_id, str) or not HEX8.fullmatch(request_id):
        raise ValueError("INVALID_REQUEST_ID")
    return request_id.encode("ascii")


def encode_request(request_id: str) -> bytes:
    """Build only the argument-free read. This module never opens a port."""
    core = b"@1|" + _request_id_bytes(request_id) + b"|" + OPERATION.encode("ascii")
    return core + b"|" + frame_crc(core) + b"\n"


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("DUPLICATE_JSON_FIELD")
        result[key] = value
    return result


def decode_reply(frame: bytes, request_id: str) -> EncoderSample:
    """Verify exactly one complete reply and preserve both fault latch sources."""
    expected_id = _request_id_bytes(request_id)
    if not frame.endswith(b"\n"):
        raise ValueError("INVALID_FRAMING")
    line = frame[:-1].removesuffix(b"\r")
    if len(line) > MAX_FRAME_BYTES or b"\n" in line or b"\r" in line:
        raise ValueError("INVALID_FRAMING")
    core, delimiter, crc = line.rpartition(b"|")
    if not delimiter:
        raise ValueError("INVALID_FRAMING")
    if frame_crc(core) != crc:
        raise ValueError("CRC_MISMATCH")
    fields = core.split(b"|", 3)
    if len(fields) != 4 or fields[0] != b"@1":
        raise ValueError("INVALID_FRAMING")
    if fields[1] != expected_id:
        raise ValueError("REQUEST_ID_MISMATCH")
    if fields[2] != b"200":
        raise ValueError("TELEMETRY_REPLY_REJECTED")
    try:
        values = json.loads(fields[3].decode("ascii"), object_pairs_hook=_unique_object)
    except (UnicodeDecodeError, ValueError, RecursionError) as exc:
        raise ValueError("INVALID_TELEMETRY_JSON") from exc
    if not isinstance(values, dict) or values.keys() != V1_FIELDS:
        raise ValueError("INVALID_TELEMETRY_FIELDS")
    version = values.pop("version")
    if type(version) is not int or version != 1:
        raise ValueError("INVALID_TELEMETRY_VERSION")
    boot_id = values["boot_session_id"]
    if not isinstance(boot_id, str) or not BOOT_ID.fullmatch(boot_id):
        raise ValueError("INVALID_BOOT_ID")
    state = values["controller_state"]
    if not isinstance(state, str) or state not in CONTROLLER_STATES:
        raise ValueError("INVALID_CONTROLLER_STATE")
    for field in BOOL_FIELDS:
        if type(values[field]) is not bool:
            raise ValueError(f"INVALID_{field.upper()}")
    for field, (low, high) in NUMERIC_FIELDS.items():
        value = values[field]
        if not isinstance(value, str) or not DECIMAL.fullmatch(value):
            raise ValueError(f"INVALID_{field.upper()}")
        number = int(value)
        if not low <= number <= high:
            raise ValueError(f"INVALID_{field.upper()}")
        values[field] = number
    sample = EncoderSample(**values)
    if sample.sequence_exhausted and sample.sample_sequence != U64_MAX:
        raise ValueError("INVALID_EXHAUSTION_FLAG")
    for ticks, overflow in (
        (sample.right_total_signed_ticks, sample.right_counter_overflow),
        (sample.left_total_signed_ticks, sample.left_counter_overflow),
    ):
        if overflow and ticks not in (I64_MIN, I64_MAX):
            raise ValueError("INVALID_OVERFLOW_FLAG")
    return sample


@dataclass(frozen=True)
class SampleDisposition:
    status: str
    missing_samples: int = 0


class EncoderSampleContinuity:
    """Observe decoded replies in request order during one evidence capture."""

    def __init__(self):
        self._boot_id: str | None = None
        self._retired_boots: set[str] = set()
        self._previous: EncoderSample | None = None
        self._terminal: str | None = None

    def observe(self, sample: EncoderSample) -> SampleDisposition:
        if sample.boot_session_id in self._retired_boots:
            return SampleDisposition("RETIRED_BOOT")
        reboot = self._boot_id is not None and self._boot_id != sample.boot_session_id
        if reboot:
            self._retired_boots.add(self._boot_id)
            self._previous = self._terminal = None
        self._boot_id = sample.boot_session_id
        if self._terminal:
            return SampleDisposition(self._terminal)

        previous = self._previous
        if previous is not None:
            if sample == previous:
                return SampleDisposition("DUPLICATE")
            if sample.sample_sequence < previous.sample_sequence:
                status = ("ROLLOVER_OR_OUT_OF_ORDER" if previous.sample_sequence == U64_MAX
                          else "OUT_OF_ORDER")
                return SampleDisposition(status)
            if sample.sample_sequence == previous.sample_sequence and not sample.sequence_exhausted:
                return SampleDisposition("CONFLICTING_DUPLICATE")
            if sample.sample_monotonic_us <= previous.sample_monotonic_us:
                return SampleDisposition("NON_INCREASING_TIMESTAMP")

        if sample.right_counter_overflow or sample.left_counter_overflow:
            self._terminal = "COUNTER_OVERFLOW"
        elif sample.sequence_exhausted:
            self._terminal = "SEQUENCE_EXHAUSTED"
        if self._terminal:
            return SampleDisposition(self._terminal)

        self._previous = sample
        if previous is None:
            return SampleDisposition("REBOOT_BASELINE" if reboot else "BASELINE")
        missing = sample.sample_sequence - previous.sample_sequence - 1
        return SampleDisposition("SAMPLE_GAP" if missing else "CONSECUTIVE", missing)
