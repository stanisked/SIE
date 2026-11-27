"""Host production-code, wire codec, and static derivative regression tests."""

import json
import re
import subprocess
from pathlib import Path

import pytest

from firmware.esp32.tools.wheel_encoder_telemetry_v1 import (
    I64_MAX,
    I64_MIN,
    U64_MAX,
    EncoderSampleContinuity,
    SampleDisposition,
    decode_reply,
    encode_request,
)
from vision_core.tools.uart_http_transport import UartHttp, frame_crc


ROOT = Path(__file__).resolve().parents[1]
V13 = ROOT / "traction_control_v4_2_uart_shadow_v13_brake_continuity"
V15 = ROOT / "traction_control_v4_2_uart_shadow_v15_cumulative_encoder_telemetry"
REQUEST_ID = "00000020"
BOOT_ID = "0123456789ABCDEF"


def sketch(directory):
    return (directory / f"{directory.name}.ino").read_text()


def function(source, name):
    return re.search(rf"^\w[^\n]*\b{name}\([^\n]*\n\{{\n.*?^\}}\n", source, re.M | re.S)[0]


@pytest.fixture(scope="module")
def harness(tmp_path_factory):
    workdir = tmp_path_factory.mktemp("encoder_harness")
    source = sketch(V15)
    functions = (
        "onRightEncoderA", "onLeftEncoderA", "resetEncoderCounts", "uartCrc16",
        "uartHex16", "uartHexIdValid", "sendUartReply", "sampleWheelEncoderTelemetry",
        "sendWheelEncoderTelemetry", "handleUartFrame",
    )
    (workdir / "wheel_encoder_sketch.inc").write_text(
        "\n".join(function(source, name) for name in functions)
    )
    executable = workdir / "check_encoder_telemetry"
    subprocess.run(
        ["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror", "-pthread",
         "-fsanitize=undefined", "-fno-sanitize-recover=all",
         "-I", str(workdir), "-I", str(V15), "-o", str(executable),
         str(Path(__file__).with_name("wheel_encoder_telemetry_sketch_harness.cpp"))],
        check=True,
    )
    return executable


def run_harness(harness, case):
    return subprocess.check_output([harness, case], timeout=10)


def test_physical_signs_and_reset_through_actual_isrs(harness):
    run_harness(harness, "motion")


def test_concurrent_pair_and_timestamp_with_mocked_gpio_service(harness):
    run_harness(harness, "concurrent")


def test_contended_snapshot_refuses_without_losing_ticks_or_consuming_sequence(harness):
    frame = run_harness(harness, "contended")
    with pytest.raises(ValueError, match="TELEMETRY_REPLY_REJECTED"):
        decode_reply(frame, REQUEST_ID)


def test_firmware_boundaries_and_lossless_uart_serialization(harness):
    frame = run_harness(harness, "boundaries")
    sample = decode_reply(frame, "00000021")
    assert sample.right_total_signed_ticks == I64_MIN
    assert sample.left_total_signed_ticks == I64_MAX
    assert sample.sample_monotonic_us == I64_MAX
    assert sample.sample_sequence == U64_MAX
    assert sample.right_counter_overflow and sample.left_counter_overflow
    assert sample.sequence_exhausted
    assert len(frame) <= 436


def test_actual_uart_dispatch_crc_and_fault_provenance(harness):
    frame = run_harness(harness, "dispatch")
    sample = decode_reply(frame, REQUEST_ID)
    assert sample.controller_state == "FAULT"
    assert sample.fault_latched and sample.steered_safety_latched


def test_only_enumerated_sketch_changes_and_no_runner_opt_in():
    baseline = sketch(V13)
    candidate = sketch(V15)
    for name in ("brake_timing.h", "command_replay_guard.h", "fault_snapshot.h",
                 "remote_stop_observer.h"):
        assert (V13 / name).read_bytes() == (V15 / name).read_bytes()
    for include in ("#include <esp_timer.h>\n", '#include "wheel_encoder_telemetry.h"\n'):
        candidate = candidate.replace(include, "", 1)
    candidate = candidate.replace(
        'FIRMWARE_BUILD_ID[] = "v4_2_uart_shadow_v15_cumulative_encoder_telemetry"',
        'FIRMWARE_BUILD_ID[] = "v4_2_uart_shadow_v13_brake_continuity"',
        1,
    )
    candidate = re.sub(
        r"\n// Both callbacks are dispatched serially by the ESP32 GPIO ISR service\.\nWheelEncoderTelemetry wheelEncoderTelemetry;\n",
        "", candidate, flags=re.S, count=1,
    )
    for wheel, phase, sign in (("Right", "RIGHT_B", "right"), ("Left", "LEFT_B", "left")):
        isr = function(candidate, f"on{wheel}EncoderA")
        expected = (
            f"void IRAM_ATTR on{wheel}EncoderA()\n{{\n"
            f"  const bool phaseBHigh = digitalRead({phase});\n"
            f"  {sign}RawCount += phaseBHigh ? 1 : -1;\n"
            f"  wheelEncoderTelemetry.record{wheel}Edge(phaseBHigh);\n"
            "}\n"
        )
        assert isr == expected
        candidate = candidate.replace(isr, function(baseline, f"on{wheel}EncoderA"), 1)

    for name in ("sampleWheelEncoderTelemetry", "sendWheelEncoderTelemetry"):
        candidate = candidate.replace(function(candidate, name) + "\n", "", 1)
    candidate = candidate.replace(
        '  } else if (fields[2] == "WHEEL_ENCODERS_V1" && count == 3) {\n'
        "    sendWheelEncoderTelemetry();\n",
        "", 1,
    )
    assert baseline == candidate
    with pytest.raises(ValueError, match="UNSUPPORTED_UART_ENDPOINT"):
        UartHttp._fields("GET", "/wheel-encoders-v1")


def payload(**changes):
    return {
        "version": 1, "boot_session_id": BOOT_ID, "sample_sequence": "1",
        "sample_monotonic_us": "100", "right_total_signed_ticks": "0",
        "left_total_signed_ticks": "0", "controller_state": "READY",
        "fault_latched": False, "steered_safety_latched": False,
        "right_counter_overflow": False, "left_counter_overflow": False,
        "sequence_exhausted": False, **changes,
    }


def wire(body, *, request_id=REQUEST_ID, code=200):
    core = f"@1|{request_id}|{code}|".encode("ascii") + body
    return core + b"|" + frame_crc(core) + b"\r\n"


def decode(values):
    return decode_reply(wire(json.dumps(values).encode()), REQUEST_ID)


def sample(seq=1, time=100, **changes):
    return decode(payload(sample_sequence=str(seq), sample_monotonic_us=str(time), **changes))


def test_request_contains_only_read_operation_and_crc():
    core = b"@1|00000020|WHEEL_ENCODERS_V1"
    assert encode_request(REQUEST_ID) == core + b"|" + frame_crc(core) + b"\n"
    with pytest.raises(ValueError, match="INVALID_REQUEST_ID"):
        encode_request("00000020|STOP")


def test_consecutive_duplicate_gap_and_order():
    tracker = EncoderSampleContinuity()
    first = sample()
    assert tracker.observe(first).status == "BASELINE"
    assert tracker.observe(first).status == "DUPLICATE"
    assert tracker.observe(sample(1, 101)).status == "CONFLICTING_DUPLICATE"
    assert tracker.observe(sample(2, 200)).status == "CONSECUTIVE"
    assert tracker.observe(sample(6, 600, right_total_signed_ticks="-13")) == SampleDisposition(
        "SAMPLE_GAP", 3
    )
    assert tracker.observe(sample(5, 500)).status == "OUT_OF_ORDER"
    assert tracker.observe(sample(7, 600)).status == "NON_INCREASING_TIMESTAMP"
    assert tracker.observe(sample(7, 599)).status == "NON_INCREASING_TIMESTAMP"
    assert tracker.observe(sample(7, 601)).status == "CONSECUTIVE"


def test_reboot_has_no_cross_boot_continuity_and_old_epoch_is_refused():
    tracker = EncoderSampleContinuity()
    assert tracker.observe(sample(10, 1000)).status == "BASELINE"
    restarted = sample(1, 10, boot_session_id="0000000000000002")
    assert tracker.observe(restarted).status == "REBOOT_BASELINE"
    assert tracker.observe(sample(11, 1100)).status == "RETIRED_BOOT"
    assert tracker.observe(restarted).status == "DUPLICATE"


def test_exhaustion_and_same_boot_rollover():
    tracker = EncoderSampleContinuity()
    assert tracker.observe(sample(U64_MAX - 1, 800)).status == "BASELINE"
    assert tracker.observe(sample(U64_MAX, 801)).status == "CONSECUTIVE"
    assert tracker.observe(sample(1, 900)).status == "ROLLOVER_OR_OUT_OF_ORDER"
    assert tracker.observe(sample(U64_MAX, 901, sequence_exhausted=True)).status == "SEQUENCE_EXHAUSTED"
    assert tracker.observe(sample(1, 950)).status == "SEQUENCE_EXHAUSTED"


def test_counter_boundary_valid_until_flag_and_invalid_until_reboot():
    tracker = EncoderSampleContinuity()
    assert tracker.observe(sample(right_total_signed_ticks=str(I64_MAX))).status == "BASELINE"
    failed = sample(2, 200, right_total_signed_ticks=str(I64_MAX), right_counter_overflow=True)
    assert tracker.observe(failed).status == "COUNTER_OVERFLOW"
    assert tracker.observe(sample(3, 300)).status == "COUNTER_OVERFLOW"
    new_boot = sample(1, 10, boot_session_id="0000000000000002")
    assert tracker.observe(new_boot).status == "REBOOT_BASELINE"


@pytest.mark.parametrize(
    ("field", "value", "reason"),
    [
        ("version", True, "INVALID_TELEMETRY_VERSION"),
        ("boot_session_id", "ab", "INVALID_BOOT_ID"),
        ("sample_sequence", "0", "INVALID_SAMPLE_SEQUENCE"),
        ("sample_sequence", str(U64_MAX + 1), "INVALID_SAMPLE_SEQUENCE"),
        ("sample_monotonic_us", "-1", "INVALID_SAMPLE_MONOTONIC_US"),
        ("sample_monotonic_us", None, "INVALID_SAMPLE_MONOTONIC_US"),
        ("sample_monotonic_us", str(I64_MAX + 1), "INVALID_SAMPLE_MONOTONIC_US"),
        ("right_total_signed_ticks", str(I64_MAX + 1), "INVALID_RIGHT_TOTAL_SIGNED_TICKS"),
        ("left_total_signed_ticks", str(I64_MIN - 1), "INVALID_LEFT_TOTAL_SIGNED_TICKS"),
        ("right_total_signed_ticks", 1, "INVALID_RIGHT_TOTAL_SIGNED_TICKS"),
        ("left_total_signed_ticks", "-0", "INVALID_LEFT_TOTAL_SIGNED_TICKS"),
        ("controller_state", [], "INVALID_CONTROLLER_STATE"),
        ("fault_latched", 1, "INVALID_FAULT_LATCHED"),
        ("right_counter_overflow", True, "INVALID_OVERFLOW_FLAG"),
        ("sequence_exhausted", True, "INVALID_EXHAUSTION_FLAG"),
    ],
)
def test_rejects_malformed_or_unrepresentable_fields(field, value, reason):
    with pytest.raises(ValueError, match=reason):
        decode(payload(**{field: value}))


def test_missing_and_duplicate_fields():
    data = payload()
    del data["fault_latched"]
    with pytest.raises(ValueError, match="INVALID_TELEMETRY_FIELDS"):
        decode(data)
    with pytest.raises(ValueError, match="INVALID_TELEMETRY_JSON"):
        decode_reply(wire(b'{"version":1,"version":1}'), REQUEST_ID)


@pytest.mark.parametrize(
    ("frame", "reason"),
    [
        (b"@1|00000020|200|{}|0000\n", "CRC_MISMATCH"),
        (wire(b"{}", request_id="00000001"), "REQUEST_ID_MISMATCH"),
        (wire(b"{}", code=400), "TELEMETRY_REPLY_REJECTED"),
        (b"@1|00000020", "INVALID_FRAMING"),
        (wire(b" " * 2048), "INVALID_FRAMING"),
        (wire(b"{}") + wire(b"{}"), "INVALID_FRAMING"),
    ],
)
def test_bad_wire_frames_are_refused(frame, reason):
    with pytest.raises(ValueError, match=reason):
        decode_reply(frame, REQUEST_ID)
