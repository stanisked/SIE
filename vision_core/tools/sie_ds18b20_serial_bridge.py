#!/usr/bin/env python3

"""Persistent ESP32 DS18B20 serial bridge for SIE H0.5b.

The bridge process opens the serial port once, validates the configured ROM
mapping, and atomically publishes the latest temperature state. The snapshot
command reads that state without reopening the serial port and emits the JSON
object expected by diagnose_h2_stereo_thermal_drift.py.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


REQUIRED_CHANNELS = {"camera_left", "camera_right", "ambient"}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def parse_mapping(value: str) -> tuple[str, str]:
    if "=" not in value:
        raise ValueError("--map must use CHANNEL=ROM")
    channel, rom = (part.strip() for part in value.split("=", 1))
    if not channel or not rom:
        raise ValueError("--map must use non-empty CHANNEL=ROM")
    rom = rom.upper()
    if len(rom) != 16 or any(char not in "0123456789ABCDEF" for char in rom):
        raise ValueError(f"invalid DS18B20 ROM: {rom}")
    return channel, rom


def build_mapping(values: list[str]) -> dict[str, str]:
    mapping = dict(parse_mapping(value) for value in values)
    if set(mapping) != REQUIRED_CHANNELS:
        raise ValueError(
            "required channels are exactly: camera_left, camera_right, ambient"
        )
    if len(set(mapping.values())) != len(mapping):
        raise ValueError("each temperature channel must use a unique ROM")
    return mapping


def temperatures_from_payload(
    payload: dict[str, Any],
    mapping: dict[str, str],
) -> tuple[dict[str, float], int]:
    if payload.get("status") != "OK":
        raise ValueError(f"ESP32 status is not OK: {payload.get('status')!r}")
    sensors = payload.get("sensors")
    if not isinstance(sensors, list):
        raise ValueError("ESP32 payload has no sensors list")

    by_rom: dict[str, dict[str, Any]] = {}
    for item in sensors:
        if not isinstance(item, dict):
            raise ValueError("ESP32 sensor record is not an object")
        rom = str(item.get("rom", "")).upper()
        if not rom:
            raise ValueError("ESP32 sensor record has no ROM")
        if rom in by_rom:
            raise ValueError(f"duplicate ROM in ESP32 payload: {rom}")
        by_rom[rom] = item

    expected_roms = set(mapping.values())
    observed_roms = set(by_rom)
    if observed_roms != expected_roms:
        missing = sorted(expected_roms - observed_roms)
        unexpected = sorted(observed_roms - expected_roms)
        raise ValueError(
            f"ROM set mismatch; missing={missing}, unexpected={unexpected}"
        )

    values: dict[str, float] = {}
    for channel, rom in mapping.items():
        item = by_rom[rom]
        if item.get("valid") is not True:
            raise ValueError(f"{channel}/{rom} is marked invalid")
        value = float(item["temperature_c"])
        if not math.isfinite(value) or not -40.0 <= value <= 125.0:
            raise ValueError(f"{channel}/{rom} has invalid temperature {value}")
        values[channel] = value

    uptime_ms = int(payload["uptime_ms"])
    if uptime_ms < 0:
        raise ValueError("negative ESP32 uptime")
    return values, uptime_ms


def decode_serial_payload(raw_line: bytes) -> dict[str, Any] | None:
    """Extract one complete JSON object from a possibly noisy serial line.

    CH340/ESP32 startup output can contain non-UTF-8 bytes before the first
    firmware JSON record. Transport noise is not a sensor failure, so an
    incomplete or undecodable line is discarded without changing the last
    validated temperature state.
    """
    start = raw_line.find(b"{")
    end = raw_line.rfind(b"}")
    if start < 0 or end < start:
        return None
    try:
        candidate = raw_line[start : end + 1].decode("utf-8", errors="strict")
        payload = json.loads(candidate)
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None
    if not isinstance(payload, dict):
        return None
    return payload


def write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def build_state(
    *,
    status: str,
    mapping: dict[str, str],
    bridge_started_at_utc: str,
    reset_detected: bool,
    temperatures_c: dict[str, float] | None = None,
    source_uptime_ms: int | None = None,
    error: str | None = None,
) -> dict[str, Any]:
    now_unix = time.time()
    return {
        "status": status,
        "updated_at_utc": utc_now(),
        "updated_at_unix_s": now_unix,
        "bridge_started_at_utc": bridge_started_at_utc,
        "reset_detected": reset_detected,
        "source_uptime_ms": source_uptime_ms,
        "rom_mapping": mapping,
        "temperatures_c": temperatures_c or {},
        "error": error,
    }


def snapshot_from_state(
    state: dict[str, Any],
    *,
    max_age_s: float,
    now_unix_s: float | None = None,
) -> dict[str, Any]:
    if state.get("status") != "OK":
        raise ValueError(
            f"temperature bridge status is {state.get('status')!r}: "
            f"{state.get('error')!r}"
        )
    if state.get("reset_detected") is True:
        raise ValueError("ESP32 reset was detected after bridge startup")
    updated = float(state["updated_at_unix_s"])
    now = time.time() if now_unix_s is None else now_unix_s
    age = now - updated
    if age < -1.0:
        raise ValueError(f"temperature state timestamp is in the future: {age:.3f}s")
    if age > max_age_s:
        raise ValueError(
            f"temperature state is stale: age={age:.3f}s, limit={max_age_s:.3f}s"
        )
    temperatures = state.get("temperatures_c")
    if not isinstance(temperatures, dict) or set(temperatures) != REQUIRED_CHANNELS:
        raise ValueError("temperature state does not contain the three required channels")
    normalized = {name: float(value) for name, value in temperatures.items()}
    if any(not math.isfinite(value) for value in normalized.values()):
        raise ValueError("temperature state contains a non-finite value")
    return {"temperatures_c": normalized}


def run_bridge(args: argparse.Namespace) -> int:
    mapping = build_mapping(args.map)
    try:
        import serial  # type: ignore
    except ImportError as error:
        raise RuntimeError(
            "pyserial is required; install it with: python3 -m pip install pyserial"
        ) from error

    state_path = Path(args.state_file)
    started_at = utc_now()
    reset_detected = False
    previous_uptime_ms: int | None = None
    last_console_update = 0.0
    dropped_transport_lines = 0
    last_transport_warning = 0.0
    last_valid_monotonic: float | None = None

    write_json_atomic(
        state_path,
        build_state(
            status="STARTING",
            mapping=mapping,
            bridge_started_at_utc=started_at,
            reset_detected=False,
        ),
    )

    connection = serial.Serial(
        port=None,
        baudrate=args.baud,
        timeout=1.0,
        dsrdtr=False,
        rtscts=False,
        exclusive=True,
    )
    connection.dtr = False
    connection.rts = False
    connection.port = args.port
    connection.open()

    try:
        connection.reset_input_buffer()
        opened_monotonic = time.monotonic()
        print(
            f"SIE DS18B20 bridge: port={args.port} baud={args.baud} "
            f"state={state_path}",
            flush=True,
        )
        print(f"ROM mapping: {json.dumps(mapping, sort_keys=True)}", flush=True)

        while True:
            raw_line = connection.readline()
            now = time.monotonic()

            if (
                previous_uptime_ms is None
                and now - opened_monotonic >= args.startup_timeout_s
            ):
                error_text = (
                    "TimeoutError: no valid three-ROM packet within "
                    f"{args.startup_timeout_s:.3f}s after serial startup"
                )
                write_json_atomic(
                    state_path,
                    build_state(
                        status="ERROR",
                        mapping=mapping,
                        bridge_started_at_utc=started_at,
                        reset_detected=False,
                        error=error_text,
                    ),
                )
                print(error_text, file=sys.stderr, flush=True)
                return 2

            if (
                last_valid_monotonic is not None
                and now - last_valid_monotonic >= args.stream_timeout_s
            ):
                error_text = (
                    "TimeoutError: validated temperature stream is stale; "
                    f"no valid packet for {now - last_valid_monotonic:.3f}s"
                )
                write_json_atomic(
                    state_path,
                    build_state(
                        status="ERROR",
                        mapping=mapping,
                        bridge_started_at_utc=started_at,
                        reset_detected=reset_detected,
                        source_uptime_ms=previous_uptime_ms,
                        error=error_text,
                    ),
                )
                print(error_text, file=sys.stderr, flush=True)
                return 2

            if not raw_line:
                continue

            payload = decode_serial_payload(raw_line)
            if payload is None:
                dropped_transport_lines += 1
                if previous_uptime_ms is not None:
                    error_text = (
                        "TransportError: malformed serial line after the "
                        "validated stream started; fault latched"
                    )
                    write_json_atomic(
                        state_path,
                        build_state(
                            status="ERROR",
                            mapping=mapping,
                            bridge_started_at_utc=started_at,
                            reset_detected=reset_detected,
                            source_uptime_ms=previous_uptime_ms,
                            error=error_text,
                        ),
                    )
                    print(error_text, file=sys.stderr, flush=True)
                    return 2
                if now - last_transport_warning >= args.console_interval_s:
                    print(
                        "Discarded non-JSON/corrupted serial line; "
                        f"total={dropped_transport_lines}",
                        file=sys.stderr,
                        flush=True,
                    )
                    last_transport_warning = now
                continue

            try:
                temperatures, uptime_ms = temperatures_from_payload(payload, mapping)
                if (
                    previous_uptime_ms is not None
                    and uptime_ms < previous_uptime_ms
                ):
                    reset_detected = True
                    error_text = "ESP32 uptime decreased after bridge startup"
                    write_json_atomic(
                        state_path,
                        build_state(
                            status="RESET_DETECTED",
                            mapping=mapping,
                            bridge_started_at_utc=started_at,
                            reset_detected=True,
                            source_uptime_ms=uptime_ms,
                            error=error_text,
                        ),
                    )
                    print(error_text, file=sys.stderr, flush=True)
                    return 2

                previous_uptime_ms = uptime_ms
                last_valid_monotonic = now
                state = build_state(
                    status="OK",
                    mapping=mapping,
                    bridge_started_at_utc=started_at,
                    reset_detected=False,
                    temperatures_c=temperatures,
                    source_uptime_ms=uptime_ms,
                )
                write_json_atomic(state_path, state)
                now = time.monotonic()
                if now - last_console_update >= args.console_interval_s:
                    print(
                        f"uptime_ms={uptime_ms} status=OK "
                        + " ".join(
                            f"{name}={value:.4f}C"
                            for name, value in temperatures.items()
                        ),
                        flush=True,
                    )
                    last_console_update = now
            except Exception as error:
                state = build_state(
                    status="ERROR",
                    mapping=mapping,
                    bridge_started_at_utc=started_at,
                    reset_detected=reset_detected,
                    error=f"{type(error).__name__}: {error}",
                )
                write_json_atomic(state_path, state)
                print(state["error"], file=sys.stderr, flush=True)
                return 2
    finally:
        connection.close()


def run_snapshot(args: argparse.Namespace) -> int:
    state_path = Path(args.state_file)
    try:
        state = json.loads(state_path.read_text(encoding="utf-8"))
        snapshot = snapshot_from_state(state, max_age_s=args.max_age_s)
    except Exception as error:
        print(f"temperature snapshot failed: {type(error).__name__}: {error}", file=sys.stderr)
        return 2
    print(json.dumps(snapshot, ensure_ascii=False, sort_keys=True))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Persistent ESP32 DS18B20 bridge for SIE H0.5b"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    bridge = subparsers.add_parser("bridge", help="read ESP32 serial continuously")
    bridge.add_argument("--port", default="/dev/ttyUSB0")
    bridge.add_argument("--baud", type=int, default=115200)
    bridge.add_argument("--state-file", required=True)
    bridge.add_argument("--map", action="append", default=[], required=True)
    bridge.add_argument("--console-interval-s", type=float, default=10.0)
    bridge.add_argument("--startup-timeout-s", type=float, default=10.0)
    bridge.add_argument("--stream-timeout-s", type=float, default=5.0)
    bridge.set_defaults(handler=run_bridge)

    snapshot = subparsers.add_parser("snapshot", help="print a fresh JSON snapshot")
    snapshot.add_argument("--state-file", required=True)
    snapshot.add_argument("--max-age-s", type=float, default=5.0)
    snapshot.set_defaults(handler=run_snapshot)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if getattr(args, "max_age_s", 1.0) <= 0:
        parser.error("--max-age-s must be positive")
    if getattr(args, "console_interval_s", 1.0) <= 0:
        parser.error("--console-interval-s must be positive")
    if getattr(args, "startup_timeout_s", 1.0) <= 0:
        parser.error("--startup-timeout-s must be positive")
    if getattr(args, "stream_timeout_s", 1.0) <= 0:
        parser.error("--stream-timeout-s must be positive")
    return int(args.handler(args))


if __name__ == "__main__":
    raise SystemExit(main())
