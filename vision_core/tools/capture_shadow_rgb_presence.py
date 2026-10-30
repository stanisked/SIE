#!/usr/bin/env python3
"""Ten-minute AR0234 person-present/absent dataset capture, camera only.

Continuously drain native V4L2 MJPEG at the camera rate and save fresh original
compressed packets at 2 Hz. No detector, stereo, ROS, UART or motors are used.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import secrets
import shutil
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros2/sie_ros2"))
from sie_ros2.latest_v4l2_capture import LatestV4L2Capture


DEVICE = "/dev/v4l/by-id/usb-DECXIN_CAMERA_DECXIN_CAMERA_01.00.00-video-index0"
WIDTH, HEIGHT, CAMERA_FPS = 1920, 1200, 30
MIB = 1024 * 1024


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for part in iter(lambda: stream.read(MIB), b""):
            h.update(part)
    return h.hexdigest()


def configure_camera(device: str) -> dict:
    if device != DEVICE or not Path(device).exists():
        raise ValueError("AR0234 stable by-id device missing or changed")
    for option in ("--set-fmt-video=width=1920,height=1200,pixelformat=MJPG",
                   "--set-parm=30", "--set-ctrl=auto_exposure=3,gain=0"):
        subprocess.run(["v4l2-ctl", "-d", device, option], check=True,
                       text=True, capture_output=True)
    controls = subprocess.run(["v4l2-ctl", "-d", device, "-C", "auto_exposure,gain"],
                              check=True, text=True, capture_output=True).stdout
    if not all(f"{name}: {value}" in controls for name, value in
               (("auto_exposure", 3), ("gain", 0))):
        raise RuntimeError(f"AR0234 controls differ from requested values: {controls}")
    return {"device": device, "width": WIDTH, "height": HEIGHT,
            "fourcc": "MJPG", "requested_stream_fps": CAMERA_FPS,
            "auto_exposure": 3, "gain": 0, "controls_readback": controls.strip()}


def compile_capture(output: Path) -> tuple[Path, str]:
    source = ROOT / "vision_core/tools/stream_v4l2_mjpeg.c"
    binary = output / "native_capture"
    subprocess.run(["cc", "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
                    str(source), "-o", str(binary)], check=True,
                   text=True, capture_output=True)
    return binary, sha256(source)


def slot_after(now: float, started: float, interval: float, current: int) -> int:
    """Skip missed deadlines; never claim a stale image belongs to a past slot."""
    return max(current, math.floor((now - started) / interval))


def save_packet(output: Path, index: int, payload: bytes, metadata: dict,
                scheduled: float, *, min_free_bytes: int) -> dict:
    if (not payload.startswith(b"\xff\xd8") or
            shutil.disk_usage(output).free - len(payload) < min_free_bytes):
        raise RuntimeError("INVALID_JPEG_OR_DISK_RESERVE_EXCEEDED")
    filename = f"images/ar0234_{index:06d}.jpg"
    with (output / filename).open("xb") as stream:
        stream.write(payload)
    return {"file": filename, "sha256": hashlib.sha256(payload).hexdigest(),
            "bytes": len(payload), "selected_at_utc": utc_now(),
            "scheduled_monotonic_s": scheduled,
            "driver_sequence": metadata["sequence"],
            "driver_timestamp_s": metadata["driver_timestamp_s"],
            "host_packet_received_monotonic_s": metadata["host_packet_received_monotonic_s"],
            "received_count": metadata["received_count"],
            "replaced_latest_count": metadata["replaced_latest_count"]}


def record_session(output: Path, *, duration_s: int = 600, sample_hz: int = 2,
                   reserve_mib: int = 1024, max_session_mib: int = 1024,
                   projector_state: str, scenario: str, device: str = DEVICE) -> dict:
    if (type(duration_s) is not int or not 1 <= duration_s <= 600
            or sample_hz not in (1, 2) or reserve_mib < 256 or max_session_mib < 256
            or projector_state not in ("on", "off") or not scenario.strip()):
        raise ValueError("INVALID_CAPTURE_SETTINGS")
    if output.exists():
        raise FileExistsError(f"refusing existing output: {output}")
    os.umask(0o077)
    output.mkdir(mode=0o700, parents=True, exist_ok=False)
    (output / "images").mkdir(mode=0o700)
    available = shutil.disk_usage(output).free
    if available < (reserve_mib + max_session_mib) * MIB:
        raise RuntimeError("INSUFFICIENT_FREE_DISK_SPACE")
    started_utc = utc_now()
    report = {"schema_version": "sie.shadow_rgb_presence_capture.v1",
              "status": "FAILED", "started_at_utc": started_utc,
              "scenario_annotation": scenario, "projector_state_operator_reported": projector_state,
              "source_code_sha256": sha256(Path(__file__)),
              "camera": None, "native_capture_source_sha256": None,
              "duration_requested_s": duration_s, "sample_hz": sample_hz,
              "samples_scheduled": duration_s * sample_hz,
              "reserve_free_mib": reserve_mib, "max_session_mib": max_session_mib,
              "frames_saved": 0, "slots_skipped": 0, "bytes_saved": 0,
              "image_decode_or_reencode": False, "raw_person_labels": "UNANNOTATED",
              "metric_measurement_authorized": False, "navigation_eligible": False,
              "actuator_access": False}
    capture = None
    started = None
    try:
        report["camera"] = configure_camera(device)
        binary, source_sha = compile_capture(output)
        report["native_capture_source_sha256"] = source_sha
        report["native_capture_binary_sha256"] = sha256(binary)
        # The native reader's private stderr file belongs with this session,
        # not to a potentially crowded or unwritable system temporary mount.
        previous_tempdir = tempfile.tempdir
        tempfile.tempdir = str(output)
        try:
            capture = LatestV4L2Capture(binary, device, WIDTH, HEIGHT, timeout=5.0)
        finally:
            tempfile.tempdir = previous_tempdir
        # Start the ten-minute clock only once the stream has delivered a frame.
        capture.snapshot()
        started = time.monotonic()
        interval = 1.0 / sample_hz
        deadline = started + duration_s
        previous_sequence = None
        index = 0
        slot = 0
        print(f"CAPTURE_STARTED {output} duration_s={duration_s} sample_hz={sample_hz}",
              flush=True)
        with (output / "frames.jsonl").open("x", encoding="utf-8") as journal:
            while slot < duration_s * sample_hz:
                slot = slot_after(time.monotonic(), started, interval, slot)
                scheduled = started + slot * interval
                if scheduled >= deadline:
                    break
                remaining = scheduled - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
                payload, meta = capture.snapshot(previous_sequence,
                                                 not_before_received_s=scheduled)
                if meta["host_packet_received_monotonic_s"] < scheduled:
                    raise RuntimeError("STALE_FRAME_SELECTED")
                previous_sequence = meta["sequence"]
                index += 1
                row = save_packet(output, index, payload, meta, scheduled,
                                  min_free_bytes=reserve_mib * MIB)
                row["slot"] = slot
                journal.write(json.dumps(row, allow_nan=False) + "\n")
                journal.flush()
                report["frames_saved"] = index
                report["bytes_saved"] += row["bytes"]
                if report["bytes_saved"] > max_session_mib * MIB:
                    raise RuntimeError("SESSION_SIZE_LIMIT_EXCEEDED")
                if index == 1 or index % 60 == 0:
                    print(f"SAVED {index} elapsed_s={time.monotonic()-started:.1f}", flush=True)
                slot += 1
        report["elapsed_monotonic_s"] = time.monotonic() - started
        report["slots_skipped"] = duration_s * sample_hz - index
        report["status"] = "COMPLETE" if report["slots_skipped"] == 0 else "COMPLETE_WITH_GAPS"
    except KeyboardInterrupt:
        report["status"] = "INTERRUPTED"
    except Exception as error:
        report["error"] = f"{type(error).__name__}: {error}"
        print("CAPTURE_FAILED", report["error"], file=sys.stderr, flush=True)
    finally:
        if capture is not None:
            try:
                capture.release()
            except Exception as error:
                report["status"] = "FAILED"
                report["cleanup_error"] = f"{type(error).__name__}: {error}"
        if started is not None:
            report["elapsed_monotonic_s"] = time.monotonic() - started
        report["finished_at_utc"] = utc_now()
        journal = output / "frames.jsonl"
        if journal.exists():
            report["frames_jsonl_sha256"] = sha256(journal)
        (output / "session.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n",
                                             encoding="utf-8")
    print("SUMMARY", json.dumps({k: report[k] for k in
          ("status", "frames_saved", "slots_skipped", "bytes_saved")}), flush=True)
    print("Saved:", output / "session.json", flush=True)
    return report


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, default=Path.home() / ".local/state/sie/evidence" /
                   ("shadow_rgb_presence_" + datetime.now().strftime("%Y%m%d_%H%M%S")
                    + "_" + secrets.token_hex(3)))
    p.add_argument("--scenario", required=True,
                   help="Operator description of person entering/leaving the frame")
    p.add_argument("--projector-state", required=True, choices=("on", "off"))
    p.add_argument("--duration-s", type=int, default=600)
    p.add_argument("--sample-hz", type=int, default=2, choices=(1, 2))
    p.add_argument("--reserve-mib", type=int, default=1024)
    p.add_argument("--max-session-mib", type=int, default=1024)
    args = p.parse_args()
    result = record_session(args.output, duration_s=args.duration_s,
                            sample_hz=args.sample_hz, reserve_mib=args.reserve_mib,
                            max_session_mib=args.max_session_mib,
                            projector_state=args.projector_state, scenario=args.scenario)
    return 0 if result["status"] in ("COMPLETE", "COMPLETE_WITH_GAPS") else 1


if __name__ == "__main__":
    raise SystemExit(main())
