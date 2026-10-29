#!/usr/bin/env python3
"""Bench-only Shadow brake/resume test with BOTH drive wheels lifted.

Starts two bounded 0.30 m segments with one command ID. A human must verify
both wheels are clear of the floor and have an accessible motor power cutoff.
This tool does not use a camera or establish person-following safety.
"""
from __future__ import annotations

import argparse
import fcntl
import json
from pathlib import Path
import secrets
import sys
import time
from urllib.parse import urlencode

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from vision_core.tools.uart_http_transport import UartHttp


BUILD = "v4_2_uart_shadow_v1_candidate"
SEGMENT_M = "0.300"
MIN_WHEEL_TRAVEL_M = .004
MAX_PAUSED_COUNT_DRIFT = 2


def verify_status(status, boot, command_id):
    if status.get("firmware_build_id") != BUILD or status.get("boot_session_id") != boot:
        raise RuntimeError("FIRMWARE_OR_BOOT_CHANGED")
    if status.get("bounded_fault_latched") or status.get("steered_safety_latched") or \
            status.get("state") == "FAULT":
        raise RuntimeError("CONTROLLER_FAULT:" + str(status.get("bounded_fault_reason")
                                                     or status.get("fault")))
    if status.get("active_command_id") != command_id:
        raise RuntimeError("ACTIVE_COMMAND_CHANGED")
    for key in ("right_encoder_raw_count", "left_encoder_raw_count",
                "right_distance_m", "left_distance_m"):
        if type(status.get(key)) not in (int, float):
            raise RuntimeError("MISSING_TELEMETRY:" + key)
    return status


def check_pause_settled(before, after):
    for status in (before, after):
        if status.get("state") != "PAUSED" or status.get("remote_stop_pending") is not False or \
                status.get("right_pwm") != 0 or status.get("left_pwm") != 0:
            raise RuntimeError("PAUSE_NOT_STATIONARY")
    drift = {wheel: abs(after[f"{wheel}_encoder_raw_count"] -
                        before[f"{wheel}_encoder_raw_count"])
             for wheel in ("right", "left")}
    if max(drift.values()) > MAX_PAUSED_COUNT_DRIFT:
        raise RuntimeError("WHEEL_MOVED_AFTER_PAUSE:" + str(drift))
    return drift


def probe(request, journal, *, clock=time.monotonic, sleep=time.sleep):
    """Run the bench probe. Always call /stop after any accepted START."""
    def call(endpoint, params=None, timeout=.55):
        url = "http://uart.local" + endpoint
        if params is not None:
            url += "?" + urlencode(params)
        row = {"endpoint": endpoint, "params": params, "started_s": clock()}
        try:
            code, body = request("GET" if endpoint == "/status" else "POST", url,
                                 timeout)
            row.update(code=code, body=body)
            return code, body
        except BaseException as error:
            row["error"] = type(error).__name__ + ": " + str(error)
            raise
        finally:
            row["completed_s"] = clock()
            journal.write(json.dumps(row, allow_nan=False) + "\n")
            journal.flush()

    code, ready = call("/status")
    if code != 200 or ready.get("firmware_build_id") != BUILD or \
            ready.get("shadow_api_version") != "v1_paused_segment" or \
            ready.get("state") != "READY" or ready.get("active_command_id") is not None or \
            ready.get("right_pwm") != 0 or ready.get("left_pwm") != 0 or \
            ready.get("bounded_fault_latched") or ready.get("steered_safety_latched") or \
            ready.get("remote_stop_pending"):
        raise RuntimeError("SHADOW_PREFLIGHT_REFUSED")
    boot = ready["boot_session_id"]
    command_id = "shadow-lift-" + secrets.token_hex(8)
    base = {"boot_session_id": boot, "command_id": command_id}
    result = {"boot_session_id": boot, "command_id": command_id,
              "segments": [], "stop_confirmed": False}
    may_have_started = False
    steer_seq = 0

    def status():
        code, body = call("/status")
        if code != 200:
            raise RuntimeError("STATUS_UNAVAILABLE")
        return verify_status(body, boot, command_id)

    def moving_segment(transition_seq):
        nonlocal steer_seq
        deadline = clock() + 2.8
        next_heartbeat = 0.0
        while clock() < deadline:
            sample = status()
            state = sample["state"]
            if sample.get("shadow_transition_seq") != transition_seq or \
                    sample.get("shadow_segments_started") != (transition_seq + 1) // 2:
                raise RuntimeError("TRANSITION_OR_SEGMENT_CHANGED")
            if state not in ("STARTING", "DRIVING"):
                raise RuntimeError("SEGMENT_ENDED_BEFORE_ACTIVE_PAUSE:" + str(state))
            # startMotion resets both raw counters at START and RESUME.
            # Forward wheel motion is right raw < 0, left raw > 0.
            right = sample["right_encoder_raw_count"]
            left = sample["left_encoder_raw_count"]
            if (state == "DRIVING" and sample["right_pwm"] > 0 and
                    sample["left_pwm"] > 0 and
                    sample["right_distance_m"] >= MIN_WHEEL_TRAVEL_M and
                    sample["left_distance_m"] >= MIN_WHEEL_TRAVEL_M and
                    right <= -3 and left >= 3):
                return {"right_encoder_raw_count": right,
                        "left_encoder_raw_count": left,
                        "right_distance_m": sample["right_distance_m"],
                        "left_distance_m": sample["left_distance_m"],
                        "right_pwm": sample["right_pwm"],
                        "left_pwm": sample["left_pwm"]}
            if clock() >= next_heartbeat:
                steer_seq += 1
                code, reply = call("/steer-update", {**base, "seq": steer_seq,
                                                      "bias_pwm": 0})
                if code != 200 or reply.get("accepted") is not True:
                    raise RuntimeError("ZERO_BIAS_HEARTBEAT_REFUSED:" + str(reply))
                next_heartbeat = clock() + .18
            sleep(.025)
        raise TimeoutError("BOTH_WHEELS_NOT_OBSERVED_WHILE_DRIVING")

    def pause_and_check(transition_seq):
        code, reply = call("/shadow-pause", {**base, "seq": transition_seq})
        if code not in (200, 202) or reply.get("accepted") is not True:
            raise RuntimeError("PAUSE_REFUSED:" + str(reply))
        deadline = clock() + 2.0
        while clock() < deadline:
            sample = status()
            if sample["state"] == "PAUSED":
                if sample.get("shadow_transition_seq") != transition_seq:
                    raise RuntimeError("PAUSE_SEQUENCE_CHANGED")
                sleep(.25)
                stable = status()
                return check_pause_settled(sample, stable), stable
            sleep(.03)
        raise TimeoutError("PAUSE_NOT_CONFIRMED")

    try:
        may_have_started = True  # A lost START ACK can still mean motion.
        code, reply = call("/shadow-start", {**base, "seq": 1,
                                              "distance_m": SEGMENT_M})
        if code != 202 or reply.get("accepted") is not True:
            raise RuntimeError("START_REFUSED:" + str(reply))
        first = moving_segment(1)
        drift, _ = pause_and_check(2)
        result["segments"].append({"movement": first, "paused_count_drift": drift})

        code, reply = call("/shadow-resume", {**base, "seq": 3,
                                               "distance_m": SEGMENT_M})
        if code != 202 or reply.get("accepted") is not True:
            raise RuntimeError("RESUME_REFUSED:" + str(reply))
        second = moving_segment(3)
        drift, _ = pause_and_check(4)
        result["segments"].append({"movement": second, "paused_count_drift": drift})
        code, reply = call("/shadow-stop", {**base, "seq": 5})
        if code not in (200, 202) or reply.get("accepted") is not True:
            raise RuntimeError("SHADOW_STOP_REFUSED:" + str(reply))
        result["result"] = "TWO_SEGMENTS_PAUSED_AND_RESUMED"
    except BaseException as error:
        result["result"] = "STOPPED_ON_REFUSAL"
        result["error"] = type(error).__name__ + ": " + str(error)
    finally:
        if may_have_started:
            if result["result"] != "TWO_SEGMENTS_PAUSED_AND_RESUMED":
                try:
                    call("/stop")  # Terminal brake even after uncertain START/RESUME.
                except BaseException as error:
                    result["stop_error"] = type(error).__name__ + ": " + str(error)
            try:
                deadline = clock() + 2.5
                while clock() < deadline:
                    code, final = call("/status")
                    if code == 200 and final.get("boot_session_id") == boot and \
                            final.get("state") == "READY" and \
                            final.get("last_command_id") == command_id and \
                            final.get("last_command_state") == "STOPPED" and \
                            final.get("active_command_id") is None and \
                            final.get("right_pwm") == final.get("left_pwm") == 0 and \
                            final.get("remote_stop_pending") is False:
                        result["stop_confirmed"] = True
                        break
                    sleep(.04)
                if not result["stop_confirmed"]:
                    result["last_status"] = final
            except BaseException as error:
                result["confirmation_error"] = type(error).__name__ + ": " + str(error)
            if not result["stop_confirmed"]:
                try:
                    call("/stop")
                except BaseException as error:
                    result["stop_error"] = type(error).__name__ + ": " + str(error)
                result["action"] = "CUT_MOTOR_POWER_NOW"
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--serial-port", required=True)
    p.add_argument("--wheels-lifted", action="store_true", required=True,
                   help="Both powered wheels supported clear of the floor")
    p.add_argument("--operator-at-cutoff", action="store_true", required=True,
                   help="Operator observing wheels with motor power cutoff at hand")
    args = p.parse_args()
    root = Path.home() / ".local/state/sie/evidence" / (
        "shadow_lifted_probe_" + str(int(time.time())) + "_" + secrets.token_hex(4))
    root.mkdir(parents=True, exist_ok=False)
    lockpath = Path.home() / ".local/state/sie/v7_operator_one_step.lock"
    lockpath.parent.mkdir(parents=True, exist_ok=True)
    with lockpath.open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with (root / "uart.jsonl").open("x") as journal:
            with UartHttp(args.serial_port) as request:
                result = probe(request, journal)
    result["journal"] = str(root / "uart.jsonl")
    print(json.dumps(result, indent=2))
    return 0 if (result.get("result") == "TWO_SEGMENTS_PAUSED_AND_RESUMED" and
                 result.get("stop_confirmed")) else 1


if __name__ == "__main__":
    sys.exit(main())
