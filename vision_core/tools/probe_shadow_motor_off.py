#!/usr/bin/env python3
"""Exercise Shadow pause/resume with motor power physically OFF.

This diagnostic intentionally sends START and RESUME; it must never be run
with motor power connected. It tests protocol state transitions, not motion.
"""
from __future__ import annotations

import argparse
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


def probe(request, *, clock=time.monotonic, sleep=time.sleep):
    def call(op, params=None):
        url = "http://uart.local" + op
        if params is not None:
            url += "?" + urlencode(params)
        return request("GET" if op == "/status" else "POST", url, .7)

    def wait_state(expected, command_id, seq, timeout=2.0):
        deadline = clock() + timeout
        while clock() < deadline:
            code, body = call("/status")
            if code != 200 or body.get("boot_session_id") != boot:
                raise RuntimeError("BOOT_OR_STATUS_CHANGED")
            if body.get("bounded_fault_latched") or body.get("steered_safety_latched"):
                raise RuntimeError("SHADOW_FAULT:" + str(body.get("bounded_fault_reason")))
            if (body.get("state") == expected and
                    body.get("active_command_id") == command_id and
                    body.get("shadow_transition_seq") == seq and
                    body.get("right_pwm") == 0 and body.get("left_pwm") == 0):
                return body
            sleep(.03)
        raise TimeoutError("SHADOW_PAUSE_NOT_CONFIRMED")

    code, ready = call("/status")
    if (code != 200 or ready.get("firmware_build_id") != BUILD or
            ready.get("shadow_api_version") != "v1_paused_segment" or
            ready.get("state") != "READY" or
            ready.get("active_command_id") is not None or
            ready.get("right_pwm") != 0 or ready.get("left_pwm") != 0 or
            ready.get("bounded_fault_latched") or
            ready.get("steered_safety_latched")):
        raise RuntimeError("SHADOW_PREFLIGHT_REFUSED")
    boot = ready["boot_session_id"]
    command_id = "shadow-off-" + secrets.token_hex(8)
    base = {"boot_session_id": boot, "command_id": command_id}
    started = False
    result = {"boot_session_id": boot, "command_id": command_id}
    try:
        started = True  # A timed-out request may already have started motion.
        code, body = call("/shadow-start", {**base, "seq": 1,
                                            "distance_m": "0.020"})
        if code != 202 or body.get("accepted") is not True:
            raise RuntimeError("SHADOW_START_REFUSED")
        code, body = call("/shadow-pause", {**base, "seq": 2})
        if code not in (200, 202):
            raise RuntimeError("SHADOW_FIRST_PAUSE_REFUSED")
        wait_state("PAUSED", command_id, 2)
        # Repeating the same pause must not allocate a second command.
        code, duplicate = call("/shadow-pause", {**base, "seq": 2})
        if code != 200 or duplicate.get("duplicate") is not True:
            raise RuntimeError("SHADOW_PAUSE_REPLAY_FAILED")
        code, body = call("/shadow-resume", {**base, "seq": 3,
                                             "distance_m": "0.020"})
        if code != 202 or body.get("accepted") is not True:
            raise RuntimeError("SHADOW_RESUME_REFUSED")
        code, body = call("/shadow-pause", {**base, "seq": 4})
        if code not in (200, 202):
            raise RuntimeError("SHADOW_SECOND_PAUSE_REFUSED")
        wait_state("PAUSED", command_id, 4)
        code, body = call("/shadow-stop", {**base, "seq": 5})
        if code not in (200, 202):
            raise RuntimeError("SHADOW_STOP_REFUSED")
        deadline = clock() + 2.0
        while clock() < deadline:
            code, final = call("/status")
            if (code == 200 and final.get("state") == "READY" and
                    final.get("last_command_id") == command_id and
                    final.get("last_command_state") == "STOPPED" and
                    final.get("active_command_id") is None and
                    final.get("right_pwm") == final.get("left_pwm") == 0 and
                    final.get("remote_stop_pending") is False):
                result["final_state"] = "READY"
                result["same_id_resumed"] = True
                result["motor_power_required"] = False
                return result
            sleep(.03)
        raise TimeoutError("SHADOW_TERMINAL_STOP_NOT_CONFIRMED")
    finally:
        if started and "final_state" not in result:
            try:
                call("/stop")
            except BaseException:
                pass


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--serial-port", required=True)
    p.add_argument("--motors-unpowered", action="store_true", required=True)
    args = p.parse_args()
    with UartHttp(args.serial_port) as request:
        print(json.dumps(probe(request)))


if __name__ == "__main__":
    main()
