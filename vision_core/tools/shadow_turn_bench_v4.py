"""One-shot lifted-wheel SHADOW v4 TURN commissioning over GPIO UART.

Never use this tool on a chassis whose powered wheels touch the floor.
No camera tracking, range inference or autonomous follow is performed here.
The human operator owns motor power cutoff; never retries motion commands.
"""
from __future__ import annotations

from datetime import datetime, timezone
import fcntl
import json
from pathlib import Path
import secrets
import time
from urllib.parse import urlencode

from vision_core.tools.uart_http_transport import UartHttp


PORT = "/dev/ttyAMA0"
BUILD = "v4_2_uart_shadow_v4_edge_turn_10deg"
API = "v2_paused_edge_turn"
LOCK = Path.home() / ".local/state/sie/v7_operator_one_step.lock"


def main():
    evidence = (Path.home() / ".local/state/sie/evidence" /
                ("shadow_turn_bench_v4_" + datetime.now().strftime("%Y%m%d_%H%M%S")))
    evidence.mkdir(parents=True, exist_ok=False)
    path = evidence / "uart_bench.jsonl"
    LOCK.parent.mkdir(parents=True, exist_ok=True)
    command_id = "sie-v4-bench-" + secrets.token_hex(6)
    started = False
    success = False
    stop_confirmed = False
    error = None

    with LOCK.open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with path.open("x") as journal:
            def emit(event, **values):
                item = {"event": event,
                        "at_utc": datetime.now(timezone.utc).isoformat(),
                        **values}
                journal.write(json.dumps(item, ensure_ascii=False,
                                         allow_nan=False) + "\n")
                journal.flush()
                if event not in ("STATUS_POLL",):
                    print(json.dumps(item, ensure_ascii=False,
                                     allow_nan=False), flush=True)

            def request(uart, method, endpoint, params=None, *, timeout=2.0):
                url = "http://uart.local" + endpoint
                if params is not None:
                    url += "?" + urlencode(params)
                try:
                    code, response = uart(method, url, timeout)
                    emit("UART_REPLY", method=method, endpoint=endpoint,
                         query=params, code=code, response=response)
                    return code, response
                except BaseException as exc:
                    emit("UART_ERROR", method=method, endpoint=endpoint,
                         query=params, error=type(exc).__name__ + ": " + str(exc))
                    raise

            def status(uart, *, log=True):
                code, data = request(uart, "GET", "/status")
                if code != 200 or not isinstance(data, dict):
                    raise RuntimeError("UART_STATUS_REJECTED")
                if (data.get("firmware_build_id") != BUILD or
                        data.get("shadow_api_version") != API):
                    raise RuntimeError("UNEXPECTED_FIRMWARE")
                if log:
                    emit("STATUS_POLL", state=data.get("state"),
                         active_command_id=data.get("active_command_id"),
                         seq=data.get("shadow_transition_seq"),
                         turn_result=data.get("shadow_turn_result"),
                         right_pwm=data.get("right_pwm"),
                         left_pwm=data.get("left_pwm"),
                         heading_rad=data.get("heading_rad"),
                         right_encoder=data.get("right_encoder_raw_count"),
                         left_encoder=data.get("left_encoder_raw_count"))
                return data

            def assert_healthy(data):
                if (data.get("bounded_fault_latched") or
                        data.get("steered_safety_latched") or
                        data.get("fault") or data.get("state") == "FAULT"):
                    raise RuntimeError("ESP32_FAULT:" + str(data.get("fault")))

            def wait_paused(uart, *, seq, turn=False, duration=6.5):
                deadline = time.monotonic() + duration
                while time.monotonic() < deadline:
                    data = status(uart)
                    assert_healthy(data)
                    if (data.get("state") == "PAUSED" and
                            data.get("active_command_id") == command_id and
                            data.get("shadow_transition_seq") == seq and
                            data.get("right_pwm") == 0 and
                            data.get("left_pwm") == 0):
                        if turn:
                            result = data.get("shadow_turn_result")
                            if result not in ("SUCCESS", "PARTIAL_PROGRESS"):
                                raise RuntimeError("TURN_NOT_CONFIRMED:" + str(result))
                        return data
                    time.sleep(.12)
                raise TimeoutError("PAUSED_NOT_CONFIRMED")

            emit("BENCH_START", evidence=str(evidence), command_id=command_id,
                 direction="RIGHT", angle_deg=5.0, forward_seed_m=0.03)

            with UartHttp(PORT) as uart:
                boot = None
                try:
                    before = status(uart)
                    assert_healthy(before)
                    if (before.get("state") != "READY" or
                            before.get("active_command_id") is not None or
                            before.get("right_pwm") != 0 or
                            before.get("left_pwm") != 0 or
                            before.get("remote_stop_pending")):
                        raise RuntimeError("CONTROLLER_NOT_READY")
                    boot = before["boot_session_id"]
                    # 30 mm segment only to enter the existing SHADOW session.
                    # It is immediately paused, without a camera-based drive loop.
                    p = {"boot_session_id": boot, "command_id": command_id,
                         "seq": 1, "distance_m": "0.030", "bias_pwm": "0"}
                    started = True  # uncertain ACK must still trigger terminal STOP
                    code, reply = request(uart, "POST", "/shadow-start", p)
                    if code != 202 or reply.get("accepted") is not True:
                        raise RuntimeError("START_REJECTED:" + str(reply))
                    time.sleep(.12)
                    p = {"boot_session_id": boot, "command_id": command_id, "seq": 2}
                    code, reply = request(uart, "POST", "/shadow-pause", p)
                    if code not in (200, 202) or reply.get("accepted") is not True:
                        raise RuntimeError("PAUSE_REJECTED:" + str(reply))
                    paused = wait_paused(uart, seq=2, duration=3.0)
                    emit("PAUSE_CONFIRMED",
                         state=paused["state"],
                         right_pwm=paused["right_pwm"],
                         left_pwm=paused["left_pwm"])

                    p = {"boot_session_id": boot, "command_id": command_id,
                         "seq": 3, "direction": "RIGHT", "angle_deg": "5.00"}
                    code, reply = request(uart, "POST", "/shadow-turn", p)
                    if code != 202 or reply.get("accepted") is not True:
                        raise RuntimeError("TURN_REJECTED:" + str(reply))
                    after = wait_paused(uart, seq=3, turn=True)
                    emit("TURN_SETTLED",
                         result=after.get("shadow_turn_result"),
                         heading_rad=after.get("heading_rad"),
                         right_encoder=after.get("right_encoder_raw_count"),
                         left_encoder=after.get("left_encoder_raw_count"),
                         right_pwm=after.get("right_pwm"),
                         left_pwm=after.get("left_pwm"))
                    if after.get("shadow_turn_result") != "SUCCESS":
                        raise RuntimeError("TURN_PARTIAL_PROGRESS")
                    success = True
                except BaseException as exc:
                    error = type(exc).__name__ + ": " + str(exc)
                    emit("BENCH_REFUSAL", error=error)
                finally:
                    # STOP is out-of-band, never requires a transition sequence.
                    # Do not resend a motor command after a timeout.
                    if started:
                        try:
                            request(uart, "POST", "/stop", timeout=2.0)
                        except BaseException as exc:
                            emit("STOP_REPLY_UNCERTAIN", error=str(exc))
                        deadline = time.monotonic() + 3.0
                        while time.monotonic() < deadline:
                            try:
                                final = status(uart)
                                if (final.get("state") in ("READY", "FAULT") and
                                        final.get("active_command_id") is None and
                                        final.get("right_pwm") == 0 and
                                        final.get("left_pwm") == 0 and
                                        not final.get("remote_stop_pending")):
                                    stop_confirmed = True
                                    emit("STOP_CONFIRMED", final_state=final["state"],
                                         right_pwm=0, left_pwm=0,
                                         last_command_id=final.get("last_command_id"))
                                    break
                            except BaseException as exc:
                                emit("STOP_STATUS_ERROR", error=str(exc))
                            time.sleep(.12)
                    else:
                        stop_confirmed = True
                    emit("BENCH_SUMMARY", turn_success=success,
                         stop_confirmed=stop_confirmed,
                         result=("PASS" if success and stop_confirmed
                                 else "REFUSED"),
                         error=error, evidence=str(evidence))

    print("EVIDENCE:", str(evidence))
    if not (success and stop_confirmed):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
