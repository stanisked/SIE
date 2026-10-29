"""Injected Shadow UART session runner; requires a validated live range provider.

There is intentionally no CLI that can start motors from the existing RGB-only
preview. The caller supplies fresh RGB identity and associated calibrated
person range. All motor traffic is journaled and uses one command ID.
"""
from __future__ import annotations

from dataclasses import dataclass
import fcntl
import json
import secrets
import time
from pathlib import Path
from urllib.parse import urlencode

from vision_core.tools.shadow_range_gate import (
    RgbTarget, TargetRange, select_target_range,
)
from vision_core.tools.shadow_supervisor import ShadowSupervisor


BUILD = "v4_2_uart_shadow_v1_candidate"
SEGMENT_M = .30


@dataclass(frozen=True)
class ShadowObservation:
    target: RgbTarget | None
    ranges: tuple[TargetRange, ...]
    camera_encoder_disagreement: bool = False


def run_session(*, request, observations, journal_path, rgb_calibration_id,
                lidar_calibration_id=None, clock=time.monotonic,
                segment_m=SEGMENT_M, max_observations=None,
                lock_path=None):
    """Use the same operator lock as the current bounded steering runner."""
    lockpath = Path(lock_path) if lock_path is not None else (
        Path.home() / ".local/state/sie/v7_operator_one_step.lock")
    lockpath.parent.mkdir(parents=True, exist_ok=True)
    with lockpath.open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return _run_session_unlocked(request=request, observations=observations,
            journal_path=journal_path, rgb_calibration_id=rgb_calibration_id,
            lidar_calibration_id=lidar_calibration_id, clock=clock,
            segment_m=segment_m, max_observations=max_observations)


def _run_session_unlocked(*, request, observations, journal_path, rgb_calibration_id,
                lidar_calibration_id=None, clock=time.monotonic,
                segment_m=SEGMENT_M, max_observations=None):
    """Run until source ends or operator interrupts; always request terminal STOP.

    `observations` must yield an observation or None frequently even if the
    camera has no new frame. This function cannot establish identity or
    distance calibration on its own. Do not call on a physical cart without
    a validated target/range producer and an independent motor cutoff.
    """
    if not .02 <= segment_m <= .30:
        raise ValueError("INVALID_SHADOW_SEGMENT")
    output = Path(journal_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    report = {"result": "NO_COMMAND", "command_id": None,
              "operations": [], "measurement_admission_changed": False}
    supervisor = None
    latest = None
    seen = 0
    last_steer_at = 0.0

    with output.open("x") as journal:
        def call(method, endpoint, params=None, timeout=.3):
            url = "http://uart.local" + endpoint
            if params is not None:
                url += "?" + urlencode(params)
            row = {"method": method, "endpoint": endpoint,
                   "query": params, "started_s": clock()}
            try:
                code, body = request(method, url, timeout)
                row.update(code=code, body=body)
                return code, body
            except BaseException as error:
                row["error"] = type(error).__name__ + ": " + str(error)
                raise
            finally:
                row["completed_s"] = clock()
                journal.write(json.dumps(row, allow_nan=False) + "\n")
                journal.flush()

        def status():
            code, body = call("GET", "/status", timeout=.6)
            if code != 200 or body.get("firmware_build_id") != BUILD or \
                    body.get("shadow_api_version") != "v1_paused_segment":
                raise RuntimeError("SHADOW_CONTROLLER_STATUS_INVALID")
            if supervisor is not None and body.get("boot_session_id") != supervisor.boot_session_id:
                raise RuntimeError("BOOT_SESSION_CHANGED")
            return body

        def reconcile(body):
            supervisor.reconcile(
                boot_session_id=body["boot_session_id"],
                active_command_id=body.get("active_command_id"),
                last_command_id=body.get("last_command_id"),
                controller_state=body.get("state"),
                right_pwm=body.get("right_pwm"), left_pwm=body.get("left_pwm"),
                fault_latched=body.get("bounded_fault_latched") or
                              body.get("steered_safety_latched"))

        def issue(intent, original_target=None):
            nonlocal last_steer_at
            endpoint = {"START": "/shadow-start", "PAUSE": "/shadow-pause",
                        "RESUME": "/shadow-resume", "STOP": "/shadow-stop",
                        "STEER": "/steer-update"}[intent.operation]
            params = {"boot_session_id": intent.boot_session_id,
                      "command_id": intent.command_id,
                      "seq": intent.transition_seq}
            if intent.operation in ("START", "RESUME"):
                params["distance_m"] = format(segment_m, ".3f")
            if intent.operation == "STEER":
                params["bias_pwm"] = intent.bias_pwm
            try:
                code, body = call("POST", endpoint, params)
            except TimeoutError:
                # A lost ACK may still mean that motors started. Status is the
                # only recovery source; no second start/resume is transmitted.
                code, body = None, None
            if code is not None and code not in (200, 202):
                observed = status()
                reconcile(observed)
                if ((intent.operation == "STEER" and supervisor.state == "PAUSED") or
                        (intent.operation == "STOP" and supervisor.state == "TERMINAL")):
                    return
                raise RuntimeError("SHADOW_OPERATION_REJECTED:" + str(body))
            deadline = clock() + .55 if intent.operation in ("START", "RESUME") else clock() + 1.6
            last_heartbeat = 0.0
            while clock() < deadline:
                observed = status()
                if observed.get("bounded_fault_latched") or observed.get("steered_safety_latched"):
                    reconcile(observed)
                    raise RuntimeError("SHADOW_CONTROLLER_FAULT")
                state = observed.get("state")
                if (intent.operation in ("START", "RESUME") and state == "STARTING"
                        and clock() - last_heartbeat >= .18
                        and original_target is not None
                        and clock() - original_target.observed_s <= .5):
                    heartbeat = supervisor.starting_heartbeat()
                    code, reply = call("POST", "/steer-update", {
                        "boot_session_id": heartbeat.boot_session_id,
                        "command_id": heartbeat.command_id,
                        "seq": heartbeat.transition_seq, "bias_pwm": 0})
                    if code != 200 or reply.get("accepted") is not True:
                        raise RuntimeError("SHADOW_START_HEARTBEAT_REFUSED")
                    last_heartbeat = clock()
                if state == "PAUSED" and observed.get("active_command_id") == intent.command_id:
                    reconcile(observed)
                    return
                if state == "DRIVING" and intent.operation in ("START", "RESUME", "STEER"):
                    seq_key = ("active_steer_update_seq" if intent.operation == "STEER"
                               else "shadow_transition_seq")
                    if observed.get(seq_key) == intent.transition_seq:
                        supervisor.acknowledge(intent,
                            boot_session_id=observed["boot_session_id"],
                            active_command_id=observed.get("active_command_id"),
                            controller_state=state, right_pwm=observed["right_pwm"],
                            left_pwm=observed["left_pwm"],
                            fault_latched=False,
                            transition_seq=observed[seq_key])
                        last_steer_at = clock()
                        return
                if state in ("PAUSED", "READY") and intent.operation in ("PAUSE", "STOP"):
                    supervisor.acknowledge(intent,
                        boot_session_id=observed["boot_session_id"],
                        active_command_id=observed.get("active_command_id"),
                        last_command_id=observed.get("last_command_id"),
                        controller_state=state, right_pwm=observed["right_pwm"],
                        left_pwm=observed["left_pwm"], fault_latched=False,
                        transition_seq=observed.get("shadow_transition_seq"))
                    return
                time.sleep(.035)
            raise TimeoutError("SHADOW_STATE_CONFIRMATION_TIMEOUT")

        try:
            ready = status()
            if ready.get("state") != "READY" or ready.get("active_command_id") is not None or \
                    ready.get("right_pwm") != 0 or ready.get("left_pwm") != 0 or \
                    ready.get("bounded_fault_latched") or ready.get("steered_safety_latched"):
                raise RuntimeError("SHADOW_CONTROLLER_NOT_READY")
            supervisor = ShadowSupervisor(ready["boot_session_id"],
                                          "sie-shadow-" + secrets.token_hex(12))
            report["command_id"] = supervisor.command_id
            for observation in observations:
                seen += 1
                if max_observations is not None and seen > max_observations:
                    break
                if observation is not None:
                    latest = observation
                target = latest.target if latest is not None else None
                ranges = list(latest.ranges) if latest is not None else []
                now = clock()
                decision = select_target_range(target, ranges, now,
                    expected_rgb_calibration_id=rgb_calibration_id,
                    expected_lidar_calibration_id=lidar_calibration_id)
                # Poll every input tick: a bounded segment can end on its own.
                observed = status()
                reconcile(observed)
                if supervisor.state in ("FAULT", "TERMINAL"):
                    break
                if observed.get("state") in ("BRAKING", "COASTING"):
                    continue
                if (supervisor.state == "ACTIVE" and decision.action == "FOLLOW_ELIGIBLE"
                        and now - last_steer_at < .15):
                    continue
                bias = 0 if target is None else max(-20, min(20, round(target.bearing_deg * 4)))
                intent = supervisor.observe(target, decision, bias_pwm=bias,
                    camera_encoder_disagreement=(latest.camera_encoder_disagreement
                                                  if latest is not None else False))
                if intent is None:
                    continue
                report["operations"].append(intent.operation)
                issue(intent, target)
            report["result"] = "SESSION_ENDED"
        except BaseException as error:
            report["result"] = "STOPPED_ON_REFUSAL"
            report["error"] = type(error).__name__ + ": " + str(error)
        finally:
            if (supervisor is not None and supervisor.state != "TERMINAL" and
                    (report["operations"] or supervisor.state != "IDLE")):
                # Terminal STOP is deliberately available even if a transition
                # acknowledgement was lost. Never resume after ambiguity.
                try:
                    call("POST", "/stop", timeout=1.0)
                    deadline = clock() + 3.0
                    while clock() < deadline:
                        final = status()
                        report["final_status"] = final
                        if (final.get("state") in ("READY", "FAULT") and
                                final.get("active_command_id") is None and
                                final.get("last_command_id") == supervisor.command_id and
                                final.get("right_pwm") == 0 and
                                final.get("left_pwm") == 0 and
                                final.get("remote_stop_pending") is False):
                            break
                        time.sleep(.04)
                    else:
                        report["result"] = "STOP_NOT_CONFIRMED"
                except BaseException as error:
                    report["result"] = "STOP_NOT_CONFIRMED"
                    report["stop_error"] = type(error).__name__ + ": " + str(error)
            journal.write(json.dumps({"summary": report}, allow_nan=False) + "\n")
    return report
