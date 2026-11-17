"""Injected Shadow UART session runner; requires a validated live range provider.

There is intentionally no CLI that can start motors from the existing RGB-only
preview. The caller supplies fresh RGB identity and associated calibrated
person range. All motor traffic is journaled and uses one command ID.
"""
from __future__ import annotations

from dataclasses import dataclass
import fcntl
import json
import math
import secrets
import time
from pathlib import Path
from urllib.parse import urlencode

from vision_core.tools.shadow_range_gate import (
    RgbTarget, TargetRange, RangeDecision, select_target_range,
    SHADOW_STOP_DISTANCE_M, SHADOW_RESUME_MARGIN_M,
    SHADOW_COMMAND_CLEARANCE_M, SHADOW_MIN_SEGMENT_M,
)
from vision_core.tools.shadow_supervisor import ShadowSupervisor


BUILD = "v4_2_uart_shadow_v3_segment_0p4"
EDGE_TURN_BUILD = "v4_2_uart_shadow_v4_edge_turn_10deg"
SEGMENT_M = .40
EDGE_TURN_MIN_PRIOR_CLEARANCE_M = 1.25
EDGE_TURN_MAX_PRIOR_AGE_S = 1.5
EDGE_TURN_MAX_ANGLE_DEG = 8.0


@dataclass(frozen=True)
class ShadowObservation:
    target: RgbTarget | None
    ranges: tuple[TargetRange, ...]
    camera_encoder_disagreement: bool = False
    edge_recovery_bearing_deg: float | None = None


def select_edge_turn(target: RgbTarget | None, edge_bearing_deg: float | None,
                     last_verified: tuple[str, float, float] | None,
                     now_s: float) -> tuple[str, float] | None:
    """Single bounded orientation correction after a fresh same-person edge.

    A recently associated front clearance is required; RGB alone cannot
    authorize a pivot near people. Caller enforces one turn per session.
    """
    if (target is None or last_verified is None or
            type(edge_bearing_deg) not in (float, int) or
            not math.isfinite(edge_bearing_deg) or
            not math.isfinite(now_s) or
            last_verified[0] != target.track_id or
            not 0 <= now_s - target.observed_s <= .5 or
            not 0 <= now_s - last_verified[1] <= EDGE_TURN_MAX_PRIOR_AGE_S or
            last_verified[2] < EDGE_TURN_MIN_PRIOR_CLEARANCE_M or
            abs(edge_bearing_deg) < 1.0):
        return None
    direction = "RIGHT" if edge_bearing_deg > 0 else "LEFT"
    angle = min(EDGE_TURN_MAX_ANGLE_DEG, abs(edge_bearing_deg))
    return direction, round(angle, 2)



def run_session(*, request, observations, journal_path, rgb_calibration_id,
                lidar_calibration_id=None, clock=time.monotonic,
                segment_m=SEGMENT_M, max_observations=None, max_segments=None,
                minimum_confidence=.8, lock_path=None,
                latest_observation=None, enable_edge_turn=False):
    """Use the same operator lock as the current bounded steering runner."""
    lockpath = Path(lock_path) if lock_path is not None else (
        Path.home() / ".local/state/sie/v7_operator_one_step.lock")
    lockpath.parent.mkdir(parents=True, exist_ok=True)
    with lockpath.open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return _run_session_unlocked(request=request, observations=observations,
            journal_path=journal_path, rgb_calibration_id=rgb_calibration_id,
            lidar_calibration_id=lidar_calibration_id, clock=clock,
            segment_m=segment_m, max_observations=max_observations,
            max_segments=max_segments, minimum_confidence=minimum_confidence,
            latest_observation=latest_observation,
            enable_edge_turn=enable_edge_turn)


def _run_session_unlocked(*, request, observations, journal_path, rgb_calibration_id,
                lidar_calibration_id=None, clock=time.monotonic,
                segment_m=SEGMENT_M, max_observations=None, max_segments=None,
                minimum_confidence=.8, latest_observation=None,
                enable_edge_turn=False):
    """Run until source ends or operator interrupts; always request terminal STOP.

    `observations` must yield an observation or None frequently even if the
    camera has no new frame. This function cannot establish identity or
    distance calibration on its own. A new RGB/range pair is required for each
    RESUME; `max_segments` bounds attempted START/RESUME segments. Do not call
    on a physical cart without a validated target/range producer and an
    independent motor cutoff.
    """
    if type(enable_edge_turn) is not bool:
        raise ValueError("INVALID_EDGE_TURN_SWITCH")
    if not SHADOW_MIN_SEGMENT_M <= segment_m <= .40:
        raise ValueError("INVALID_SHADOW_SEGMENT")
    if max_segments is not None and (type(max_segments) is not int or
                                     max_segments < 1):
        raise ValueError("INVALID_MAX_SEGMENTS")
    if (type(minimum_confidence) not in (float, int) or
            not 0 < minimum_confidence <= 1 or
            not math.isfinite(minimum_confidence)):
        raise ValueError("INVALID_MINIMUM_CONFIDENCE")
    output = Path(journal_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    report = {"result": "NO_COMMAND", "command_id": None,
              "operations": [], "segments_started": 0,
              "reused_pair_refusals": 0,
              "measurement_admission_changed": False,
              "edge_turns_started": 0}
    supervisor = None
    latest = None
    seen = 0
    last_steer_at = 0.0
    last_segment_pair = None
    last_decision_key = None
    last_edge_recovery_track = None
    last_verified_range = None
    post_turn_reobserve_after_s = None

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
            code, body = call("GET", "/status", timeout=2.0)
            contract = (body.get("firmware_build_id"),
                        body.get("shadow_api_version"))
            valid_contracts = {(BUILD, "v1_paused_segment"),
                               (EDGE_TURN_BUILD, "v2_paused_edge_turn")}
            if (code != 200 or contract not in valid_contracts or
                    (enable_edge_turn and contract !=
                        (EDGE_TURN_BUILD, "v2_paused_edge_turn"))):
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

        def fresh_steering_observation(expected_track_id, after_observed_s):
            """New current same-person range only, never RGB-edge-only guidance."""
            candidate = latest_observation() if latest_observation is not None else None
            if (candidate is None or candidate.camera_encoder_disagreement
                    or candidate.edge_recovery_bearing_deg is not None):
                return None
            target = candidate.target
            if (target is None or target.track_id != expected_track_id
                    or target.observed_s <= after_observed_s):
                return None
            decision = select_target_range(target, list(candidate.ranges), clock(),
                expected_rgb_calibration_id=rgb_calibration_id,
                expected_lidar_calibration_id=lidar_calibration_id,
                minimum_confidence=minimum_confidence)
            if decision.action != "FOLLOW_ELIGIBLE":
                return None
            return target, decision

        def issue(intent, original_target=None, selected_range=None):
            nonlocal last_steer_at
            endpoint = {"START": "/shadow-start", "PAUSE": "/shadow-pause",
                        "RESUME": "/shadow-resume", "STOP": "/shadow-stop",
                        "STEER": "/steer-update",
                        "TURN": "/shadow-turn"}[intent.operation]
            params = {"boot_session_id": intent.boot_session_id,
                      "command_id": intent.command_id,
                      "seq": intent.transition_seq}
            if intent.operation in ("START", "RESUME"):
                # The next finite step must fit before the hold boundary.
                # Allow 20 mm for rounding and unobserved motion after capture.
                available = selected_range.distance_m - (
                    SHADOW_STOP_DISTANCE_M + SHADOW_RESUME_MARGIN_M +
                    SHADOW_COMMAND_CLEARANCE_M)
                if available < SHADOW_MIN_SEGMENT_M:
                    raise RuntimeError("SHADOW_STEP_CLEARANCE_TOO_SMALL")
                params["distance_m"] = format(min(segment_m, available), ".3f")
                params["bias_pwm"] = str(intent.bias_pwm)
            if intent.operation == "STEER":
                params["bias_pwm"] = intent.bias_pwm
            if intent.operation == "TURN":
                params["direction"] = intent.turn_direction
                params["angle_deg"] = format(intent.turn_angle_deg, ".2f")
            try:
                code, body = call("POST", endpoint, params)
            except TimeoutError:
                # A lost ACK may still mean that motors started. Status is the
                # only recovery source; no second start/resume is transmitted.
                code, body = None, None
            if code is not None and code not in (200, 202):
                observed = status()
                reconcile(observed)

                if intent.operation == "STEER" and code == 409 and (
                    body or {}
                ).get("reason") == "NO_MATCHING_ACTIVE_STEERED_FORWARD":
                    # The short forward segment may already be braking.
                    # Never retry a rejected STEER or acknowledge it as applied.
                    settle_deadline = clock() + 1.6
                    while True:
                        if (
                            observed.get("boot_session_id") != intent.boot_session_id
                            or observed.get("active_command_id") != intent.command_id
                            or observed.get("bounded_fault_latched")
                            or observed.get("steered_safety_latched")
                            or observed.get("remote_stop_pending")
                        ):
                            break
                        if (
                            observed.get("state") == "PAUSED"
                            and observed.get("right_pwm") == 0
                            and observed.get("left_pwm") == 0
                            and supervisor.state == "PAUSED"
                        ):
                            return
                        if (
                            observed.get("state") not in ("BRAKING", "COASTING")
                            or clock() >= settle_deadline
                        ):
                            break
                        time.sleep(.04)
                        observed = status()
                        reconcile(observed)

                if intent.operation == "STOP" and supervisor.state == "TERMINAL":
                    return
                raise RuntimeError("SHADOW_OPERATION_REJECTED:" + str(body))
            deadline = (clock() + 5.5 if intent.operation == "TURN" else
                        clock() + .55 if intent.operation in ("START", "RESUME")
                        else clock() + 1.6)
            last_heartbeat = 0.0
            while clock() < deadline:
                observed = status()
                if observed.get("bounded_fault_latched") or observed.get("steered_safety_latched"):
                    reconcile(observed)
                    raise RuntimeError("SHADOW_CONTROLLER_FAULT")
                state = observed.get("state")
                # ESP32 accepts steering only after DRIVING begins.
                # Do not issue /steer-update while STARTING.
                # The normal ACTIVE loop handles fresh steering after
                # the START/RESUME acknowledgement below.
                if (intent.operation == "TURN"
                        and state == "PAUSED"
                        and observed.get("active_command_id") == intent.command_id
                        and observed.get("shadow_transition_seq") == intent.transition_seq
                        and observed.get("shadow_turn_result") in
                            ("SUCCESS", "PARTIAL_PROGRESS")
                        and observed.get("right_pwm") == 0
                        and observed.get("left_pwm") == 0):
                    supervisor.acknowledge(intent,
                        boot_session_id=observed["boot_session_id"],
                        active_command_id=observed.get("active_command_id"),
                        controller_state="PAUSED",
                        right_pwm=0, left_pwm=0, fault_latched=False,
                        transition_seq=observed["shadow_transition_seq"])
                    journal.write(json.dumps({
                        "event": "EDGE_TURN_SETTLED",
                        "at_s": clock(),
                        "result": observed["shadow_turn_result"],
                        "heading_rad": observed.get("heading_rad"),
                        "right_encoder_raw_count":
                            observed.get("right_encoder_raw_count"),
                        "left_encoder_raw_count":
                            observed.get("left_encoder_raw_count"),
                        "right_pwm": 0, "left_pwm": 0,
                    }, allow_nan=False) + "\n")
                    journal.flush()
                    return
                if (intent.operation != "TURN" and state == "PAUSED"
                        and observed.get("active_command_id") == intent.command_id):
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
                        if intent.operation == "STEER":
                            last_steer_at = clock()
                        elif intent.operation in ("START", "RESUME"):
                            # The ROS producer continued while UART STARTING.
                            # Send one newly observed steering correction as
                            # soon as the controller confirms DRIVING. No
                            # duplicate sample and no edge-only bearing.
                            fresh = fresh_steering_observation(
                                supervisor.track_id,
                                original_target.observed_s)
                            if fresh is not None:
                                newest_target, newest_decision = fresh
                                newest_bias = max(-20, min(20,
                                    round(newest_target.bearing_deg * 4)))
                                if newest_bias != intent.bias_pwm:
                                    steer = supervisor.observe(
                                        newest_target, newest_decision,
                                        bias_pwm=newest_bias)
                                    if steer is not None and steer.operation == "STEER":
                                        report["operations"].append("STEER")
                                        issue(steer, newest_target, newest_decision.selected)
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
                    expected_lidar_calibration_id=lidar_calibration_id,
                    minimum_confidence=minimum_confidence)
                if (decision.selected is not None and target is not None
                        and decision.action in ("FOLLOW_ELIGIBLE", "HOLD")
                        and post_turn_reobserve_after_s is None):
                    last_verified_range = (
                        target.track_id, decision.selected.observed_s,
                        decision.selected.distance_m)
                if post_turn_reobserve_after_s is not None:
                    if (target is None or decision.selected is None
                            or target.observed_s <= post_turn_reobserve_after_s
                            or decision.selected.observed_s <= post_turn_reobserve_after_s):
                        decision = RangeDecision(
                            "PAUSE", "POST_TURN_FRESH_PAIR_REQUIRED")
                    else:
                        post_turn_reobserve_after_s = None
                # Log reason changes, not every 50-ms status tick.
                decision_key = (decision.action, decision.reason,
                                target.track_id if target is not None else None)
                if decision_key != last_decision_key:
                    journal.write(json.dumps({
                        "event": "SHADOW_RANGE_DECISION", "at_s": now,
                        "action": decision.action, "reason": decision.reason,
                        "track_id": decision_key[2],
                        "distance_m": (decision.selected.distance_m
                                       if decision.selected is not None else None),
                        "supervisor_state": supervisor.state,
                    }, allow_nan=False) + "\n")
                    journal.flush()
                    last_decision_key = decision_key
                # RGB-only edge guidance records orientation recovery need.
                # UART v3 has no paused in-place TURN endpoint; do not
                # improvise an unsafe forward segment to chase the edge.
                if (latest is not None and
                        latest.edge_recovery_bearing_deg is not None
                        and decision.action == "PAUSE"
                        and decision.reason == "NO_FRESH_ASSOCIATED_RANGE"):
                    recovery_track = target.track_id if target else None
                    if recovery_track != last_edge_recovery_track:
                        journal.write(json.dumps({
                            "event": "EDGE_ORIENTATION_RECOVERY_REQUIRED",
                            "at_s": now,
                            "track_id": target.track_id if target else None,
                            "bearing_deg": latest.edge_recovery_bearing_deg,
                            "turn_direction": ("RIGHT" if
                                latest.edge_recovery_bearing_deg > 0 else "LEFT"),
                            "motor_operation": None,
                            "reason": "UART_V3_HAS_NO_PAUSED_TURN",
                        }, allow_nan=False) + "\n")
                        journal.flush()
                    last_edge_recovery_track = recovery_track
                else:
                    last_edge_recovery_track = None
                # Poll every input tick: a bounded segment can end on its own.
                observed = status()
                reconcile(observed)
                if supervisor.state in ("FAULT", "TERMINAL"):
                    break
                if observed.get("state") in ("BRAKING", "COASTING"):
                    continue
                if supervisor.state == "PAUSED":
                    if max_segments is not None and report["segments_started"] >= max_segments:
                        report["reason"] = "SEGMENT_LIMIT_REACHED"
                        break
                    if (enable_edge_turn and report["edge_turns_started"] == 0
                            and latest is not None and not latest.camera_encoder_disagreement
                            and latest.edge_recovery_bearing_deg is not None
                            and decision.action == "PAUSE"
                            and decision.reason == "NO_FRESH_ASSOCIATED_RANGE"
                            and target is not None and target.track_id == supervisor.track_id):
                        recovery = select_edge_turn(
                            target, latest.edge_recovery_bearing_deg,
                            last_verified_range, now)
                        if recovery is not None:
                            direction, angle = recovery
                            intent = supervisor.turn(direction, angle)
                            report["operations"].append("TURN")
                            report["edge_turns_started"] += 1
                            issue(intent, target)
                            post_turn_reobserve_after_s = clock()
                            last_verified_range = None
                            continue
                    # A settled segment cannot resume on the observations
                    # that authorized it, even while their age is below .5 s.
                    if (decision.action == "FOLLOW_ELIGIBLE" and
                            last_segment_pair is not None and
                            (target.observed_s <= last_segment_pair[0] or
                             decision.selected.observed_s <= last_segment_pair[1])):
                        report["reused_pair_refusals"] += 1
                        continue
                if (supervisor.state == "ACTIVE" and decision.action == "FOLLOW_ELIGIBLE"
                        and now - last_steer_at < .15):
                    continue
                bias = 0 if target is None else max(-20, min(20, round(target.bearing_deg * 4)))
                # A new RGB track cannot inherit the old motor command.
                # End the paused session with a confirmed STOP.
                if (supervisor.state == "PAUSED"
                        and supervisor.track_id is not None
                        and target is not None
                        and target.track_id != supervisor.track_id
                        and decision.action == "FOLLOW_ELIGIBLE"):
                    intent = supervisor.observe(
                        target, decision, operator_stop=True)
                    if intent is None or intent.operation != "STOP":
                        raise RuntimeError("TRACK_CHANGE_STOP_NOT_ISSUED")
                    report["operations"].append("STOP")
                    issue(intent, target, decision.selected)
                    report["reason"] = "NEW_TRACK_REQUIRES_NEW_SESSION"
                    break

                intent = supervisor.observe(target, decision, bias_pwm=bias,
                    camera_encoder_disagreement=(latest.camera_encoder_disagreement
                                                  if latest is not None else False))
                if intent is None:
                    continue
                report["operations"].append(intent.operation)
                if intent.operation in ("START", "RESUME"):
                    last_segment_pair = (target.observed_s,
                                         decision.selected.observed_s)
                    report["segments_started"] += 1
                issue(intent, target, decision.selected)
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
