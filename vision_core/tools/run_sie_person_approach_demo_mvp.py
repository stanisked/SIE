#!/usr/bin/env python3
"""Supervised metric bearing -> turn/reobserve -> forward/reobserve person approach."""
from __future__ import annotations

import argparse
import json
import secrets
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sie_core.supervised_bounded_executor import (  # noqa: E402
    ExecutionContractError, SUPERVISED_PERSON_APPROACH_SESSION,
    authorize_person_approach_session_action, execute_one_supervised_command,
    fetch_bounded_status,
)
from vision_core.person_approach.bounded_bridge import plan_bounded_command  # noqa: E402
from vision_core.person_approach.spatial_navigation import (  # noqa: E402
    evaluate_person_navigation_window, navigation_decision,
)
from vision_core.person_depth_fusion.live import LiveFusionError, build_live_runtime  # noqa: E402
from vision_core.person_localization.yolo11_person_upper_body_runtime import (  # noqa: E402
    OnnxRuntimeYolo11PersonUpperBodyObserver,
)
from vision_core.tools.run_sie_static_target_mvp import (  # noqa: E402
    DEFAULT_TEMPERATURE_DISABLED_POLICY, DEFAULT_YOLO_MODEL,
    YOLO_CONFIDENCE_THRESHOLD, fresh_supervised_execution_plan,
    parse_yolo_confidence_threshold,
)


SESSION_CONFIRMATION = SUPERVISED_PERSON_APPROACH_SESSION
EXECUTION_SCOPE = SUPERVISED_PERSON_APPROACH_SESSION


def _positive(value: str) -> float:
    try:
        result = float(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("value must be positive and finite") from error
    if result <= 0.0 or result != result or result in {float("inf"), float("-inf")}:
        raise argparse.ArgumentTypeError("value must be positive and finite")
    return result


def _nonnegative(value: str) -> float:
    result = _positive(value) if value != "0" else 0.0
    return result


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--project-root", type=Path, required=True)
    parser.add_argument("--ar-intrinsic", type=Path, required=True)
    parser.add_argument("--safe-distance-m", type=_positive, required=True)
    parser.add_argument("--bearing-deadband-deg", type=_nonnegative, required=True)
    parser.add_argument("--jsonl-output", type=Path, required=True)
    parser.add_argument("--yolo-model", type=Path, default=DEFAULT_YOLO_MODEL)
    parser.add_argument("--yolo-confidence-threshold", type=parse_yolo_confidence_threshold, default=YOLO_CONFIDENCE_THRESHOLD)
    parser.add_argument("--stereo-policy", type=Path, default=PROJECT_ROOT / DEFAULT_TEMPERATURE_DISABLED_POLICY)
    parser.add_argument("--person-threshold", type=float, choices=(0.4, 0.5), default=0.5)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--base-url")
    parser.add_argument(
        "--authorization-mode", choices=(SUPERVISED_PERSON_APPROACH_SESSION,),
        default=SUPERVISED_PERSON_APPROACH_SESSION,
        help="required with --execute; one confirmation authorizes this approach session only",
    )
    parser.add_argument("--timeout-s", type=_positive, default=2.0)
    parser.add_argument("--poll-interval-s", type=_positive, default=0.2)
    parser.add_argument("--terminal-timeout-s", type=_positive, default=8.0)
    return parser.parse_args(argv)


def _record(*, session_id: str, operator_session_confirmation: str | None,
            action_command_ids: list[str], state: str, index: int, navigation: dict[str, Any] | None,
            plan: dict[str, Any] | None = None, executor: dict[str, Any] | None = None,
            reason: str | None = None) -> dict[str, Any]:
    return json.loads(json.dumps({
        "schema_version": "sie.person_approach_supervised_demo_mvp.v1",
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "authorization_mode": SUPERVISED_PERSON_APPROACH_SESSION,
        "session_id": session_id,
        "operator_session_confirmation": operator_session_confirmation,
        "action_command_ids": list(action_command_ids),
        "execution_scope": EXECUTION_SCOPE,
        "automatic_capability_qualification_changed": False,
        "state": state, "action_index": index,
        "result": navigation.get("result") if type(navigation) is dict else "BLOCKED",
        "reason": reason if reason is not None else navigation.get("reason") if type(navigation) is dict else None,
        "metric_navigation": navigation, "chosen_action": None if type(navigation) is not dict else navigation.get("action"),
        "command_id": None if type(plan) is not dict else plan.get("command_id"),
        "planned_command": plan, "terminal_esp32_record": None if type(executor) is not dict else executor.get("terminal_status"),
        "executor": executor, "network_performed": bool(executor is not None),
        "motor_command_performed": bool(type(executor) is dict and executor.get("motor_command_performed") is True),
        "reobserve_required": state in {"REOBSERVE_AFTER_TURN", "REOBSERVE_AFTER_FORWARD"},
        "image_center_used_as_gate": False,
    }, allow_nan=False, sort_keys=True))


def _write(stream: Any, value: dict[str, Any]) -> None:
    text = json.dumps(value, allow_nan=False, sort_keys=True)
    print(text)
    stream.write(text + "\n")
    stream.flush()


def _session_authorization(
    plan: dict[str, Any], *, session_id: str, operator_session_confirmation: str,
) -> dict[str, Any]:
    return authorize_person_approach_session_action(
        planned_command=plan,
        session_id=session_id,
        operator_session_confirmation=operator_session_confirmation,
    )


def main() -> int:
    args = parse_args()
    runtime = None
    session_id = "session-" + secrets.token_hex(12)
    operator_session_confirmation: str | None = None
    action_command_ids: list[str] = []
    try:
        if not args.execute or not args.base_url:
            raise ExecutionContractError("--execute and --base-url are required")
        if args.authorization_mode != SUPERVISED_PERSON_APPROACH_SESSION:
            raise ExecutionContractError("--execute requires SUPERVISED_PERSON_APPROACH_SESSION")
        confirmed = input(f"Для supervised session введи {SESSION_CONFIRMATION}:\n> ").strip()
        if confirmed != SESSION_CONFIRMATION:
            raise ExecutionContractError("SUPERVISED_SESSION_NOT_CONFIRMED")
        operator_session_confirmation = confirmed
        observer = OnnxRuntimeYolo11PersonUpperBodyObserver(args.yolo_model, confidence_threshold=args.yolo_confidence_threshold)
        runtime = build_live_runtime(model=args.model, reference=args.reference, project_root=args.project_root,
                                     person_threshold=args.person_threshold, stereo_policy_path=args.stereo_policy.resolve(),
                                     primary_image_observer=observer)
        runtime.start()
        with args.jsonl_output.open("a", encoding="utf-8") as stream:
            state, action_index, decision_sequence = "OBSERVE", 0, 0
            while True:
                cycles = [runtime.cycle(f"{session_id}-a{action_index:04d}-c{number}") for number in range(1, 6)]
                navigation = evaluate_person_navigation_window(
                    cycles, safe_distance_m=args.safe_distance_m,
                    bearing_deadband_deg=args.bearing_deadband_deg,
                )
                _write(stream, _record(session_id=session_id, operator_session_confirmation=operator_session_confirmation, action_command_ids=action_command_ids, state=state, index=action_index, navigation=navigation))
                if navigation.get("result") == "ARRIVED":
                    _write(stream, _record(session_id=session_id, operator_session_confirmation=operator_session_confirmation, action_command_ids=action_command_ids, state="ARRIVED", index=action_index, navigation=navigation))
                    return 0
                if navigation.get("result") not in {"TURN_REQUIRED", "FORWARD_REQUIRED"}:
                    _write(stream, _record(session_id=session_id, operator_session_confirmation=operator_session_confirmation, action_command_ids=action_command_ids, state="BLOCKED", index=action_index, navigation=navigation))
                    return 2
                status_code, status = fetch_bounded_status(base_url=args.base_url, timeout_s=args.timeout_s)
                boot_session_id = status.get("boot_session_id") if status_code == 200 and type(status) is dict else None
                if type(boot_session_id) is not str:
                    _write(stream, _record(session_id=session_id, operator_session_confirmation=operator_session_confirmation, action_command_ids=action_command_ids, state="BLOCKED", index=action_index, navigation=navigation, reason="STATUS_UNAVAILABLE_BEFORE_COMMAND"))
                    return 2
                decision_sequence += 1
                decision = navigation_decision(navigation, sequence=decision_sequence, timestamp=datetime.now(timezone.utc).isoformat())
                bridge = plan_bounded_command({"decision": decision, "evidence_window": cycles, "boot_session_id": boot_session_id, "previous_terminal_motion_outcome": None, "freshness_reference_utc": datetime.now(timezone.utc).isoformat()}).to_dict()
                if bridge.get("result") != "PLANNED_BOUNDED_COMMAND":
                    _write(stream, _record(session_id=session_id, operator_session_confirmation=operator_session_confirmation, action_command_ids=action_command_ids, state="BLOCKED", index=action_index, navigation=navigation, plan=bridge, reason=bridge.get("block_reason")))
                    return 2
                plan = fresh_supervised_execution_plan(bridge)
                authorization = _session_authorization(
                    plan,
                    session_id=session_id,
                    operator_session_confirmation=operator_session_confirmation,
                )
                action_state = "TURN_IF_NEEDED" if navigation["result"] == "TURN_REQUIRED" else "FORWARD_STEP"
                _write(stream, _record(session_id=session_id, operator_session_confirmation=operator_session_confirmation, action_command_ids=action_command_ids, state=action_state, index=action_index, navigation=navigation, plan=plan))
                executor = execute_one_supervised_command(planned_command=plan, authorization=authorization, base_url=args.base_url,
                                                          timeout_s=args.timeout_s, poll_interval_s=args.poll_interval_s,
                                                          terminal_timeout_s=args.terminal_timeout_s)
                terminal = executor.get("terminal_status") if type(executor) is dict else None
                if executor.get("motor_command_performed") is True:
                    action_command_ids.append(plan["command_id"])
                if executor.get("result") != "AWAIT_REOBSERVATION" or not isinstance(terminal, dict) or terminal.get("last_command_state") in {"FAULT", "STOPPED"}:
                    _write(stream, _record(session_id=session_id, operator_session_confirmation=operator_session_confirmation, action_command_ids=action_command_ids, state="BLOCKED", index=action_index, navigation=navigation, plan=plan, executor=executor, reason="TERMINAL_OR_HTTP_FAILURE"))
                    return 2
                state = "REOBSERVE_AFTER_TURN" if navigation["result"] == "TURN_REQUIRED" else "REOBSERVE_AFTER_FORWARD"
                _write(stream, _record(session_id=session_id, operator_session_confirmation=operator_session_confirmation, action_command_ids=action_command_ids, state=state, index=action_index, navigation=navigation, plan=plan, executor=executor))
                action_index += 1
    except (ExecutionContractError, LiveFusionError, RuntimeError, ValueError, OSError) as error:
        print(json.dumps({
            "result": "BLOCKED", "reason": str(error),
            "authorization_mode": SUPERVISED_PERSON_APPROACH_SESSION,
            "session_id": session_id,
            "operator_session_confirmation": operator_session_confirmation,
            "action_command_ids": action_command_ids,
            "execution_scope": EXECUTION_SCOPE,
            "automatic_capability_qualification_changed": False,
            "network_performed": False, "motor_command_performed": False,
        }, allow_nan=False, sort_keys=True))
        return 2
    finally:
        if runtime is not None:
            runtime.close()


if __name__ == "__main__":
    raise SystemExit(main())
