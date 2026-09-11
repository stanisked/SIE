#!/usr/bin/env python3
"""Observe one static person target, make a bridge plan, and optionally execute it once."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sie_core.supervised_bounded_executor import (  # noqa: E402
    ExecutionContractError,
    authorize_operator_trial,
    execute_one_supervised_command,
    fetch_bounded_status,
)
from vision_core.person_approach.bounded_bridge import plan_bounded_command  # noqa: E402
from vision_core.person_approach.metric_first_target_supervisor import (  # noqa: E402
    MetricFirstTargetSupervisor,
    WINDOW_SIZE,
)
from vision_core.person_depth_fusion.live import LiveFusionError, build_live_runtime  # noqa: E402
from vision_core.tools.prepare_ar0234_yaw_observations import load_optical_axis_cx  # noqa: E402


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--project-root", type=Path, required=True)
    parser.add_argument("--ar-intrinsic", type=Path, required=True)
    parser.add_argument("--center-tolerance-px", type=float, required=True)
    parser.add_argument("--person-threshold", type=float, choices=(0.4, 0.5), default=0.5)
    parser.add_argument("--execute", action="store_true", help="allow one confirmed bounded HTTP command")
    parser.add_argument("--base-url", help="explicit ESP32 http:// host, required with --execute")
    parser.add_argument("--authorization-mode", choices=("QUALIFIED", "SUPERVISED_EXPERIMENTAL_TRIAL"), default="QUALIFIED")
    parser.add_argument("--experimental-reason")
    parser.add_argument("--timeout-s", type=float, default=2.0)
    parser.add_argument("--poll-interval-s", type=float, default=0.2)
    parser.add_argument("--terminal-timeout-s", type=float, default=8.0)
    return parser.parse_args(argv)


def bridge_envelope(
    *, supervision: object, cycles: object, boot_session_id: object
) -> dict[str, Any] | None:
    """Reuse the metric decision and exact shared window without rewriting either."""
    if type(supervision) is not dict or type(cycles) is not list:
        return None
    decision = supervision.get("metric_decision")
    if type(decision) is not dict or decision.get("status") != "ADVANCE":
        return None
    return {
        "decision": decision,
        "evidence_window": cycles,
        "boot_session_id": boot_session_id,
        "previous_terminal_motion_outcome": None,
    }


def _combined(result: str, reason: str | None, *, supervision: dict[str, Any] | None, bridge: dict[str, Any] | None, executor: dict[str, Any] | None, network: bool) -> dict[str, Any]:
    return {
        "schema_version": "sie.static_target_supervised_mvp.v1",
        "result": result,
        "reason": reason,
        "supervision": supervision,
        "bridge_plan": bridge,
        "executor": executor,
        "network_performed": network,
        "motor_command_performed": bool(executor and executor.get("motor_command_performed") is True),
        "reobserve_required": True,
    }


def main() -> int:
    args = parse_args()
    runtime = None
    try:
        if args.execute and not args.base_url:
            raise ExecutionContractError("--execute requires --base-url")
        initial_status: dict[str, Any] | None = None
        boot_session_id: str | None = None
        network = False
        if args.execute:
            status_code, initial_status = fetch_bounded_status(base_url=args.base_url, timeout_s=args.timeout_s)
            network = True
            if status_code != 200 or type(initial_status.get("boot_session_id")) is not str:
                print(json.dumps(_combined("BLOCKED_PREFLIGHT", "STATUS_UNAVAILABLE_BEFORE_OBSERVATION", supervision=None, bridge=None, executor=None, network=network), allow_nan=False, sort_keys=True))
                return 0
            boot_session_id = initial_status["boot_session_id"]
        runtime = build_live_runtime(model=args.model, reference=args.reference, project_root=args.project_root, person_threshold=args.person_threshold)
        runtime.start()
        supervisor = MetricFirstTargetSupervisor(live_runtime=runtime, optical_axis_cx_px=load_optical_axis_cx(args.ar_intrinsic), center_tolerance_px=args.center_tolerance_px)
        cycles = [runtime.cycle(f"static-target-mvp-{index:06d}") for index in range(1, WINDOW_SIZE + 1)]
        supervision = supervisor.process_shared_window(cycles)
        if not args.execute:
            print(json.dumps(_combined("AWAIT_OPERATOR_EXECUTION", "RE-RUN_WITH_--execute_TO_FETCH_FRESH_SESSION_AND_ALLOW_ONE_COMMAND", supervision=supervision, bridge=None, executor=None, network=False), allow_nan=False, sort_keys=True))
            return 0
        envelope = bridge_envelope(supervision=supervision, cycles=cycles, boot_session_id=boot_session_id)
        if envelope is None:
            print(json.dumps(_combined("NO_EXECUTION_PLAN", "METRIC_ADVANCE_NOT_AVAILABLE_FROM_THIS_SHARED_WINDOW", supervision=supervision, bridge=None, executor=None, network=network), allow_nan=False, sort_keys=True))
            return 0
        plan = plan_bounded_command(envelope).to_dict()
        if plan.get("result") != "PLANNED_BOUNDED_COMMAND":
            print(json.dumps(_combined("NO_EXECUTION_PLAN", plan.get("block_reason"), supervision=supervision, bridge=plan, executor=None, network=network), allow_nan=False, sort_keys=True))
            return 0
        command_id = plan["command_id"]
        confirmation = input(f"Подтверди ровно этот command_id для одного шага: {command_id}\n> ").strip()
        authorization = authorize_operator_trial(planned_command=plan, confirmation_command_id=confirmation, authorization_mode=args.authorization_mode, experimental_reason=args.experimental_reason)
        executor = execute_one_supervised_command(planned_command=plan, authorization=authorization, base_url=args.base_url, timeout_s=args.timeout_s, poll_interval_s=args.poll_interval_s, terminal_timeout_s=args.terminal_timeout_s)
        print(json.dumps(_combined(executor["result"], executor.get("reason"), supervision=supervision, bridge=plan, executor=executor, network=True), allow_nan=False, sort_keys=True))
    except (ExecutionContractError, LiveFusionError, ValueError, RuntimeError) as error:
        print(json.dumps(_combined("BLOCKED", str(error), supervision=None, bridge=None, executor=None, network=False), allow_nan=False, sort_keys=True))
        return 2
    finally:
        if runtime is not None:
            runtime.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
