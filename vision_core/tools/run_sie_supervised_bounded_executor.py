#!/usr/bin/env python3
"""Run one explicitly confirmed bounded ESP32 command for a supervised MVP."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sie_core.supervised_bounded_executor import (
    ExecutionContractError,
    authorize_operator_trial,
    execute_one_supervised_command,
    validate_planned_bounded_command,
)


def _plan(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if type(value) is not dict:
        raise ExecutionContractError("plan JSON must be an object")
    return value


def parser() -> argparse.ArgumentParser:
    value = argparse.ArgumentParser(description=__doc__)
    value.add_argument("plan_json", type=Path, help="bridge PlannedBoundedCommand JSON")
    value.add_argument("--execute", action="store_true", help="allow one real HTTP command after local confirmation")
    value.add_argument("--base-url", help="explicit ESP32 http:// host, required with --execute")
    value.add_argument("--confirm-command-id", help="must exactly equal the plan command_id")
    value.add_argument("--authorization-mode", choices=("QUALIFIED", "SUPERVISED_EXPERIMENTAL_TRIAL"), default="QUALIFIED")
    value.add_argument("--experimental-reason", help="required for SUPERVISED_EXPERIMENTAL_TRIAL")
    value.add_argument("--timeout-s", type=float, default=2.0)
    value.add_argument("--poll-interval-s", type=float, default=0.2)
    value.add_argument("--terminal-timeout-s", type=float, default=8.0)
    return value


def main() -> int:
    args = parser().parse_args()
    try:
        command = validate_planned_bounded_command(_plan(args.plan_json))
        if not args.execute:
            result = {
                "schema_version": "sie.supervised_bounded_executor.v1",
                "result": "AWAIT_OPERATOR_CONFIRMATION",
                "reason": "PASS_--execute_AND_EXACT_--confirm-command-id_TO_SEND_ONE_COMMAND",
                "planned_command": command,
                "network_performed": False,
                "motor_command_performed": False,
                "reobserve_required": True,
            }
        else:
            if not args.base_url or not args.confirm_command_id:
                raise ExecutionContractError("--execute requires --base-url and --confirm-command-id")
            authorization = authorize_operator_trial(
                planned_command=command,
                confirmation_command_id=args.confirm_command_id,
                authorization_mode=args.authorization_mode,
                experimental_reason=args.experimental_reason,
            )
            result = execute_one_supervised_command(
                planned_command=command,
                authorization=authorization,
                base_url=args.base_url,
                timeout_s=args.timeout_s,
                poll_interval_s=args.poll_interval_s,
                terminal_timeout_s=args.terminal_timeout_s,
            )
    except (ExecutionContractError, OSError, json.JSONDecodeError) as error:
        result = {
            "schema_version": "sie.supervised_bounded_executor.v1",
            "result": "BLOCKED_EXECUTION_CONTRACT",
            "reason": str(error),
            "network_performed": False,
            "motor_command_performed": False,
            "reobserve_required": True,
        }
    print(json.dumps(result, allow_nan=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
