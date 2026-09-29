"""ROS 2 supervisor for the bounded-forward supervised trolley MVP.

Disabled mode preserves the phase-1 contract.  Execution mode is a separate
operator-confirmed session which can issue only one 0.10 m POST per fresh
FORWARD_REOBSERVE decision, never retries POST, and requires a new observation
after every terminal motion outcome.
"""
from __future__ import annotations

import hashlib
import json
import secrets
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .contracts import (
    ContractError,
    decode_json,
    encode,
    supervisor_state,
    validate_navigation,
)

PROFILE_SCHEMA = "sie.actuator.supervised_bounded_forward_profile.v1"
PROFILE_STATUS = "ACTIVE_SUPERVISED_BOUNDED_FORWARD_MVP"
SESSION_CONFIRMATION = "SUPERVISED_PERSON_APPROACH_SESSION"


def _utc(value: object) -> datetime:
    if type(value) is not str:
        raise ContractError("decision timestamp must be text")
    try:
        result = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as error:
        raise ContractError("decision timestamp must be ISO-8601") from error
    if result.tzinfo is None:
        raise ContractError("decision timestamp must be timezone-aware")
    return result.astimezone(timezone.utc)


class SupervisorContractNode(Node):
    def __init__(self) -> None:
        super().__init__("sie_supervisor_contract")
        self.declare_parameter("input_topic", "/sie/navigation/decision")
        self.declare_parameter("output_topic", "/sie/supervisor/state")
        self.declare_parameter("execution_enabled", False)
        self.declare_parameter("project_root", "")
        self.declare_parameter("execution_profile", "")
        self.declare_parameter("base_url", "")
        self.declare_parameter("operator_session_confirmation", "")
        self.declare_parameter("timeout_s", 2.0)
        self.declare_parameter("poll_interval_s", 0.2)
        self.declare_parameter("terminal_timeout_s", 8.0)
        self.declare_parameter("maximum_steps", 1)
        self.declare_parameter(
            "audit_jsonl",
            "~/.local/state/sie/streams/supervised_bounded_forward_mvp.jsonl",
        )

        self.execution_enabled = bool(
            self.get_parameter("execution_enabled").value
        )
        self.publisher = self.create_publisher(
            String, self.get_parameter("output_topic").value, 10
        )
        self.subscription = self.create_subscription(
            String, self.get_parameter("input_topic").value, self._callback, 10
        )
        self.session_id: str | None = None
        self.steps_performed = 0
        self.reobserve_after_utc: datetime | None = None
        self.seen_decision_ids: set[str] = set()
        self.session_block_reason: str | None = None

        if not self.execution_enabled:
            self.get_logger().info("phase 1: actuator bridge disabled")
            return

        project_root = Path(
            str(self.get_parameter("project_root").value)
        ).expanduser()
        profile_path = Path(
            str(self.get_parameter("execution_profile").value)
        ).expanduser()
        if not project_root.is_absolute() or not project_root.is_dir():
            raise RuntimeError("project_root must be an existing absolute directory")
        if not profile_path.is_absolute() or not profile_path.is_file():
            raise RuntimeError("execution_profile must be an existing absolute file")
        if str(project_root) not in sys.path:
            sys.path.insert(0, str(project_root))

        from sie_core.supervised_bounded_executor import (
            ExecutionContractError,
            authorize_person_approach_session_action,
            execute_one_supervised_command,
            fetch_bounded_status,
        )

        self.ExecutionContractError = ExecutionContractError
        self.authorize_action = authorize_person_approach_session_action
        self.execute_one = execute_one_supervised_command
        self.fetch_status = fetch_bounded_status
        self.profile_path = profile_path
        profile_bytes = profile_path.read_bytes()
        self.profile_sha256 = hashlib.sha256(profile_bytes).hexdigest()
        self.profile = json.loads(profile_bytes.decode("utf-8"))
        self._validate_profile()

        self.base_url = str(self.get_parameter("base_url").value).rstrip("/")
        confirmation = str(
            self.get_parameter("operator_session_confirmation").value
        )
        if self.base_url != self.profile["base_url"]:
            raise RuntimeError("base_url does not exactly match execution profile")
        if confirmation != SESSION_CONFIRMATION:
            raise RuntimeError("operator session confirmation is absent or invalid")
        self.operator_confirmation = confirmation
        self.timeout_s = float(self.get_parameter("timeout_s").value)
        self.poll_interval_s = float(
            self.get_parameter("poll_interval_s").value
        )
        self.terminal_timeout_s = float(
            self.get_parameter("terminal_timeout_s").value
        )
        if min(self.timeout_s, self.poll_interval_s, self.terminal_timeout_s) <= 0:
            raise RuntimeError("executor timeouts must be positive")
        envelope = self.profile["command_envelope"]
        profile_max_steps = int(envelope["maximum_steps_per_session"])
        self.max_steps = int(self.get_parameter("maximum_steps").value)
        if self.max_steps < 1 or self.max_steps > profile_max_steps:
            raise RuntimeError("maximum_steps exceeds the activated profile")
        self.maximum_decision_age_s = float(
            envelope["maximum_decision_age_s"]
        )
        self.maximum_measurement_age_s = float(
            envelope["maximum_measurement_age_s"]
        )
        self.session_id = "ros2-session-" + secrets.token_hex(12)
        self.audit_path = Path(
            str(self.get_parameter("audit_jsonl").value)
        ).expanduser()
        self.audit_path.parent.mkdir(parents=True, exist_ok=True)
        with self.audit_path.open("a", encoding="utf-8"):
            pass
        self.get_logger().warning(
            "SUPERVISED bounded-forward bridge armed; "
            f"session={self.session_id}; max_steps={self.max_steps}; "
            "0.10 m only; no POST retry; re-observation mandatory"
        )

    def _validate_profile(self) -> None:
        profile = self.profile
        authorization = profile.get("authorization", {})
        envelope = profile.get("command_envelope", {})
        controller = profile.get("controller_requirements", {})
        if (
            profile.get("schema_version") != PROFILE_SCHEMA
            or profile.get("status") != PROFILE_STATUS
            or profile.get("adapter_id") != "esp32_zk5ad_sgm37_520"
            or profile.get("capability_id") != "bounded_forward_0.10_m"
            or profile.get("motion_api_version") != "v2_bounded_motion_api"
            or authorization.get("mode") != SESSION_CONFIRMATION
            or authorization.get("operator_confirmation") != SESSION_CONFIRMATION
            or authorization.get("automatic_capability_qualification_changed")
            is not False
            or authorization.get("ros_navigation_execution_authorized") is not False
            or authorization.get("supervisor_session_execution_authorized") is not True
            or envelope.get("method") != "POST"
            or envelope.get("endpoint") != "/move-forward"
            or float(envelope.get("distance_m", -1)) != 0.10
            or int(envelope.get("maximum_steps_per_session", 0)) not in range(1, 4)
            or envelope.get("post_retry_permitted") is not False
            or envelope.get("reobserve_after_every_step") is not True
            or float(envelope.get("maximum_decision_age_s", -1)) != 1.0
            or float(envelope.get("maximum_measurement_age_s", -1)) != 5.0
            or controller.get("ready_state") != "READY"
            or controller.get("bounded_fault_latched") is not False
            or controller.get("active_command_id") is not None
            or controller.get("active_braking_required") is not True
        ):
            raise RuntimeError("execution profile violates bounded-forward MVP")

    def _state(
        self,
        decision: dict[str, Any],
        *,
        result: str,
        reason: str,
        motor_command_performed: bool = False,
        network_performed: bool = False,
        command_id: str | None = None,
        executor: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        return {
            "schema_version": "sie.supervisor.state.v2",
            "state_id": f"supervisor:{decision['decision_id']}",
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "decision_id": decision["decision_id"],
            "measurement_id": decision["measurement_id"],
            "result": result,
            "reason": reason,
            "recommended_action": decision["recommended_action"],
            "actuator_bridge": "SUPERVISED_BOUNDED_FORWARD_MVP_V1",
            "session_id": self.session_id,
            "execution_profile": str(self.profile_path),
            "execution_profile_sha256": self.profile_sha256,
            "command_id": command_id,
            "steps_performed": self.steps_performed,
            "maximum_steps_per_session": self.max_steps,
            "network_performed": network_performed,
            "motor_command_performed": motor_command_performed,
            "reobserve_required": self.reobserve_after_utc is not None,
            "dynamic_fusion_authorized": False,
            "automatic_capability_qualification_changed": False,
            "executor": executor,
        }

    def _emit(self, state: dict[str, Any]) -> None:
        text = encode(state)
        if self.execution_enabled:
            with self.audit_path.open("a", encoding="utf-8") as stream:
                stream.write(text + "\n")
                stream.flush()
        output = String()
        output.data = text
        self.publisher.publish(output)

    def _callback(self, message: String) -> None:
        try:
            decision = validate_navigation(decode_json(message.data))
            if not self.execution_enabled:
                self._emit(supervisor_state(decision))
                return
            state = self._execute_decision(decision)
            self._emit(state)
        except (ContractError, OSError, RuntimeError, ValueError) as error:
            if self.execution_enabled:
                self.session_block_reason = f"SUPERVISOR_ERROR:{error}"
            self.get_logger().warning(f"rejected navigation decision: {error}")

    def _execute_decision(self, decision: dict[str, Any]) -> dict[str, Any]:
        decision_id = decision["decision_id"]
        if decision_id in self.seen_decision_ids:
            return self._state(
                decision, result="BLOCKED", reason="DUPLICATE_DECISION"
            )
        self.seen_decision_ids.add(decision_id)

        if self.session_block_reason is not None:
            return self._state(
                decision, result="BLOCKED", reason=self.session_block_reason
            )
        decision_time = _utc(decision["timestamp"])
        measurement_time = _utc(decision["measurement_timestamp"])
        now = datetime.now(timezone.utc)
        decision_age_s = (now - decision_time).total_seconds()
        measurement_age_s = (now - measurement_time).total_seconds()
        if (
            decision_age_s < 0
            or decision_age_s > self.maximum_decision_age_s
        ):
            return self._state(
                decision, result="BLOCKED", reason="DECISION_STALE_OR_FROM_FUTURE"
            )
        if (
            measurement_age_s < 0
            or measurement_age_s > self.maximum_measurement_age_s
        ):
            return self._state(
                decision, result="BLOCKED", reason="MEASUREMENT_STALE_OR_FROM_FUTURE"
            )
        if self.reobserve_after_utc is not None:
            if measurement_time <= self.reobserve_after_utc:
                return self._state(
                    decision,
                    result="AWAIT_REOBSERVATION",
                    reason="NEW_POST_TERMINAL_OBSERVATION_REQUIRED",
                )
            self.reobserve_after_utc = None
        if self.steps_performed >= self.max_steps:
            return self._state(
                decision, result="BLOCKED", reason="SESSION_STEP_LIMIT_REACHED"
            )
        if (
            decision["result"] != "RECOMMENDED"
            or decision["recommended_action"] != "FORWARD_REOBSERVE"
            or decision["execution_authorized"] is not False
        ):
            return self._state(
                decision,
                result="BLOCKED",
                reason=f"NAVIGATION_{decision['reason']}",
            )

        status_code, status = self.fetch_status(
            base_url=self.base_url, timeout_s=self.timeout_s
        )
        if (
            status_code != 200
            or status.get("motion_api_version") != "v2_bounded_motion_api"
            or status.get("state") != "READY"
            or status.get("bounded_fault_latched") is not False
            or status.get("active_command_id") is not None
            or type(status.get("boot_session_id")) is not str
        ):
            self.session_block_reason = "CONTROLLER_PREFLIGHT_NOT_READY"
            return self._state(
                decision, result="BLOCKED", reason=self.session_block_reason,
                network_performed=True,
            )

        identity = "\n".join(
            (self.session_id or "", decision_id, status["boot_session_id"])
        )
        command_id = "ros2-" + hashlib.sha256(
            identity.encode("utf-8")
        ).hexdigest()[:59]
        plan = {
            "schema_version": "sie.person_approach_bounded_bridge.v1",
            "result": "PLANNED_BOUNDED_COMMAND",
            "decision_id": decision_id,
            "method": "POST",
            "endpoint": "/move-forward",
            "query": {
                "boot_session_id": status["boot_session_id"],
                "command_id": command_id,
                "distance_m": "0.1",
            },
            "command_id": command_id,
            "evidence_cycle_ids": [],
            "evidence_measurement_ids": [decision["measurement_id"]],
            "reference_frame": "ov9281_physical_left_optical_frame",
            "units": "m",
            "freshness_diagnostics": {
                "decision_age_s": decision_age_s,
                "maximum_decision_age_s": self.maximum_decision_age_s,
                "measurement_age_s": measurement_age_s,
                "maximum_measurement_age_s": self.maximum_measurement_age_s,
            },
            "network_performed": False,
            "reobserve_required": True,
        }
        authorization = self.authorize_action(
            planned_command=plan,
            session_id=self.session_id,
            operator_session_confirmation=self.operator_confirmation,
        )
        self._emit(
            self._state(
                decision,
                result="EXECUTING",
                reason="ONE_BOUNDED_FORWARD_POST",
                command_id=command_id,
                network_performed=True,
            )
        )
        executor = self.execute_one(
            planned_command=plan,
            authorization=authorization,
            base_url=self.base_url,
            timeout_s=self.timeout_s,
            poll_interval_s=self.poll_interval_s,
            terminal_timeout_s=self.terminal_timeout_s,
        )
        terminal = executor.get("terminal_status")
        terminal_state = (
            terminal.get("last_command_state")
            if type(terminal) is dict
            else None
        )
        motor_performed = executor.get("motor_command_performed") is True
        network_performed = executor.get("network_performed") is True
        safe_terminal = (
            executor.get("result") == "AWAIT_REOBSERVATION"
            and terminal_state in {"SUCCESS", "PARTIAL_PROGRESS"}
            and terminal.get("bounded_fault_latched") is False
        )
        if not safe_terminal:
            self.session_block_reason = (
                f"EXECUTOR_{executor.get('result')}:{terminal_state}"
            )
            return self._state(
                decision,
                result="BLOCKED",
                reason=self.session_block_reason,
                motor_command_performed=motor_performed,
                network_performed=network_performed,
                command_id=command_id,
                executor=executor,
            )

        if motor_performed:
            self.steps_performed += 1
        self.reobserve_after_utc = datetime.now(timezone.utc)
        return self._state(
            decision,
            result="AWAIT_REOBSERVATION",
            reason=f"TERMINAL_{terminal_state}",
            motor_command_performed=motor_performed,
            network_performed=network_performed,
            command_id=command_id,
            executor=executor,
        )


def main() -> None:
    rclpy.init()
    node = SupervisorContractNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
