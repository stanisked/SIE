"""Pure Shadow supervisory state machine. No UART or motor I/O here.

The adapter must journal intents, send them once with the supplied IDs, and
confirm state and zero PWM before invoking acknowledge(). A timeout is
ambiguous: reconcile with controller status, never mint another command ID.
"""
from __future__ import annotations

from dataclasses import dataclass

from vision_core.tools.shadow_range_gate import RangeDecision, RgbTarget


@dataclass(frozen=True)
class ShadowIntent:
    operation: str
    boot_session_id: str
    command_id: str
    transition_seq: int
    bias_pwm: int = 0
    range_source: str | None = None
    association_evidence_id: str | None = None


class ShadowSupervisor:
    """One command ID for the whole session, including every pause/resume."""

    def __init__(self, boot_session_id: str, command_id: str):
        if not boot_session_id or not command_id:
            raise ValueError("SESSION_ID_REQUIRED")
        self.boot_session_id = boot_session_id
        self.command_id = command_id
        self.state = "IDLE"
        self.track_id = None
        self.seq = 0
        self.steer_seq = 0
        self.pending = None

    def observe(self, target: RgbTarget | None, decision: RangeDecision,
                *, bias_pwm: int = 0, operator_stop: bool = False,
                camera_encoder_disagreement: bool = False) -> ShadowIntent | None:
        if self.state in ("TERMINAL", "FAULT") or self.pending:
            return None
        if type(bias_pwm) is not int or not -20 <= bias_pwm <= 20:
            raise ValueError("INVALID_BIAS_PWM")
        if operator_stop:
            if self.state == "IDLE":
                self.state = "TERMINAL"
                return None
            return self._issue("STOP")
        if camera_encoder_disagreement:
            if self.state == "ACTIVE":
                return self._issue("PAUSE")
            return None
        # Losing a target never releases its identity. A different RGB track
        # cannot silently take over this command.
        matched = target is not None and (self.track_id is None or
                                           target.track_id == self.track_id)
        follow = matched and decision.action == "FOLLOW_ELIGIBLE" and \
            decision.selected is not None and \
            decision.selected.target_track_id == target.track_id
        if not follow:
            if self.state == "ACTIVE":
                return self._issue("PAUSE")
            return None
        if self.state == "IDLE":
            self.track_id = target.track_id
            return self._issue("START", bias_pwm, decision)
        if self.state == "PAUSED":
            return self._issue("RESUME", bias_pwm, decision)
        if self.state == "ACTIVE":
            return self._issue("STEER", bias_pwm, decision)
        return None

    def _issue(self, operation, bias_pwm=0, decision=None):
        if operation == "STEER":
            self.steer_seq += 1
            seq = self.steer_seq
        else:
            self.seq += 1
            seq = self.seq
        intent = ShadowIntent(
            operation, self.boot_session_id, self.command_id, seq,
            bias_pwm,
            decision.selected.source if decision else None,
            decision.selected.association_evidence_id if decision else None)
        self.pending = intent
        return intent

    def starting_heartbeat(self) -> ShadowIntent:
        """Zero-bias heartbeat while awaiting STARTING -> DRIVING."""
        if self.pending is None or self.pending.operation not in ("START", "RESUME"):
            raise ValueError("SHADOW_HEARTBEAT_NOT_STARTING")
        self.steer_seq += 1
        return ShadowIntent("STEER", self.boot_session_id, self.command_id,
                            self.steer_seq, 0)

    def acknowledge(self, intent: ShadowIntent, *, boot_session_id: str,
                    active_command_id: str | None, controller_state: str,
                    right_pwm: int, left_pwm: int, fault_latched: bool,
                    transition_seq: int,
                    last_command_id: str | None = None):
        """Apply a fully observed controller status; never infer from POST ACK."""
        if self.pending != intent or boot_session_id != self.boot_session_id or \
                transition_seq != intent.transition_seq:
            raise ValueError("SHADOW_ACK_MISMATCH")
        if fault_latched or controller_state == "FAULT":
            self.state = "FAULT"
            self.pending = None
            return
        if intent.operation == "STOP":
            expected_state, expected_id, next_state = "READY", None, "TERMINAL"
        elif intent.operation == "PAUSE":
            expected_state, expected_id, next_state = "PAUSED", self.command_id, "PAUSED"
        else:
            expected_state, expected_id, next_state = "DRIVING", self.command_id, "ACTIVE"
        if controller_state != expected_state or active_command_id != expected_id:
            raise ValueError("SHADOW_STATE_NOT_CONFIRMED")
        if intent.operation == "STOP" and last_command_id != self.command_id:
            raise ValueError("SHADOW_STOP_ID_NOT_CONFIRMED")
        if intent.operation in ("PAUSE", "STOP") and (right_pwm or left_pwm):
            raise ValueError("SHADOW_BRAKE_NOT_CONFIRMED")
        self.state = next_state
        self.pending = None

    def reconcile(self, *, boot_session_id: str, active_command_id: str | None,
                  last_command_id: str | None, controller_state: str,
                  right_pwm: int, left_pwm: int, fault_latched: bool):
        """Observe an unsolicited segment completion or terminal fault."""
        if boot_session_id != self.boot_session_id:
            raise ValueError("BOOT_SESSION_CHANGED")
        if fault_latched or controller_state == "FAULT":
            self.state = "FAULT"
            self.pending = None
            return
        if controller_state == "PAUSED" and active_command_id == self.command_id:
            if right_pwm != 0 or left_pwm != 0:
                raise ValueError("SHADOW_BRAKE_NOT_CONFIRMED")
            self.state = "PAUSED"
            self.pending = None
            return
        if controller_state == "READY" and last_command_id == self.command_id:
            if right_pwm != 0 or left_pwm != 0:
                raise ValueError("SHADOW_STOP_NOT_CONFIRMED")
            self.state = "TERMINAL"
            self.pending = None
            return
        if active_command_id not in (self.command_id, None):
            raise ValueError("ACTIVE_COMMAND_CHANGED")

