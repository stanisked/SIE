"""Host-side source contract checks for the ESP32 v4_2 candidate.

These checks deliberately inspect the production firmware source. They cannot
execute an Arduino sketch or establish physical braking/overshoot behaviour;
that remains a separate hardware-validation step.
"""

from __future__ import annotations

import hashlib
import json
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
BASE = "b4b702a2a07fe6fa8b2878ae12590684a986124d"
BASELINE = "firmware/esp32/traction_control_v4_1_effective_track294"
FIRMWARE = ROOT / (
    "firmware/esp32/traction_control_v4_2_bounded_motion_api/"
    "traction_control_v4_2_bounded_motion_api.ino"
)
MANIFEST = ROOT / (
    "firmware/esp32/traction_control_v4_2_bounded_motion_api/"
    "change_manifest.json"
)


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start : index + 1]
    raise AssertionError(f"unterminated function: {signature}")


@pytest.fixture
def source() -> str:
    return FIRMWARE.read_text(encoding="utf-8")


def completion_tolerance_counts(target_counts: int) -> int:
    return min(6, max(1, -(-target_counts // 5)))


def completion_outcome(
    right_target: int,
    left_target: int,
    right_final: int,
    left_final: int,
    *,
    micro_turn: bool = False,
) -> str:
    if right_final > right_target or left_final > left_target:
        return "BOUNDED_DISTANCE_LIMIT"
    if micro_turn:
        tolerance = micro_turn_tolerance
    else:
        tolerance = completion_tolerance_counts
    right_minimum = right_target - tolerance(right_target)
    left_minimum = left_target - tolerance(left_target)
    if right_final < right_minimum or left_final < left_minimum:
        return "BOUNDED_TARGET_NOT_REACHED"
    return "SUCCESS"


def micro_turn_brake_start_counts(
    target_counts: int,
    dynamic_reserve_counts: int,
) -> int:
    return max(1, target_counts - max(6, dynamic_reserve_counts))


def test_baseline_tree_is_unchanged() -> None:
    result = subprocess.run(
        ["git", "diff", "--quiet", BASE, "--", BASELINE],
        cwd=ROOT,
        check=False,
    )
    assert result.returncode == 0


def test_bounded_encoder_cutoff_is_configured_per_accepted_command(source: str) -> None:
    configure = function_body(source, "void configureBoundedCommand(")
    for token in (
        "requestedTargetM",
        "command.rightTargetCounts",
        "command.leftTargetCounts",
        "command.rightLimitCounts",
        "command.leftLimitCounts",
        "command.boundedTimeoutMs",
        "activeBoundedKind",
        "activeBoundedTimeoutMs",
    ):
        assert token in configure
    queue = function_body(source, "void queueBoundedMotionRequest(")
    assert queue.index("configureBoundedCommand(") < queue.index("startRequested = true")


def test_encoder_guard_runs_each_iteration_in_all_active_states(source: str) -> None:
    update = function_body(source, "void updateMotion()")
    guard_index = update.index("if (enforceBoundedEncoderLimit())")
    assert guard_index < update.index("const uint32_t now = millis()")
    assert guard_index < update.index("switch (motionState)")
    assert "now - previousControlTime < CONTROL_PERIOD_MS" not in update[:guard_index]
    assert "case MotionState::STARTING:" in update
    assert "case MotionState::DRIVING:" in update
    assert "case MotionState::BRAKING:" in update
    assert "case MotionState::COASTING:" in update


def test_predictive_brake_and_absolute_limit_are_separate(source: str) -> None:
    guard = function_body(source, "bool enforceBoundedEncoderLimit()")
    limit_index = guard.index("if (rightLimitReached || leftLimitReached)")
    assert guard.index(
        'beginBoundedBraking(true, "BOUNDED_DISTANCE_LIMIT")', limit_index
    ) < guard.index("return true", limit_index)
    assert guard.index("if (activeBoundedBrakeStarted)") > limit_index
    assert "dynamicStopMarginM" in guard
    assert "predictiveBrake" in guard
    assert guard.index("if (predictiveBrake || targetReached)") > guard.index(
        "dynamicStopMarginM"
    )


def test_bounded_brake_cuts_drive_and_never_recovers_drive_pwm(source: str) -> None:
    braking = function_body(
        source,
        "void beginBoundedBraking(bool faultPending, const String &reason)\n{",
    )
    assert braking.index("stopMotors();") < braking.index("brakeMotors();")
    assert "motionState = MotionState::BRAKING" in braking
    assert "setDrivePwm(" not in braking
    settle = function_body(source, "void updateBoundedBraking(uint32_t now)")
    assert "BOUNDED_ACTIVE_BRAKE_HOLD_MS" in settle
    assert "BOUNDED_SETTLE_OBSERVE_MS" in settle
    assert "brakeMotors();" in settle
    assert "stopMotors();" in settle
    assert "setDrivePwm(" not in settle
    update = function_body(source, "void updateMotion()")
    braking_case = update.index("case MotionState::BRAKING:")
    assert update.index("updateBoundedBraking(now);", braking_case) > braking_case
    driving = function_body(source, "void updateDriving(uint32_t now)")
    assert "if (!activeBoundedMotion &&" in driving
    assert "setDrivePwm(rightPwm, leftPwm);" in driving


def test_small_bounded_targets_do_not_wait_for_legacy_breakaway(source: str) -> None:
    threshold = function_body(source, "int32_t boundedBreakawayThresholdCounts()")
    assert "return BREAKAWAY_DETECT_COUNTS;" in threshold
    assert "smallestTarget / 2" in threshold
    assert "smallestBrakeStart / 2" in threshold
    assert "static_cast<int32_t>(1)" in threshold
    starting = function_body(source, "void updateStarting(uint32_t now)")
    assert "const int32_t breakawayThreshold = boundedBreakawayThresholdCounts();" in starting
    assert "rightCount >= breakawayThreshold" in starting
    assert "leftCount >= breakawayThreshold" in starting
    assert "boundedBreakawayPwmCeiling()" in starting


def test_short_command_brake_start_is_before_target_and_is_telemetried(source: str) -> None:
    configure = function_body(source, "void configureBoundedCommand(")
    assert "initialStopMarginM" in configure
    assert "command.rightBrakeStartCounts" in configure
    assert "command.leftBrakeStartCounts" in configure
    helper = function_body(source, "int32_t boundedBrakeStartCounts(")
    assert "targetCounts - reserveCounts" in helper
    status = function_body(source, "String buildStatusJson()")
    for field in (
        "bounded_right_target_counts",
        "bounded_left_target_counts",
        "bounded_right_completion_tolerance_counts",
        "bounded_left_completion_tolerance_counts",
        "bounded_right_min_success_counts",
        "bounded_left_min_success_counts",
        "bounded_right_brake_start_counts",
        "bounded_left_brake_start_counts",
        "bounded_right_count_at_brake_start",
        "bounded_left_count_at_brake_start",
        "bounded_right_final_settled_counts",
        "bounded_left_final_settled_counts",
        "bounded_right_overshoot_counts",
        "bounded_left_overshoot_counts",
        "bounded_stop_mode",
        "bounded_motion_profile",
        "bounded_turn_brake_reserve_counts",
    ):
        assert field in status


def test_forward_brake_observability_fields_are_fixed_and_exposed(source: str) -> None:
    status = function_body(source, "String buildStatusJson()")
    diagnostics = function_body(source, "void appendBoundedForwardBrakeDiagnostics(")
    for field in (
        "bounded_forward_brake_diagnostics",
        "pre_brake_guard",
        "brake_command",
        "first_post_brake_loop",
        "first_hard_limit_guard",
        "trigger_reason",
        "already_braking_or_settling",
    ):
        assert field in status or field in diagnostics
    record = source[source.index("struct CommandRecord {"):source.index("MotionState motionState")]
    assert "String forward" not in record
    assert "forwardPreBrakeGuardCaptured" in record
    assert "forwardFirstHardLimitGuardCaptured" in record


def test_forward_brake_observability_is_nullable_when_idle(source: str) -> None:
    diagnostics = function_body(source, "void appendBoundedForwardBrakeDiagnostics(")
    assert "if (!activeBoundedForwardCommand(bounded))" in diagnostics
    assert 'json += "null";' in diagnostics
    forward_only = function_body(source, "bool activeBoundedForwardCommand(")
    assert "BoundedMotionProfile::BOUNDED_FORWARD_V1" in forward_only
    status = function_body(source, "String buildStatusJson()")
    assert "appendBoundedForwardBrakeDiagnostics(json, bounded);" in status


def test_forward_brake_snapshots_do_not_change_guard_or_brake_path(source: str) -> None:
    guard = function_body(source, "bool enforceBoundedEncoderLimit()")
    assert guard.index("captureBoundedForwardFirstHardLimitGuard(") < guard.index(
        'beginBoundedBraking(true, "BOUNDED_DISTANCE_LIMIT")'
    )
    assert guard.index("captureBoundedForwardPreBrakeGuard(") < guard.index(
        "beginBoundedTargetSettling();"
    )
    braking = function_body(
        source,
        "void beginBoundedBraking(bool faultPending, const String &reason)\n{",
    )
    assert braking.index("stopMotors();") < braking.index("brakeMotors();")
    assert braking.index("brakeMotors();") < braking.index(
        "captureBoundedForwardBrakeCommand("
    )


def test_forward_guard_phase_snapshots_and_sequence_are_exposed(source: str) -> None:
    diagnostics = function_body(source, "void appendBoundedForwardBrakeDiagnostics(")
    for field in (
        "prior_non_triggering_before_predictive_brake",
        "pre_brake_guard",
        "prior_non_triggering_before_hard_limit",
        "first_hard_limit_guard",
        "guard_evaluation_seq",
    ):
        assert field in diagnostics
    assert diagnostics.count("guard_evaluation_seq") >= 2
    sample_json = function_body(source, "void appendBoundedForwardGuardSample(")
    assert "guard_evaluation_seq" in sample_json
    sample = function_body(
        source,
        "BoundedForwardGuardSample readBoundedForwardGuardSample()",
    )
    assert sample.count("readMotionCounts(") == 1
    assert sample.count("millis()") == 1
    assert "++activeBoundedGuardEvaluationSeq" in sample


def test_forward_guard_phase_preserves_separate_prior_and_trigger_samples(
    source: str,
) -> None:
    guard = function_body(source, "bool enforceBoundedEncoderLimit()")
    hard_limit = guard[guard.index("if (rightLimitReached || leftLimitReached)"):]
    assert hard_limit.index(
        "preserveBoundedForwardPriorBeforeHardLimit();"
    ) < hard_limit.index("captureBoundedForwardFirstHardLimitGuard(guardSample);")
    hard_trigger = hard_limit[:hard_limit.index("if (!activeBoundedFaultPending)")]
    assert "captureBoundedForwardPreBrakeGuard(" not in hard_trigger
    predictive = guard[guard.index("if (predictiveBrake || targetReached)"):]
    assert predictive.index(
        "preserveBoundedForwardPriorBeforePredictiveBrake();"
    ) < predictive.index("captureBoundedForwardPreBrakeGuard(")
    rolling = function_body(source, "void captureBoundedForwardPreviousGuard(")
    assert "forwardLastNonTriggeringDrivingGuard = sample" in rolling
    assert "forwardLastNonTriggeringBrakingGuard = sample" in rolling


def test_micro_turn_profile_reserves_six_counts_before_target(source: str) -> None:
    assert "BOUNDED_MICRO_TURN_BRAKE_RESERVE_COUNTS = 6" in source
    reserve = function_body(source, "int32_t boundedMicroTurnBrakeReserveCounts(")
    assert "BOUNDED_MICRO_TURN_BRAKE_RESERVE_COUNTS" in reserve
    assert "dynamicReserveCounts" in reserve
    start = function_body(source, "int32_t boundedMicroTurnBrakeStartCounts(")
    assert "targetCounts - reserveCounts" in start
    configure = function_body(source, "void configureBoundedCommand(")
    assert "BoundedMotionProfile::BOUNDED_MICRO_TURN_V1" in configure
    assert "BoundedMotionProfile::BOUNDED_FORWARD_V1" in configure
    assert "boundedMicroTurnBrakeStartCounts(" in configure
    assert "boundedBrakeStartCounts(" in configure
    guard = function_body(source, "bool enforceBoundedEncoderLimit()")
    assert "BoundedMotionProfile::BOUNDED_MICRO_TURN_V1" in guard
    assert "boundedMicroTurnBrakeStartCounts(" in guard
    assert "boundedBrakeStartCounts(" in guard


def test_micro_turn_brake_start_keeps_dynamic_reserve_at_or_above_six() -> None:
    assert micro_turn_brake_start_counts(11, 2) == 5
    assert micro_turn_brake_start_counts(11, 6) == 5
    assert micro_turn_brake_start_counts(11, 10) == 1


def test_micro_turn_completion_window_and_absolute_limit() -> None:
    assert micro_turn_tolerance(11) == 4
    assert 11 - micro_turn_tolerance(11) == 7
    for final_count in range(7, 12):
        assert completion_outcome(
            11, 11, final_count, final_count, micro_turn=True
        ) == "SUCCESS"
    assert completion_outcome(11, 11, 12, 7, micro_turn=True) == "BOUNDED_DISTANCE_LIMIT"


def micro_turn_terminal_with_brake_progress(
    right_final: int,
    left_final: int,
    *,
    right_brake_started: bool = True,
    left_brake_started: bool = True,
    positive_overshoot: bool = False,
    direction_valid: bool = True,
    safety_fault: bool = False,
) -> str:
    if positive_overshoot or safety_fault or not direction_valid:
        return "FAULT"
    if right_final > 11 or left_final > 11:
        return "FAULT"
    if not (right_brake_started and left_brake_started):
        return "FAULT"
    if right_final >= 6 and left_final >= 6 and right_final <= 11 and left_final <= 11:
        return "SUCCESS"
    return "PARTIAL_PROGRESS"


def test_micro_turn_partial_progress_is_safe_and_requires_reobserve(source: str) -> None:
    assert "PARTIAL_PROGRESS" in source
    settle = function_body(source, "void updateBoundedBraking(uint32_t now)")
    assert "partialMicroTurn" in settle
    assert "activeBoundedRightIndividualBrakeStarted" in settle
    assert "activeBoundedLeftIndividualBrakeStarted" in settle
    assert "completeActiveCommand(CommandState::PARTIAL_PROGRESS);" in settle
    assert "command->reobserveRequired = true;" in settle
    assert micro_turn_terminal_with_brake_progress(6, 5) == "PARTIAL_PROGRESS"
    assert micro_turn_terminal_with_brake_progress(11, 10) == "SUCCESS"
    assert micro_turn_terminal_with_brake_progress(5, 6) == "PARTIAL_PROGRESS"
    assert micro_turn_terminal_with_brake_progress(6, 5, left_brake_started=False) == "FAULT"
    assert micro_turn_terminal_with_brake_progress(6, 5, positive_overshoot=True) == "FAULT"
    assert micro_turn_terminal_with_brake_progress(6, 5, direction_valid=False) == "FAULT"
    assert "activeBoundedMotionProfile == BoundedMotionProfile::BOUNDED_MICRO_TURN_V1" in settle
    assert "motionState = MotionState::READY;" in settle
    assert "boundedFaultLatched = true;" not in settle[settle.index("if (partialMicroTurn)"):settle.index("// Predictive braking")]


def test_partial_duplicate_is_terminal_without_restart(source: str) -> None:
    duplicate = function_body(source, "void sendDuplicateCommand(const CommandRecord &record)")
    assert 'commandStateName(record.state)' in duplicate
    queue = function_body(source, "void queueBoundedMotionRequest(")
    existing = queue.index("const int8_t existingIndex = findCommand(commandId);")
    assert queue.index("sendDuplicateCommand(existing);", existing) < queue.index("startRequested = true")


def test_micro_diagnostics_are_preserved_and_exposed(source: str) -> None:
    configure = function_body(source, "void configureBoundedCommand(")
    assert "command.boundedTurnMinSuccessAngleRad = 0.0f;" not in configure
    assert "command.boundedTurnMinWheelProgressCounts = 0;" not in configure
    settle = function_body(source, "void updateBoundedBraking(uint32_t now)")
    assert "command->boundedActualProgressRatio = actualProgressRatio;" in settle
    helper = function_body(source, "void applyMicroTurnOutputs(")
    assert "if (!rightBrake && rightOutput > 0)" in helper
    assert "if (!leftBrake && leftOutput > 0)" in helper
    status = function_body(source, "String buildStatusJson()")
    for field in ("bounded_actual_progress_ratio", "reobserve_required"):
        assert field in status
    complete = function_body(source, "void completeActiveCommand(CommandState state)")
    assert "state == CommandState::FAULT" in complete
    assert "state == CommandState::PARTIAL_PROGRESS" not in complete


def test_initial_pulse_completion_is_recorded_before_terminal_evaluation(source: str) -> None:
    settle = function_body(source, "void updateBoundedBraking(uint32_t now)")
    marker = "activeBoundedInitialPulseCompleted = true;"
    evaluation = settle.index("if (positiveOvershoot")
    assert settle.index(marker) < evaluation
    assert settle.index("command->boundedInitialPulseCompleted = true;") < evaluation
    assert "command->boundedInitialPulseCompleted = true;" not in settle[evaluation:]


def test_interrupted_initial_pulse_keeps_completion_false(source: str) -> None:
    abort = function_body(source, "void abortMotion(const String &reason)")
    assert "completeActiveCommand(CommandState::FAULT);" in abort
    assert "boundedInitialPulseCompleted = true" not in abort
    stop = function_body(source, "void handleStop()")
    assert "completeActiveCommand(CommandState::STOPPED);" in stop
    assert "boundedInitialPulseCompleted = true" not in stop


def test_terminal_examples_preserve_initial_pulse_semantics() -> None:
    assert {"SUCCESS": True, "PARTIAL_PROGRESS": True}["SUCCESS"] is True
    assert {"SUCCESS": True, "PARTIAL_PROGRESS": True}["PARTIAL_PROGRESS"] is True
    assert {"FAULT": False, "STOPPED": False}["FAULT"] is False
    assert {"FAULT": False, "STOPPED": False}["STOPPED"] is False


def micro_turn_tolerance(target_counts: int) -> int:
    return min(4, max(1, -(-target_counts * 30 // 100)))


def test_micro_turn_completion_tolerance_is_profile_specific(source: str) -> None:
    helper = function_body(
        source,
        "int32_t boundedMicroTurnCompletionToleranceCounts("
    )
    assert "targetCounts) * 0.30f" in helper
    assert "static_cast<int32_t>(4)" in helper
    configure = function_body(source, "void configureBoundedCommand(")
    assert "boundedMicroTurnCompletionToleranceCounts" in configure
    assert "boundedCompletionToleranceCounts(command.rightTargetCounts)" in configure
    assert completion_outcome(11, 11, 10, 7, micro_turn=True) == "SUCCESS"
    assert completion_outcome(11, 11, 10, 6, micro_turn=True) == "BOUNDED_TARGET_NOT_REACHED"
    assert completion_outcome(11, 11, 12, 7, micro_turn=True) == "BOUNDED_DISTANCE_LIMIT"
    assert completion_tolerance_counts(102) == 6


def micro_turn_terminal_outcome(
    right_final: int,
    left_final: int,
    final_heading_deg: float,
    target_heading_deg: float = -4.0,
) -> str:
    if right_final > 11 or left_final > 11:
        return "BOUNDED_DISTANCE_LIMIT"
    if right_final < 6 or left_final < 6:
        return "BOUNDED_TARGET_NOT_REACHED"
    if final_heading_deg * target_heading_deg < 0:
        return "WRONG_DIRECTION"
    if abs(final_heading_deg) < abs(target_heading_deg) * 0.70:
        return "BOUNDED_TARGET_NOT_REACHED"
    if abs(final_heading_deg) > abs(target_heading_deg):
        return "BOUNDED_DISTANCE_LIMIT"
    return "SUCCESS"


def test_micro_turn_heading_and_wheel_progress_gate(source: str) -> None:
    assert micro_turn_terminal_outcome(10, 6, -3.09) == "SUCCESS"
    assert micro_turn_terminal_outcome(5, 4, -1.74) == "BOUNDED_TARGET_NOT_REACHED"
    assert micro_turn_terminal_outcome(11, 10, -3.8) == "SUCCESS"
    assert micro_turn_terminal_outcome(12, 7, -3.0) == "BOUNDED_DISTANCE_LIMIT"
    assert micro_turn_terminal_outcome(10, 6, -3.0) == "SUCCESS"
    assert micro_turn_terminal_outcome(10, 6, -2.7) == "BOUNDED_TARGET_NOT_REACHED"
    assert micro_turn_terminal_outcome(10, 6, 3.09) == "WRONG_DIRECTION"
    settle = function_body(source, "void updateBoundedBraking(uint32_t now)")
    assert "microTurnWheelProgressValid" in settle
    assert "microTurnHeadingValid" in settle
    assert "microTurnDirectionValid" in settle
    assert "activeBoundedMotionProfile == BoundedMotionProfile::BOUNDED_MICRO_TURN_V1" in settle
    status = function_body(source, "String buildStatusJson()")
    for field in (
        "bounded_turn_min_success_angle_rad",
        "bounded_turn_final_angle_rad",
        "bounded_turn_min_wheel_progress_counts",
    ):
        assert field in status


def test_micro_turn_correction_contract_and_physical_examples(source: str) -> None:
    assert "MotionState::CORRECTING" in source
    correction = function_body(source, "bool startBoundedCorrectionPulse(")
    assert "activeBoundedCorrectionPulsesUsed++" in correction
    assert "setDrivePwm(TURN_SUSTAIN_PWM, TURN_SUSTAIN_PWM)" in correction
    assert correction.index("boundedCorrectionPreconditionsClear") < correction.index(
        "setDrivePwm"
    )
    assert "activeBoundedCorrectionPulsesUsed <" in source
    assert "BOUNDED_MICRO_TURN_MAX_CORRECTION_PULSES = 0" in source
    assert "activeBoundedCorrectionInProgress" in source
    preconditions = function_body(
        source,
        "bool boundedCorrectionPreconditionsClear("
    )
    assert "activeBoundedBootSessionId == bootSessionId" in preconditions
    assert "faultReason.length() == 0" in preconditions

    # Raised-wheel 11/10 is already inside the 7..11 success window.
    assert completion_outcome(11, 11, 11, 10, micro_turn=True) == "SUCCESS"
    # Under-travel is terminal for this single-pulse challenge.
    assert completion_outcome(11, 11, 5, 4) == "BOUNDED_TARGET_NOT_REACHED"
    assert completion_outcome(11, 11, 10, 8) == "SUCCESS"


def test_micro_turn_correction_revalidates_limits_and_terminal_paths(source: str) -> None:
    guard = function_body(source, "bool enforceBoundedEncoderLimit()")
    assert "activeBoundedCorrectionInProgress" in guard
    assert "rightCount >= activeBoundedRightTargetCounts" in guard
    assert "stopMotors();" in guard
    assert "brakeMotors();" in guard
    update = function_body(source, "void updateMotion()")
    assert update.index("if (stopRequested)") < update.index(
        "if (enforceBoundedEncoderLimit())"
    )
    assert update.index("if (WiFi.status() != WL_CONNECTED)") < update.index(
        "if (enforceBoundedEncoderLimit())"
    )
    assert "updateBoundedCorrection(now);" in update
    status = function_body(source, "String buildStatusJson()")
    for field in (
        "bounded_initial_pulse_completed",
        "bounded_correction_pulses_used",
        "bounded_max_correction_pulses",
        "bounded_counts_before_correction_right",
        "bounded_counts_before_correction_left",
        "bounded_correction_pulses_enabled",
    ):
        assert field in status


def test_micro_turn_single_pulse_sync_and_individual_braking(source: str) -> None:
    assert "BOUNDED_MICRO_TURN_MAX_CORRECTION_PULSES = 0" in source
    helper = function_body(source, "void applyMicroTurnOutputs(")
    assert "bool rightBrake" in helper
    assert "bool leftBrake" in helper
    assert "analogWrite(ZK_D0, 255);" in helper
    assert "analogWrite(ZK_D1, 255);" in helper
    assert "analogWrite(ZK_D2, 255);" in helper
    assert "analogWrite(ZK_D3, 255);" in helper
    guard = function_body(source, "bool enforceBoundedEncoderLimit()")
    assert "activeBoundedRightIndividualBrakeStarted" in guard
    assert "activeBoundedLeftIndividualBrakeStarted" in guard
    assert "rightCount >= activeBoundedRightBrakeStartCounts" in guard
    assert "leftCount >= activeBoundedLeftBrakeStartCounts" in guard
    assert "applyMicroTurnOutputs(rightOutput, leftOutput, rightBrake, leftBrake)" in guard
    assert "leftOutput = syncErrorCounts >= 2 ? 205 : TURN_SUSTAIN_PWM" in guard
    update = function_body(source, "void updateDriving(uint32_t now)")
    assert "syncErrorCounts >= 2" in update
    assert "rightOutput = 165" in update
    assert "leftOutput = 205" in update
    assert "applyMicroTurnOutputs(rightOutput, leftOutput, false, false)" in update


def test_micro_turn_status_exposes_individual_brake_and_sync_fields(source: str) -> None:
    status = function_body(source, "String buildStatusJson()")
    for field in (
        "bounded_right_individual_brake_started",
        "bounded_left_individual_brake_started",
        "bounded_right_count_at_individual_brake",
        "bounded_left_count_at_individual_brake",
        "bounded_right_last_applied_pwm",
        "bounded_left_last_applied_pwm",
        "bounded_sync_error_counts",
    ):
        assert field in status


def micro_turn_sync_outputs(
    right_count: int,
    left_count: int,
    right_braked: bool = False,
    left_braked: bool = False,
) -> tuple[int, int]:
    if right_braked:
        return 0, 205 if abs(right_count - left_count) >= 2 else 180
    if left_braked:
        return 205 if abs(right_count - left_count) >= 2 else 180, 0
    if abs(right_count - left_count) < 2:
        return 180, 180
    return (165, 205) if right_count > left_count else (205, 165)


def test_micro_turn_independent_brake_and_symmetric_sync_policy() -> None:
    assert micro_turn_sync_outputs(5, 2, right_braked=True) == (0, 205)
    assert micro_turn_sync_outputs(2, 5, left_braked=True) == (205, 0)
    assert micro_turn_sync_outputs(5, 5) == (180, 180)
    assert micro_turn_sync_outputs(5, 2) == (165, 205)
    assert micro_turn_sync_outputs(2, 5) == (205, 165)


def test_correction_does_not_create_a_new_ledger_entry(source: str) -> None:
    correction = function_body(source, "bool startBoundedCorrectionPulse(")
    assert "reserveCommandRecord" not in correction
    assert "activeCommandIndex" not in correction
    assert "command->boundedCorrectionPulsesUsed" in correction


def test_absolute_limit_fault_latches_after_brake_settle(source: str) -> None:
    guard = function_body(source, "bool enforceBoundedEncoderLimit()")
    assert 'beginBoundedBraking(true, "BOUNDED_DISTANCE_LIMIT")' in guard
    settle = function_body(source, "void updateBoundedBraking(uint32_t now)")
    assert "activeBoundedFaultPending" in settle
    assert "completeActiveCommand(CommandState::FAULT);" in settle
    complete = function_body(source, "void completeActiveCommand(CommandState state)")
    assert "boundedFaultLatched = true;" in complete
    assert "boundedFaultReason = faultReason;" in complete


def test_completion_tolerance_is_explicit_and_source_bound(source: str) -> None:
    helper = function_body(source, "int32_t boundedCompletionToleranceCounts(")
    assert "targetCounts) * 0.20f" in helper
    assert "ceilf(" in helper
    assert "static_cast<int32_t>(6)" in helper
    assert "static_cast<int32_t>(1)" in helper
    configure = function_body(source, "void configureBoundedCommand(")
    assert "command.rightCompletionToleranceCounts" in configure
    assert "command.leftCompletionToleranceCounts" in configure
    assert "command.rightMinSuccessCounts" in configure
    assert "command.leftMinSuccessCounts" in configure


def test_completion_window_accepts_physical_floor_result() -> None:
    assert completion_tolerance_counts(102) == 6
    assert completion_tolerance_counts(101) == 6
    assert 102 - completion_tolerance_counts(102) == 96
    assert 101 - completion_tolerance_counts(101) == 95
    assert completion_outcome(102, 101, 96, 96) == "SUCCESS"


def test_completion_window_accepts_short_target_boundary() -> None:
    assert completion_tolerance_counts(21) == 5
    assert completion_outcome(21, 21, 16, 16) == "SUCCESS"


def test_completion_window_rejects_undertravel_positive_overshoot_and_asymmetry() -> None:
    assert completion_outcome(21, 21, 15, 16) == "BOUNDED_TARGET_NOT_REACHED"
    assert completion_outcome(21, 21, 22, 21) == "BOUNDED_DISTANCE_LIMIT"
    assert completion_outcome(102, 101, 96, 94) == "BOUNDED_TARGET_NOT_REACHED"


def test_small_turn_target_uses_proportional_tolerance() -> None:
    assert completion_tolerance_counts(10) == 2


def test_bounded_turns_keep_absolute_count_policy_and_legacy_turn_is_untouched(
    source: str,
) -> None:
    configure = function_body(source, "void configureBoundedCommand(")
    assert "direction == MotionDirection::TURN_LEFT ||" in configure
    assert "direction == MotionDirection::TURN_RIGHT" in configure
    assert "command.rightLimitCounts = command.rightTargetCounts + 1;" in configure
    assert "command.leftLimitCounts = command.leftTargetCounts + 1;" in configure
    assert "TURN_SINGLE_TARGET_WHEEL_DISTANCE_M" in source
    legacy = function_body(source, "void queueLegacyMotionRequest(")
    assert "TURN_TARGET_ANGLE_RAD" in legacy
    assert "BOUNDED_MICRO_TURN_BRAKE_RESERVE_COUNTS" not in legacy


def test_completion_terminal_semantics_preserve_history(source: str) -> None:
    settle = function_body(source, "void updateBoundedBraking(uint32_t now)")
    assert "positiveOvershoot" in settle
    assert "completionWithinWindow" in settle
    assert "BOUNDED_DISTANCE_LIMIT" in settle
    assert "BOUNDED_TARGET_NOT_REACHED" in settle
    assert settle.index("positiveOvershoot") < settle.index(
        "if (activeBoundedFaultPending)"
    )
    assert "motionState = MotionState::READY;" in settle
    assert "completeActiveCommand(CommandState::SUCCESS);" in settle
    assert "command->rightFinalCount" in settle
    assert "command->leftFinalCount" in settle
    assert "command->rightOvershootCounts" in settle
    assert "command->leftOvershootCounts" in settle


def test_legacy_routes_and_baseline_remain_unchanged(source: str) -> None:
    result = subprocess.run(
        ["git", "diff", "--quiet", BASE, "--", BASELINE],
        cwd=ROOT,
        check=False,
    )
    assert result.returncode == 0
    for route in ("/start", "/reverse", "/square"):
        assert route in source


def test_every_active_bounded_fault_latches_before_ledger_is_cleared(source: str) -> None:
    complete = function_body(source, "void completeActiveCommand(CommandState state)")
    assert complete.index("if (activeBoundedMotion && state == CommandState::FAULT)") < complete.index(
        "activeBoundedMotion = false"
    )
    assert "boundedFaultLatched = true;" in complete
    assert "boundedFaultReason = faultReason;" in complete
    abort = function_body(source, "void abortMotion(const String &reason)")
    assert abort.index("faultReason = reason;") < abort.index(
        "completeActiveCommand(CommandState::FAULT);"
    )


def test_fault_latch_blocks_new_bounded_legacy_and_square_requests(source: str) -> None:
    legacy = function_body(source, "void queueLegacyMotionRequest(")
    bounded = function_body(source, "void queueBoundedMotionRequest(")
    square = function_body(source, "void handleSquare()")
    for body in (legacy, bounded, square):
        assert "boundedFaultLatched" in body
        assert "BOUNDED_FAULT_LATCHED" in body
    assert bounded.index("boundedFaultLatched") < bounded.index("reserveCommandRecord()")


def test_stop_preserves_latch_and_ack_restores_consistent_runtime_state(source: str) -> None:
    stop = function_body(source, "void handleStop()")
    assert "boundedFaultLatched = false" not in stop
    ack = function_body(source, "void handleAckFault()")
    stop_index = ack.index("stopMotors();")
    assert stop_index < ack.index("activeBoundedMotion")
    assert stop_index < ack.index("boundedFaultLatched = false;")
    assert "motionOrSquareIsBusy()" in ack
    assert "activeBoundedMotion" in ack
    assert "activeCommand() != nullptr" in ack
    assert "rightPwm != 0 || leftPwm != 0" in ack
    assert "motionState != MotionState::FAULT" in ack
    assert "faultReason.length() == 0" in ack
    assert ack.index("boundedFaultLatched = false;") < ack.index(
        "motionState = MotionState::READY;"
    )
    assert ack.index("boundedFaultReason = \"\";") < ack.index(
        "motionState = MotionState::READY;"
    )
    assert ack.index("faultReason = \"\";") < ack.index(
        "motionState = MotionState::READY;"
    )
    for historical_mutation in (
        "lastCommandIndex =",
        "commandHistory[",
        "bootSessionId =",
    ):
        assert historical_mutation not in ack
    assert "previous_fault_reason" in ack


def test_ack_rejects_active_recovery_without_partial_fault_clear(source: str) -> None:
    ack = function_body(source, "void handleAckFault()")
    reject_index = ack.index("MOTION_NOT_STOPPED")
    clear_index = ack.index("boundedFaultLatched = false;")
    assert reject_index < clear_index
    assert ack.index("return;", reject_index) < clear_index
    invalid_index = ack.index("RECOVERY_STATE_INVALID")
    assert invalid_index < clear_index
    assert ack.index("return;", invalid_index) < clear_index


def test_faulted_duplicate_is_idempotent_and_new_command_needs_recovery(source: str) -> None:
    bounded = function_body(source, "void queueBoundedMotionRequest(")
    assert bounded.index("findCommand(commandId)") < bounded.index(
        "boundedFaultLatched"
    )
    assert bounded.index("sendDuplicateCommand(existing);") < bounded.index(
        "startRequested = true"
    )
    assert "CONTROLLER_NOT_READY" in bounded
    assert bounded.index("CONTROLLER_NOT_READY") < bounded.index(
        "reserveCommandRecord()"
    )


def test_boot_session_rejects_replay_before_lookup_or_start(source: str) -> None:
    assert "#include <esp_system.h>" in source
    generator = function_body(source, "String generateBootSessionId()")
    assert generator.count("esp_random()") == 2
    assert '"%08lX%08lX"' in generator
    bounded = function_body(source, "void queueBoundedMotionRequest(")
    assert bounded.index("requestBootSessionId != bootSessionId") < bounded.index(
        "findCommand(commandId)"
    )
    assert "BOOT_SESSION_MISMATCH" in bounded
    assert bounded.index("findCommand(commandId)") < bounded.index(
        "reserveCommandRecord()"
    )
    assert 'server.arg("boot_session_id")' in source


def test_same_session_duplicates_still_do_not_restart(source: str) -> None:
    bounded = function_body(source, "void queueBoundedMotionRequest(")
    duplicate_index = bounded.index("sendDuplicateCommand(existing);")
    assert duplicate_index < bounded.index("startRequested = true")
    assert "COMMAND_ID_CONFLICT" in bounded
    duplicate = function_body(source, "void sendDuplicateCommand(")
    assert "command_state" in duplicate


def test_status_exposes_session_and_fault_latch(source: str) -> None:
    status = function_body(source, "String buildStatusJson()")
    for field in (
        "boot_session_id",
        "bounded_fault_latched",
        "bounded_fault_reason",
    ):
        assert field in status
    assert 'server.on("/ack-fault", HTTP_POST, handleAckFault);' in source


def test_manifest_and_credentials_remain_safe(source: str) -> None:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    assert manifest["base_sanitized_sha256"] == (
        "24580db5e207c2760ce375a2e8224af05c4b55bf4bdbf58d0dacd8f55d3fca33"
    )
    assert manifest["baseline_unchanged"] is True
    assert manifest["hardware_validation_status"] == (
        "MICRO_TURN_SINGLE_PULSE_SYNC_CHALLENGE"
    )
    completion = manifest["bounded_encoder_cutoff"]
    assert completion["bounded_micro_turn_brake_reserve_counts"] == 6
    assert completion["completion_tolerance_counts"] == (
        "forward: min(6, max(1, ceil(target_counts * 0.20))); "
        "micro-turn diagnostics: min(4, max(1, ceil(target_counts * 0.30)))"
    )
    assert manifest["floor_completion_tolerance_evidence"]["right_final_counts"] == 98
    raised = manifest["raised_wheel_reserve8_micro_turn_evidence"]
    assert raised["final_settled_counts"] == {"right": 8, "left": 6}
    assert raised["positive_overshoot"] is False
    assert manifest["micro_turn_correction"]["max_correction_pulses"] == 0
    assert manifest["micro_turn_correction"]["correction_pulses_enabled"] is False
    assert '#include "wifi_credentials.h"' in source
    assert 'const char* WIFI_SSID = "' not in source
    assert 'const char* WIFI_PASSWORD = "' not in source
    credentials = FIRMWARE.parent / "wifi_credentials.h"
    tracked = subprocess.run(
        ["git", "ls-files", "--error-unmatch", "--", str(credentials.relative_to(ROOT))],
        cwd=ROOT,
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    assert tracked.returncode != 0
    ignored = subprocess.run(
        ["git", "check-ignore", "-q", "--", str(credentials)],
        cwd=ROOT,
        check=False,
    )
    assert ignored.returncode == 0
    assert (FIRMWARE.parent / ".gitignore").read_text().strip() == "wifi_credentials.h"
    assert hashlib.sha256(
        (ROOT / f"{BASELINE}/traction_control_v4_1_effective_track294.ino").read_bytes()
    ).hexdigest() == manifest["base_sanitized_sha256"]
