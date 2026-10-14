"""Task-budget decision preview. No hardware adapter or motion publication."""
import math


def approach_control_preview(*, admitted_measurement=False, measurement_id=None,
        clearance_m=None, clearance_error_bound_m=None, exposure_age_s=None,
        closing_speed_bound_m_s=None, target_closing_speed_bound_m_s=None,
        measured_stop_distance_bound_m=None, measured_stop_time_bound_s=None,
        aligned_in_robot_frame=False):
    result = {"status": "STOP", "preview_only": True, "motor_command_performed": False,
              "requested_working_speed_m_s": .15, "recommended_speed_m_s": 0.,
              "stand_off_from_robot_front_m": 1., "source_measurement_id": measurement_id,
              "reasons": [], "policy_status": "DRAFT_NOT_ACTIVATED"}
    reasons = result["reasons"]
    if not admitted_measurement or not isinstance(measurement_id, str) or not measurement_id:
        reasons.append("NO_ADMITTED_METRIC_MEASUREMENT")
    values = {"ROBOT_FRONT_CLEARANCE": clearance_m, "PHYSICAL_ERROR_BOUND": clearance_error_bound_m,
              "EXPOSURE_AGE": exposure_age_s, "CLOSING_SPEED_BOUND": closing_speed_bound_m_s,
              "TARGET_CLOSING_SPEED_BOUND": target_closing_speed_bound_m_s,
              "MEASURED_STOP_DISTANCE_BOUND": measured_stop_distance_bound_m,
              "MEASURED_STOP_TIME_BOUND": measured_stop_time_bound_s}
    for name, value in values.items():
        if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
            reasons.append(name + "_UNKNOWN_OR_INVALID")
    if not aligned_in_robot_frame:
        reasons.append("ROBOT_FRAME_ALIGNMENT_NOT_ESTABLISHED")
    if reasons:
        return result
    error_limit = .10 if clearance_m <= 2. else .05 * clearance_m
    result["required_error_bound_m"] = error_limit
    if clearance_error_bound_m > error_limit:
        reasons.append("PHYSICAL_ERROR_EXCEEDS_TASK_REQUIREMENT")
    if exposure_age_s > .25:
        reasons.append("EXPOSURE_AGE_EXCEEDS_250MS")
    if measured_stop_distance_bound_m > .025 or measured_stop_time_bound_s > .5:
        reasons.append("DRIVE_STOPPING_EXCEEDS_TASK_REQUIREMENT")
    margin = (clearance_error_bound_m + closing_speed_bound_m_s * exposure_age_s
              + measured_stop_distance_bound_m
              + target_closing_speed_bound_m_s * measured_stop_time_bound_s + .10)
    result["required_clearance_m"] = 1. + margin
    if clearance_m <= 1. + margin:
        reasons.append("STOPPING_CLEARANCE_BUDGET_EXHAUSTED")
    if not reasons:
        result.update(status="APPROACH", recommended_speed_m_s=.15)
    return result
