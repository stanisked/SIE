from sie_ros2.contracts import (
    ContractError,
    navigation_decision,
    supervisor_state,
    validate_perception,
)


def measurement(**overrides):
    value = {
        "schema_version": "sie.perception.measurement.v1",
        "measurement_id": "measurement:test:1",
        "timestamp": "2026-09-22T10:00:00+00:00",
        "status": "SUCCESS",
        "reference_frame": "rectified_left_optical_frame",
        "units": "m",
        "range_m": 3.0,
        "bearing_deg": 0.0,
        "confidence": 0.9,
        "calibration": {
            "calibration_id": "stereo_calibration_v6",
            "sha256": "a" * 64,
        },
    }
    value.update(overrides)
    return value


def test_forward_is_recommendation_not_authorization():
    decision = navigation_decision(
        validate_perception(measurement()),
        safe_distance_m=2.0,
        bearing_deadband_deg=2.0,
        min_confidence=0.5,
    )
    assert decision["result"] == "RECOMMENDED"
    assert decision["recommended_action"] == "FORWARD_REOBSERVE"
    assert decision["execution_authorized"] is False
    state = supervisor_state(decision)
    assert state["actuator_bridge"] == "DISABLED_PHASE_1"
    assert state["motor_command_performed"] is False


def test_invalid_calibration_hash_is_rejected():
    invalid = measurement(calibration={"calibration_id": "v6", "sha256": "not-a-hash"})
    try:
        validate_perception(invalid)
    except ContractError as error:
        assert "SHA-256" in str(error)
    else:
        raise AssertionError("invalid calibration hash was accepted")


def test_low_confidence_blocks_navigation():
    decision = navigation_decision(
        validate_perception(measurement(confidence=0.2)),
        safe_distance_m=2.0,
        bearing_deadband_deg=2.0,
        min_confidence=0.5,
    )
    assert decision["result"] == "BLOCKED"
    assert decision["recommended_action"] == "STOP"
    assert decision["reason"] == "LOW_CONFIDENCE"
