from sie_ros2.contracts import (
    ContractError,
    ar0234_target_suitability,
    ar0234_unique_non_edge_candidate_selection,
    navigation_decision,
    supervisor_state,
    validate_ar0234_observation,
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


def ar0234_no_target(**overrides):
    value = {
        "schema_version": "sie.ar0234.yolo11_person_upper_body_observation.v1",
        "observation_id": "ar0234-yolo11-person-upper-body:ar0234-ros2-00000312",
        "evidence_id": "ar0234-yolo11-person-upper-body:ar0234-ros2-00000312",
        "source_cycle_id": "ar0234-ros2-00000312",
        "captured_at_utc": "2026-09-28T11:13:36.019926+00:00",
        "reference_frame": "ar0234_image_frame",
        "units": "px",
        "entity_type": "person",
        "class_name": "person_upper_body",
        "model_sha256": "d" * 64,
        "confidence_threshold": 0.4,
        "frame_size_px": {"width": 1920, "height": 1200},
        "model_input_size_px": {"width": 640, "height": 640},
        "raw_detection_count": 6113,
        "detection_count": 0,
        "detections": [],
        "eligible_detection_count": 0,
        "target_status": "NO_TARGET",
        "bbox_xyxy_px": None,
        "center_x_px": None,
        "confidence": None,
        "truncated_left": None,
        "truncated_right": None,
        "truncated_top": None,
        "truncated_bottom": None,
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
    assert decision["measurement_timestamp"] == measurement()["timestamp"]
    assert decision["timestamp"] != decision["measurement_timestamp"]
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


def test_ar0234_no_target_observation_is_valid_but_not_measurement():
    observation = validate_ar0234_observation(ar0234_no_target())
    assert observation["target_status"] == "NO_TARGET"
    assert observation["units"] == "px"


def test_ar0234_observation_rejects_invalid_model_hash():
    try:
        validate_ar0234_observation(ar0234_no_target(model_sha256="not-a-hash"))
    except ContractError as error:
        assert "SHA-256" in str(error)
    else:
        raise AssertionError("invalid model hash was accepted")



def test_unique_non_edge_candidate_is_selected_from_raw_multiple_targets():
    observation = ar0234_no_target(
        confidence_threshold=0.4,
        target_status="MULTIPLE_TARGETS",
        detection_count=2,
        eligible_detection_count=2,
        detections=[
            {
                "bbox_xyxy_px": [771.023, 110.556, 1570.953, 1087.371],
                "center_x_px": 1170.988,
                "confidence": 0.682353,
                "truncated_left": False,
                "truncated_right": False,
                "truncated_top": False,
                "truncated_bottom": False,
            },
            {
                "bbox_xyxy_px": [0.0, 132.312, 101.043, 1108.576],
                "center_x_px": 50.5215,
                "confidence": 0.553209,
                "truncated_left": True,
                "truncated_right": False,
                "truncated_top": False,
                "truncated_bottom": False,
            },
        ],
    )
    selection = ar0234_unique_non_edge_candidate_selection(observation)
    assert selection["disposition"] == "SELECTED_FOR_FURTHER_INTERPRETATION"
    assert selection["reason"] == "UNIQUE_NON_EDGE_CANDIDATE"
    assert selection["selected_candidate_index"] == 0
    assert selection["metric_measurement_authorized"] is False


def test_multiple_non_edge_candidates_remain_rejected():
    candidate = {
        "bbox_xyxy_px": [500.0, 100.0, 900.0, 1000.0],
        "center_x_px": 700.0,
        "confidence": 0.7,
        "truncated_left": False,
        "truncated_right": False,
        "truncated_top": False,
        "truncated_bottom": False,
    }
    observation = ar0234_no_target(
        confidence_threshold=0.4,
        target_status="MULTIPLE_TARGETS",
        detection_count=2,
        eligible_detection_count=2,
        detections=[candidate, {**candidate, "bbox_xyxy_px": [1000.0, 100.0, 1400.0, 1000.0], "center_x_px": 1200.0}],
    )
    selection = ar0234_unique_non_edge_candidate_selection(observation)
    assert selection["disposition"] == "REJECTED"
    assert selection["reason"] == "MULTIPLE_NON_EDGE_CANDIDATES"
    assert selection["metric_measurement_authorized"] is False


def test_near_right_model_grid_edge_is_not_a_non_edge_candidate():
    observation = ar0234_no_target(
        confidence_threshold=0.4,
        target_status="MULTIPLE_TARGETS",
        detection_count=2,
        eligible_detection_count=2,
        detections=[
            {
                "bbox_xyxy_px": [770.946075, 10.91391, 1190.738434, 1149.689117],
                "center_x_px": 980.8422545,
                "confidence": 0.927669,
                "truncated_left": False,
                "truncated_right": False,
                "truncated_top": False,
                "truncated_bottom": False,
            },
            {
                "bbox_xyxy_px": [1772.615295, 220.982346, 1919.851135, 1200.0],
                "center_x_px": 1846.233215,
                "confidence": 0.626962,
                "truncated_left": False,
                "truncated_right": False,
                "truncated_top": False,
                "truncated_bottom": True,
            },
        ],
    )
    selection = ar0234_unique_non_edge_candidate_selection(observation)
    assert selection["disposition"] == "SELECTED_FOR_FURTHER_INTERPRETATION"
    assert selection["selected_candidate_index"] == 0
    assert selection["edge_margin_px"] == 3.0

def test_edge_truncated_target_is_not_eligible_for_measurement():
    observation = ar0234_no_target(
        target_status="SINGLE_TARGET",
        detection_count=1,
        eligible_detection_count=1,
        detections=[{"source": "test"}],
        bbox_xyxy_px=[0.0, 0.0, 124.54, 1032.26],
        center_x_px=62.27,
        confidence=0.676108,
        truncated_left=True,
        truncated_top=True,
        truncated_right=False,
        truncated_bottom=False,
    )
    interpretation = ar0234_target_suitability(observation)
    assert interpretation["disposition"] == "REJECTED"
    assert interpretation["reason"] == "EDGE_TRUNCATED:left,top"
    assert interpretation["geometry_eligible"] is False
    assert interpretation["metric_measurement_authorized"] is False
