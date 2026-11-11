from datetime import datetime, timezone, timedelta

from vision_core.tools.shadow_live_bridge import (
    AR_INTRINSIC_SHA256, observation_from_audit,
)
from vision_core.tools.shadow_range_gate import ACTIVE_STEREO_SHA256, select_target_range


def sample():
    return {
        "status": "ASSOCIATED_DIAGNOSTIC_ONLY",
        "cycle_id": "cycle-1", "audit_id": "dynamic-association-audit:cycle-1",
        "timestamp": "2026-11-11T12:00:00+00:00",
        "reference_frame": "ov9281_physical_left_optical_frame",
        "calibration": {"ov9281_stereo_sha256": ACTIVE_STEREO_SHA256,
                        "ar0234_intrinsic_sha256": AR_INTRINSIC_SHA256},
        "ar_target_track": {"confirmation": {
            "state": "CONFIRMED_BY_CURRENT_RGB_STEREO_ASSOCIATION",
            "track_id": "person-1"}},
        "association": {"x_m": .15, "y_m": 0., "z_m": 1.5,
                        "confidence": .91},
    }


def convert(row, age=.12):
    return observation_from_audit(row, rim_forward_of_optical_center_m=.0125,
        received_mono_s=20.,
        received_utc=datetime(2026, 11, 11, 12, 0, tzinfo=timezone.utc)
        + timedelta(seconds=age))


def test_current_association_maps_to_one_target_and_front_distance():
    result = convert(sample())
    assert result.target.track_id == result.ranges[0].target_track_id == "person-1"
    assert abs(result.ranges[0].distance_m -
               (1.5 * 0.9659258262890683 - .210 - .0125)) < 1e-9
    decision = select_target_range(result.target, list(result.ranges), 20.,
                                   expected_rgb_calibration_id=AR_INTRINSIC_SHA256)
    assert decision.action == "FOLLOW_ELIGIBLE"


def test_missing_ownership_wrong_calibration_and_stale_frame_refuse():
    row = sample()
    row["ar_target_track"]["confirmation"]["track_id"] = None
    assert convert(row).target is None
    row = sample()
    row["calibration"]["ov9281_stereo_sha256"] = "0" * 64
    assert convert(row).ranges == ()
    assert convert(sample(), age=2.).target is None
