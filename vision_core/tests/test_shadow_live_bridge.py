from datetime import datetime, timezone, timedelta

from vision_core.tools.shadow_live_bridge import (
    AR_INTRINSIC_SHA256, observation_from_audit, preview_record,
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


def test_preview_preserves_stereo_refusal_and_capture_latency():
    row = sample()
    row.update(status="ASSOCIATION_UNAVAILABLE", reason="FRUSTUM_DEPTH_SAMPLES_OUT_OF_RANGE",
               association=None, stereo_geometry={
                   "lr_consistent_disparity_sample_count": 300,
                   "in_range_depth_sample_count": 0,
                   "pre_range_depth_diagnostic": {
                       "depth_percentiles_m": [2.7, 2.8, 3., 3.2, 3.4, 3.6, 3.8]}})
    result = preview_record(row, convert(row), now_s=20.,
        received_utc=datetime(2026, 11, 11, 12, 0, 1, tzinfo=timezone.utc))
    assert result["audit_reason"] == "FRUSTUM_DEPTH_SAMPLES_OUT_OF_RANGE"
    assert result["lr_samples"] == 300
    assert result["capture_to_receive_s"] == 1.
    assert result["decision"] == "PAUSE"
    assert result["semantic_source"] is None


def test_preview_keeps_all_points_and_numeric_candidate_separate():
    row=sample()
    row['shadow_rgb_continuity']={'status':'MATCHED_CURRENT_RGB_DIAGNOSTIC_ONLY',
                                  'track_id':'person-1'}
    row['shape_diagnostic']={
        'mask_depth':{'depth_layers':[{'sample_count':120,'median_depth_m':1.2},
                                      {'sample_count':600,'median_depth_m':2.4}],
                      'candidate_sample_count':600,'candidate_mad_m':.02},
        'front_geometry':{'point_set':'ALL_CURRENT_MASK_OWNED_STEREO_POINTS',
                          'known_term_m':{'p05':.95,'median':2.1}},
        'candidate_layer_front_geometry':{
            'point_set':'NUMERIC_CANDIDATE_DEPTH_LAYER_DIAGNOSTIC_ONLY',
            'point_count':600,'known_term_m':{'p05':2.1,'median':2.11}}}
    row['source_snapshot_evidence']={'status':'REQUESTED_AFTER_PRIMARY_PUBLISH',
        'file':'cycle-1.npz','mask_sha256':'mask-hash'}
    result=preview_record(row,convert(row),now_s=20.)
    assert result['front_known_p05_m']==.95
    assert result['candidate_front_known_p05_m']==2.1
    assert result['candidate_front_point_count']==600
    assert result['rgb_continuity_track_id']=='person-1'
    assert result['mask_depth_layers']==row['shape_diagnostic']['mask_depth']['depth_layers']
    assert result['source_snapshot_evidence']==row['source_snapshot_evidence']
