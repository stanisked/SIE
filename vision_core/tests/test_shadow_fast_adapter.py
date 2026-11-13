"""Exact same-cycle Shadow binding using the fast preview schema."""
import copy
import unittest

from vision_core.person_localization.fast_local_depth_consistency import STABLE_LOCAL_PATCH_POLICY
from vision_core.tools.shadow_fast_adapter import (
    AR_INTRINSIC_SHA256, SHADOW_V3_SHA256, front_profile_sha256,
    from_control_preview, match_v3_to_mask_instance,
)
from vision_core.tools.shadow_range_gate import ACTIVE_STEREO_SHA256, select_target_range
from vision_core.tools.shadow_supervisor import ShadowSupervisor


def source():
    cid = "fast-cycle-1"
    sha = "mask-hash"
    person = {"instance_index": 0, "confidence": .91}
    provenance = {"source_cycle_id": cid, "mask_sha256": sha,
                  "instance_index": 0, "model_sha256": "seg-model",
                  "rgb_arrays_sha256": "rgb-array", "fusion_arrays_sha256": "fusion-array"}
    return {
        "cycle_id": cid, "schema_version": "sie.ar0234_ov9281.control_input_preview.v1",
        "status": "MASKED_DEPTH_SHADOW_DIAGNOSTIC_ONLY",
        "calibration": {"ar0234_intrinsic_sha256": AR_INTRINSIC_SHA256,
                        "ov9281_stereo_sha256": ACTIVE_STEREO_SHA256},
        "stereo_resolution": {"mode": "half_640x400"},
        "ar0234_dynamic_target": {"disposition": "OBSERVED", "current_eligible_count": 1,
            "candidate": person},
        "target_geometry": {"accepted": True},
        "segmentation_observation": {"cycle_id": cid,
            "status": "CURRENT_INSTANCES_DIAGNOSTIC_ONLY",
            "model_sha256": "seg-model", "arrays_sha256": "rgb-array",
            "rgb_payload_sha256": "rgb-pixels", "instances": [person],
            "instance_mask_sha256": [sha]},
        "shadow_track_confirmation": {"state": "CONFIRMED_BY_CURRENT_RGB_STEREO_ASSOCIATION",
            "track_id": "person-1", "confirmed_observation_id": f"segmentation-preview:{cid}:0"},
        "shadow_target_bearing_deg": 3.5,
        "shadow_v3_mask_match": {"status": "BOUND_TO_CURRENT_MASK_INSTANCE",
            "v3_model_sha256": SHADOW_V3_SHA256, "source_cycle_id": cid,
            "source_rgb_arrays_sha256": "rgb-array", "rgb_payload_sha256": "rgb-pixels",
            "mask_instance_index": 0},
        "shape_diagnostic": {"cycle_id": cid, "mask_sha256": sha,
            "model_sha256": "seg-model", "arrays_sha256": "fusion-array",
            "mask": {"status": "CURRENT_FRAME_MASK_DIAGNOSTIC_ONLY",
                     "instance_index": 0, "source_rgb_arrays_sha256": "rgb-array"},
            "mask_depth": {"status": "COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY",
                           "candidate_sample_count": 200, "plausible_layer_count": 1},
            "front_geometry": {"status": "PARTIAL_FRONT_GEOMETRY_DIAGNOSTIC_ONLY",
                "known_term_m": {"p05": 1.5}, "point_count": 200,
                "point_set": "ALL_CURRENT_MASK_OWNED_STEREO_POINTS",
                "source_depth_status": "COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY",
                "profile_sha256": front_profile_sha256(), "provenance": dict(provenance)},
            "candidate_layer_front_geometry": {
                "status": "PARTIAL_FRONT_GEOMETRY_DIAGNOSTIC_ONLY",
                "known_term_m": {"p05": 1.7}, "point_count": 200,
                "point_set": "NUMERIC_CANDIDATE_DEPTH_LAYER_DIAGNOSTIC_ONLY",
                "semantic_ownership_verified": False,
                "source_depth_status": "COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY",
                "profile_sha256": front_profile_sha256(), "provenance": dict(provenance)}},
        "frame_freshness": {"capture_backend": "latest_v4l2",
            "endpoint": "RESULT_READY_BEFORE_PUBLICATION; not a motor command",
            "frames": {"ar0234": {"driver_timestamp_s": 9.75,
                                     "host_packet_received_monotonic_s": 9.78,
                                     "flags": 0x12000},
                       "ov9281": {"driver_timestamp_s": 9.76,
                                    "host_packet_received_monotonic_s": 9.79,
                                    "flags": 0x12000}},
            "driver_declared_soe_to_result_s": {"ar0234": .20, "ov9281": .19}},
    }


def convert(row, now=10.):
    return from_control_preview(row, now_s=now,
        rim_forward_of_optical_center_m=.0125,
        expected_front_profile_sha256=front_profile_sha256(),
        expected_segmentation_model_sha256="seg-model")


def convert_mask_only(row, now=10., local_preview=False):
    return from_control_preview(row, now_s=now,
        rim_forward_of_optical_center_m=.0125,
        expected_front_profile_sha256=front_profile_sha256(),
        expected_segmentation_model_sha256="seg-model",
        semantic_source="segmentation_mask",local_preview=local_preview)


class FastAdapterTests(unittest.TestCase):
    def test_mad_refusal_uses_only_bound_stable_patches_in_motor_off_preview(self):
        row=source()
        row.pop('shadow_v3_mask_match')
        row['shadow_semantic_source']='COCO_CURRENT_INSTANCE_MASK'
        row['shape_diagnostic']['mask_depth']['status']='DEPTH_MAD_EXCEEDED'
        for name in ('front_geometry','candidate_layer_front_geometry'):
            row['shape_diagnostic'][name]['source_depth_status']='DEPTH_MAD_EXCEEDED'
        row.pop('shadow_track_confirmation')
        row['shadow_rgb_continuity']={'status':'MATCHED_CURRENT_RGB_DIAGNOSTIC_ONLY',
                                      'track_id':'person-1'}
        row['shape_diagnostic']['stable_candidate_front_geometry']={
            'status':'STABLE_CANDIDATE_FRONT_DIAGNOSTIC_ONLY',
            'point_set':'STABLE_LOCAL_PATCHES_NUMERIC_CANDIDATE_LAYER_DIAGNOSTIC_ONLY',
            'candidate_sample_count':200,'point_count':150,
            'source_depth_status':'DEPTH_MAD_EXCEEDED',
            'profile_sha256':front_profile_sha256(),
            'provenance':dict(row['shape_diagnostic']['front_geometry']['provenance']),
            'known_term_m':{'p05':1.6},'semantic_ownership_verified':False,
            'distance_from_front_m':None,'actuator_access':False,
            'stable_local_patches':{'status':'STABLE_LOCAL_PATCHES_DIAGNOSTIC_ONLY',
                'policy':dict(STABLE_LOCAL_PATCH_POLICY),'selected_point_count':150,
                'selected_patch_count':3,'selected_held_out_residual':{'sample_count':150}}}
        self.assertIsNone(convert_mask_only(row)[0].target)
        observation,reason=convert_mask_only(row,local_preview=True)
        self.assertEqual(reason,'CURRENT_STABLE_LOCAL_PREVIEW')
        self.assertEqual(observation.target.track_id,'person-1')
        self.assertAlmostEqual(observation.ranges[0].distance_m,1.5875)
        self.assertTrue(observation.ranges[0].association_evidence_id.startswith('fast-local-preview:'))
        from vision_core.tools.shadow_live_bridge import preview_record
        record=preview_record(row,observation,now_s=10.,minimum_confidence=.4)
        self.assertEqual(record['track_id'],'person-1')
        self.assertEqual(record['decision'],'FOLLOW_ELIGIBLE')
        self.assertEqual(record['stable_front_point_count'],150)
        missing_bearing=copy.deepcopy(row)
        missing_bearing.pop('shadow_target_bearing_deg')
        self.assertEqual(convert_mask_only(missing_bearing,local_preview=True)[1],
                         'FAST_EVIDENCE_FIELD_MISSING')
        for mutate in (
            lambda x:x['shadow_rgb_continuity'].update(track_id=''),
            lambda x:x['shadow_rgb_continuity'].update(status='UNMATCHED'),
            lambda x:x['shape_diagnostic']['stable_candidate_front_geometry']['provenance'].update(mask_sha256='other'),
            lambda x:x['shape_diagnostic']['stable_candidate_front_geometry']['stable_local_patches']['policy'].update(maximum_residual_mad_m=.5),
            lambda x:x['shape_diagnostic']['stable_candidate_front_geometry'].update(point_count=99),
            lambda x:x['ar0234_dynamic_target'].update(current_eligible_count=2),
            lambda x:x['shape_diagnostic']['mask_depth'].update(plausible_layer_count=2),
        ):
            with self.subTest(mutate=mutate):
                changed=copy.deepcopy(row);mutate(changed)
                self.assertIsNone(convert_mask_only(changed,local_preview=True)[0].target)

    def test_candidate_layer_requires_exact_count_and_provenance(self):
        row = source()
        row["shape_diagnostic"].pop("candidate_layer_front_geometry")
        observation, reason = convert(row)
        self.assertIsNone(observation.target)
        self.assertEqual(reason, "FRONT_LAYER_BINDING_UNVERIFIED")
        row = source()
        row["shape_diagnostic"]["candidate_layer_front_geometry"]["point_count"] = 250
        self.assertEqual(convert(row)[1], "FRONT_LAYER_BINDING_UNVERIFIED")
        row = source()
        row["shape_diagnostic"]["candidate_layer_front_geometry"]["provenance"]["mask_sha256"] = "old"
        self.assertEqual(convert(row)[1], "FRONT_LAYER_BINDING_UNVERIFIED")

    def test_separate_segmentation_semantics_has_explicit_provenance(self):
        row = source()
        row.pop("shadow_v3_mask_match")
        row["shadow_semantic_source"] = "COCO_CURRENT_INSTANCE_MASK"
        observation, reason = convert_mask_only(row)
        self.assertEqual(reason, "CURRENT_FAST_MASK_OBSERVATION")
        self.assertIsNotNone(observation.target)
        self.assertIsNone(convert(row)[0].target)
        row.pop("shadow_semantic_source")
        self.assertIsNone(convert_mask_only(row)[0].target)

    def test_segmentation_semantics_pauses_for_two_instances_or_age(self):
        row = source()
        row.pop("shadow_v3_mask_match")
        row["shadow_semantic_source"] = "COCO_CURRENT_INSTANCE_MASK"
        row["segmentation_observation"]["instances"].append(
            {"instance_index": 1, "confidence": .8})
        row["segmentation_observation"]["instance_mask_sha256"].append("second")
        self.assertIsNone(convert_mask_only(row)[0].target)
        row["segmentation_observation"]["instances"].pop()
        row["segmentation_observation"]["instance_mask_sha256"].pop()
        self.assertIsNone(convert_mask_only(row, now=10.51)[0].target)

    def test_exact_cycle_yields_one_target_and_front_range(self):
        observation, reason = convert(source())
        self.assertEqual(reason, "CURRENT_FAST_MASK_OBSERVATION")
        self.assertEqual(observation.target.track_id, observation.ranges[0].target_track_id)
        self.assertAlmostEqual(observation.ranges[0].distance_m, 1.6875)
        self.assertEqual(select_target_range(observation.target, list(observation.ranges), 10.,
            expected_rgb_calibration_id=AR_INTRINSIC_SHA256,
            minimum_confidence=.4).action, "FOLLOW_ELIGIBLE")

    def test_missing_lost_ambiguous_or_reused_mask_pauses(self):
        for mutate in (
            lambda row: row["ar0234_dynamic_target"].update(disposition="UNAVAILABLE"),
            lambda row: row["ar0234_dynamic_target"].update(current_eligible_count=2),
            lambda row: row["shape_diagnostic"]["mask"].update(source_rgb_arrays_sha256="old"),
            lambda row: row["segmentation_observation"]["instance_mask_sha256"].__setitem__(0, "other"),
            lambda row: row["shape_diagnostic"]["front_geometry"]["provenance"].update(source_cycle_id="old"),
            lambda row: row.pop("shadow_track_confirmation"),
            lambda row: row["shadow_track_confirmation"].update(confirmed_observation_id="old"),
            lambda row: row["shape_diagnostic"]["mask_depth"].update(plausible_layer_count=2),
            lambda row: row["shadow_v3_mask_match"].update(status="V3_MASK_MATCH_UNAVAILABLE"),
            lambda row: row["shadow_v3_mask_match"].update(rgb_payload_sha256="old"),
            lambda row: row.update(status="CANCELLED_SHUTDOWN"),
            lambda row: row["segmentation_observation"].update(model_sha256="wrong"),
            lambda row: row["frame_freshness"]["frames"]["ar0234"].update(flags=0),
        ):
            with self.subTest(mutate=mutate):
                row = copy.deepcopy(source())
                mutate(row)
                observation, reason = convert(row)
                self.assertIsNone(observation.target)
                self.assertNotEqual(reason, "CURRENT_FAST_MASK_OBSERVATION")

    def test_driver_age_to_decision_not_host_cycle_duration(self):
        row = source()
        self.assertIsNone(convert(row, now=10.51)[0].target)
        row["frame_freshness"]["frames"]["ov9281"]["driver_timestamp_s"] = 9.0
        self.assertIsNone(convert(row)[0].target)

    def test_v3_box_needs_unique_same_cycle_segmentation_instance(self):
        segmentation = {"cycle_id": "one", "arrays_sha256": "rgb",
                        "rgb_payload_sha256": "pixels",
                        "instances": [{"instance_index": 0,
                                       "bbox_xyxy_px": [80, 60, 210, 430]},
                                      {"instance_index": 1,
                                       "bbox_xyxy_px": [300, 55, 420, 425]}]}
        v3 = {"source_cycle_id": "one", "model_sha256": SHADOW_V3_SHA256,
              "target_status": "SINGLE_TARGET", "detection_count": 1,
              "bbox_xyxy_px": [95, 70, 185, 170]}
        self.assertEqual(match_v3_to_mask_instance(v3, segmentation,
            segmentation["instances"][0], rgb_payload_sha256="pixels")["status"],
            "BOUND_TO_CURRENT_MASK_INSTANCE")
        self.assertEqual(match_v3_to_mask_instance(v3, segmentation,
            segmentation["instances"][1], rgb_payload_sha256="pixels")["status"],
            "V3_MASK_MATCH_UNAVAILABLE")
        v3["source_cycle_id"] = "old"
        self.assertEqual(match_v3_to_mask_instance(v3, segmentation,
            segmentation["instances"][0], rgb_payload_sha256="pixels")["status"],
            "V3_MASK_MATCH_UNAVAILABLE")

    def test_one_command_pauses_on_loss_and_does_not_take_new_identity(self):
        supervisor = ShadowSupervisor("boot-1", "shadow-1")
        first, _ = convert(source())
        decide = lambda observation: select_target_range(
            observation.target, list(observation.ranges), 10.,
            expected_rgb_calibration_id=AR_INTRINSIC_SHA256,
            minimum_confidence=.4)
        start = supervisor.observe(first.target, decide(first))
        self.assertEqual(start.operation, "START")
        supervisor.acknowledge(start, boot_session_id="boot-1", active_command_id="shadow-1",
            controller_state="DRIVING", right_pwm=50, left_pwm=50,
            fault_latched=False, transition_seq=start.transition_seq)
        lost = source()
        lost["ar0234_dynamic_target"]["disposition"] = "UNAVAILABLE"
        empty, _ = convert(lost)
        pause = supervisor.observe(empty.target, decide(empty))
        self.assertEqual(pause.operation, "PAUSE")
        supervisor.acknowledge(pause, boot_session_id="boot-1", active_command_id="shadow-1",
            controller_state="PAUSED", right_pwm=0, left_pwm=0,
            fault_latched=False, transition_seq=pause.transition_seq)
        reacquired = source()
        reacquired["shadow_track_confirmation"]["track_id"] = "new-person"
        other, _ = convert(reacquired)
        self.assertIsNone(supervisor.observe(other.target, decide(other)))
        multiple = source()
        multiple["ar0234_dynamic_target"]["current_eligible_count"] = 2
        ambiguous, _ = convert(multiple)
        self.assertIsNone(supervisor.observe(ambiguous.target, decide(ambiguous)))


if __name__ == "__main__":
    unittest.main()
