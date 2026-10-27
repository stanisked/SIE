from copy import deepcopy
from pathlib import Path
import tempfile
import unittest

from firmware.esp32.tests.test_remote_stop_trial import Harness
from vision_core.tests.test_control_depth_admission_draft import source
from vision_core.tools.run_v7_supervised_one_step import (
    check_observation, execute_observed_step, REVIEWED_SHA, SOURCE_HASHES, EXTRINSIC_SHA, MODEL_SHA, main)


def current_audit():
    a = source()
    cid = a['cycle_id']
    a.update(schema_version='sie.ar0234_ov9281.control_input_preview.v1',
        activated_depth_range_m=[.5, 4.5],
        calibration={'ar0234_intrinsic_sha256': SOURCE_HASHES['ar0234_intrinsic'],
            'ov9281_stereo_sha256': SOURCE_HASHES['ov9281_stereo_calibration'],
            'ar0234_to_physical_left_extrinsic_sha256': EXTRINSIC_SHA},
        existing_static_profile_preview={'required': True, 'profile_sha256': REVIEWED_SHA},
        existing_profile_static_gate={'cycle_id': cid, 'profile_sha256': REVIEWED_SHA, 'status': 'PASS'})
    a['working_targets'].update(exposure_to_command_age_max_s=.5,
        freshness_policy_id='experimental_freshness_500ms_v1')
    a['shape_diagnostic'].update(cycle_id=cid, model_sha256=MODEL_SHA)
    a['shape_diagnostic']['fast_local_consistency']['provenance']['model_sha256'] = MODEL_SHA
    a['shape_diagnostic']['mask_depth']['status'] = 'COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY'
    candidate = {'instance_index': 0, 'confidence': .9}
    a['ar0234_dynamic_target'].update(candidate=candidate, current_eligible_count=1)
    a['segmentation_observation'] = {'cycle_id': cid, 'model_sha256': MODEL_SHA, 'arrays_sha256': 'rgb', 'instances': [candidate]}
    a['frame_freshness']['frames'] = {k: {'driver_timestamp_s': 1.,
        'host_packet_received_monotonic_s': 1.01, 'flags': 73729} for k in ('ar0234', 'ov9281')}
    return a


class OneStepTests(unittest.TestCase):
    def test_default_opens_no_ros_or_network(self):
        self.assertEqual(main([]), 0)

    def test_fresh_bound_evidence_keeps_unknowns_and_stop(self):
        a = current_audit(); before = deepcopy(a)
        r = check_observation(a, .9, 1.1)
        self.assertEqual(a, before)
        self.assertEqual(r['diagnostic_decision']['status'], 'STOP')
        self.assertIn('physical_depth_error_bound', r['missing_checks'])
        self.assertFalse(r['diagnostic_is_admitted_measurement'])
        self.assertFalse(r['esp32_acceptance_age_established'])

    def test_stale_preconfirmation_future_missing_or_mismatched_evidence_refused(self):
        for edit in (
            lambda a: a['existing_profile_static_gate'].update(status='PENDING'),
            lambda a: a['calibration'].update(ov9281_stereo_sha256='other'),
            lambda a: a['shape_diagnostic'].update(cycle_id='old'),
            lambda a: a['segmentation_observation'].update(arrays_sha256='other'),
            lambda a: a['shape_diagnostic']['mask_depth'].update(status='DEPTH_MAD_EXCEEDED'),
            lambda a: a['frame_freshness']['frames']['ar0234'].update(driver_timestamp_s=.89),
            lambda a: a['frame_freshness']['frames']['ar0234'].update(driver_timestamp_s=2.),
            lambda a: a['frame_freshness']['frames'].pop('ov9281'),
            lambda a: a['working_targets'].update(exposure_to_command_age_max_s=.25),
            lambda a: a['ar0234_dynamic_target']['candidate'].update(instance_index=1),
            lambda a: a['frame_freshness']['frames']['ar0234'].update(flags=0),
        ):
            a = current_audit(); edit(a)
            with self.assertRaises(ValueError):
                check_observation(a, .9, 1.1)
        with self.assertRaises(ValueError):
            check_observation(current_audit(), .9, 1.500001)

    def run_trial(self, mode='normal', preflight_delay=0):
        h = Harness(mode); h.time = 1.1; req = h.request
        def request(method, url, timeout):
            if method == 'GET' and h.command is None:
                h.time += preflight_delay
            return req(method, url, timeout)
        with tempfile.TemporaryDirectory() as tmp:
            r = execute_observed_step(current_audit(), armed_at=.9,
                base_url='http://127.0.0.1', output=Path(tmp)/'trial',
                request=request, clock=h.clock, sleep=h.sleep)
        return r, h

    def test_one_move_one_stop_matching_terminal_and_source_identity(self):
        r, h = self.run_trial()
        self.assertEqual(r['result'], 'REMOTE_STOP_OBSERVED_DIAGNOSTIC_ONLY')
        self.assertEqual([p for m, p in h.calls if m == 'POST'], ['/move-forward', '/stop'])
        self.assertEqual(r['external_pre_move_evidence']['source_cycle_id'], 'one')

    def test_preflight_before_frame_keeps_fresh_evidence(self):
        h=Harness()
        _,ready=h.request('GET','http://127.0.0.1/status',.5)
        h.calls.clear()
        h.time=1.1
        with tempfile.TemporaryDirectory() as tmp:
            r=execute_observed_step(current_audit(),armed_at=.9,
                base_url='http://127.0.0.1',output=Path(tmp)/'trial',
                request=h.request,clock=h.clock,sleep=h.sleep,
                ready_status=ready,ready_status_at=1.0)
        self.assertEqual(r['result'],'REMOTE_STOP_OBSERVED_DIAGNOSTIC_ONLY')
        self.assertEqual(r['external_pre_move_evidence']['source_cycle_id'],'one')
        self.assertEqual(h.calls[0],('POST','/move-forward'))

    def test_preflight_before_operator_arm_is_refused(self):
        h=Harness()
        _,ready=h.request('GET','http://127.0.0.1/status',.5)
        h.calls.clear()
        h.time=1.1
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(ValueError,'CONTROLLER_STATUS_BEFORE_OPERATOR_ARM'):
                execute_observed_step(current_audit(),armed_at=.9,
                    base_url='http://127.0.0.1',output=Path(tmp)/'trial',
                    request=h.request,clock=h.clock,sleep=h.sleep,
                    ready_status=ready,ready_status_at=.8)
        self.assertEqual(h.calls,[])

    def test_supervised_rgb_forward_trial_accepts_mad_refusal(self):
        a=current_audit()
        a['shape_diagnostic']['mask_depth'].update(
            status='DEPTH_MAD_EXCEEDED', candidate_mad_m=.08)
        h=Harness(); h.time=1.1
        with tempfile.TemporaryDirectory() as tmp:
            r=execute_observed_step(a,armed_at=.9,
                base_url='http://127.0.0.1',output=Path(tmp)/'trial',
                request=h.request,clock=h.clock,sleep=h.sleep,
                supervised_rgb_only=True)
        self.assertEqual(r['result'],'REMOTE_STOP_OBSERVED_DIAGNOSTIC_ONLY')
        self.assertEqual([route for method,route in h.calls if method=='POST'],
                         ['/move-forward','/stop'])
        self.assertTrue(r['external_pre_move_evidence']['supervised_rgb_only'])

    def test_supervised_rgb_target_does_not_require_coherent_mask_depth(self):
        a=current_audit()
        a['shape_diagnostic']['mask_depth'].update(
            status='DEPTH_MAD_EXCEEDED', candidate_mad_m=.08)
        with self.assertRaisesRegex(ValueError,'LEGACY_MASK_QUALITY_REFUSED'):
            check_observation(a,.9,1.1)
        evidence=check_observation(a,.9,1.1,supervised_rgb_only=True)
        self.assertEqual(evidence['mask_depth_status'],'DEPTH_MAD_EXCEEDED')
        self.assertTrue(evidence['supervised_rgb_only'])
        self.assertEqual(evidence['diagnostic_decision']['status'],'STOP')
        a['frame_freshness']['frames']['ar0234']['driver_timestamp_s']=.5
        with self.assertRaises(ValueError):
            check_observation(a,.9,1.1,supervised_rgb_only=True)

    def test_slow_preflight_causes_no_move_and_no_stop(self):
        r, h = self.run_trial(preflight_delay=.401)
        self.assertEqual(r['movement_posts'], 0)
        self.assertEqual(r['stop_posts'], 0)
        self.assertEqual(h.calls, [('GET', '/status')])

    def test_move_timeout_or_interrupt_still_attempts_one_stop(self):
        for mode in ('move_transport', 'interrupt', 'stop_transport', 'reboot'):
            r, h = self.run_trial(mode)
            self.assertEqual([p for m, p in h.calls if m == 'POST'], ['/move-forward', '/stop'])
            self.assertNotEqual(r['result'], 'REMOTE_STOP_OBSERVED_DIAGNOSTIC_ONLY')


if __name__ == '__main__':
    unittest.main()
