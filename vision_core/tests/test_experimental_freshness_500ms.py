from copy import deepcopy
import unittest

from vision_core.tests.test_control_depth_admission_draft import source
from vision_core.person_localization.control_depth_admission_draft import evaluate_depth_admission_draft
from vision_core.person_localization.control_depth_quality_shadow import assess_control_depth


class ExperimentalFreshnessTests(unittest.TestCase):
    def audit(self, age):
        a = source()
        a['working_targets'].update(exposure_to_command_age_max_s=.5,
            freshness_policy_id='experimental_freshness_500ms_v1')
        a['frame_freshness']['driver_declared_soe_to_result_s']['ar0234'] = age
        return a

    def test_boundary_and_no_motion_authority(self):
        for age, expected in ((.499, 'PASS'), (.5, 'PASS'), (.500001, 'FAIL')):
            a = self.audit(age)
            before = deepcopy(a)
            r = evaluate_depth_admission_draft(a)
            self.assertEqual(a, before)
            self.assertEqual(r['checks']['driver_frame_age']['status'], expected)
            self.assertEqual(r['checks']['exposure_to_command_age']['status'], 'UNKNOWN')
            self.assertEqual(r['checks']['exposure_to_command_age']['requirement'], .5)
            self.assertIn('physical_depth_error_bound', r['missing_checks'])
            self.assertEqual(r['decision_preview'], 'STOP')
            self.assertFalse(r['actuator_access'])
            q = assess_control_depth(a)
            self.assertEqual('DRIVER_DECLARED_FRAME_AGE_EXCEEDS_TARGET' in q['refusal_reasons'], expected == 'FAIL')

    def test_old_audits_and_unlabelled_limits_use_original_policy(self):
        for labelled in (False, True):
            a = source()
            a['frame_freshness']['driver_declared_soe_to_result_s']['ar0234'] = .3
            if labelled:
                a['working_targets']['exposure_to_command_age_max_s'] = .5
            r = evaluate_depth_admission_draft(a)
            self.assertEqual(r['policy_id'], 'control_depth_admission_draft_v1')
            self.assertEqual(r['checks']['driver_frame_age']['status'], 'FAIL')

    def test_missing_age_never_passes(self):
        a = self.audit(.4)
        a['frame_freshness'] = {}
        self.assertEqual(evaluate_depth_admission_draft(a)['checks']['driver_frame_age']['status'], 'UNKNOWN')


if __name__ == '__main__':
    unittest.main()
