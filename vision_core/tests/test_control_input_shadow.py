"""Current-cycle binding regressions; no camera, ROS or HTTP access."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from vision_core.person_localization.control_input_shadow import evaluate_control_input_shadow
from vision_core.tests.test_control_depth_admission_draft import source


class ControlInputShadowTests(unittest.TestCase):
    def test_binds_current_candidate_without_admitting_or_changing_input(self):
        a = source(); before = deepcopy(a)
        result = evaluate_control_input_shadow(a)
        self.assertEqual(a, before)
        self.assertEqual(result['control_input_shadow']['candidate_optical_z_m'], 2.7)
        self.assertEqual(result['decision_preview']['source_cycle_id'], 'one')
        self.assertIn('activated_metric_range', result['decision_preview']['failed_checks'])
        self.assertIn('physical_depth_error_bound', result['decision_preview']['missing_checks'])
        self.assertEqual(result['decision_preview']['status'], 'STOP')
        self.assertIsNone(result['decision_preview']['source_measurement_id'])
        self.assertFalse(result['control_input_shadow']['actuator_access'])

    def test_stale_binding_and_age_have_separate_concrete_reasons(self):
        a = source()
        a['shape_diagnostic']['fast_local_consistency']['provenance']['source_cycle_id'] = 'old'
        a['frame_freshness']['driver_declared_soe_to_result_s']['ov9281'] = .3
        r = evaluate_control_input_shadow(a)['decision_preview']
        self.assertIn('DRAFT_CHECK_FAILED:current_local_binding', r['reasons'])
        self.assertIn('DRAFT_CHECK_FAILED:driver_frame_age', r['reasons'])

    def test_cancelled_missing_target_and_identity_cannot_reuse_previous_permission(self):
        for edit, expected in (
            (lambda a:a.update(status='CANCELLED_SHUTDOWN'), 'completed_current_cycle'),
            (lambda a:a['ar0234_dynamic_target'].update(disposition='UNAVAILABLE'), 'current_target_geometry'),
            (lambda a:a.update(cycle_id=None), 'source_cycle_identity')):
            a = source(); edit(a)
            a['decision_preview'] = {'status':'APPROACH', 'recommended_speed_m_s':.15}
            a['depth_admission_draft'] = {'status':'ADMITTED'}
            r = evaluate_control_input_shadow(a)['decision_preview']
            self.assertEqual(r['status'], 'STOP')
            self.assertEqual(r['recommended_speed_m_s'], 0)
            self.assertIn(expected, r['failed_checks'])

    def test_assessment_exception_still_returns_a_stop(self):
        with patch('vision_core.person_localization.control_input_shadow.evaluate_depth_admission_draft',
                   side_effect=ValueError('bad policy')):
            r = evaluate_control_input_shadow(source())
        self.assertEqual(r['decision_preview']['status'], 'STOP')
        self.assertEqual(r['decision_preview']['assessment_error'], 'ValueError')
        self.assertIsNone(r['depth_admission_draft'])

    def test_malformed_shape_produces_stop_instead_of_crashing(self):
        a = source(); a['shape_diagnostic'] = 'invalid'
        r = evaluate_control_input_shadow(a)
        self.assertEqual(r['decision_preview']['status'], 'STOP')
        self.assertIsNone(r['control_input_shadow']['candidate_optical_z_m'])

    def test_replay_writes_same_cycle_bound_preview_and_preserves_source_bytes(self):
        root = Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory() as temp:
            p = Path(temp); journal = p/'run'; journal.mkdir()
            f = journal/'ar0234-ov9281-control-preview-one.json'
            f.write_text(json.dumps(source())); digest = hashlib.sha256(f.read_bytes()).hexdigest()
            out = p/'replay.json'
            run = subprocess.run([sys.executable, str(root/'vision_core/tools/analyze_control_depth_quality_shadow.py'),
                                  '--journal-root', str(journal), '--output', str(out)],
                                 capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stderr)
            r = json.loads(out.read_text()); row = r['rows'][0]
            self.assertEqual(row['decision_preview']['source_cycle_id'], row['cycle_id'])
            self.assertEqual(row['decision_preview']['status'], 'STOP')
            self.assertTrue(r['inputs_unchanged'])
            self.assertEqual(hashlib.sha256(f.read_bytes()).hexdigest(), digest)


if __name__ == '__main__':
    unittest.main()
