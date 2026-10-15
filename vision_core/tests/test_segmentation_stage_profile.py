import sys
from pathlib import Path
import unittest
from unittest.mock import patch
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from profile_segmentation_stages import Calls,derive,instrument,active_session_owner


class StageProfileTests(unittest.TestCase):
    def test_wrappers_keep_result_exception_and_restore_methods(self):
        ops=SimpleNamespace(process_mask_native=lambda:'mask',scale_masks=lambda:'scaled')
        session=SimpleNamespace(run=lambda:'tensor')
        predictor=SimpleNamespace(model=SimpleNamespace(session=session),preprocess=lambda:'input',inference=lambda:session.run(),postprocess=lambda:ops.process_mask_native())
        original=ops.process_mask_native
        with patch.dict(sys.modules,{'ultralytics.utils':SimpleNamespace(ops=ops)}):
            with instrument(SimpleNamespace(model=SimpleNamespace(predictor=predictor))) as calls:
                self.assertEqual(predictor.inference(),'tensor')
                self.assertEqual(predictor.postprocess(),'mask')
                self.assertEqual(calls.counts['ort_run'],1)
            self.assertIs(ops.process_mask_native,original)
        c=Calls()
        def fail():raise ValueError('same')
        with self.assertRaisesRegex(ValueError,'same'):c.wrap('failure',fail)()
        self.assertEqual(c.counts['failure'],1)

    def test_delegated_session_is_used_instead_of_inactive_outer_shadow(self):
        calls=[]
        actual=SimpleNamespace(run=lambda:calls.append('active'))
        shadow=SimpleNamespace(run=lambda:calls.append('inactive'))
        leaf=SimpleNamespace(session=actual)
        outer=SimpleNamespace(session=shadow,backend=leaf)
        self.assertIs(active_session_owner(outer),leaf)
        ops=SimpleNamespace(process_mask_native=lambda:None,scale_masks=lambda:None)
        predictor=SimpleNamespace(model=outer,preprocess=lambda:None,
            inference=lambda:leaf.session.run(),postprocess=lambda:None)
        with patch.dict(sys.modules,{'ultralytics.utils':SimpleNamespace(ops=ops)}):
            with instrument(SimpleNamespace(model=SimpleNamespace(predictor=predictor))) as measured:
                predictor.inference()
                self.assertEqual(measured.counts['ort_run'],1)
        self.assertEqual(calls,['active'])
        outer.backend=outer
        with self.assertRaisesRegex(ValueError,'cycle'):active_session_owner(outer)

    def test_nested_stages_not_summed_and_missing_coverage_refuses(self):
        calls=Calls();calls.seconds.update(preprocess=.01,inference=.06,ort_run=.05,postprocess=.03,process_mask_native=.02,scale_masks=.015)
        calls.counts.update({k:1 for k in calls.seconds})
        result=derive(.12,calls,1)
        self.assertAlmostEqual(result['outside_predictor_stages_s'],.02)
        self.assertAlmostEqual(result['postprocess_outside_mask_native_s'],.01)
        self.assertAlmostEqual(result['mask_native_outside_scale_s'],.005)
        calls.counts['scale_masks']=0
        with self.assertRaisesRegex(ValueError,'not reached'):derive(.12,calls,1)


if __name__=='__main__':unittest.main()
