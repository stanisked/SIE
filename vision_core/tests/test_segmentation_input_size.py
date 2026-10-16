import sys
from pathlib import Path
import unittest
import json
import tempfile
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from benchmark_segmentation_input_size import inner_roi,depth_for,delta,summarize,challenge_reference,sha256


class InputSizeTests(unittest.TestCase):
    def test_new_challenge_is_explicit_and_bound_to_original_checkpoint(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);checkpoint=root/'source.pt';checkpoint.write_bytes(b'frozen checkpoint')
            parent=root/'original.json'
            parent.write_text(json.dumps({'source_model':str(checkpoint),'source_model_sha256':sha256(checkpoint)}))
            review=root/'review.json'
            review.write_text(json.dumps({'status':'OFFLINE_INPUT_SIZE_CHALLENGE_ONLY',
                'reference':{'comparison_file':str(parent),'comparison_sha256':sha256(parent)}}))
            path,status=challenge_reference(review,[256,384],[(192,320)],True)
            self.assertEqual(path,checkpoint)
            self.assertEqual(status,'OFFLINE_192X320_VS_REVIEWED_256X384_ONLY')
            for shape,sizes in (([416,640],[(192,320)]),([256,384],[(160,256)])):
                with self.assertRaises(ValueError):challenge_reference(review,shape,sizes,True)
            with self.assertRaises(ValueError):challenge_reference(review,[256,384],[(192,320)],False)
            checkpoint.write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError,'checkpoint changed'):
                challenge_reference(review,[256,384],[(192,320)],True)
            parent.write_text('{}')
            with self.assertRaisesRegex(ValueError,'parent comparison changed'):
                challenge_reference(review,[256,384],[(192,320)],True)

    def test_runtime_roi_and_missing_depth(self):
        self.assertEqual(inner_roi({'bbox_xyxy_px':[100,100,500,1100]}),[180.,400.,420.,950.])
        self.assertIsNone(delta({}, {'candidate_depth_m':2.},'candidate_depth_m'))
        self.assertAlmostEqual(delta({'candidate_depth_m':2.},{'candidate_depth_m':2.1},'candidate_depth_m'),.1)

    def test_no_target_never_fuses_and_valid_target_uses_its_own_mask(self):
        stamp='2026-10-15T08:00:00+00:00'
        with patch('preview_mask_surface.evaluate') as evaluate:
            self.assertEqual(depth_for([],[],stamp,None,{})[2]['status'],'NO_CURRENT_ELIGIBLE_TARGET')
            evaluate.assert_not_called()
            instance={'instance_index':0,'bbox_xyxy_px':[500,200,1000,1100],'confidence':.9}
            mask=np.ones((1200,1920),bool)
            evaluate.return_value=({'mask_depth':{'status':'DEPTH_MAD_EXCEEDED','candidate_depth_m':2.}},None)
            result=depth_for([instance],[mask],stamp,'same-disparity',{})
            self.assertEqual(result[2]['status'],'DEPTH_MAD_EXCEEDED')
            self.assertIs(evaluate.call_args.args[1],mask)
            self.assertEqual(evaluate.call_args.args[0],'same-disparity')
            self.assertFalse(evaluate.call_args.kwargs['prepare_local'])

    def test_summary_retains_refusal_and_correspondence_failure(self):
        row={'reference_s':.4,'candidate_s':.2,'matching':{'pairs':[], 'unmatched_reference_count':1,'unmatched_candidate_count':0},
             'selection_comparison':{'same_disposition':False,'both_unavailable':False,'selected_pair':None},
             'reference_geometry':{'accepted':True},'candidate_geometry':None,
             'reference_depth':{'status':'COHERENT'},'candidate_depth':{'status':'NO_CURRENT_ELIGIBLE_TARGET'},'depth_delta_m':None}
        summary=summarize([row])
        self.assertEqual(summary['selected_correspondence_failures'],1)
        self.assertEqual(summary['unmatched_instances'],1)
        self.assertEqual(summary['depth_status_transitions'],{'COHERENT -> NO_CURRENT_ELIGIBLE_TARGET':1})
        self.assertIsNone(summary['absolute_depth_delta_max_m'])


if __name__=='__main__':unittest.main()
