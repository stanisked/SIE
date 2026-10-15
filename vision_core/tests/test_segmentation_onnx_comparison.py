"""Comparison diagnostics: exact pairing, differences, refusals and source hashes."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'vision_core/tools'))
from benchmark_segmentation_onnx import match_instances, selection, selection_comparison, load_cases, sha256, FixedInputModel, geometry_checker


def instance(index,box,confidence=.9):
    return {'instance_index':index,'bbox_xyxy_px':box,'confidence':confidence}


class OnnxComparisonTests(unittest.TestCase):
    def test_order_swap_uses_one_to_one_geometry_and_selected_correspondence(self):
        a=[instance(0,[20,20,60,100]),instance(1,[100,20,140,100],.6)]
        b=[instance(0,a[1]['bbox_xyxy_px'],.6),instance(1,a[0]['bbox_xyxy_px'])]
        x=np.zeros((160,160),bool);x[20:100,20:60]=1
        y=np.zeros_like(x);y[20:100,100:140]=1
        result=match_instances(a,[x,y],b,[y,x])
        self.assertEqual([(p['reference_index'],p['candidate_index']) for p in result['pairs']],[(0,1),(1,0)])
        self.assertTrue(all(p['mask_iou']==1 and p['mask_different_pixels']==0 for p in result['pairs']))
        stamp='2026-10-15T08:00:00+00:00'
        choices=selection_comparison(selection(a,stamp),selection(b,stamp),result)
        self.assertTrue(choices['selected_correspondence_unambiguous'])

    def test_missing_person_mask_difference_and_equal_score_tie_are_visible(self):
        a=[instance(0,[20,20,60,100]),instance(1,[100,20,140,100])]
        mask=np.ones((4,4),bool);different=mask.copy();different[0,0]=False
        result=match_instances(a,[mask,mask],a[:1],[different])
        self.assertEqual(result['unmatched_reference_count'],1)
        self.assertEqual(result['pairs'][0]['mask_different_pixels'],1)
        self.assertEqual(result['pairs'][0]['mask_iou'],15/16)
        tied=match_instances([a[0],a[0]],[mask,mask],[a[0],a[0]],[mask,mask])
        self.assertTrue(all(p['ambiguous_equal_score'] for p in tied['pairs']))
        self.assertEqual(match_instances([],[],[],[])['pairs'],[])
        self.assertEqual(match_instances(a,[mask,mask],[],[])['unmatched_reference_count'],2)

    def test_selection_disagreement_is_not_promoted_to_agreement(self):
        stamp='2026-10-15T08:00:00+00:00';a=[instance(0,[20,20,60,100])]
        result=selection_comparison(selection(a,stamp),selection([],stamp),match_instances(a,[np.ones((2,2),bool)],[],[]))
        self.assertFalse(result['same_disposition'])
        self.assertIsNone(result['selected_pair'])

    def test_actual_runtime_geometry_is_preserved_for_comparison(self):
        check=geometry_checker()
        self.assertTrue(check(instance(0,[500,200,900,1000]))['accepted'])
        self.assertFalse(check(instance(0,[500,200,600,1000]))['accepted'])
        self.assertFalse(check(instance(0,[0,200,400,1000]))['accepted'])
        self.assertIsNone(check(None))

    def test_case_binding_and_fixed_input_wrapper(self):
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory)/'cycle';folder.mkdir()
            (folder/'audit.json').write_text(json.dumps({'cycle_id':'cycle','timestamp':'2026-10-15T08:00:00+00:00'}))
            source=folder/'ar0234_source.jpg';source.write_bytes(b'fixture')
            cases=load_cases(Path(directory))
            self.assertEqual(cases[0]['source_sha256'][str(source)],sha256(source))
            source.unlink()
            with self.assertRaises(ValueError):load_cases(Path(directory))
        class Model:
            def predict(self,**kwargs):return kwargs
        actual=FixedInputModel(Model(),[416,640]).predict(imgsz=640,rect=True,source='array')
        self.assertEqual(actual['imgsz'],[416,640]);self.assertFalse(actual['rect']);self.assertFalse(actual['dnn'])


if __name__=='__main__':unittest.main()
