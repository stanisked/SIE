import ast
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
from typing import Any
import unittest
from unittest.mock import patch

from vision_core.person_localization import existing_static_profile_preview as mod
from vision_core.person_localization.control_input_shadow import evaluate_control_input_shadow
from vision_core.tests.test_control_depth_admission_draft import source
from vision_core.tools.replay_existing_static_profile_v2 import replay


def profile():
    return {'status':'ACTIVE_CONDITIONAL_SUPERVISED_DECISION_RECOMMENDATION_ONLY',
        'authorization':{'execution_authorized':False,'actuator_bridge':'DISABLED_PHASE_1',
                         'metric_measurement_range_m':[.5,4.5]},
        'calibration_profile':{'source_sha256':mod.SOURCE_HASHES.copy(),
            'ar0234_to_ov9281_physical_left_extrinsic':{'sha256':mod.EXTRINSIC_SHA}},
        'target_geometry_policy':{'schema_version':'sie.ar0234.upper_body_static_gate.v1',
            'allow_bottom_truncation':True,'dynamic_fusion_permitted':False,'require_single_target':True,
            'require_untruncated_edges':['top','left','right'],'static_scene_only':True},
        'temporal_static_gate':{'schema_version':'sie.temporal.static_gate.v1','apply_before_stereo_fusion':True,
            'candidate_max_age_s':5.,'max_area_relative_change':.1,'max_center_delta_px':12.,
            'minimum_consecutive_observations':2}}


def candidate(dx=0, bottom=False):
    return {'bbox_xyxy_px':[100+dx,100,300+dx,600],
            'truncated_top':False,'truncated_left':False,'truncated_right':False,'truncated_bottom':bottom}


class StaticProfilePreviewTests(unittest.TestCase):
    def test_exact_profile_sha_and_execution_prohibition(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'profile.json';p.write_text(json.dumps(profile()))
            with self.assertRaises(ValueError): mod.load_existing_static_profile(p)
            digest=hashlib.sha256(p.read_bytes()).hexdigest()
            with patch.object(mod,'REVIEWED_SHA',digest):
                self.assertEqual(mod.load_existing_static_profile(p),profile())
            bad=profile();bad['authorization']['execution_authorized']=True
            p.write_text(json.dumps(bad));digest=hashlib.sha256(p.read_bytes()).hexdigest()
            with patch.object(mod,'REVIEWED_SHA',digest), self.assertRaises(ValueError):
                mod.load_existing_static_profile(p)

    def test_pending_then_pass_and_bottom_exception_only(self):
        g=mod.StaticProfilePreviewGate(profile())
        self.assertEqual(g.evaluate('a',candidate(bottom=True),1,0.)['status'],'PENDING')
        self.assertEqual(g.evaluate('b',candidate(bottom=True),1,.2)['status'],'PASS')
        for edge in ('top','left','right'):
            q=candidate();q['truncated_'+edge]=True
            self.assertEqual(g.evaluate(edge,q,1,1.)['status'],'FAIL')
        q=candidate();q.pop('truncated_top')
        self.assertEqual(g.evaluate('missing',q,1,2.)['status'],'FAIL')

    def test_motion_single_target_gap_and_clock_reset(self):
        g=mod.StaticProfilePreviewGate(profile());g.evaluate('a',candidate(),1,0.)
        self.assertEqual(g.evaluate('b',candidate(13),1,.2)['reason'],'MOTION_DETECTED')
        self.assertEqual(g.evaluate('c',candidate(13),1,.4)['status'],'PASS')
        self.assertEqual(g.evaluate('d',candidate(13),1,6.)['status'],'PENDING')
        self.assertEqual(g.evaluate('d',candidate(13),1,6.1)['status'],'FAIL')
        g.evaluate('e',candidate(),1,7.)
        self.assertEqual(g.evaluate('f',candidate(),2,7.2)['status'],'FAIL')
        self.assertEqual(g.evaluate('g',candidate(),1,7.4)['status'],'PENDING')
        self.assertEqual(g.evaluate('h',candidate(),1,7.3)['status'],'FAIL')

    def test_area_change_refuses_and_recovers_with_two_observations(self):
        g=mod.StaticProfilePreviewGate(profile());g.evaluate('a',candidate(),1,0.)
        q=candidate();q['bbox_xyxy_px']=[88,100,312,600]
        self.assertEqual(g.evaluate('b',q,1,.2)['reason'],'MOTION_DETECTED')
        self.assertEqual(g.evaluate('c',q,1,.4)['status'],'PASS')

    def test_static_gate_must_belong_to_the_current_cycle(self):
        a=source();a['existing_static_profile_preview']={'required':True,'profile_sha256':mod.REVIEWED_SHA}
        a['existing_profile_static_gate']={'cycle_id':'old','profile_sha256':mod.REVIEWED_SHA,'status':'PASS'}
        self.assertIn('existing_static_profile_binding',evaluate_control_input_shadow(a)['decision_preview']['failed_checks'])
        a['existing_profile_static_gate']['cycle_id']='one'
        a['existing_profile_static_gate']['status']='PENDING'
        self.assertIn('existing_static_profile_applicability',evaluate_control_input_shadow(a)['decision_preview']['failed_checks'])

    def test_temporal_result_matches_existing_static_producer(self):
        root=Path(__file__).resolve().parents[2]
        path=root/'ros2/sie_ros2/sie_ros2/ar0234_ov9281_metric_measurement_node.py'
        tree=ast.parse(path.read_text());nodes=[n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef)
            and n.name in ('_reset_static_gate','_temporal_static_reason')]
        clock=SimpleNamespace(value=0.);namespace={'time':SimpleNamespace(monotonic=lambda:clock.value),
            'math':math,'Any':Any}
        exec(compile(ast.Module(body=nodes,type_ignores=[]),str(path),'exec'),namespace)
        ref=SimpleNamespace(_static_candidate=None,_static_confirmation_observations=0,
            static_candidate_max_age_s=5.,static_max_center_delta_px=12.,
            static_max_area_relative_change=.1,static_confirmation_count=2)
        ref._reset_static_gate=lambda:namespace['_reset_static_gate'](ref)
        g=mod.StaticProfilePreviewGate(profile())
        for i,(t,dx) in enumerate([(0.,0),(.2,1),(.4,15),(.6,15),(6.,15),(6.2,15)]):
            q=candidate(dx);clock.value=t
            old=namespace['_temporal_static_reason'](ref,q)
            new=g.evaluate(str(i),q,1,t)
            self.assertEqual(new['status']=='PASS',old is None)
            if old:self.assertEqual(new['reason'],old.split(':')[0])

    def test_replay_is_counterfactual_preserves_sources_and_stops(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);j=p/'run';j.mkdir();prof=p/'profile.json';prof.write_text('not read in this test')
            hashes={}
            for i in range(2):
                a=source();a['cycle_id']=str(i);a['timestamp']=f'2026-10-21T10:00:0{i}+00:00'
                a['ar0234_dynamic_target'].update(candidate=candidate(),current_eligible_count=1)
                a['calibration']={'ar0234_intrinsic_sha256':mod.SOURCE_HASHES['ar0234_intrinsic'],
                    'ov9281_stereo_sha256':mod.SOURCE_HASHES['ov9281_stereo_calibration'],
                    'ar0234_to_physical_left_extrinsic_sha256':mod.EXTRINSIC_SHA}
                f=j/f'ar0234-ov9281-control-preview-{i}.json';f.write_text(json.dumps(a));hashes[f]=f.read_bytes()
            # Supply a pinned test profile while keeping source-integrity checks real.
            with patch('vision_core.tools.replay_existing_static_profile_v2.load_existing_static_profile',return_value=profile()), \
                 patch('vision_core.tools.replay_existing_static_profile_v2.REVIEWED_SHA',hashlib.sha256(prof.read_bytes()).hexdigest()), \
                 patch.object(mod,'REVIEWED_SHA',hashlib.sha256(prof.read_bytes()).hexdigest()), \
                 patch('vision_core.person_localization.control_input_shadow.REVIEWED_SHA',hashlib.sha256(prof.read_bytes()).hexdigest()):
                result=replay(j,prof)
            self.assertEqual(result['summary']['static_gate'],{'PENDING':1,'PASS':1})
            self.assertNotIn('activated_metric_range',result['summary']['scenario_failed_checks'])
            self.assertNotIn('existing_static_profile_binding',result['summary']['scenario_failed_checks'])
            self.assertEqual(result['summary']['scenario_failed_checks']['existing_static_profile_applicability'],1)
            self.assertEqual(result['summary']['preview'],{'STOP':2})
            self.assertFalse(result['static_gate_executed_before_historical_stereo'])
            self.assertTrue(all(f.read_bytes()==raw for f,raw in hashes.items()))


if __name__=='__main__':unittest.main()
