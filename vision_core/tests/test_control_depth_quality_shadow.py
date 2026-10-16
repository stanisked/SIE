from copy import deepcopy
import unittest
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from vision_core.person_localization.control_depth_quality_shadow import assess_control_depth


def source():
    return {'cycle_id':'one','status':'MASKED_DEPTH_SHADOW_DIAGNOSTIC_ONLY',
        'working_targets':{'exposure_to_command_age_max_s':.25},
        'processing_timing':{'host_capture_start_to_result_ready_s':.1,'endpoint':'PRIMARY'},
        'shape_diagnostic':{'mask_sha256':'mask','mask_depth':{
            'status':'DEPTH_MAD_EXCEEDED','candidate_depth_m':2.8,'candidate_mad_m':.08,
            'selected_sample_count':1000,'candidate_sample_count':1000,'plausible_layer_count':1},
            'local_surface':{'status':'SKIPPED_BY_DIAGNOSTIC_SAMPLING'}}}


def local(a):
    return {'cycle_id':a['cycle_id'],'status':'LOCAL_SURFACE_DIAGNOSTIC_ONLY',
        'provenance':{'source_cycle_id':a['cycle_id'],'mask_sha256':'mask',
                      'mask_depth':deepcopy(a['shape_diagnostic']['mask_depth'])},
        'local_surface':{'status':'LOCAL_SURFACE_DIAGNOSTIC_ONLY',
            'pooled_held_out_axial_residual':{'mad_m':.02,'absolute_p90_m':.07},
            'rgb_coverage':{'mask_area_in_fitted_bins_fraction':.99},'unfitted_patch_fraction':.08}}


class QualityShadowTests(unittest.TestCase):
    def test_global_mad_is_preserved_not_reused_as_independent_refusal(self):
        a=source();before=deepcopy(a);r=assess_control_depth(a,local(a))
        self.assertEqual(a,before)
        self.assertEqual(r['status'],'SHADOW_UNDETERMINED')
        self.assertEqual(r['legacy_mask_status'],'DEPTH_MAD_EXCEEDED')
        self.assertEqual(r['local_consistency']['status'],'OBSERVED_NOT_ADMITTED')
        self.assertFalse(r['suitable_for_control'])
        self.assertIsNone(r['local_consistency']['physical_error_bound_m'])

    def test_later_frame_or_other_mask_never_inherits_local_evidence(self):
        a=source();result=local(a)
        for field in ('cycle','mask','depth'):
            b=deepcopy(result)
            if field=='cycle':b['cycle_id']='other'
            if field=='mask':b['provenance']['mask_sha256']='other'
            if field=='depth':b['provenance']['mask_depth']['candidate_depth_m']=3.
            r=assess_control_depth(a,b)
            self.assertIn('LOCAL_RESULT_BINDING_MISMATCH',r['refusal_reasons'])
            self.assertEqual(r['local_consistency']['status'],'UNAVAILABLE')
        a['cycle_id']='two'
        self.assertEqual(assess_control_depth(a)['local_consistency']['status'],'UNAVAILABLE')

    def test_freshness_support_layers_and_cancel_refuse_separately(self):
        a=source();a['processing_timing']['host_capture_start_to_result_ready_s']=.356
        a['shape_diagnostic']['mask_depth']={'selected_sample_count':20,'plausible_layer_count':2}
        r=assess_control_depth(a)
        self.assertEqual(set(r['refusal_reasons']),{'NO_SUPPORTED_CURRENT_DEPTH_CANDIDATE','DEPTH_LAYER_AMBIGUITY'})
        self.assertIsNone(r['freshness']['exposure_to_command_age_s'])
        a['status']='CANCELLED_SHUTDOWN'
        self.assertIn('CURRENT_CYCLE_NOT_USABLE',assess_control_depth(a)['refusal_reasons'])


    def test_driver_age_separate_from_cycle_duration_and_no_command_admission(self):
        a=source();a['processing_timing']['host_capture_start_to_result_ready_s']=.3
        a['frame_freshness']={'capture_backend':'latest_v4l2',
            'endpoint':'RESULT_READY_BEFORE_PUBLICATION; not a motor command',
            'driver_declared_soe_to_result_s':{'ar0234':.245,'ov9281':.24},
            'within_250ms_by_driver_metadata':False,'maximum_driver_declared_age_s':99.}
        before=deepcopy(a);r=assess_control_depth(a)
        self.assertEqual(a,before)
        self.assertEqual(r['status'],'SHADOW_UNDETERMINED')
        f=r['freshness'];self.assertEqual(f['driver_metadata']['maximum_driver_declared_age_s'],.245)
        self.assertEqual(f['driver_metadata']['status'],'WITHIN_DECLARED_TARGET')
        self.assertTrue(f['performance']['host_duration_exceeds_numeric_age_target'])
        self.assertIn('EXPOSURE_TO_COMMAND_AGE_NOT_ESTABLISHED',r['missing_evidence'])
        self.assertFalse(r['suitable_for_control'])
        self.assertIsNone(f['exposure_to_command_age_s'])
        # A fast cycle cannot hide an old image. Recompute from both ages,
        # not from a cached success flag or a cached maximum.
        a['processing_timing']['host_capture_start_to_result_ready_s']=.1
        a['frame_freshness']['driver_declared_soe_to_result_s']['ov9281']=.251
        a['frame_freshness']['within_250ms_by_driver_metadata']=True
        r=assess_control_depth(a)
        self.assertEqual(r['refusal_reasons'],['DRIVER_DECLARED_FRAME_AGE_EXCEEDS_TARGET'])
        # Missing, partial, negative, NaN and boolean values never establish age.
        for ages in ({},{'ar0234':.1},{'ar0234':.1,'ov9281':-.1},
                     {'ar0234':float('nan'),'ov9281':.1},{'ar0234':True,'ov9281':.1}):
            a['frame_freshness']['driver_declared_soe_to_result_s']=ages
            r=assess_control_depth(a)
            self.assertEqual(r['freshness']['driver_metadata']['status'],'UNAVAILABLE_OR_INVALID')
            self.assertIn('DRIVER_FRAME_AGE_UNAVAILABLE_OR_INVALID',r['missing_evidence'])
            self.assertFalse(r['metric_measurement_authorized'])
        a['frame_freshness']['driver_declared_soe_to_result_s']={'ar0234':.25,'ov9281':.25}
        self.assertEqual(assess_control_depth(a)['freshness']['driver_metadata']['status'],'WITHIN_DECLARED_TARGET')
        a['frame_freshness']['endpoint']='COMMAND'
        self.assertEqual(assess_control_depth(a)['freshness']['driver_metadata']['status'],'UNAVAILABLE_OR_INVALID')

    def test_offline_replay_preserves_recorded_assessment_and_inputs(self):
        root=Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'ar0234-ov9281-control-preview-one.json'
            a=source();a['processing_timing']['host_capture_start_to_result_ready_s']=.3
            a['control_depth_quality_shadow']={'status':'SHADOW_REFUSED',
                'refusal_reasons':['HOST_PROCESSING_EXCEEDS_DECLARED_AGE_TARGET']}
            a['frame_freshness']={'capture_backend':'latest_v4l2',
                'endpoint':'RESULT_READY_BEFORE_PUBLICATION; not a motor command',
                'driver_declared_soe_to_result_s':{'ar0234':.24,'ov9281':.23}}
            path.write_text(json.dumps(a));before=path.read_bytes()
            output=Path(directory)/'replay.json'
            result=subprocess.run([sys.executable,str(root/'vision_core/tools/analyze_control_depth_quality_shadow.py'),
                '--journal-root',directory,'--output',str(output)],capture_output=True,text=True,check=True)
            report=json.loads(output.read_text())
            self.assertEqual(path.read_bytes(),before)
            self.assertTrue(report['inputs_unchanged'])
            self.assertEqual(report['summary']['recorded_to_replayed'],{'SHADOW_REFUSED -> SHADOW_UNDETERMINED':1})
            self.assertEqual(report['rows'][0]['recorded_primary'],a['control_depth_quality_shadow'])
            self.assertEqual(report['summary']['driver_frame_age'],{'WITHIN_DECLARED_TARGET':1})
            self.assertIn('INPUTS_UNCHANGED: True',result.stdout)
            self.assertFalse(report['actuator_access'])

    def test_missing_and_nonfinite_evidence_never_passes(self):
        for a in ({},source()):
            if a:
                a['shape_diagnostic']['mask_depth']['candidate_depth_m']=float('nan')
            r=assess_control_depth(a)
            self.assertEqual(r['status'],'SHADOW_REFUSED')
            self.assertFalse(r['actuator_access'])
            self.assertFalse(r['local_result_reusable_for_other_frames'])


if __name__=='__main__':unittest.main()
