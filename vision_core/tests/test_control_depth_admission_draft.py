from copy import deepcopy
import unittest
from vision_core.person_localization.control_depth_admission_draft import evaluate_depth_admission_draft


def source():
    return {'cycle_id':'one','status':'MASKED_DEPTH_SHADOW_DIAGNOSTIC_ONLY',
        'activated_depth_range_m':[.5,2.],
        'ar0234_dynamic_target':{'disposition':'OBSERVED'},'target_geometry':{'accepted':True},
        'working_targets':{'exposure_to_command_age_max_s':.25},
        'processing_timing':{'host_capture_start_to_result_ready_s':.3},
        'frame_freshness':{'capture_backend':'latest_v4l2','endpoint':'RESULT_READY_BEFORE_PUBLICATION; not a motor command',
            'driver_declared_soe_to_result_s':{'ar0234':.24,'ov9281':.2}},
        'shape_diagnostic':{'reference_frame':'rectified_left_optical_frame','unit':'m',
            'model_sha256':'model','arrays_sha256':'fusion','mask_sha256':'mask',
            'mask':{'instance_index':0,'source_rgb_arrays_sha256':'rgb'},
            'mask_depth':{'status':'DEPTH_MAD_EXCEEDED','candidate_depth_m':2.7,'candidate_sample_count':1000,
                'selected_sample_count':1100,'plausible_layer_count':1,'candidate_mad_m':.08},
            'fast_local_consistency':{'status':'LOCAL_CROSSFIT_DIAGNOSTIC_ONLY',
                'provenance':{'source_cycle_id':'one','mask_sha256':'mask','instance_index':0,
                    'model_sha256':'model','rgb_arrays_sha256':'rgb','fusion_arrays_sha256':'fusion'},
                'pooled_held_out_axial_residual':{'sample_count':1000,'mad_m':.01,'absolute_p90_m':.04,'absolute_max_m':.1},
                'fitted_sample_fraction':.98}}}


class DraftAdmissionTests(unittest.TestCase):
    def test_good_diagnostics_do_not_invent_limits_or_activate_motion(self):
        a=source();before=deepcopy(a);r=evaluate_depth_admission_draft(a)
        self.assertEqual(a,before)
        self.assertEqual(r['sensor_prerequisites'],'PRESENT_NOT_ADMITTED')
        self.assertEqual(r['failed_checks'],['activated_metric_range'])
        self.assertIn('local_tail_limit',r['missing_checks'])
        self.assertIn('physical_depth_error_bound',r['missing_checks'])
        self.assertAlmostEqual(r['required_physical_depth_error_bound_m'],.135)
        self.assertEqual(r['checks']['driver_frame_age']['status'],'PASS')
        self.assertFalse(r['suitable_for_control']);self.assertEqual(r['decision_preview'],'STOP')
        a['shape_diagnostic']['mask_depth']['candidate_depth_m']=1.5
        r=evaluate_depth_admission_draft(a)
        self.assertEqual(r['failed_checks'],[])
        self.assertEqual(r['status'],'DRAFT_BLOCKED_MISSING_EVIDENCE')
        self.assertEqual(r['required_physical_depth_error_bound_m'],.1)
        self.assertFalse(r['actuator_access'])

    def test_support_layer_age_mask_target_and_units_fail_independently(self):
        for name,edit in (
            ('candidate_support',lambda a:a['shape_diagnostic']['mask_depth'].update(candidate_sample_count=99)),
            ('candidate_support',lambda a:a['shape_diagnostic']['mask_depth'].update(candidate_sample_count=True)),
            ('unique_plausible_layer',lambda a:a['shape_diagnostic']['mask_depth'].update(plausible_layer_count=2)),
            ('driver_frame_age',lambda a:a['frame_freshness']['driver_declared_soe_to_result_s'].update(ov9281=.251)),
            ('current_local_binding',lambda a:a['shape_diagnostic']['fast_local_consistency']['provenance'].update(source_cycle_id='old')),
            ('current_target_geometry',lambda a:a['ar0234_dynamic_target'].update(disposition='UNAVAILABLE')),
            ('metric_diagnostic_frame',lambda a:a['shape_diagnostic'].update(unit='mm')),
            ('local_residual_support',lambda a:a['shape_diagnostic']['fast_local_consistency']['pooled_held_out_axial_residual'].update(mad_m=-1.))):
            a=source();edit(a);r=evaluate_depth_admission_draft(a)
            self.assertIn(name,r['failed_checks'],name)
            self.assertEqual(r['sensor_prerequisites'],'FAIL')
            self.assertFalse(r['metric_measurement_authorized'])

    def test_missing_evidence_and_fabricated_booleans_never_admit(self):
        a=source();a['frame_freshness']={};a['physical_error_validated']=True
        a['motion_authorized']=True;a['shape_diagnostic']['fast_local_consistency']={}
        r=evaluate_depth_admission_draft(a)
        self.assertIn('driver_frame_age',r['missing_checks'])
        self.assertIn('physical_depth_error_bound',r['missing_checks'])
        self.assertFalse(r['actuator_access'])
        self.assertEqual(evaluate_depth_admission_draft({})['decision_preview'],'STOP')


if __name__=='__main__':unittest.main()
