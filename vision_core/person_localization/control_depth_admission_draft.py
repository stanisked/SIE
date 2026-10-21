"""Explicit draft admission checks. Diagnostic replay only, never motion authority."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from .control_depth_quality_shadow import assess_control_depth,finite_number

POLICY=Path(__file__).resolve().parents[1]/'config/diagnostics/control_depth_admission_draft_v1.json'


def load_policy(audit=None):
    targets=(audit or {}).get('working_targets') or {}
    experimental=(targets.get('freshness_policy_id') == 'experimental_freshness_500ms_v1'
        and targets.get('exposure_to_command_age_max_s') == .5)
    path=POLICY.with_name('control_depth_admission_experimental_500ms_v1.json') if experimental else POLICY
    raw=path.read_bytes();policy=json.loads(raw)
    if policy['status']!='DRAFT_NOT_ACTIVATED' or policy['actuator_access'] is not False:
        raise ValueError('only non-actuating draft policy supported')
    return policy,hashlib.sha256(raw).hexdigest()


def evaluate_depth_admission_draft(audit):
    policy,sha=load_policy(audit);quality=assess_control_depth(audit)
    shape=audit.get('shape_diagnostic') or {};depth=shape.get('mask_depth') or {}
    fast=shape.get('fast_local_consistency') or {};residual=fast.get('pooled_held_out_axial_residual') or {}
    checks={}
    def check(name,condition,observed,requirement,source):
        checks[name]={'status':'UNKNOWN' if condition is None else 'PASS' if condition else 'FAIL',
            'observed':observed,'requirement':requirement,'source':source}
    valid_cycle=audit.get('status') in ('MASKED_DEPTH_SHADOW_DIAGNOSTIC_ONLY','SHAPE_AWARE_SHADOW_DIAGNOSTIC_ONLY')
    check('completed_current_cycle',valid_cycle,audit.get('status'),'current completed diagnostic cycle','cycle status')
    target=audit.get('ar0234_dynamic_target') or {};geometry=audit.get('target_geometry') or {}
    check('current_target_geometry',target.get('disposition')=='OBSERVED' and geometry.get('accepted') is True,
        {'selection':target.get('disposition'),'geometry':geometry.get('accepted')},'observed current target and accepted geometry','existing current-frame target gates')
    check('metric_diagnostic_frame',shape.get('reference_frame')=='rectified_left_optical_frame' and shape.get('unit')=='m',
        {'frame':shape.get('reference_frame'),'unit':shape.get('unit')},'rectified_left_optical_frame, m','existing diagnostic geometry')
    z=depth.get('candidate_depth_m');count=depth.get('candidate_sample_count')
    check('candidate_support',finite_number(z) and z>0 and type(count) is int and count>=policy['minimum_candidate_samples'],
        {'depth_m':z,'sample_count':count},policy['minimum_candidate_samples'],policy['sources']['minimum_candidate_samples'])
    layers=depth.get('plausible_layer_count')
    check('unique_plausible_layer',type(layers) is int and layers==policy['required_plausible_layers'],layers,1,policy['sources']['required_plausible_layers'])
    local=quality['local_consistency'];bound=local.get('fast_binding')=='EXACT_SOURCE_FRAME_AND_MASK'
    check('current_local_binding',bound,local.get('fast_binding'),'exact current cycle/mask/model/RGB/fusion binding','existing worker and quality-shadow provenance')
    tested=residual.get('sample_count')
    numeric=all(finite_number(residual.get(k)) and residual[k]>=0 for k in ('mad_m','absolute_p90_m','absolute_max_m'))
    check('local_residual_support',bound and numeric and type(tested) is int and tested>=policy['minimum_candidate_samples'],
        {'sample_count':tested,'residual':deepcopy(residual)},'at least 100 evaluated points and finite nonnegative residual statistics','existing 100-point support reused for diagnostic applicability; not an accuracy claim')
    freshness=quality['freshness']['driver_metadata']
    age=freshness.get('maximum_driver_declared_age_s')
    age_known=freshness['status'] not in ('UNAVAILABLE_OR_INVALID','AGE_TARGET_UNAVAILABLE')
    check('driver_frame_age',age<=policy['maximum_driver_frame_age_s'] if age_known else None,
        age,policy['maximum_driver_frame_age_s'],policy['sources']['maximum_driver_frame_age_s'])
    prerequisite_names=list(checks)
    # UNSET numerical profile limits are visible unknowns, never implicit passes.
    for name,value,key in (
        ('local_mad_limit',residual.get('mad_m'),'local_mad_max_m'),
        ('local_tail_limit',residual.get('absolute_p90_m'),'local_absolute_p90_max_m'),
        ('fitted_support_limit',fast.get('fitted_sample_fraction'),'minimum_fitted_sample_fraction'),
        ('rgb_coverage_limit',None,'minimum_rgb_coverage_fraction')):
        check(name,None,value,policy[key],policy['sources']['local_limits'])
    activated=audit.get('activated_depth_range_m')
    valid_range=(isinstance(activated,list) and len(activated)==2 and
        all(finite_number(v) for v in activated) and 0<activated[0]<activated[1])
    in_range=(activated[0]<=z<=activated[1]) if valid_range and finite_number(z) and z>0 else None
    check('activated_metric_range',in_range,{'candidate_depth_m':z,'activated_range_m':activated},
        'inside existing activated Measurement range','diagnostic range expansion is not activation')
    requirement=policy['physical_error_requirement']
    required_error=(requirement['up_to_2m_max_error_m'] if z<=2 else requirement['above_2m_max_relative_error']*z) if finite_number(z) and z>0 else None
    for name,explanation in (
        ('physical_depth_error_bound','Applicable validated physical-error model for this geometry/calibration/ownership/capture state'),
        ('exposure_to_command_age','Independently established timing and command delivery age, not driver-to-result flags'),
        ('robot_front_clearance','Surveyed camera-to-base transform, footprint and target-surface clearance; median z is insufficient'),
        ('approved_measurement_applicability','Applicable activated Measurement profile; diagnostic range is not approval'),
        ('dynamic_association_error_budget','R6: timing/association contribution <=0.02m requires exposure-pair uncertainty and applicable relative motion bound'),
        ('closing_motion_bound','Applicable current relative/target closing-motion bounds'),
        ('drive_stopping_bound','Measured stop bounds under relevant load, floor, supply and controller/watchdog state'),
        ('path_obstacle_observation','Observation of obstacles in the swept path independent of selected semantic target')):
        requirement_value = required_error if name=='physical_depth_error_bound' else (
            policy['maximum_driver_frame_age_s'] if name=='exposure_to_command_age' else None)
        check(name,None,None,requirement_value,explanation)
    failed=[k for k,v in checks.items() if v['status']=='FAIL'];unknown=[k for k,v in checks.items() if v['status']=='UNKNOWN']
    prereq_failed=[k for k in prerequisite_names if checks[k]['status']=='FAIL']
    prereq_unknown=[k for k in prerequisite_names if checks[k]['status']=='UNKNOWN']
    return {'status':'DRAFT_REFUSED' if failed else 'DRAFT_BLOCKED_MISSING_EVIDENCE',
        'reference_frame':'rectified_left_optical_frame','unit':'m','confidence':None,
        'confidence_status':'DRAFT_NOT_VALIDATED',
        'cycle_id':audit.get('cycle_id'),'policy_id':policy['policy_id'],'policy_sha256':sha,
        'policy_status':policy['status'],
        'sensor_prerequisites':'FAIL' if prereq_failed else 'UNKNOWN' if prereq_unknown else 'PRESENT_NOT_ADMITTED',
        'checks':checks,'failed_checks':failed,'missing_checks':unknown,
        'legacy_global_mad_status':depth.get('status'),'legacy_global_mad_m':depth.get('candidate_mad_m'),
        'required_physical_depth_error_bound_m':required_error,
        'error_requirement_frame':'rectified_left_optical_frame diagnostic comparison only; robot-front requirement remains unresolved',
        'decision_preview':'STOP','suitable_for_control':False,'metric_measurement_authorized':False,
        'navigation_eligible':False,'actuator_access':False,'motor_command_performed':False,
        'limitations':['PRESENT_NOT_ADMITTED does not pass unset residual/coverage or physical admission criteria.',
            'Physical missing evidence cannot be filled by caller booleans or cached local results.',
            'No speed, stop command, movement bridge, activation or legacy MAD policy is changed.']}
