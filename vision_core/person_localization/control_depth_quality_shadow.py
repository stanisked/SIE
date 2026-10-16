"""Diagnostic evidence assessment only. Never admits Measurement or motion.

No new numeric quality thresholds. Existing support/layer dispositions are
reported separately from global MAD; local residuals lack a validated physical
error/admission policy. Exposure-to-command freshness is not inferred from host
processing time. Deferred diagnostics apply exclusively to their source cycle.
"""
from copy import deepcopy
import math


def finite_number(value):
    return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)


def assess_control_depth(audit,local_result=None):
    cid=audit.get('cycle_id')
    shape=audit.get('shape_diagnostic') or {}
    depth=shape.get('mask_depth') or {}
    timing=audit.get('processing_timing') or {}
    host=timing.get('host_capture_start_to_result_ready_s')
    target=(audit.get('working_targets') or {}).get('exposure_to_command_age_max_s')
    reasons=[];unknown=[]
    candidate=depth.get('candidate_depth_m')
    has_candidate=finite_number(candidate) and candidate>0
    support=depth.get('candidate_sample_count')
    supported=has_candidate and isinstance(support,int) and not isinstance(support,bool) and support>0
    if not supported:reasons.append('NO_SUPPORTED_CURRENT_DEPTH_CANDIDATE')
    unique=depth.get('plausible_layer_count')==1
    if depth.get('plausible_layer_count') is None:
        unknown.append('LAYER_ASSESSMENT_UNAVAILABLE')
    elif not unique:reasons.append('DEPTH_LAYER_AMBIGUITY')
    local=shape.get('local_surface') or {}
    binding='PRIMARY_CURRENT_FRAME'
    if local_result is not None:
        provenance=local_result.get('provenance') or {}
        bound=(bool(cid) and local_result.get('cycle_id')==cid
               and provenance.get('source_cycle_id')==cid
               and provenance.get('mask_depth')==depth
               and provenance.get('mask_sha256')==shape.get('mask_sha256')
               and bool(shape.get('mask_sha256')))
        binding='EXACT_SOURCE_FRAME' if bound else 'REJECTED_FRAME_BINDING'
        if not bound:
            reasons.append('LOCAL_RESULT_BINDING_MISMATCH');local={}
        elif local_result.get('status')=='LOCAL_SURFACE_DIAGNOSTIC_ONLY':
            local=local_result.get('local_surface') or {}
        else:local={'status':'LOCAL_SURFACE_UNAVAILABLE'}
    residual=local.get('pooled_held_out_axial_residual') or {}
    available=all(finite_number(residual.get(k)) for k in ('mad_m','absolute_p90_m'))
    unknown.append('LOCAL_ERROR_TO_PHYSICAL_ACCURACY_POLICY_NOT_VALIDATED' if available else 'CURRENT_FRAME_LOCAL_CONSISTENCY_UNAVAILABLE')
    host_exceeded=(finite_number(host) and finite_number(target) and target>0 and host>target)
    # A cycle can wait for a new exposure. Its duration is not the age of it.
    frame=audit.get('frame_freshness') or {}
    ages=frame.get('driver_declared_soe_to_result_s') or {}
    valid_target=finite_number(target) and target>0
    valid_driver=(frame.get('capture_backend')=='latest_v4l2'
        and frame.get('endpoint')=='RESULT_READY_BEFORE_PUBLICATION; not a motor command'
        and isinstance(ages,dict) and set(ages)=={'ar0234','ov9281'}
        and all(finite_number(v) and v>=0 for v in ages.values()))
    driver_status='UNAVAILABLE_OR_INVALID'
    driver_max=max(ages.values()) if valid_driver else None
    if valid_driver and valid_target:
        driver_status='WITHIN_DECLARED_TARGET' if driver_max<=target else 'DECLARED_TARGET_EXCEEDED'
        if driver_max>target:reasons.append('DRIVER_DECLARED_FRAME_AGE_EXCEEDS_TARGET')
    elif valid_driver:
        driver_status='AGE_TARGET_UNAVAILABLE'
        unknown.append('FRAME_AGE_TARGET_UNAVAILABLE')
    else:unknown.append('DRIVER_FRAME_AGE_UNAVAILABLE_OR_INVALID')
    unknown.extend(['EXPOSURE_TO_COMMAND_AGE_NOT_ESTABLISHED','PHYSICAL_DEPTH_ERROR_BOUND_NOT_ESTABLISHED'])
    if audit.get('status') in ('CANCELLED_SHUTDOWN','PROCESSING_REFUSED','TARGET_UNAVAILABLE'):
        reasons.append('CURRENT_CYCLE_NOT_USABLE')
    return {'schema_version':'sie.control_depth_quality_shadow.v1','assessment_revision':2,'cycle_id':cid,
        'status':'SHADOW_REFUSED' if reasons else 'SHADOW_UNDETERMINED',
        'suitable_for_control':False,'metric_measurement_authorized':False,
        'navigation_eligible':False,'actuator_access':False,
        'reference_frame':'rectified_left_optical_frame','unit':'m',
        'confidence':None,'confidence_status':'NOT_VALIDATED',
        'refusal_reasons':reasons,'missing_evidence':unknown,
        'legacy_mask_status':depth.get('status'),'legacy_global_mad_m':depth.get('candidate_mad_m'),
        'support':{'status':'EXISTING_CANDIDATE_SUPPORT_PRESENT' if supported else 'UNAVAILABLE',
                   'selected_sample_count':depth.get('selected_sample_count'),'candidate_sample_count':support},
        'layers':{'status':'SINGLE_EXISTING_PLAUSIBLE_LAYER' if unique else 'AMBIGUOUS_OR_UNAVAILABLE',
                  'plausible_layer_count':depth.get('plausible_layer_count'),'depth_layers':deepcopy(depth.get('depth_layers',[]))},
        'local_consistency':{'status':'OBSERVED_NOT_ADMITTED' if available else 'UNAVAILABLE',
            'source_status':local.get('status'),'binding':binding,'source_cycle_id':cid,
            'held_out_residual':deepcopy(residual), 'rgb_coverage':deepcopy(local.get('rgb_coverage')),
            'unfitted_patch_fraction':local.get('unfitted_patch_fraction'),
            'physical_error_bound_m':None},
        'freshness':{'status':'EXPOSURE_TO_COMMAND_AGE_NOT_ESTABLISHED',
            'driver_metadata':{'status':driver_status,
                'driver_declared_soe_to_result_s':deepcopy(ages) if valid_driver else {},
                'maximum_driver_declared_age_s':driver_max,'endpoint':frame.get('endpoint'),
                'physical_exposure_timing_independently_verified':False,
                'command_age_established':False},
            'performance':{'host_duration_exceeds_numeric_age_target':bool(host_exceeded),
                'duration_is_frame_age':False},
            'source_primary_host_processing_s':host,'declared_age_target_s':target,
            'exposure_to_command_age_s':None,'host_endpoint':timing.get('endpoint'),
            'local_completion_is_new_camera_observation':False},
        'local_result_reusable_for_other_frames':False,'quality_thresholds_added':False}
