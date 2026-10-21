"""Bind a diagnostic camera cycle to its draft STOP preview, never an Action.

No threshold or metric admission is introduced here. A caller's claimed
permissions and old assessments are ignored; the existing draft is recomputed
from this cycle. Physical bounds remain unknown rather than inferred from MAD.
"""
from .control_depth_admission_draft import evaluate_depth_admission_draft
from .control_depth_quality_shadow import finite_number
from .existing_static_profile_preview import REVIEWED_SHA


def evaluate_control_input_shadow(audit):
    cid = audit.get('cycle_id')
    if not isinstance(cid, str) or not cid.strip():
        cid = None
    try:
        draft = evaluate_depth_admission_draft(audit)
        failed = list(draft['failed_checks'])
        missing = list(draft['missing_checks'])
        assessment_error = None
    except Exception as error:
        # A diagnostic failure must still produce a STOP preview for this cycle.
        draft = None
        failed, missing = [], ['depth_admission_assessment']
        assessment_error = type(error).__name__
    context = audit.get('existing_static_profile_preview') or {}
    if context.get('required') is True:
        gate = audit.get('existing_profile_static_gate') or {}
        if (context.get('profile_sha256') != REVIEWED_SHA
                or gate.get('cycle_id') != cid or not cid
                or gate.get('profile_sha256') != context.get('profile_sha256')):
            failed.append('existing_static_profile_binding')
        elif gate.get('status') != 'PASS':
            failed.append('existing_static_profile_applicability')
    if cid is None:
        failed.append('source_cycle_identity')
    shape = audit.get('shape_diagnostic')
    shape = shape if isinstance(shape, dict) else {}
    depth = shape.get('mask_depth')
    depth = depth if isinstance(depth, dict) else {}
    z = depth.get('candidate_depth_m')
    z = z if finite_number(z) and z > 0 else None
    reasons = ['NO_ADMITTED_METRIC_MEASUREMENT']
    reasons += ['DRAFT_CHECK_FAILED:' + key for key in failed]
    reasons += ['DRAFT_CHECK_UNKNOWN:' + key for key in missing]
    preview = {
        'status': 'STOP', 'preview_only': True,
        'motor_command_performed': False, 'recommended_speed_m_s': 0.,
        'source_cycle_id': cid, 'source_measurement_id': None,
        'policy_status': 'DRAFT_NOT_ACTIVATED',
        'depth_admission_status': draft['status'] if draft else 'ASSESSMENT_UNAVAILABLE',
        'failed_checks': failed, 'missing_checks': missing, 'reasons': reasons,
        'assessment_error': assessment_error,
    }
    return {
        'depth_admission_draft': draft,
        'control_input_shadow': {
            'status': 'BLOCKED_DIAGNOSTIC_ONLY', 'source_cycle_id': cid,
            'reference_frame': 'rectified_left_optical_frame', 'unit': 'm',
            'confidence': None, 'confidence_status': 'NOT_VALIDATED',
            'candidate_optical_z_m': z,
            'candidate_is_robot_front_clearance': False,
            'legacy_mask_status': depth.get('status'),
            'sensor_prerequisites': draft['sensor_prerequisites'] if draft else 'UNKNOWN',
            'metric_measurement_authorized': False, 'actuator_access': False,
            'canonical_decision_created': False,
        },
        'decision_preview': preview,
    }
