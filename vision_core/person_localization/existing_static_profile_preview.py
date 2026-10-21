"""Diagnostic use of the reviewed existing static v2 profile, never activation."""
import hashlib
import json
import math
from pathlib import Path
from .control_depth_quality_shadow import finite_number

REVIEWED_SHA = 'c835a304cc2758843747400c30f09a567b5ac74ebfd7908ea8ee14a013f519d9'
SOURCE_HASHES = {
    'ar0234_intrinsic': 'e9454ed3d93ab36a6b159aa8c8e2356353cca8a6b0480e73c371199937ae7096',
    'ov9281_stereo_calibration': '39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52',
}
EXTRINSIC_SHA = '10b5fdd9b3467ba5e04a8bd6d2daad8440382798374c39bfd4b9037c2bf5b9cc'


def load_existing_static_profile(path):
    raw = Path(path).read_bytes()
    if hashlib.sha256(raw).hexdigest() != REVIEWED_SHA:
        raise ValueError('existing static v2 profile SHA mismatch')
    p = json.loads(raw)
    if (p.get('status') != 'ACTIVE_CONDITIONAL_SUPERVISED_DECISION_RECOMMENDATION_ONLY'
            or p['authorization'].get('execution_authorized') is not False
            or p['authorization'].get('actuator_bridge') != 'DISABLED_PHASE_1'
            or p['authorization'].get('metric_measurement_range_m') != [.5, 4.5]
            or p['calibration_profile']['source_sha256'] != SOURCE_HASHES
            or p['calibration_profile']['ar0234_to_ov9281_physical_left_extrinsic']['sha256'] != EXTRINSIC_SHA):
        raise ValueError('existing static v2 applicability mismatch')
    StaticProfilePreviewGate(p)  # Validate the exact policy interpreted below.
    return p


class StaticProfilePreviewGate:
    def __init__(self, profile):
        self.geometry = profile['target_geometry_policy']
        self.temporal = profile['temporal_static_gate']
        if self.geometry != {
            'schema_version': 'sie.ar0234.upper_body_static_gate.v1',
            'allow_bottom_truncation': True, 'dynamic_fusion_permitted': False,
            'require_single_target': True, 'require_untruncated_edges': ['top', 'left', 'right'],
            'static_scene_only': True}:
            raise ValueError('unsupported static geometry policy')
        if self.temporal != {
            'schema_version': 'sie.temporal.static_gate.v1', 'apply_before_stereo_fusion': True,
            'candidate_max_age_s': 5., 'max_area_relative_change': .1,
            'max_center_delta_px': 12., 'minimum_consecutive_observations': 2}:
            raise ValueError('unsupported temporal static policy')
        self.reset()

    def reset(self):
        self.previous = None
        self.count = 0

    def evaluate(self, cycle_id, candidate, eligible_count, monotonic_s):
        result = {'cycle_id': cycle_id, 'profile_sha256': REVIEWED_SHA,
                  'status': 'FAIL', 'reason': None, 'visual_static_only': True,
                  'physical_stillness_proven': False, 'actuator_access': False}
        def refuse(reason):
            self.reset(); result['reason'] = reason
            return result
        if not isinstance(cycle_id, str) or not cycle_id or not finite_number(monotonic_s):
            return refuse('INVALID_CYCLE_OR_CLOCK')
        if type(eligible_count) is not int or eligible_count != 1 or not isinstance(candidate, dict):
            return refuse('SINGLE_CURRENT_TARGET_REQUIRED')
        edges = [candidate.get('truncated_' + e) for e in ('top', 'left', 'right', 'bottom')]
        if any(type(v) is not bool for v in edges) or any(edges[:3]):
            return refuse('STATIC_PROFILE_TARGET_EDGE_REFUSED')
        b = candidate.get('bbox_xyxy_px')
        if not isinstance(b, list) or len(b) != 4 or not all(finite_number(v) for v in b):
            return refuse('INVALID_TARGET_BBOX')
        x1, y1, x2, y2 = b
        if x2 <= x1 or y2 <= y1:
            return refuse('INVALID_TARGET_BBOX')
        current = {'cycle_id': cycle_id, 'time': monotonic_s, 'x': (x1+x2)/2,
                   'y': (y1+y2)/2, 'area': (x2-x1)*(y2-y1)}
        previous = self.previous
        if previous is not None and (previous['cycle_id'] == cycle_id or monotonic_s <= previous['time']):
            return refuse('DUPLICATE_CYCLE_OR_NONINCREASING_CLOCK')
        self.previous = current
        if previous is None or monotonic_s - previous['time'] > self.temporal['candidate_max_age_s']:
            self.count = 1
            result.update(status='PENDING', reason='TEMPORAL_STABILITY_PENDING')
            return result
        delta = math.hypot(current['x']-previous['x'], current['y']-previous['y'])
        area_change = abs(current['area']-previous['area'])/previous['area']
        result.update(center_delta_px=delta, area_relative_change=area_change)
        if delta > self.temporal['max_center_delta_px'] or area_change > self.temporal['max_area_relative_change']:
            self.count = 1
            result['reason'] = 'MOTION_DETECTED'
            return result
        self.count += 1
        result.update(status='PASS', reason='VISUAL_STATIC_GATE_PASSED', consecutive_observations=self.count)
        return result
