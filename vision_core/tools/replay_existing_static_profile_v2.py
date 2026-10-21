#!/usr/bin/env python3
"""Compare static-v2 applicability on saved audits; never rewrites source audits.

This is a counterfactual policy replay. A post-hoc gate cannot establish that
the historical stereo ran after that gate, or approve the historical result.
"""
import argparse
from collections import Counter
from copy import deepcopy
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from vision_core.person_localization.existing_static_profile_preview import (
    load_existing_static_profile, StaticProfilePreviewGate, REVIEWED_SHA, SOURCE_HASHES, EXTRINSIC_SHA)
from vision_core.person_localization.control_input_shadow import evaluate_control_input_shadow


def replay(journal_root, profile_path):
    profile = load_existing_static_profile(profile_path)
    gate = StaticProfilePreviewGate(profile)
    hashes = {str(profile_path): REVIEWED_SHA}
    rows = []; seen = set(); last_run = None
    for path in sorted(journal_root.rglob('ar0234-ov9281-control-preview-*.json')):
        raw = path.read_bytes(); hashes[str(path)] = hashlib.sha256(raw).hexdigest()
        a = json.loads(raw); cid = a.get('cycle_id')
        if not isinstance(cid, str) or not cid or cid in seen:
            raise ValueError('missing/duplicate source cycle id')
        seen.add(cid)
        run = str(path.parent)
        if run != last_run: gate.reset(); last_run = run
        cal = a.get('calibration') or {}
        if any(cal.get(k) != v for k,v in {
            'ar0234_intrinsic_sha256': SOURCE_HASHES['ar0234_intrinsic'],
            'ov9281_stereo_sha256': SOURCE_HASHES['ov9281_stereo_calibration'],
            'ar0234_to_physical_left_extrinsic_sha256': EXTRINSIC_SHA}.items()):
            raise ValueError('source audit calibration differs from reviewed profile')
        dynamic = a.get('ar0234_dynamic_target') or {}
        candidate = dynamic.get('candidate') if dynamic.get('disposition') == 'OBSERVED' else None
        stamp = datetime.fromisoformat(a['timestamp'])
        if stamp.tzinfo is None: raise ValueError('source timestamp lacks timezone')
        result = gate.evaluate(cid, candidate, dynamic.get('current_eligible_count'), stamp.timestamp())
        if a.get('status') in ('CANCELLED_SHUTDOWN','PROCESSING_REFUSED'):
            gate.reset(); result.update(status='FAIL', reason='SOURCE_CYCLE_NOT_USABLE')
        scenario = deepcopy(a)
        scenario['activated_depth_range_m'] = profile['authorization']['metric_measurement_range_m']
        scenario['existing_static_profile_preview'] = {'required':True, 'profile_sha256':REVIEWED_SHA}
        scenario['existing_profile_static_gate'] = result
        original = evaluate_control_input_shadow(a)
        alternative = evaluate_control_input_shadow(scenario)
        rows.append({'cycle_id':cid, 'static_gate':result,
                     'recorded_range_m':a.get('activated_depth_range_m'),
                     'scenario_range_m':scenario['activated_depth_range_m'],
                     'original_failed_checks':original['decision_preview']['failed_checks'],
                     'scenario_preview':alternative['decision_preview']})
    if not rows: raise ValueError('no source audits')
    if not all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in hashes.items()):
        raise ValueError('source inputs changed')
    return {'status':'COUNTERFACTUAL_STATIC_PROFILE_REPLAY_ONLY', 'inputs_unchanged':True,
            'static_gate_executed_before_historical_stereo':False,
            'clock_basis':'recorded host UTC cycle start; live gate uses monotonic evaluation time',
            'metric_measurement_authorized':False, 'actuator_access':False,
            'source_sha256':hashes, 'rows':rows,
            'summary':{'cycles':len(rows), 'static_gate':dict(Counter(r['static_gate']['status'] for r in rows)),
                'static_reasons':dict(Counter(r['static_gate']['reason'] for r in rows)),
                'original_failed_checks':dict(Counter(k for r in rows for k in r['original_failed_checks'])),
                'scenario_failed_checks':dict(Counter(k for r in rows for k in r['scenario_preview']['failed_checks'])),
                'scenario_missing_checks':dict(Counter(k for r in rows for k in r['scenario_preview']['missing_checks'])),
                'preview':dict(Counter(r['scenario_preview']['status'] for r in rows))}}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--journal-root',type=Path,required=True)
    p.add_argument('--profile',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args = p.parse_args()
    if args.output.exists(): p.error('output already exists')
    result = replay(args.journal_root,args.profile)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream: json.dump(result,stream,indent=2,allow_nan=False)
    print('STATIC_V2_REPLAY:',result['summary'])
    print('INPUTS_UNCHANGED:',result['inputs_unchanged'])
    print('Saved:',args.output)


if __name__ == '__main__': main()
