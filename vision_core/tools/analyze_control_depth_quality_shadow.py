#!/usr/bin/env python3
"""Replay the non-actuating quality assessment on compact existing journals."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from vision_core.person_localization.control_depth_quality_shadow import assess_control_depth
from vision_core.person_localization.control_depth_admission_draft import POLICY
from vision_core.person_localization.control_input_shadow import evaluate_control_input_shadow


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--journal-root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():p.error('output already exists')
    local={};hashes={str(POLICY):hashlib.sha256(POLICY.read_bytes()).hexdigest()}
    def read(path):
        data=path.read_bytes();hashes[str(path)]=hashlib.sha256(data).hexdigest()
        return json.loads(data)
    for path in sorted(args.journal_root.rglob('local-surface-*.json')):
        result=read(path);cid=result['cycle_id']
        if cid in local:raise ValueError('duplicate local cycle')
        local[cid]=result
    rows=[];seen=set()
    for path in sorted(args.journal_root.rglob('ar0234-ov9281-control-preview-*.json')):
        audit=read(path);cid=audit['cycle_id']
        if cid in seen:raise ValueError('duplicate primary cycle')
        seen.add(cid)
        if audit.get('status')=='CANCELLED_SHUTDOWN':continue
        primary=assess_control_depth(audit)
        delayed=assess_control_depth(audit,local[cid]) if cid in local else None
        bound=evaluate_control_input_shadow(audit)
        rows.append({'cycle_id':cid,'recorded_primary':audit.get('control_depth_quality_shadow'),
            'primary':primary,'same_frame_delayed':delayed,'draft_admission':bound['depth_admission_draft'],
            'control_input_shadow':bound['control_input_shadow'],'decision_preview':bound['decision_preview']})
        print(cid[-8:],primary['status'],'reasons=',','.join(primary['refusal_reasons']),
              'local=',(delayed or primary)['local_consistency']['status'])
    if not rows:raise ValueError('no completed primary cycles')
    if set(local)-seen:raise ValueError('orphan local cycles')
    unchanged=all(hashlib.sha256(Path(path).read_bytes()).hexdigest()==sha for path,sha in hashes.items())
    if not unchanged:raise ValueError('inputs changed')
    summary={'cycles':len(rows),'primary':dict(Counter(r['primary']['status'] for r in rows)),
        'refusal_reasons':dict(Counter(x for r in rows for x in r['primary']['refusal_reasons'])),
        'missing_evidence':dict(Counter(x for r in rows for x in r['primary']['missing_evidence'])),
        'driver_frame_age':dict(Counter(r['primary']['freshness']['driver_metadata']['status'] for r in rows)),
        'host_duration_exceeds_numeric_age_target':sum(r['primary']['freshness']['performance']['host_duration_exceeds_numeric_age_target'] for r in rows),
        'recorded_to_replayed':dict(Counter((r['recorded_primary'] or {}).get('status','ABSENT')+' -> '+r['primary']['status'] for r in rows)),
        'draft_admission':{'statuses':dict(Counter(r['draft_admission']['status'] for r in rows)),
            'sensor_prerequisites':dict(Counter(r['draft_admission']['sensor_prerequisites'] for r in rows)),
            'failed_checks':dict(Counter(k for r in rows for k in r['draft_admission']['failed_checks'])),
            'missing_checks':dict(Counter(k for r in rows for k in r['draft_admission']['missing_checks']))},
        'control_input_shadow':{'statuses':dict(Counter(r['control_input_shadow']['status'] for r in rows)),
            'decision_preview':dict(Counter(r['decision_preview']['status'] for r in rows)),
            'reasons':dict(Counter(reason for r in rows for reason in r['decision_preview']['reasons']))},
        'includes_first_cycle':True,
        'local_matches':sum(r['same_frame_delayed'] is not None for r in rows),
        'local_binding_refusals':sum('LOCAL_RESULT_BINDING_MISMATCH' in (r['same_frame_delayed'] or {}) .get('refusal_reasons',[]) for r in rows)}
    report={'status':'CONTROL_DEPTH_QUALITY_SHADOW_REPLAY_ONLY','rows':rows,'summary':summary,
        'inputs_unchanged':unchanged,'source_sha256':hashes,'metric_measurement_authorized':False,'actuator_access':False,
        'code_sha256':{str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in (Path(__file__),ROOT/'vision_core/person_localization/control_depth_quality_shadow.py',ROOT/'vision_core/person_localization/control_depth_admission_draft.py',ROOT/'vision_core/person_localization/control_input_shadow.py',POLICY)}}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    print('QUALITY_SHADOW:',summary);print('INPUTS_UNCHANGED:',unchanged);print('Saved:',args.output)


if __name__=='__main__':main()
