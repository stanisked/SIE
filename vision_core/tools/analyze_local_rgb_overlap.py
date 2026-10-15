#!/usr/bin/env python3
"""Estimate local-worker/RGB interval overlap from an existing preview journal."""
import argparse
from collections import defaultdict
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import statistics


def seconds(value):
    value=float(value)
    if not math.isfinite(value) or value<0:raise ValueError('invalid duration')
    return value


def utc(value):
    result=datetime.fromisoformat(value)
    if result.tzinfo is None:raise ValueError('timezone required')
    return result.timestamp()


def overlap(a,b):return max(0.,min(a[1],b[1])-max(a[0],b[0]))


def analyze(primary,local):
    by_id={}
    for a in primary:
        cid=a['cycle_id']
        if cid in by_id:raise ValueError('duplicate primary cycle')
        by_id[cid]=a
    intervals=[];seen=set()
    for a in local:
        cid=a['cycle_id']
        if cid in seen or cid not in by_id:raise ValueError('duplicate/orphan local cycle')
        seen.add(cid)
        if a.get('provenance',{}).get('source_cycle_id')!=cid:raise ValueError('local source binding mismatch')
        # Completion wall timestamp is captured just before duration serialization.
        # This reconstructs a request interval, not a CPU execution trace.
        end=utc(a['completed_at_utc']);duration=seconds(a['submit_to_result_s']);wait=seconds(a['queue_wait_s'])
        if wait>duration:raise ValueError('queue wait exceeds duration')
        intervals.append((cid,end-duration+wait,end))
    first={}
    for a in primary:
        run=a['cycle_id'].rsplit('-',1)[0]
        first[run]=min(first.get(run,a['cycle_id']),a['cycle_id'])
    rows=[];excluded=defaultdict(int)
    for a in sorted(primary,key=lambda a:a['cycle_id']):
        cid=a['cycle_id'];run=cid.rsplit('-',1)[0]
        if a.get('status')=='CANCELLED_SHUTDOWN':excluded['cancelled']+=1;continue
        if cid==first[run]:excluded['first_cycle']+=1;continue
        timeline=a.get('perception_timeline_s',{})
        if 'rgb_call_start_after_cycle_start_s' not in timeline or 'rgb_call_end_after_cycle_start_s' not in timeline:
            excluded['missing_rgb_timeline']+=1;continue
        base=utc(a['timestamp']);start=seconds(timeline['rgb_call_start_after_cycle_start_s']);end=seconds(timeline['rgb_call_end_after_cycle_start_s'])
        if end<start:raise ValueError('reversed RGB interval')
        hits=[{'source_cycle_id':source,'overlap_s':overlap((base+start,base+end),(lo,hi))}
              for source,lo,hi in intervals if source.rsplit('-',1)[0]==run and overlap((base+start,base+end),(lo,hi))>0]
        rows.append({'cycle_id':cid,'group':'LOCAL_REQUEST_OVERLAP' if hits else 'NO_RECORDED_LOCAL_OVERLAP',
            'local_intervals':hits,'rgb_call_s':end-start,
            'prediction_extract_s':a.get('segmentation_observation',{}).get('prediction_extract_s'),
            'host_s':seconds(a['processing_timing']['host_capture_start_to_result_ready_s'])})
    groups={}
    for label in ('LOCAL_REQUEST_OVERLAP','NO_RECORDED_LOCAL_OVERLAP'):
        selected=[r for r in rows if r['group']==label];metrics={}
        for key in ('host_s','rgb_call_s','prediction_extract_s'):
            values=[seconds(r[key]) for r in selected if r[key] is not None]
            metrics[key]={'n':len(values),'median':statistics.median(values) if values else None,'max':max(values) if values else None}
        groups[label]={'cycles':len(selected),'within_250ms':sum(r['host_s']<=.25 for r in selected),'metrics':metrics}
    return {'rows':rows,'groups':groups,'excluded':dict(excluded),'local_intervals':len(intervals),
        'limitations':['UTC-reconstructed request intervals, not exact CPU execution; wall clock adjustment can distort overlap.',
            'No-overlap means no recorded local request overlap, not an idle CPU.',
            'Observational association does not establish causation; scene, CPU frequency and other work can differ.',
            'Host time is not exposure-to-command age; no quality or motion policy changes.']}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--journal-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    if args.output.exists():p.error('new output required')
    paths=sorted(args.journal_root.rglob('ar0234-ov9281-control-preview-*.json'));locals_=sorted(args.journal_root.rglob('local-surface-*.json'))
    if not paths:raise ValueError('no primary journal')
    if not locals_:raise ValueError('no saved local intervals for comparison')
    hashes={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in paths+locals_}
    report=analyze([json.loads(f.read_text()) for f in paths],[json.loads(f.read_text()) for f in locals_])
    for label,group in report['groups'].items():print('LOCAL_RGB_OVERLAP',label,json.dumps(group),flush=True)
    print('OVERLAP_EXCLUDED',json.dumps(report['excluded']))
    if any(hashlib.sha256(Path(f).read_bytes()).hexdigest()!=h for f,h in hashes.items()):raise ValueError('input changed')
    report.update(status='OFFLINE_LOCAL_RGB_OVERLAP_DIAGNOSTIC_ONLY',inputs_unchanged=True,source_sha256=hashes,
                  code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),actuator_access=False)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    print('INPUTS_UNCHANGED: True Saved:',args.output)


if __name__=='__main__':main()
