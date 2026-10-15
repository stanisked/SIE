#!/usr/bin/env python3
"""Read existing compact timing journals without inventing missing stage timers."""
import argparse
from collections import Counter,defaultdict
import hashlib
import json
import math
from pathlib import Path
import statistics


def number(value):
    return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value) and value>=0


def timing_row(a):
    shape=a.get('shape_diagnostic') or {};seg=a.get('segmentation_observation') or {}
    execution=a.get('stereo_execution') or {};worker=execution.get('worker_timings_s') or {}
    raw={'host':(a.get('processing_timing') or {}).get('host_capture_start_to_result_ready_s'),
        'rgb_rpc':a.get('rgb_segmentation_round_trip_s'),'rgb_inference_extract':seg.get('mask_s',shape.get('mask_s')),
        'stereo_worker':a.get('stereo_worker_s'),'fusion_rpc':shape.get('round_trip_s'),
        'mask_depth_prepare':shape.get('mask_depth_prepare_s'),
        'left_match':worker.get('left_compute_s'),'right_match':worker.get('right_compute_s')}
    out={k:v for k,v in raw.items() if number(v)}
    for name,outer,inner in (('rgb_rpc_outside_inference','rgb_rpc','rgb_inference_extract'),
                            ('fusion_rpc_outside_prepare','fusion_rpc','mask_depth_prepare')):
        if outer in out and inner in out:out[name]=out[outer]-out[inner]
    if all(k in out for k in ('stereo_worker','left_match','right_match')):
        out['stereo_worker_minus_longest_match']=out['stereo_worker']-max(out['left_match'],out['right_match'])
    if all(k in out for k in ('host','rgb_rpc','stereo_worker','fusion_rpc')):
        # A proxy only: branch start/queue offsets and capture durations were not recorded.
        out['parallel_branch_max_proxy']=max(out['rgb_rpc'],out['stereo_worker'])
        out['host_unallocated_proxy']=out['host']-out['parallel_branch_max_proxy']-out['fusion_rpc']
    return {'cycle_id':a['cycle_id'],'source_status':a.get('status'),
        'local_schedule':(a.get('local_surface_async') or {}).get('status'),
        'timing_s':out,'missing_timers':[k for k in raw if k not in out]}


def summarize(rows):
    values=defaultdict(list)
    for row in rows:
        for key,value in row['timing_s'].items():values[key].append(value)
    return {key:{'n':len(v),'median_s':statistics.median(v),'max_s':max(v),'min_s':min(v)} for key,v in values.items()}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--journal-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():p.error('new output required')
    records=[];hashes={};seen=set()
    for path in sorted(args.journal_root.rglob('ar0234-ov9281-control-preview-*.json')):
        blob=path.read_bytes();a=json.loads(blob);hashes[str(path)]=hashlib.sha256(blob).hexdigest()
        if a['cycle_id'] in seen:raise ValueError('duplicate cycle')
        seen.add(a['cycle_id']);records.append(a)
    if not records:raise ValueError('no primary journals')
    first={};excluded=Counter();rows=[]
    for a in records:
        run=a['cycle_id'].rsplit('-',1)[0]
        first.setdefault(run,a['cycle_id'])
        if a.get('status')=='CANCELLED_SHUTDOWN':excluded['cancelled']+=1;continue
        if a['cycle_id']==first[run]:excluded['first_recorded_cycle_per_run']+=1;continue
        rows.append(timing_row(a))
    summary=summarize(rows)
    for key,stats in summary.items():
        print(key,'n=',stats['n'],'median_ms=',round(stats['median_s']*1000,2),'max_ms=',round(stats['max_s']*1000,2))
    limitations=['Capture AR/stereo duration and future.result wait were not separately recorded.',
        'RGB and stereo overlap: never sum their durations as exclusive wall time.',
        'host_unallocated_proxy includes capture, branch scheduling offsets, target selection and other unrecorded work; not a measured capture timer.',
        'RPC-minus-worker deltas include RAM serialization/hash/load/response and scheduling; not pure IPC.',
        'stereo_worker-minus-longest-match includes remap/gray/resize/LR and queue/overlap effects; not an isolated rectification timer.',
        'Background local work is outside primary path, but can compete for CPU. Publish/journal after primary endpoint excluded.',
        'Per-cycle differences are summarized directly; differences of medians are not used. Negative proxies retained.']
    unchanged=all(hashlib.sha256(Path(path).read_bytes()).hexdigest()==sha for path,sha in hashes.items())
    if not unchanged:raise ValueError('journal changed during analysis')
    report={'status':'EXISTING_TIMERS_ANALYSIS_ONLY','rows':rows,'summary':summary,'excluded':dict(excluded),
        'source_statuses':dict(Counter(a.get('status') for a in records)),
        'by_local_schedule':{key:summarize([r for r in rows if r['local_schedule']==key]) for key in sorted({r['local_schedule'] for r in rows},key=str)},
        'limitations':limitations,'source_sha256':hashes,'inputs_unchanged':unchanged,
        'tool_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    print('TIMING_ROWS:',len(rows),'EXCLUDED:',dict(excluded))
    print('UNRECORDED: capture_AR, capture_stereo, exact_stereo_wait, exact_branch_overlap')
    print('Saved:',args.output)


if __name__=='__main__':main()
