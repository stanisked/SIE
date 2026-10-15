#!/usr/bin/env python3
"""Compare full vs target-cropped SGBM on frozen pairs, without activating crop."""
import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import statistics
import sys
import time
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
import cv2
import numpy as np
from analyze_target_stereo_crop import crop_plan,runtime_frustum_margin
from vision_core.person_localization.diagnostic_stereo_resolution import half_disparity


def restore_crop(raw_maps,box,full_shape,configs):
    """Common crop origin: disparity unchanged; restore global LR rounding/grid."""
    x1,y1,x2,y2=box;h,w=full_shape
    if not 0<=x1<x2<=w or not 0<=y1<y2<=h:raise ValueError('invalid crop box')
    restored=[]
    for raw,config in zip(raw_maps,configs):
        if raw.shape!=(y2-y1,x2-x1) or raw.dtype!=np.int16:raise ValueError('unexpected cropped map')
        invalid=(config['minDisparity']-1)*16
        full=np.full(full_shape,invalid,np.int16);full[y1:y2,x1:x2]=raw;restored.append(full)
    if len(restored)!=2:raise ValueError('expected both matching directions')
    return restored


def array_summary(values):
    if not len(values):return {'n':0,'median':None,'absolute_p90':None,'absolute_max':None}
    return {'n':len(values),'median':float(np.median(values)),
            'absolute_p90':float(np.percentile(np.abs(values),90)),'absolute_max':float(np.max(np.abs(values)))}


def sample_comparison(first,second,width,fb):
    a=first['v']*width+first['u'];b=second['v']*width+second['u']
    common,ia,ib=np.intersect1d(a,b,return_indices=True)
    lost=np.setdiff1d(a,b);gained=np.setdiff1d(b,a)
    disparity_delta=second['d'][ib].astype(float)-first['d'][ia].astype(float)
    z1=fb/first['d'][ia];z2=fb/second['d'][ib]
    depth_delta=z2-z1
    order=np.argsort(np.abs(depth_delta))[::-1][:20]
    return {'reference_owned_samples':len(a),'crop_owned_samples':len(b),'common_owned_samples':len(common),
        'lost_sample_count':len(lost),'gained_sample_count':len(gained),
        'lost_xy_px':[[int(i%width),int(i//width)] for i in lost],
        'gained_xy_px':[[int(i%width),int(i//width)] for i in gained],
        'disparity_delta_px':array_summary(disparity_delta),'depth_delta_m':array_summary(depth_delta),
        'changed_common_disparities':int(np.count_nonzero(disparity_delta)),
        'largest_common_depth_changes':[{'xy_px':[int(common[i]%width),int(common[i]//width)],
            'reference_depth_m':float(z1[i]),'crop_depth_m':float(z2[i]),'delta_m':float(depth_delta[i])} for i in order]}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('evidence-root','ar-intrinsic','stereo-calibration','activation','output'):
        p.add_argument('--'+key,type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():p.error('new output required')
    from analyze_ar0234_stereo_component_ambiguity import load_calibrations,sha256
    from replay_shadow_mask_depth_range import replay
    from preview_mask_surface import prepare_stereo_points,evaluate_prepared
    from benchmark_stereo_sgbm_parallel import runtime_parameters,compute_pair
    from benchmark_stereo_resolution import scaled_configs
    cp=json.loads(args.activation.read_text())['calibration_profile'];ex=cp['ar0234_to_ov9281_physical_left_extrinsic'];extrinsic=Path(ex['path']).expanduser()
    if sha256(args.ar_intrinsic)!=cp['source_sha256']['ar0234_intrinsic'] or sha256(args.stereo_calibration)!=cp['source_sha256']['ov9281_stereo_calibration'] or sha256(extrinsic)!=ex['sha256']:
        raise ValueError('calibration/activation mismatch')
    cal=load_calibrations(args.ar_intrinsic,args.stereo_calibration,extrinsic)
    configs,code=runtime_parameters();configs=scaled_configs(configs,.5);margin=runtime_frustum_margin()
    cv2.setNumThreads(1)
    matchers={label:[cv2.StereoSGBM_create(**c) for c in configs] for label in ('full','crop')}
    hashes={str(f):sha256(f) for f in (args.ar_intrinsic,args.stereo_calibration,args.activation,extrinsic)};rows=[]
    with ThreadPoolExecutor(max_workers=2) as pool:
        for path in sorted(args.evidence_root.rglob('audit.json')):
            verified=replay(path,cal,.5,4.5)
            if 'mask_sha256' not in verified:raise ValueError('expected verified saved mask')
            audit=json.loads(path.read_text());cid=audit['cycle_id'];folder=path.parent
            roi=audit['stereo_geometry']['ar_inner_roi_xyxy_px'];mp=folder/f'{cid}.png'
            mask=cv2.imread(str(mp),cv2.IMREAD_GRAYSCALE)==255
            images=[]
            for side in ('left','right'):
                f=folder/f'ov9281_physical_{side}_rectified.jpg';hashes[str(f)]=sha256(f)
                color=cv2.imread(str(f))
                if color is None or color.shape!=(800,1280,3):raise ValueError('invalid rectified stereo evidence')
                images.append(cv2.resize(cv2.cvtColor(color,cv2.COLOR_BGR2GRAY),(640,400),interpolation=cv2.INTER_AREA))
            for f in (path,mp,folder/f'{cid}.json',folder/'ar0234_source.jpg',folder/'disparity_lr_consistent_px.npy'):hashes[str(f)]=sha256(f)
            plan_start=time.perf_counter();plan=crop_plan(roi,cal,configs,diagnostic_margin_full_px=margin)
            plan_s=time.perf_counter()-plan_start
            x1,y1,x2,y2=plan['common_crop_xyxy_px']
            copy_start=time.perf_counter()
            crop_images=[im[y1:y2,x1:x2].copy() for im in images]
            crop_copy_s=time.perf_counter()-copy_start
            if x2-x1<=64+7:raise ValueError('crop too narrow for unchanged matcher')
            for label,pair in (('full',images),('crop',crop_images)):compute_pair(matchers[label],*pair,executor=pool)
            for repeat in range(2):
                timing={};results={};samples={};maps={};lrs={}
                order=('full','crop') if repeat==0 else ('crop','full')
                for label in order:
                    start=time.perf_counter();raw,_=compute_pair(matchers[label],*(images if label=='full' else crop_images),executor=pool)
                    matched=time.perf_counter();timing[label]={'match_pair_s':matched-start}
                    if label=='crop':raw=restore_crop(raw,plan['common_crop_xyxy_px'],images[0].shape,configs)
                    disparity,lr=half_disparity(*raw);maps[label]=raw;lrs[label]=lr
                    valid=np.where(lr,disparity,np.nan)
                    after_lr=time.perf_counter();timing[label]['restore_lr_s']=after_lr-matched
                    points=prepare_stereo_points(valid,cal,scale=.5,cycle_id=cid)
                    result,selected=evaluate_prepared(points,mask,roi,defer_local=True,prepare_local=True)
                    timing[label]['geometry_ownership_s']=time.perf_counter()-after_lr
                    timing[label]['match_lr_depth_s']=time.perf_counter()-start
                    results[label]=result['mask_depth'];samples[label]=selected
                delta=sample_comparison(samples['full'],samples['crop'],640,abs(float(cal['P2'][0,3]))/1000*.5)
                ref=samples['full'];u,v=ref['u'].astype(int),ref['v'].astype(int)
                xr=np.rint(u-maps['full'][0][v,u].astype(np.float32)/16.).astype(int)
                support={'right_raw_changed_at_reference_lookup':int(np.count_nonzero(maps['full'][1][v,xr]!=maps['crop'][1][v,xr])),
                    'reference_target_lost_lr':int((~lrs['crop'][v,u]).sum()),
                    'left_raw_changed_at_reference_target':int(np.count_nonzero(maps['full'][0][v,u]!=maps['crop'][0][v,u])),
                    'lr_mask_changed_inside_crop':int(np.count_nonzero(lrs['full'][y1:y2,x1:x2]!=lrs['crop'][y1:y2,x1:x2]))}
                a,b=results['full'].get('candidate_depth_m'),results['crop'].get('candidate_depth_m')
                row={'cycle_id':cid,'repeat':repeat,'order':order,'plan':plan,'plan_s':plan_s,'crop_copy_s':crop_copy_s,
                    'timing':timing,'full_depth':results['full'],'crop_depth':results['crop'],
                    'median_depth_delta_m':b-a if a is not None and b is not None else None,
                    'depth_report_exact':results['full']==results['crop'],'target_points':delta,'lr_comparison':support}
                rows.append(row)
                if repeat==0:print('CROP_MATCH',cid[-8:],results['full']['status'],'->',results['crop']['status'],
                    'depth_delta_m=',row['median_depth_delta_m'],'lost=',delta['lost_sample_count'],'gained=',delta['gained_sample_count'],
                    'changed_common=',delta['changed_common_disparities'],flush=True)
    if not rows:raise ValueError('no recorded cycles')
    unchanged=all(sha256(Path(f))==digest for f,digest in hashes.items())
    if not unchanged:raise ValueError('source evidence changed')
    summary={'comparisons':len(rows),'depth_reports_exact':sum(r['depth_report_exact'] for r in rows),
        'status_transitions':dict(Counter(r['full_depth']['status']+' -> '+r['crop_depth']['status'] for r in rows)),
        'reference_owned_samples_total':sum(r['target_points']['reference_owned_samples'] for r in rows),
        'lost_samples_total':sum(r['target_points']['lost_sample_count'] for r in rows),
        'gained_samples_total':sum(r['target_points']['gained_sample_count'] for r in rows),
        'changed_common_disparities_total':sum(r['target_points']['changed_common_disparities'] for r in rows),
        'max_absolute_median_depth_delta_m':max((abs(r['median_depth_delta_m']) for r in rows if r['median_depth_delta_m'] is not None),default=None)}
    for label in ('full','crop'):
        print('CROP_TIMING',label,json.dumps({key:statistics.median(r['timing'][label][key] for r in rows) for key in rows[0]['timing'][label]}),flush=True)
    print('ROI_PLAN median_s=',statistics.median(r['plan_s'] for r in rows),'crop_copy_median_s=',statistics.median(r['crop_copy_s'] for r in rows))
    print('CROP_MATCH_SUMMARY',json.dumps(summary),flush=True)
    paths=(Path(__file__),ROOT/'vision_core/tools/analyze_target_stereo_crop.py',ROOT/'vision_core/tools/preview_mask_surface.py',ROOT/'vision_core/person_localization/diagnostic_stereo_resolution.py')
    report={'status':'OFFLINE_CROPPED_MATCHER_COMPARISON_ONLY','rows':rows,'summary':summary,
        'inputs_unchanged':unchanged,'source_sha256':hashes,'calibration_sha256':cal['sha'],'runtime_source_sha256':code,
        'code_sha256':{str(f):sha256(f) for f in paths},'matcher_configs':configs,
        'environment':{'opencv':cv2.__version__,'numpy':np.__version__},'actuator_access':False,
        'limitations':['Frozen saved RGB ROI/mask; no live detection, capture, rectification or local fitting.',
            'Dense ROI-plan cost recorded separately; cropped timing excludes plan and prebuilt crop copy.',
            'Same disparity range/block/P1/P2/speckle settings; changed aggregation context remains visible.',
            'Restore to original half-frame coordinates before LR rounding/grid; no resize, fill, smoothing or depth correction.',
            'Current-frame ROI becomes available after RGB; faster cropped matching alone is not full-cycle speedup.',
            'No new quality tolerance or runtime switch; numeric differences are recorded, not hidden.']}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    print('INPUTS_UNCHANGED:',unchanged,'Saved:',args.output)


if __name__=='__main__':main()
