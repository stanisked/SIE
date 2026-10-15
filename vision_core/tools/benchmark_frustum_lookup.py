#!/usr/bin/env python3
"""Compare startup lookup crop plans with dense saved-ROI plans, without rollout."""
import argparse
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


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('evidence-root','ar-intrinsic','stereo-calibration','activation','output'):
        p.add_argument('--'+key,type=Path,required=True)
    p.add_argument("--compare-blocks",action="store_true",help="compare the same lookup grid with/without exact block extrema")
    args=p.parse_args()
    if args.output.exists():p.error('new output required')
    from analyze_ar0234_stereo_component_ambiguity import load_calibrations,sha256
    from analyze_target_stereo_crop import crop_plan,runtime_frustum_margin,support_loss
    from stereo_frustum_lookup import FrustumLookup,BlockFrustumLookup
    from replay_shadow_mask_depth_range import replay
    from preview_mask_surface import prepare_stereo_points
    from compare_shadow_mask_depth_median import owned
    from benchmark_stereo_sgbm_parallel import runtime_parameters,compute_pair
    from benchmark_stereo_resolution import scaled_configs
    from vision_core.person_localization.diagnostic_stereo_resolution import half_disparity
    cp=json.loads(args.activation.read_text())['calibration_profile'];ex=cp['ar0234_to_ov9281_physical_left_extrinsic'];extrinsic=Path(ex['path']).expanduser()
    if sha256(args.ar_intrinsic)!=cp['source_sha256']['ar0234_intrinsic'] or sha256(args.stereo_calibration)!=cp['source_sha256']['ov9281_stereo_calibration'] or sha256(extrinsic)!=ex['sha256']:
        raise ValueError('calibration/activation mismatch')
    cal=load_calibrations(args.ar_intrinsic,args.stereo_calibration,extrinsic)
    configs,code=runtime_parameters();configs=scaled_configs(configs,.5);margin=runtime_frustum_margin()
    cv2.setNumThreads(1)
    start=time.perf_counter();lookup=FrustumLookup(cal);startup_s=time.perf_counter()-start
    print('LOOKUP_STARTUP seconds=',round(startup_s,4),'table_bytes=',lookup.table.nbytes,flush=True)
    reference_lookup=lookup if args.compare_blocks else None
    block_startup_s=None
    if args.compare_blocks:
        start=time.perf_counter();lookup=BlockFrustumLookup(lookup);block_startup_s=time.perf_counter()-start
        print('BLOCK_INDEX_STARTUP seconds=',round(block_startup_s,6),'index_bytes=',lookup.index_bytes,flush=True)
    hashes={str(f):sha256(f) for f in (args.ar_intrinsic,args.stereo_calibration,args.activation,extrinsic)};rows=[]
    matchers=[cv2.StereoSGBM_create(**c) for c in configs]
    with ThreadPoolExecutor(max_workers=2) as pool:
        for path in sorted(args.evidence_root.rglob('audit.json')):
            verified=replay(path,cal,.5,4.5)
            if 'mask_sha256' not in verified:raise ValueError('expected verified mask')
            audit=json.loads(path.read_text());cid=audit['cycle_id'];folder=path.parent
            roi=audit['stereo_geometry']['ar_inner_roi_xyxy_px'];mp=folder/f'{cid}.png'
            mask=cv2.imread(str(mp),cv2.IMREAD_GRAYSCALE)==255
            interior=cv2.erode(mask.astype('uint8'),np.ones((5,5),'uint8'),borderType=cv2.BORDER_CONSTANT,borderValue=0)
            images=[]
            for side in ('left','right'):
                f=folder/f'ov9281_physical_{side}_rectified.jpg';hashes[str(f)]=sha256(f)
                color=cv2.imread(str(f))
                if color is None or color.shape!=(800,1280,3):raise ValueError('invalid stereo input')
                images.append(cv2.resize(cv2.cvtColor(color,cv2.COLOR_BGR2GRAY),(640,400),interpolation=cv2.INTER_AREA))
            for f in (path,mp,folder/f'{cid}.json',folder/'ar0234_source.jpg',folder/'disparity_lr_consistent_px.npy'):hashes[str(f)]=sha256(f)
            maps,_=compute_pair(matchers,*images,executor=pool);disparity,lr=half_disparity(*maps)
            points=prepare_stereo_points(np.where(lr,disparity,np.nan),cal,scale=.5)
            selected=owned(points['xy'],interior,roi);u,v,d=(points[k][selected] for k in ('u','v','d'))
            for repeat in range(2):
                plans={};timing={};order=('dense','lookup') if repeat==0 else ('lookup','dense')
                for label in order:
                    start=time.perf_counter();plans[label]=crop_plan(roi,cal,configs,diagnostic_margin_full_px=margin,
                        frustum_lookup=lookup if label=='lookup' else reference_lookup)
                    timing[label]=time.perf_counter()-start
                a,b=(plans[k]['common_crop_xyxy_px'] for k in ('dense','lookup'))
                losses={k:support_loss(plans[k],u,v,d) for k in plans}
                rows.append({'cycle_id':cid,'repeat':repeat,'order':order,'timing_s':timing,'plans':plans,
                    'plans_exact':plans['dense']==plans['lookup'],'crop_bounds_exact':a==b,'lookup_contains_dense_crop':b[0]<=a[0] and b[1]<=a[1] and b[2]>=a[2] and b[3]>=a[3],
                    'max_crop_coordinate_delta_px':max(abs(x-y) for x,y in zip(a,b)),'observed_support':losses})
    if not rows:raise ValueError('no saved cycles')
    unchanged=all(sha256(Path(f))==digest for f,digest in hashes.items())
    if not unchanged:raise ValueError('source inputs changed')
    summary={'comparisons':len(rows),'exact_crop_bounds':sum(r['crop_bounds_exact'] for r in rows),
        'exact_complete_plans':sum(r['plans_exact'] for r in rows),
        'lookup_contains_dense_crop':sum(r['lookup_contains_dense_crop'] for r in rows),
        'max_crop_coordinate_delta_px':max(r['max_crop_coordinate_delta_px'] for r in rows),
        'lookup_outside_crop_total':sum(r['observed_support']['lookup']['outside_crop_count'] for r in rows),
        'lookup_additional_support_loss_total':sum(r['observed_support']['lookup']['additional_crop_support_loss_count'] for r in rows),
        'target_samples_total':sum(r['observed_support']['lookup']['target_sample_count'] for r in rows),
        'lookup_area_fraction_median':statistics.median(r['plans']['lookup']['area_fraction'] for r in rows)}
    for label in ('dense','lookup'):
        print('BLOCK_PLAN_TIMING' if args.compare_blocks else 'FRUSTUM_TIMING',('reference' if label=='dense' else 'blocked') if args.compare_blocks else label,'median_s=',round(statistics.median(r['timing_s'][label] for r in rows),6),
            'max_s=',round(max(r['timing_s'][label] for r in rows),6),flush=True)
    if args.compare_blocks:
        summary['comparison_basis']='same integer lookup, exact tile reductions versus whole-region reductions'
        summary['lookup_contains_reference_crop']=summary.pop('lookup_contains_dense_crop')
    print('BLOCK_PLAN_SUMMARY' if args.compare_blocks else 'FRUSTUM_SUMMARY',json.dumps(summary),flush=True)
    paths=(Path(__file__),ROOT/'vision_core/tools/stereo_frustum_lookup.py',ROOT/'vision_core/tools/analyze_target_stereo_crop.py')
    report={'status':'OFFLINE_STARTUP_FRUSTUM_LOOKUP_COMPARISON_ONLY','rows':rows,'summary':summary,'startup_s':startup_s,
        'block_comparison':args.compare_blocks,'block_index_startup_s':block_startup_s,
        'block_index_bytes':getattr(lookup,'index_bytes',0),'table_bytes':lookup.table.nbytes,'source_sha256':hashes,'inputs_unchanged':unchanged,
        'calibration_sha256':cal['sha'],'runtime_source_sha256':code,'code_sha256':{str(f):sha256(f) for f in paths},
        'actuator_access':False,'limitations':['Lookup integer lattice covers RGB cells touched by ROI, but is not a continuous distortion/uncertainty bound.',
            'Block comparison uses exactly the same integer lattice and padding; ordinary dense mode uses a different fractional sample grid.',
            'Observed support on this scene is not cropped-disparity equivalence.',
            'If crop boundaries differ, previous cropped-matcher results cannot be assumed unchanged.',
            'Startup cost/memory recorded separately; no cached prior-frame target, no runtime switch.']}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    print('INPUTS_UNCHANGED:',unchanged,'Saved:',args.output)


if __name__=='__main__':main()
