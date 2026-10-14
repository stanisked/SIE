#!/usr/bin/env python3
"""Offline resolution challenge on frozen stereo pairs and current-frame masks.

Recompute full-resolution reference from the SAME JPEGs; saved live NPY maps are
not bitwise references for decoded JPEGs. No camera, runtime/profile change or
physical accuracy certification. Do not pick a winning scale automatically.
"""
from __future__ import annotations
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import statistics
import time
import cv2
import numpy as np
from benchmark_stereo_sgbm_parallel import runtime_parameters, compute_pair, consistency, sha256
from analyze_ar0234_stereo_component_ambiguity import load_calibrations
from replay_shadow_mask_depth_range import replay
from preview_mask_surface import evaluate


def scaled_configs(configs, scale):
    if scale not in (1.,.5,.25):
        raise ValueError('unsupported diagnostic scale')
    count=int(math.ceil(configs[0]['numDisparities']*scale/16)*16)
    result=[dict(c,numDisparities=count) for c in configs]
    result[1]['minDisparity']=-count
    # Block/speckle parameters stay unchanged, so their physical image support
    # increases at smaller scales. Record this algorithm change explicitly.
    return result


def paired_delta(reference, candidate, scale, fb):
    h,w=candidate.shape
    y,x=np.indices((h,w))
    fy=np.rint((y+.5)/scale-.5).astype(int)
    fx=np.rint((x+.5)/scale-.5).astype(int)
    full=reference[fy,fx]
    valid=np.isfinite(full)&np.isfinite(candidate)&(full>0)&(candidate>0)
    delta=fb*scale/candidate[valid]-fb/full[valid]
    return {'common_valid_pixels':int(valid.sum()),
            'reference_valid_at_candidate_centres':int(np.isfinite(full).sum()),
            'candidate_valid_pixels':int(np.isfinite(candidate).sum()),
            'signed_depth_delta_median_m':float(np.median(delta)) if len(delta) else None,
            'absolute_depth_delta_p90_m':float(np.percentile(np.abs(delta),90)) if len(delta) else None,
            'comparison':'All common LR-valid pixels, nearest full-resolution pixel centre; not physical truth.'}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--evidence-root',type=Path,required=True)
    p.add_argument('--ar-intrinsic',type=Path,required=True)
    p.add_argument('--stereo-calibration',type=Path,required=True)
    p.add_argument('--activation',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--cycles',type=int,default=0,help='0: every saved cycle')
    args=p.parse_args()
    if args.output.exists() or args.cycles<0:
        p.error('use new output and nonnegative cycle count')
    activation=json.loads(args.activation.read_text())
    cp=activation['calibration_profile']; ex=cp['ar0234_to_ov9281_physical_left_extrinsic']
    extrinsic=Path(ex['path']).expanduser()
    if (sha256(args.ar_intrinsic)!=cp['source_sha256']['ar0234_intrinsic'] or
        sha256(args.stereo_calibration)!=cp['source_sha256']['ov9281_stereo_calibration'] or
        sha256(extrinsic)!=ex['sha256']):
        raise ValueError('activation/calibration mismatch')
    cal=load_calibrations(args.ar_intrinsic,args.stereo_calibration,extrinsic)
    configs,code_sha=runtime_parameters()
    files=sorted(args.evidence_root.rglob('audit.json'))
    if not files:
        p.error('no frozen audit files; compact control-preview journals do not contain images')
    if args.cycles:
        files=files[:args.cycles]
    rows=[]; sources={}; previous_threads=cv2.getNumThreads()
    try:
        cv2.setNumThreads(1)
        with ThreadPoolExecutor(max_workers=2) as pool:
            matchers={s:[cv2.StereoSGBM_create(**c) for c in scaled_configs(configs,s)] for s in (1.,.5,.25)}
            warmed=set()
            for path in files:
                audit=json.loads(path.read_text()); cid=audit['cycle_id']
                if path.parent.name!=cid:
                    raise ValueError('cycle directory mismatch')
                original=replay(path,cal,.5,4.5)  # verifies mask/source/model/calibration binding
                if 'mask_sha256' not in original:
                    rows.append({'cycle_id':cid,'status':'SKIPPED_NO_VERIFIED_MASK',
                                 'reason':original['mask_depth']['status']})
                    continue
                paths=[path.parent/f'ov9281_physical_{side}_rectified.jpg' for side in ('left','right')]
                mask_path=path.parent/f'{cid}.png'
                for source in [path,*paths,mask_path,path.parent/f'{cid}.json',
                               path.parent/'ar0234_source.jpg',path.parent/'disparity_lr_consistent_px.npy']:
                    sources[str(source)]=sha256(source)
                color=[cv2.imread(str(f)) for f in paths]
                if any(a is None or a.shape!=(800,1280,3) for a in color):
                    raise ValueError('expected physical left/right rectified evidence')
                images=[cv2.cvtColor(a,cv2.COLOR_BGR2GRAY) for a in color]
                mask=cv2.imread(str(mask_path),cv2.IMREAD_GRAYSCALE)==255
                baseline=None; full_diag=None
                for scale in (1.,.5,.25):
                    pair=images if scale==1 else [cv2.resize(a,None,fx=scale,fy=scale,interpolation=cv2.INTER_AREA) for a in images]
                    if scale not in warmed:
                        compute_pair(matchers[scale],*pair,executor=pool)
                        warmed.add(scale)
                    started=time.monotonic()
                    maps,match_s=compute_pair(matchers[scale],*pair,executor=pool)
                    lr=consistency(maps,tolerance=1.5*scale)
                    disparity=np.where(lr,maps[0].astype(np.float32)/16.,np.nan)
                    stereo_s=time.monotonic()-started
                    start_diag=time.monotonic()
                    diagnostic=evaluate(disparity,mask,cal,audit['stereo_geometry']['ar_inner_roi_xyxy_px'],scale=scale)
                    diag_s=time.monotonic()-start_diag
                    depth=diagnostic['mask_depth']; held=diagnostic['local_surface']['pooled_held_out_axial_residual']
                    row={'cycle_id':cid,'scale':scale,'size':[pair[0].shape[1],pair[0].shape[0]],
                         'match_pair_s':match_s,'match_and_lr_s':stereo_s,'mask_surface_evaluation_s':diag_s,
                         'lr_tolerance_small_px':1.5*scale,'mask_surface':diagnostic}
                    if scale==1:
                        baseline=disparity; full_diag=depth
                    else:
                        row['paired_stereo_delta']=paired_delta(baseline,disparity,scale,abs(float(cal['P2'][0,3]))/1000.)
                        a,b=depth.get('candidate_depth_m'),full_diag.get('candidate_depth_m')
                        row['mask_median_delta_m']=a-b if a is not None and b is not None else None
                    rows.append(row)
                    print(cid[-8:],'scale=',scale,'stereo_s=',round(stereo_s,4),
                          depth['status'],'n=',depth.get('selected_sample_count'),
                          'depth_m=',depth.get('candidate_depth_m'),'local_MAD_mm=',
                          round(held['mad_m']*1000,1) if 'mad_m' in held else None,flush=True)
    finally:
        cv2.setNumThreads(previous_threads)
    unchanged=all(sha256(Path(path))==value for path,value in sources.items())
    report={'status':'OFFLINE_RESOLUTION_CHALLENGE_ONLY','rows':rows,'inputs_unchanged':unchanged,
            'calibration_sha256':cal['sha'],'activation_sha256':sha256(args.activation),
            'source_sha256':sources,'runtime_source_sha256':code_sha,'tool_sha256':sha256(Path(__file__)),
            'analysis_source_sha256':{str(f):sha256(f) for f in (
                Path(__file__).with_name('preview_mask_surface.py'),
                Path(__file__).with_name('compare_shadow_mask_depth_median.py'),
                Path(__file__).resolve().parents[1]/'person_localization/local_surface_diagnostic.py',
                Path(__file__).resolve().parents[1]/'person_localization/instance_mask_stereo_fusion.py')},
            'environment':{'opencv':cv2.__version__,'numpy':np.__version__},
            'method':{'opencv_threads':1,'parallel_matchers':2,'scales':[1,.5,.25],
                      'matcher_configs':{str(s):scaled_configs(configs,s) for s in (1.,.5,.25)},
                      'depth_range_m':[.5,4.5],'image_resize':'INTER_AREA; pixel-centre geometry',
                      'no_fill_no_smoothing':True},
            'limitations':['JPEG baseline differs from original live arrays.',
                           'Full-resolution depth is a numerical reference, not physical truth.',
                           'Segmentation inference, capture, rectification and resize time excluded from stereo timer.',
                           '7px block and speckle parameters unchanged: spatial support changes with scale.',
                           'No automatic runtime switch, new quality threshold, Measurement or motor command.']}
    for scale in (1.,.5,.25):
        subset=[r for r in rows if r.get('scale')==scale]
        if subset:
            print('SCALE',scale,'median_stereo_s=',round(statistics.median(r['match_and_lr_s'] for r in subset),4),flush=True)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report,stream,indent=2,allow_nan=False);stream.write('\n')
    print('INPUTS_UNCHANGED:',unchanged,'Saved:',args.output)
    if not unchanged:
        raise SystemExit(2)


if __name__=='__main__':
    main()
