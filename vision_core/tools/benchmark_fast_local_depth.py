#!/usr/bin/env python3
"""Compare current-frame local diagnostics on verified saved masks. No new gate."""
import argparse
import hashlib
import json
from pathlib import Path
import statistics
import sys
import time
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))


def fast_stats(samples):
    import numpy as np
    from analyze_shadow_local_surfaces import coarse_coverage
    from vision_core.person_localization.fast_local_depth_consistency import analyze_fast_local_depth
    started=time.perf_counter()
    report,data=analyze_fast_local_depth(samples['u'],samples['v'],samples['d'],float(samples['fb']),
        stride=int(samples['stride']),tile_px=int(32*float(samples['scale'])))
    fitted=time.perf_counter()
    report['rgb_coverage']=coarse_coverage(samples['interior'],samples['roi'],samples['xy'],data['evaluated'])
    return report,{'fit_s':fitted-started,'coverage_s':time.perf_counter()-fitted}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('evidence-root','ar-intrinsic','stereo-calibration','activation','output'):
        p.add_argument('--'+key,type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():p.error('new output required')
    import cv2
    import numpy as np
    from preview_mask_surface import evaluate,local_surface_stats
    from replay_shadow_mask_depth_range import replay
    from analyze_ar0234_stereo_component_ambiguity import load_calibrations,sha256
    cp=json.loads(args.activation.read_text())['calibration_profile']
    ex=cp['ar0234_to_ov9281_physical_left_extrinsic'];extrinsic=Path(ex['path']).expanduser()
    if sha256(args.ar_intrinsic)!=cp['source_sha256']['ar0234_intrinsic'] or sha256(args.stereo_calibration)!=cp['source_sha256']['ov9281_stereo_calibration'] or sha256(extrinsic)!=ex['sha256']:
        raise ValueError('activation/calibration SHA mismatch')
    cal=load_calibrations(args.ar_intrinsic,args.stereo_calibration,extrinsic)
    sources={str(path):sha256(path) for path in (args.activation,args.ar_intrinsic,args.stereo_calibration,extrinsic)}
    rows=[];cv2.setNumThreads(1)
    for path in sorted(args.evidence_root.rglob('audit.json')):
        audit=json.loads(path.read_text());cid=audit['cycle_id']
        if path.parent.name!=cid:raise ValueError('source cycle mismatch')
        verified=replay(path,cal,.5,4.5)
        if 'mask_sha256' not in verified:raise ValueError('verified saved instance mask required')
        folder=path.parent;mp=folder/f'{cid}.png';dp=folder/'disparity_lr_consistent_px.npy'
        for source in (path,mp,dp,folder/f'{cid}.json',folder/'ar0234_source.jpg'):
            sources[str(source)]=sha256(source)
        mask=cv2.imread(str(mp),cv2.IMREAD_GRAYSCALE)==255
        disparity=np.load(dp,allow_pickle=False)
        roi=audit['stereo_geometry']['ar_inner_roi_xyxy_px']
        primary,samples=evaluate(disparity,mask,cal,roi,defer_local=True,prepare_local=True)
        fingerprint={key:hashlib.sha256(value.tobytes()).hexdigest() for key,value in samples.items()}
        # Both algorithms evaluate the identical owned samples and coarse RGB ROI.
        fast_stats(samples);local_surface_stats(samples)
        for repeat in range(2):
            outputs={};timing={};parts=None
            order=('reference','fast') if repeat==0 else ('fast','reference')
            for label in order:
                started=time.perf_counter()
                if label=='fast':outputs[label],parts=fast_stats(samples)
                else:outputs[label]=local_surface_stats(samples)
                timing[label]=time.perf_counter()-started
            if any(hashlib.sha256(value.tobytes()).hexdigest()!=fingerprint[key] for key,value in samples.items()):
                raise ValueError('owned sample mutation')
            row={'cycle_id':cid,'repeat':repeat,'order':order,'mask_sha256':sha256(mp),
                'calibration_sha256':cal['sha'],'legacy_mask_depth':primary['mask_depth'],
                'reference_local':outputs['reference'],'fast_local':outputs['fast'],
                'timing_s':timing,'fast_parts_s':parts}
            rows.append(row)
            if repeat==0:
                a=outputs['reference']['pooled_held_out_axial_residual'];b=outputs['fast']['pooled_held_out_axial_residual']
                print(cid[-8:],'legacy=',primary['mask_depth']['status'],
                    'reference_MAD_mm=',None if 'mad_m' not in a else round(a['mad_m']*1000,2),
                    'fast_MAD_mm=',None if 'mad_m' not in b else round(b['mad_m']*1000,2),
                    'fast_p90_mm=',None if 'absolute_p90_m' not in b else round(b['absolute_p90_m']*1000,2),
                    'fitted_fraction=',round(outputs['fast']['fitted_sample_fraction'],4),flush=True)
    if not rows:raise ValueError('no verified source cycles')
    unchanged=all(sha256(path)==digest for path,digest in sources.items())
    if not unchanged:raise ValueError('source evidence changed')
    summary={label:{'median_s':statistics.median(r['timing_s'][label] for r in rows),
        'max_s':max(r['timing_s'][label] for r in rows)} for label in ('reference','fast')}
    files=(Path(__file__),ROOT/'vision_core/person_localization/fast_local_depth_consistency.py',
        ROOT/'vision_core/person_localization/local_surface_diagnostic.py',Path(__file__).with_name('preview_mask_surface.py'),
        Path(__file__).with_name('analyze_shadow_local_surfaces.py'))
    report={'status':'OFFLINE_FAST_LOCAL_CONSISTENCY_DIAGNOSTIC_ONLY','rows':rows,'summary':summary,
        'inputs_unchanged':unchanged,'source_sha256':sources,'code_sha256':{str(f):sha256(f) for f in files},
        'environment':{'numpy':np.__version__,'opencv':cv2.__version__},
        'actuator_access':False,'metric_measurement_authorized':False,'quality_thresholds_added':False,
        'limitations':['Reference is robust one-fold Huber; candidate is two-fold ordinary least squares: no equivalence claim.',
            'Both use the same saved full-resolution disparity/mask samples; live half-resolution timing remains unmeasured.',
            'Coverage describes occupied RGB bins, not dense valid depth or semantic certainty.',
            'No physical ground truth, new quality threshold, control permission or live configuration change.']}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    print('FAST_LOCAL_TIMING:',summary)
    print('INPUTS_UNCHANGED:',unchanged,'Saved:',args.output)


if __name__=='__main__':main()
