#!/usr/bin/env python3
"""Compare reference/optimized mask ownership on identical frozen inputs."""
import argparse
import hashlib
import json
from pathlib import Path
import statistics
import sys
import time
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('evidence-root','ar-intrinsic','stereo-calibration','activation','output'):
        p.add_argument('--'+key,type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():p.error('new output required')
    import cv2
    import numpy as np
    from preview_mask_surface import evaluate
    from analyze_ar0234_stereo_component_ambiguity import load_calibrations,sha256
    from replay_shadow_mask_depth_range import replay
    cp=json.loads(args.activation.read_text())['calibration_profile'];ex=cp['ar0234_to_ov9281_physical_left_extrinsic'];extrinsic=Path(ex['path']).expanduser()
    if sha256(args.ar_intrinsic)!=cp['source_sha256']['ar0234_intrinsic'] or sha256(args.stereo_calibration)!=cp['source_sha256']['ov9281_stereo_calibration'] or sha256(extrinsic)!=ex['sha256']:
        raise ValueError('calibration/activation mismatch')
    cal=load_calibrations(args.ar_intrinsic,args.stereo_calibration,extrinsic)
    hashes={};rows=[];cv2.setNumThreads(1)
    for path in sorted(args.evidence_root.rglob('audit.json')):
        audit=json.loads(path.read_text());cid=audit['cycle_id']
        verified=replay(path,cal,.5,4.5)
        if 'mask_sha256' not in verified:raise ValueError('expected verified saved mask')
        folder=path.parent;mp=folder/f'{cid}.png';dp=folder/'disparity_lr_consistent_px.npy'
        for f in (path,mp,dp,folder/f'{cid}.json',folder/'ar0234_source.jpg'):hashes[str(f)]=sha256(f)
        mask=cv2.imread(str(mp),cv2.IMREAD_GRAYSCALE)==255
        disparity=np.load(dp,allow_pickle=False)
        roi=audit['stereo_geometry']['ar_inner_roi_xyxy_px']
        # Stored full-resolution arrays are shared, not resized or recomputed.
        # This isolates mask ownership, independent of stereo matching speed.
        for prepare in (False,True):
            for backend in ('reference_numpy','opencv'):
                evaluate(disparity,mask,cal,roi,defer_local=True,prepare_local=prepare,ownership_backend=backend)
            for repeat in range(2):
                result={};samples={};timing={}
                order=('reference_numpy','opencv') if repeat==0 else ('opencv','reference_numpy')
                for backend in order:
                    start=time.perf_counter()
                    result[backend],samples[backend]=evaluate(disparity,mask,cal,roi,defer_local=True,prepare_local=prepare,ownership_backend=backend)
                    timing[backend]=time.perf_counter()-start
                if result['reference_numpy']!=result['opencv']:raise ValueError('depth/status mismatch')
                if prepare:
                    for key in samples['reference_numpy']:
                        if not np.array_equal(samples['reference_numpy'][key],samples['opencv'][key],equal_nan=True):raise ValueError('local sample mismatch')
                rows.append({'cycle_id':cid,'prepare_local':prepare,'repeat':repeat,'timing_s':timing,'exact':True})
    if not rows:raise ValueError('no verified cycles')
    unchanged=all(sha256(Path(f))==digest for f,digest in hashes.items())
    if not unchanged:raise ValueError('inputs changed')
    for prepare in (False,True):
        group=[r for r in rows if r['prepare_local']==prepare]
        print('OWNERSHIP prepare_local=',prepare,'exact=',len(group),'/',len(group),
              'reference_s=',round(statistics.median(r['timing_s']['reference_numpy'] for r in group),6),
              'optimized_s=',round(statistics.median(r['timing_s']['opencv'] for r in group),6))
    code=[Path(__file__),Path(__file__).with_name('preview_mask_surface.py'),ROOT/'vision_core/person_localization/instance_mask_stereo_fusion.py']
    report={'status':'OFFLINE_OWNERSHIP_EQUIVALENCE_ONLY','rows':rows,'inputs_unchanged':unchanged,'source_sha256':hashes,
        'code_sha256':{str(f):sha256(f) for f in code},'calibration_sha256':cal['sha'],
        'environment':{'opencv':cv2.__version__,'numpy':np.__version__},'actuator_access':False,
        'limitations':['Frozen full-resolution disparity, no live latency claim.','No physical accuracy validation or quality policy change.']}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    print('INPUTS_UNCHANGED:',unchanged,'Saved:',args.output)


if __name__=='__main__':main()
