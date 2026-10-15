#!/usr/bin/env python3
"""Compare geometry after RGB vs geometry after stereo while RGB is running."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import statistics
import sys
import time
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'ros2/sie_ros2/sie_ros2'))
from benchmark_segmentation_onnx import load_cases,sha256,selection,geometry_checker
from benchmark_segmentation_input_size import inner_roi
from benchmark_concurrent_segmentation_threads import signature


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('evidence-root','model','report','ar-intrinsic','stereo-calibration','activation','output'):
        p.add_argument('--'+key,type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():p.error('new output required')
    import cv2
    import numpy as np
    from preview_mask_client import PreviewMaskClient
    from preview_mask_surface import prepare_stereo_points
    from analyze_ar0234_stereo_component_ambiguity import load_calibrations
    from benchmark_stereo_sgbm_parallel import runtime_parameters,compute_pair
    from benchmark_stereo_resolution import scaled_configs
    from vision_core.person_localization.diagnostic_stereo_resolution import half_disparity
    cv2.setNumThreads(1)
    cp=json.loads(args.activation.read_text())['calibration_profile'];ex=cp['ar0234_to_ov9281_physical_left_extrinsic'];extrinsic=Path(ex['path']).expanduser()
    if sha256(args.ar_intrinsic)!=cp['source_sha256']['ar0234_intrinsic'] or sha256(args.stereo_calibration)!=cp['source_sha256']['ov9281_stereo_calibration'] or sha256(extrinsic)!=ex['sha256']:
        raise ValueError('calibration/activation mismatch')
    cal=load_calibrations(args.ar_intrinsic,args.stereo_calibration,extrinsic)
    cal_rpc={k:cal[k] for k in ('P1','P2','R1','R','T','K','D','size')}
    cases=load_cases(args.evidence_root);sources={str(f):sha256(f) for f in (args.model,args.report,args.ar_intrinsic,args.stereo_calibration,args.activation,extrinsic)}
    for case in cases:sources.update(case['source_sha256'])
    configs,code=runtime_parameters();matchers=[cv2.StereoSGBM_create(**c) for c in scaled_configs(configs,.5)]
    prepared=[];check_geometry=geometry_checker()
    for case in cases:
        images=[]
        for side in ('left','right'):
            path=Path(case['image']).parent/f'ov9281_physical_{side}_rectified.jpg';sources[str(path)]=sha256(path)
            color=cv2.imread(str(path))
            if color is None or color.shape!=(800,1280,3):raise ValueError('invalid stereo evidence')
            images.append(cv2.resize(cv2.cvtColor(color,cv2.COLOR_BGR2GRAY),(640,400),interpolation=cv2.INTER_AREA))
        prepared.append((cv2.imread(case['image']),images))
    client=None;rows=[]
    try:
        client=PreviewMaskClient(sys.executable,ROOT/'vision_core/tools/control_preview_mask_worker.py',args.model,
            engine='onnxruntime',comparison_report=args.report,ort_threads=2)
        with ThreadPoolExecutor(max_workers=2) as stereo_pool,ThreadPoolExecutor(max_workers=1) as owner:
            def stereo_job(images,cid,early):
                start=time.perf_counter();maps,_=compute_pair(matchers,*images,executor=stereo_pool)
                left,valid=half_disparity(*maps);disparity=np.where(valid,left,np.nan)
                geometry_start=time.perf_counter()
                points=prepare_stereo_points(disparity,cal,scale=.5,cycle_id=cid) if early else None
                return maps,disparity,points,time.perf_counter()-start,time.perf_counter()-geometry_start
            # Warm transport/inference, matching and both geometry paths outside timing.
            for early in (False,True):
                rgb,images=prepared[0];cid='warm-'+str(early)
                future=owner.submit(stereo_job,images,cid,early)
                seg=client.infer(cid,rgb,diagnostic_mask_hashes=True)
                _,disparity,points,_,_=future.result()
                choice=selection(seg['instances'],cases[0]['timestamp']);candidate=choice.get('candidate')
                if candidate and check_geometry(candidate)['accepted']:
                    client.evaluate_selected(cid,disparity,cal_rpc,candidate,inner_roi(candidate),[.5,4.5],seg,
                        scale=.5,defer_local=True,prepare_local=False,prepared_geometry=points)
            for repeat in range(2):
                for index,(case,(rgb,images)) in enumerate(zip(cases,prepared)):
                    variants=('current','early_geometry') if (index+repeat)%2==0 else ('early_geometry','current')
                    result={};signatures={};maps={};timings={}
                    for label in variants:
                        cid=f'{case["cycle_id"]}-{repeat}-{label}';start=time.perf_counter()
                        future=owner.submit(stereo_job,images,cid,label=='early_geometry')
                        seg=client.infer(cid,rgb,diagnostic_mask_hashes=True)
                        choice=selection(seg['instances'],case['timestamp']);candidate=choice.get('candidate')
                        signed,disparity,points,stereo_s,geometry_s=future.result()
                        if candidate is None or not check_geometry(candidate)['accepted']:
                            result[label]={'status':'NO_CURRENT_ELIGIBLE_TARGET'};fusion_s=0.
                        else:
                            fusion=client.evaluate_selected(cid,disparity,cal_rpc,candidate,inner_roi(candidate),[.5,4.5],seg,
                                scale=.5,defer_local=True,prepare_local=False,prepared_geometry=points)
                            if 'mask_depth' not in fusion:raise ValueError('fusion failed: '+str(fusion))
                            result[label]=fusion['mask_depth'];fusion_s=fusion['round_trip_s']
                        timings[label]={'joint_with_depth_s':time.perf_counter()-start,'rgb_rpc_s':seg['round_trip_s'],
                            'stereo_geometry_job_s':stereo_s,'early_geometry_s':geometry_s,'fusion_rpc_s':fusion_s}
                        signatures[label]=signature(seg);maps[label]=signed
                        if seg['rgb_payload_sha256']!=hashlib.sha256(rgb.tobytes()).hexdigest():raise ValueError('RGB changed')
                    exact=(result['current']==result['early_geometry'] and signatures['current']==signatures['early_geometry']
                        and all(np.array_equal(a,b) for a,b in zip(maps['current'],maps['early_geometry'])))
                    rows.append({'cycle_id':case['cycle_id'],'repeat':repeat,'order':variants,'timing':timings,
                        'exact':exact,'depth':result})
                    if not exact:raise ValueError('geometry scheduling changed output; no runtime switch')
        unchanged=all(sha256(path)==digest for path,digest in sources.items())
        if not unchanged:raise ValueError('source inputs changed')
        for label in ('current','early_geometry'):
            print('GEOMETRY_OVERLAP',label,'joint_median_s=',round(statistics.median(r['timing'][label]['joint_with_depth_s'] for r in rows),4),
                'rgb_median_s=',round(statistics.median(r['timing'][label]['rgb_rpc_s'] for r in rows),4),
                'fusion_median_s=',round(statistics.median(r['timing'][label]['fusion_rpc_s'] for r in rows),4),flush=True)
        paths=(Path(__file__),ROOT/'vision_core/tools/preview_mask_surface.py',ROOT/'vision_core/tools/control_preview_mask_worker.py',ROOT/'ros2/sie_ros2/sie_ros2/preview_mask_client.py')
        report={'status':'OFFLINE_PREPARED_GEOMETRY_OVERLAP_ONLY','rows':rows,'source_sha256':sources,
            'inputs_unchanged':unchanged,'calibration_sha256':cal['sha'],'runtime_source_sha256':code,
            'code_sha256':{str(path):sha256(path) for path in paths},'model':client.engine_metadata,
            'actuator_access':False,'limitations':['No capture/rectification/local fits in timing; predecoded rectified JPEGs.',
                'Benchmark RGB hashing and ROI helper overhead enabled in both variants.',
                'Early prepared point transfer can cost more bytes than disparity transfer.',
                'Exact agreement is not physical depth accuracy or live latency certification.']}
        args.output.parent.mkdir(parents=True,exist_ok=True)
        with args.output.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
        print('GEOMETRY_EXACT:',len(rows),'/',len(rows),'INPUTS_UNCHANGED:',unchanged,'Saved:',args.output)
    finally:
        if client is not None:client.close()


if __name__=='__main__':main()
