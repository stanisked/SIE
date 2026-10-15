#!/usr/bin/env python3
"""Actual companion inference alongside two SGBM tasks, on frozen JPEGs only."""
from concurrent.futures import ThreadPoolExecutor
import argparse
import hashlib
import json
from pathlib import Path
import statistics
import sys
import time
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'ros2/sie_ros2/sie_ros2'))
from benchmark_segmentation_onnx import load_cases,sha256


def signature(result):
    if result.get('status')!='CURRENT_INSTANCES_DIAGNOSTIC_ONLY':raise ValueError('inference refused')
    hashes=result.get('instance_mask_sha256')
    if not isinstance(hashes,list) or len(hashes)!=len(result['instances']):raise ValueError('missing mask evidence')
    return {'instances':result['instances'],'mask_sha256':hashes, 'rgb_payload_sha256':result.get('rgb_payload_sha256')}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('evidence-root','model','report','output'):
        p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--experiment',choices=['helper_threads','rgb_transport'],default='helper_threads')
    args=p.parse_args()
    if args.output.exists():p.error('new output required')
    import cv2
    import numpy as np
    from preview_mask_client import PreviewMaskClient
    from benchmark_stereo_sgbm_parallel import runtime_parameters,compute_pair
    from benchmark_stereo_resolution import scaled_configs
    cv2.setNumThreads(1)
    cases=load_cases(args.evidence_root);sources={str(f):sha256(f) for f in (args.model,args.report)}
    for case in cases:sources.update(case['source_sha256'])
    configs,code=runtime_parameters();matchers=[cv2.StereoSGBM_create(**c) for c in scaled_configs(configs,.5)]
    prepared=[]
    for case in cases:
        stereo=[]
        for side in ('left','right'):
            path=Path(case['image']).parent/f'ov9281_physical_{side}_rectified.jpg';sources[str(path)]=sha256(path)
            image=cv2.imread(str(path))
            if image is None or image.shape!=(800,1280,3):raise ValueError('invalid stereo image')
            stereo.append(cv2.resize(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),(640,400),interpolation=cv2.INTER_AREA))
        rgb=cv2.imread(case['image'])
        if rgb is None or rgb.shape!=(1200,1920,3):raise ValueError('invalid RGB')
        prepared.append((rgb,stereo))
    clients={};rows=[];metadata={}
    variants=(('npz',None,'npz'),('mmap',None,'mmap')) if args.experiment=='rgb_transport' else (('current',None,'npz'),('helper1',1,'npz'))
    labels=tuple(v[0] for v in variants)
    try:
        for label,threads,transport in variants:
            client=PreviewMaskClient(sys.executable,ROOT/'vision_core/tools/control_preview_mask_worker.py',args.model,
                engine='onnxruntime',comparison_report=args.report,ort_threads=2,cpu_helper_threads=threads,rgb_transport=transport)
            clients[label]=client;metadata[label]=client.engine_metadata
        with ThreadPoolExecutor(max_workers=2) as stereo_pool,ThreadPoolExecutor(max_workers=1) as owner:
            for label,client in clients.items():
                future=owner.submit(compute_pair,matchers,*prepared[0][1],executor=stereo_pool)
                client.infer('warm-'+label,prepared[0][0],diagnostic_mask_hashes=True);future.result()
            for repeat in range(2):
                for index,(case,(rgb,stereo)) in enumerate(zip(cases,prepared)):
                    order=labels if (repeat+index)%2==0 else labels[::-1]
                    results={};maps={};timing={}
                    for label in order:
                        start=time.perf_counter()
                        future=owner.submit(compute_pair,matchers,*stereo,executor=stereo_pool)
                        results[label]=clients[label].infer(f'{case["cycle_id"]}-{repeat}-{label}',rgb,diagnostic_mask_hashes=True)
                        maps[label],stereo_s=future.result()
                        timing[label]={'joint_s':time.perf_counter()-start,'inference_s':results[label]['mask_s'],
                            'rgb_round_trip_s':results[label]['round_trip_s'],'stereo_s':stereo_s}
                    expected_rgb_sha=hashlib.sha256(rgb.tobytes()).hexdigest()
                    if any(r.get('rgb_payload_sha256')!=expected_rgb_sha for r in results.values()):raise ValueError('RGB payload changed')
                    exact=signature(results[labels[0]])==signature(results[labels[1]])
                    stereo_exact=all(np.array_equal(a,b) for a,b in zip(maps[labels[0]],maps[labels[1]]))
                    rows.append({'cycle_id':case['cycle_id'],'repeat':repeat,'order':order,'timing':timing,
                        'rgb_instances_and_masks_exact':exact,'stereo_maps_exact':stereo_exact})
                    if not exact or not stereo_exact:raise ValueError('numerical difference; do not change live threads')
        unchanged=all(sha256(path)==digest for path,digest in sources.items())
        if not unchanged:raise ValueError('inputs changed')
        for label in clients:
            print('RGB_TRANSPORT' if args.experiment=='rgb_transport' else 'THREAD_BUDGET',label,'metadata=',metadata[label]['cpu_helper_threads'],
                'joint_median_s=',round(statistics.median(r['timing'][label]['joint_s'] for r in rows),4),
                'rgb_median_s=',round(statistics.median(r['timing'][label]['rgb_round_trip_s'] for r in rows),4),
                'stereo_median_s=',round(statistics.median(r['timing'][label]['stereo_s'] for r in rows),4),flush=True)
        report={'status':'OFFLINE_CONCURRENT_TRANSPORT_COMPARISON_ONLY' if args.experiment=='rgb_transport' else 'OFFLINE_CONCURRENT_THREAD_COMPARISON_ONLY','experiment':args.experiment,'rows':rows,'metadata':metadata,
            'source_sha256':sources,'inputs_unchanged':unchanged,'runtime_source_sha256':code,
            'code_sha256':{str(f):sha256(f) for f in (Path(__file__),ROOT/'vision_core/tools/control_preview_mask_worker.py',ROOT/'ros2/sie_ros2/sie_ros2/preview_mask_client.py',ROOT/'vision_core/person_localization/rgb_shared_buffer.py')},
            'actuator_access':False,'limitations':['No capture, local fits, projection or depth calculation in joint timer.',
                'Mask hashing enabled for comparison adds IPC/work, identically in both variants.',
                'Two resident model workers; only one inference active at a time. No live thread switch.',
                'Exact masks/boxes/stereo on this scene are not universal accuracy validation.']}
        args.output.parent.mkdir(parents=True,exist_ok=True)
        with args.output.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
        print('EXACT:',len(rows),'/',len(rows),'INPUTS_UNCHANGED:',unchanged,'Saved:',args.output)
    finally:
        for client in clients.values():client.close()


if __name__=='__main__':main()
