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
from benchmark_segmentation_onnx import load_cases,sha256,selection,geometry_checker


def signature(result):
    if result.get('status')!='CURRENT_INSTANCES_DIAGNOSTIC_ONLY':raise ValueError('inference refused')
    hashes=result.get('instance_mask_sha256')
    if not isinstance(hashes,list) or len(hashes)!=len(result['instances']):raise ValueError('missing mask evidence')
    return {'instances':result['instances'],'mask_sha256':hashes, 'rgb_payload_sha256':result.get('rgb_payload_sha256')}


def numerical_difference(reference,candidate,stamp):
    a,b=signature(reference),signature(candidate)
    pairs=list(zip(a['instances'],b['instances']))
    box=[abs(float(x)-float(y)) for first,second in pairs for x,y in zip(first['bbox_xyxy_px'],second['bbox_xyxy_px'])]
    confidence=[abs(first['confidence']-second['confidence']) for first,second in pairs]
    first,second=selection(a['instances'],stamp),selection(b['instances'],stamp)
    geometry=geometry_checker()
    return {'instance_count_reference':len(a['instances']),'instance_count_candidate':len(b['instances']),
        'instances_exact':a['instances']==b['instances'],'mask_hashes_exact':a['mask_sha256']==b['mask_sha256'],
        'mask_hash_mismatches_in_result_order':sum(x!=y for x,y in zip(a['mask_sha256'],b['mask_sha256']))+abs(len(a['mask_sha256'])-len(b['mask_sha256'])),
        'max_bbox_coordinate_delta_px_in_result_order':max(box) if box else None,
        'max_confidence_delta_in_result_order':max(confidence) if confidence else None,
        'selection_disposition_changed':first['disposition']!=second['disposition'],
        'geometry_status_changed':(geometry(first.get('candidate')) or {}).get('accepted')!=(geometry(second.get('candidate')) or {}).get('accepted'),
        'reference_selection':first,'candidate_selection':second,
        'reference_instances':a['instances'],'candidate_instances':b['instances'],
        'reference_mask_sha256':a['mask_sha256'],'candidate_mask_sha256':b['mask_sha256'],
        'comparison_basis':'Model result order; no object identity inferred from index.'}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('evidence-root','model','report','output'):
        p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--experiment',choices=['helper_threads','rgb_transport','active_ort'],default='helper_threads')
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
    if args.experiment=='active_ort':
        variants=(('auto',None,'npz',None),('ort1',None,'npz',1),('ort2',None,'npz',2))
    elif args.experiment=='rgb_transport':
        variants=(('npz',None,'npz',None),('mmap',None,'mmap',None))
    else:
        variants=(('current',None,'npz',None),('helper1',1,'npz',None))
    labels=tuple(v[0] for v in variants)
    try:
        for label,threads,transport,active in variants:
            client=PreviewMaskClient(sys.executable,ROOT/'vision_core/tools/control_preview_mask_worker.py',args.model,
                engine='onnxruntime',comparison_report=args.report,ort_threads=2,cpu_helper_threads=threads,rgb_transport=transport,active_ort_threads=active)
            clients[label]=client;metadata[label]=client.engine_metadata
        with ThreadPoolExecutor(max_workers=2) as stereo_pool,ThreadPoolExecutor(max_workers=1) as owner:
            for label,client in clients.items():
                future=owner.submit(compute_pair,matchers,*prepared[0][1],executor=stereo_pool)
                client.infer('warm-'+label,prepared[0][0],diagnostic_mask_hashes=True);future.result()
            for repeat in range(2):
                for index,(case,(rgb,stereo)) in enumerate(zip(cases,prepared)):
                    offset=(repeat+index)%len(labels)
                    order=labels[offset:]+labels[:offset]
                    if repeat%2:order=order[::-1]
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
                    exact=all(signature(results[labels[0]])==signature(results[label]) for label in labels[1:])
                    stereo_exact=all(np.array_equal(a,b) for label in labels[1:] for a,b in zip(maps[labels[0]],maps[label]))
                    differences={label:numerical_difference(results[labels[0]],results[label],case['timestamp']) for label in labels[1:]}
                    rows.append({'cycle_id':case['cycle_id'],'repeat':repeat,'order':order,'timing':timing,
                        'rgb_instances_and_masks_exact':exact,'stereo_maps_exact':stereo_exact,'differences':differences})
                    # Preserve differences as experimental evidence, never activate a candidate.
        unchanged=all(sha256(path)==digest for path,digest in sources.items())
        if not unchanged:raise ValueError('inputs changed')
        for label in clients:
            print('ACTIVE_ORT_BUDGET' if args.experiment=='active_ort' else 'RGB_TRANSPORT' if args.experiment=='rgb_transport' else 'THREAD_BUDGET',label,'metadata=',metadata[label].get('active_ort_session') if args.experiment=='active_ort' else metadata[label]['cpu_helper_threads'],
                'joint_median_s=',round(statistics.median(r['timing'][label]['joint_s'] for r in rows),4),
                'rgb_median_s=',round(statistics.median(r['timing'][label]['rgb_round_trip_s'] for r in rows),4),
                'stereo_median_s=',round(statistics.median(r['timing'][label]['stereo_s'] for r in rows),4),flush=True)
        for label in labels[1:]:
            differences=[r['differences'][label] for r in rows]
            summary={'instance_different_rows':sum(not d['instances_exact'] for d in differences),
                'mask_different_rows':sum(not d['mask_hashes_exact'] for d in differences),
                'count_different_rows':sum(d['instance_count_reference']!=d['instance_count_candidate'] for d in differences),
                'selection_status_changes':sum(d['selection_disposition_changed'] for d in differences),
                'geometry_status_changes':sum(d['geometry_status_changed'] for d in differences),
                'max_bbox_delta_px_in_result_order':max((d['max_bbox_coordinate_delta_px_in_result_order'] or 0) for d in differences),
                'max_confidence_delta_in_result_order':max((d['max_confidence_delta_in_result_order'] or 0) for d in differences)}
            print('NUMERICAL_DIFF',label,json.dumps(summary),flush=True)
        all_exact=all(r['rgb_instances_and_masks_exact'] and r['stereo_maps_exact'] for r in rows)
        report={'all_exact':all_exact,'runtime_switch_authorized':False,'status':'OFFLINE_ACTIVE_ORT_COMPARISON_ONLY' if args.experiment=='active_ort' else 'OFFLINE_CONCURRENT_TRANSPORT_COMPARISON_ONLY' if args.experiment=='rgb_transport' else 'OFFLINE_CONCURRENT_THREAD_COMPARISON_ONLY','experiment':args.experiment,'rows':rows,'metadata':metadata,
            'source_sha256':sources,'inputs_unchanged':unchanged,'runtime_source_sha256':code,
            'code_sha256':{str(f):sha256(f) for f in (Path(__file__),ROOT/'vision_core/tools/control_preview_mask_worker.py',ROOT/'ros2/sie_ros2/sie_ros2/preview_mask_client.py',ROOT/'vision_core/person_localization/rgb_shared_buffer.py',ROOT/'vision_core/tools/active_ort_session.py')},
            'actuator_access':False,'limitations':['No capture, local fits, projection or depth calculation in joint timer.',
                'Mask hashing enabled for comparison adds IPC/work, identically in both variants.',
                'One resident worker per variant; only one inference active at a time. No live thread switch.',
                'Exact masks/boxes/stereo on this scene are not universal accuracy validation.']}
        args.output.parent.mkdir(parents=True,exist_ok=True)
        with args.output.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
        print('EXACT:',sum(r['rgb_instances_and_masks_exact'] and r['stereo_maps_exact'] for r in rows),'/',len(rows),'INPUTS_UNCHANGED:',unchanged,'Saved:',args.output)
    finally:
        for client in clients.values():client.close()


if __name__=='__main__':main()
