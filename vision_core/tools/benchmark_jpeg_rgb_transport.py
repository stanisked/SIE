#!/usr/bin/env python3
"""Compare original JPEG vs decoded mmap input on frozen frames; no live switch."""
import argparse
import hashlib
import json
from pathlib import Path
import statistics
import sys
import time
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'ros2/sie_ros2/sie_ros2'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('evidence-root','model','report','ar-intrinsic','stereo-calibration','activation','output'):
        p.add_argument('--'+key,type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():p.error('new output required')
    import cv2
    import numpy as np
    from preview_mask_client import PreviewMaskClient
    from benchmark_segmentation_onnx import load_cases,sha256,selection,geometry_checker
    from benchmark_concurrent_segmentation_threads import signature,numerical_difference
    from analyze_ar0234_stereo_component_ambiguity import load_calibrations
    cp=json.loads(args.activation.read_text())['calibration_profile'];ex=cp['ar0234_to_ov9281_physical_left_extrinsic'];extrinsic=Path(ex['path']).expanduser()
    for path,expected in ((args.ar_intrinsic,cp['source_sha256']['ar0234_intrinsic']),
                          (args.stereo_calibration,cp['source_sha256']['ov9281_stereo_calibration']),(extrinsic,ex['sha256'])):
        if sha256(path)!=expected:raise ValueError('calibration/activation SHA mismatch')
    cal=load_calibrations(args.ar_intrinsic,args.stereo_calibration,extrinsic)
    cases=load_cases(args.evidence_root);sources={str(f):sha256(f) for f in (args.model,args.report,args.activation,args.ar_intrinsic,args.stereo_calibration,extrinsic)}
    prepared=[]
    for case in cases:
        sources.update(case['source_sha256']);folder=Path(case['image']).parent
        audit=json.loads((folder/'audit.json').read_text())
        if audit['calibration']!=cal['sha']:raise ValueError('saved calibration mismatch')
        disparity_path=folder/'disparity_lr_consistent_px.npy';sources[str(disparity_path)]=sha256(disparity_path)
        disparity=np.load(disparity_path,allow_pickle=False)
        if disparity.shape!=(800,1280):raise ValueError('expected original full-resolution saved disparity')
        payload=Path(case['image']).read_bytes();decoded=cv2.imdecode(np.frombuffer(payload,np.uint8),cv2.IMREAD_COLOR)
        if decoded is None or decoded.shape!=(1200,1920,3):raise ValueError('invalid saved RGB JPEG')
        expected=hashlib.sha256(decoded.tobytes()).hexdigest()
        prepared.append((case,payload,expected,disparity,audit['stereo_geometry']['ar_inner_roi_xyxy_px']))
    cv2.setNumThreads(1);rows=[];geometry=geometry_checker()
    client=PreviewMaskClient(sys.executable,ROOT/'vision_core/tools/control_preview_mask_worker.py',args.model,
        engine='onnxruntime',comparison_report=args.report,ort_threads=2,rgb_transport='mmap')
    try:
        payload=prepared[0][1]
        for _ in range(2):
            client.infer('warm-reference',cv2.imdecode(np.frombuffer(payload,np.uint8),cv2.IMREAD_COLOR))
            client.infer_jpeg('warm-jpeg',payload)
        for repeat in range(2):
            for index,(case,payload,expected,disparity,roi) in enumerate(prepared):
                results={};depths={};timing={};choices={}
                order=('reference','jpeg') if (repeat+index)%2==0 else ('jpeg','reference')
                for label in order:
                    cid=f'jpeg-comparison-{repeat}-{index}-{label}'
                    started=time.perf_counter()
                    if label=='reference':
                        rgb=cv2.imdecode(np.frombuffer(payload,np.uint8),cv2.IMREAD_COLOR)
                        result=client.infer(cid,rgb,diagnostic_mask_hashes=True)
                    else:result=client.infer_jpeg(cid,payload,diagnostic_mask_hashes=True)
                    timing[label]={'decode_transport_infer_s':time.perf_counter()-started,'rpc_s':result['round_trip_s'],
                        'worker_jpeg_decode_s':result.get('worker_jpeg_decode_s')}
                    results[label]=result
                    if result.get('rgb_payload_sha256')!=expected:raise ValueError('decoded RGB pixel mismatch')
                    chosen=selection(result['instances'],case['timestamp']);choices[label]=chosen
                    geom=geometry(chosen.get('candidate'))
                    if chosen['disposition']=='OBSERVED' and geom and geom['accepted']:
                        value=client.evaluate_selected(cid,disparity,cal,chosen['candidate'],roi,[.5,4.5],result,
                            scale=1.,defer_local=True,prepare_local=False)
                        depths[label]=value['mask_depth']
                    else:depths[label]={'status':'NOT_EVALUATED_CURRENT_TARGET_UNAVAILABLE'}
                difference=numerical_difference(results['reference'],results['jpeg'],case['timestamp'])
                exact=signature(results['reference'])==signature(results['jpeg'])
                row={'source_cycle_id':case['cycle_id'],'repeat':repeat,'order':order,'timing':timing,
                    'decoded_rgb_exact':True,'instances_masks_exact':exact,'difference':difference,
                    'selection_exact':choices['reference']==choices['jpeg'],
                    'depth_exact':depths['reference']==depths['jpeg'],'depths':depths,
                    'jpeg_bytes':len(payload),'decoded_rgb_bytes':1200*1920*3}
                rows.append(row)
        unchanged=all(sha256(Path(f))==digest for f,digest in sources.items())
        if not unchanged:raise ValueError('source inputs changed')
        summary={'comparisons':len(rows),'decoded_rgb_exact':sum(r['decoded_rgb_exact'] for r in rows),
            'instances_masks_exact':sum(r['instances_masks_exact'] for r in rows),
            'selection_exact':sum(r['selection_exact'] for r in rows),'depth_exact':sum(r['depth_exact'] for r in rows),
            'evaluated_depth_rows':sum('candidate_depth_m' in r['depths']['reference'] for r in rows)}
        for label in ('reference','jpeg'):
            values=[r['timing'][label]['decode_transport_infer_s'] for r in rows]
            print('JPEG_TRANSPORT',label,'median_s=',round(statistics.median(values),6),'max_s=',round(max(values),6),flush=True)
        print('JPEG_QUALITY',json.dumps(summary),flush=True)
        report={'status':'OFFLINE_JPEG_TRANSPORT_COMPARISON_ONLY','rows':rows,'summary':summary,
            'inputs_unchanged':unchanged,'source_sha256':sources,'calibration_sha256':cal['sha'],
            'environment':{'opencv':cv2.__version__,'numpy':np.__version__},
            'engine':client.engine_metadata,'runtime_switch_authorized':False,'actuator_access':False,
            'code_sha256':{str(f):sha256(f) for f in (Path(__file__),ROOT/'vision_core/tools/control_preview_mask_worker.py',ROOT/'ros2/sie_ros2/sie_ros2/preview_mask_client.py',ROOT/'vision_core/person_localization/jpeg_frame_transport.py')},
            'limitations':['Single isolated NN worker, no camera capture, no concurrent stereo or exposure-to-command timing.',
                'Reference timing includes parent JPEG decode and mmap RPC; candidate includes worker decode and JPEG RPC.',
                'Diagnostic pixel/mask fingerprint work is included in timings for both variants.',
                'Depth uses same frozen full-resolution disparity and saved inner ROI, not live half-resolution cropped matching.',
                'Original saved JPEG bytes, no re-encoding; numerical differences are retained, no acceptance tolerance.']}
        args.output.parent.mkdir(parents=True,exist_ok=True)
        with args.output.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
        print('INPUTS_UNCHANGED:',unchanged,'Saved:',args.output)
    finally:client.close()

if __name__=='__main__':main()
