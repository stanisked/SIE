#!/usr/bin/env python3
"""Isolated segmentation/SciPy worker; cycle-bound temporary arrays, no JPEGs."""
import argparse
import contextlib
import hashlib
import json
import os
from pathlib import Path
import sys
import time

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def emit(record):
    print(json.dumps(record,allow_nan=False,separators=(',',':')),flush=True)


def configure_helper_threads(threads):
    if threads is None:return
    if threads not in (1,2):raise ValueError('unsupported helper threads')
    import torch
    import cv2
    torch.set_num_threads(threads)
    cv2.setNumThreads(threads)
    verify_helper_threads(threads)


def verify_helper_threads(threads):
    if threads is None:return
    import torch
    import cv2
    actual=(torch.get_num_threads(),cv2.getNumThreads())
    if actual!=(threads,threads):
        raise ValueError(f'helper thread mismatch: requested={threads}, actual={actual}')


@contextlib.contextmanager
def request_bundle(request):
    import numpy as np
    if request.get('rgb_transport')=='mmap':
        if request.get('operation')!='infer_instances':raise ValueError('mmap is RGB inference only')
        from vision_core.person_localization.rgb_shared_buffer import receive_rgb
        rgb=receive_rgb(request['rgb_shared'],request['cycle_id'],request['arrays_sha256'])
        yield {'cycle_id':np.asarray(request['cycle_id']),'rgb':rgb}
    else:
        path=Path(request['arrays'])
        if path.name!='current.npz' or digest(path)!=request['arrays_sha256']:
            raise ValueError('array provenance mismatch')
        with np.load(path,allow_pickle=False) as bundle:yield bundle


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',type=Path,required=True)
    parser.add_argument('--engine',choices=['pytorch','onnxruntime'],default='pytorch')
    parser.add_argument('--comparison-report',type=Path)
    parser.add_argument('--ort-threads',type=int,default=2)
    parser.add_argument('--cpu-helper-threads',type=int,choices=[1,2])
    args=parser.parse_args()
    os.environ['YOLO_AUTOINSTALL']='false' 
    try:
        with contextlib.redirect_stdout(sys.stderr):
            import numpy as np
            from ultralytics import YOLO
            from preview_mask_surface import evaluate, segment_array, CurrentInstanceMasks
            if args.engine=='onnxruntime':
                if args.comparison_report is None or args.ort_threads<1:
                    raise ValueError('ONNX requires comparison report and positive threads')
                from onnx_segmentation_backend import OnnxSegmentationBackend
                model=OnnxSegmentationBackend(args.model,args.comparison_report,args.ort_threads)
                engine_metadata=model.metadata
            else:
                model=YOLO(str(args.model),task='segment')
                engine_metadata={'engine':'pytorch'}
            if model.task!='segment' or model.names.get(0)!='person':
                raise ValueError('expected COCO person segmentation model')
            # Ultralytics/backend warm-up may reset torch thread defaults.
            configure_helper_threads(args.cpu_helper_threads)
    except Exception as error:
        emit({'status':'UNAVAILABLE','reason':type(error).__name__,'error_detail':str(error)[:400]})
        return
    model_sha=digest(args.model)
    code_sha={str(path.name):digest(path) for path in (
        Path(__file__),Path(__file__).with_name('preview_mask_surface.py'),
        Path(__file__).resolve().parents[1]/'person_localization/local_surface_diagnostic.py',
        Path(__file__).resolve().parents[1]/'person_localization/instance_mask_stereo_fusion.py',
        Path(__file__).resolve().parents[1]/'person_localization/rgb_shared_buffer.py')}
    if args.engine=='onnxruntime':
        code_sha['onnx_backend']=digest(Path(__file__).with_name('onnx_segmentation_backend.py'))
    engine_metadata={**engine_metadata,'cpu_helper_threads':{
        'requested':args.cpu_helper_threads,
        'torch':getattr(sys.modules.get('torch'),'get_num_threads',lambda:None)(),
        'opencv':getattr(sys.modules.get('cv2'),'getNumThreads',lambda:None)()}}
    emit({'status':'READY','model_sha256':model_sha,'segmentation_engine':engine_metadata})
    current_masks=CurrentInstanceMasks()
    for line in sys.stdin:
        request={}
        try:
            request=json.loads(line)
            path=Path(request['arrays']) if 'arrays' in request else None
            with request_bundle(request) as bundle:
                if str(bundle['cycle_id'].item())!=request['cycle_id']:
                    raise ValueError('array cycle mismatch')
                if request.get('operation')=='infer_instances':
                    started=time.monotonic()
                    with contextlib.redirect_stdout(sys.stderr):
                        instances=current_masks.infer(request['cycle_id'],request['arrays_sha256'],bundle['rgb'],model)
                    inference_s=time.monotonic()-started
                    verify_helper_threads(args.cpu_helper_threads)
                    fingerprints={}
                    if request.get('diagnostic_mask_hashes'):
                        fingerprints={'rgb_payload_sha256':hashlib.sha256(bundle['rgb'].tobytes()).hexdigest(),'instance_mask_sha256':[hashlib.sha256(mask.tobytes()).hexdigest()
                            for mask in current_masks.current[3]]}
                    emit({**fingerprints,'status':'CURRENT_INSTANCES_DIAGNOSTIC_ONLY','instances':instances,
                          'mask_s':inference_s,'cycle_id':request['cycle_id'],
                          'arrays_sha256':request['arrays_sha256'],'model_sha256':model_sha,
                          'input_transport':request.get('rgb_transport','npz'),
                          'code_sha256':code_sha,'class_name':'person','inference_count':1,'segmentation_engine':engine_metadata})
                    continue
                source=bundle['rgb'] if 'rgb' in bundle else None
                cal={k:bundle[k] for k in ('P1','P2','R1','R','T','K','D','size')}
                started=time.monotonic()
                if request.get('operation')=='evaluate_selected_instance':
                    mask=current_masks.take(request['cycle_id'],request['rgb_arrays_sha256'],
                                            request['instance_index'],request['bbox_xyxy_px'])
                    info={'status':'CURRENT_FRAME_MASK_DIAGNOSTIC_ONLY',
                          'instance_index':request['instance_index'],
                          'source_rgb_arrays_sha256':request['rgb_arrays_sha256'],
                          'selection':'EXACT_SAME_MODEL_RESULT_INDEX'}
                    mask_s=0.  # inference was already timed in the preceding response
                else:
                    with contextlib.redirect_stdout(sys.stderr):
                        mask,info=segment_array(source,request['candidate'],model)
                    mask_s=time.monotonic()-started
                if mask is None:
                    result={'status':'MASK_UNAVAILABLE','mask':info}
                else:
                    started=time.monotonic()
                    deferred=bool(request.get('defer_local',False))
                    evaluated=evaluate(bundle['disparity'],mask,cal,request['roi'],*request['range'],scale=request.get('scale',1.),defer_local=deferred,prepare_local=bool(request.get('prepare_local',True)))
                    if deferred:
                        result,samples=evaluated
                        if samples is not None:
                            job=path.parent/'surface_job.npz'
                            np.savez(job,cycle_id=request['cycle_id'],**samples)
                            result['deferred_surface']={'file':job.name,'sha256':digest(job),'retained':False,'transport':'TEMPORARY_RAM_ONLY'}
                        result['mask_depth_prepare_s']=time.monotonic()-started
                    else:
                        result=evaluated
                        result['surface_s']=time.monotonic()-started
                    result.update(mask=info,
                                  mask_sha256=hashlib.sha256(mask.tobytes()).hexdigest())
                result.update(segmentation_engine=engine_metadata,code_sha256=code_sha,mask_s=mask_s,cycle_id=request['cycle_id'],
                              arrays_sha256=request['arrays_sha256'],model_sha256=model_sha)
                emit(result)
        except Exception as error:
            current_masks.current=None
            emit({'cycle_id':request.get('cycle_id'),'status':'MASK_SURFACE_UNAVAILABLE',
                  'reason':type(error).__name__,'error_detail':str(error)[:400]})


if __name__=='__main__':
    main()
