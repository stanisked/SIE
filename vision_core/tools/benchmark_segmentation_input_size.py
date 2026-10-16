#!/usr/bin/env python3
"""Offline smaller-input ONNX challenge. No runtime or admission changes."""
import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import shutil
import statistics
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from benchmark_segmentation_onnx import (load_cases,sha256,selection,selection_comparison,
    match_instances,geometry_checker,FixedInputModel,environment,tensor_fingerprint)


def inner_roi(candidate):
    """Execute actual runtime ROI with its declared defaults."""
    import ast
    from types import SimpleNamespace
    path=ROOT/'ros2/sie_ros2/sie_ros2/ar0234_ov9281_dynamic_association_audit_node.py'
    tree=ast.parse(path.read_text());defaults={}
    for n in ast.walk(tree):
        if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='declare_parameter' and len(n.args)==2 and isinstance(n.args[0],ast.Constant):
            name=n.args[0].value
            if isinstance(name,str) and name.startswith('ar_inner_roi_'):
                defaults[name]=ast.literal_eval(n.args[1])
    method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='_ar_inner_roi')
    method.returns=None
    for arg in method.args.args:arg.annotation=None
    namespace={'ContractError':ValueError}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),str(path),'exec'),namespace)
    return list(namespace['_ar_inner_roi'](SimpleNamespace(**defaults),candidate))


def depth_for(instances,masks,stamp,disparity,cal):
    from preview_mask_surface import evaluate
    choice=selection(instances,stamp);candidate=choice.get('candidate')
    geometry=geometry_checker()(candidate)
    if candidate is None or not geometry['accepted']:
        return choice,geometry,{'status':'NO_CURRENT_ELIGIBLE_TARGET'}
    result,_=evaluate(disparity,masks[candidate['instance_index']],cal,inner_roi(candidate),
                      .5,4.5,scale=.5,defer_local=True,prepare_local=False)
    return choice,geometry,result['mask_depth']


def delta(first,second,key):
    a,b=first.get(key),second.get(key)
    return b-a if a is not None and b is not None else None


def summarize(rows):
    overlaps=[p['mask_iou'] for r in rows for p in r['matching']['pairs'] if p['mask_iou'] is not None]
    values=[abs(r['depth_delta_m']) for r in rows if r['depth_delta_m'] is not None]
    return {'rows':len(rows),'reference_median_s':statistics.median(r['reference_s'] for r in rows),
        'candidate_median_s':statistics.median(r['candidate_s'] for r in rows),
        'mask_iou_median':statistics.median(overlaps) if overlaps else None,
        'mask_iou_min':min(overlaps) if overlaps else None,
        'selection_disposition_changes':sum(not r['selection_comparison']['same_disposition'] for r in rows),
        'selected_correspondence_failures':sum(not c['both_unavailable'] and c.get('selected_correspondence_unambiguous') is not True for c in (r['selection_comparison'] for r in rows)),
        'geometry_status_changes':sum((r['reference_geometry'] or {}).get('accepted')!=(r['candidate_geometry'] or {}).get('accepted') for r in rows),
        'unmatched_instances':sum(r['matching']['unmatched_reference_count']+r['matching']['unmatched_candidate_count'] for r in rows),
        'depth_status_transitions':dict(Counter(r['reference_depth']['status']+' -> '+r['candidate_depth']['status'] for r in rows)),
        'absolute_depth_delta_median_m':statistics.median(values) if values else None,
        'absolute_depth_delta_max_m':max(values) if values else None}


def candidate_backend(path,size,first):
    import onnxruntime as ort
    from ultralytics import YOLO
    from preview_mask_surface import infer_instances
    raw=YOLO(str(path),task='segment');wrapped=FixedInputModel(raw,list(size))
    infer_instances(first,wrapped)
    backend=raw.predictor.model
    if not hasattr(backend,'session') or backend.device.type!='cpu':raise ValueError('unsupported ONNX backend')
    opts=ort.SessionOptions();opts.intra_op_num_threads=2;opts.inter_op_num_threads=1
    opts.execution_mode=ort.ExecutionMode.ORT_SEQUENTIAL
    opts.add_session_config_entry('session.intra_op.allow_spinning','0')
    backend.session=ort.InferenceSession(str(path),sess_options=opts,providers=['CPUExecutionProvider'])
    if backend.session.get_inputs()[0].shape!=[1,3,*size] or backend.session.get_providers()!=['CPUExecutionProvider']:
        raise ValueError('candidate shape/provider mismatch')
    for _ in range(2):infer_instances(first,wrapped)
    return raw,wrapped



def challenge_reference(report_path,shape,sizes,current_256=False):
    """Keep this new challenge distinct from the previously reviewed rollout."""
    report=json.loads(Path(report_path).read_text())
    if not current_256:
        if list(shape)!=[416,640]:raise ValueError('expected reviewed 416x640 baseline')
        return Path(report['source_model']),'OFFLINE_INPUT_SIZE_CHALLENGE_ONLY'
    if list(shape)!=[256,384] or report.get('status')!='OFFLINE_INPUT_SIZE_CHALLENGE_ONLY':
        raise ValueError('expected reviewed 256x384 baseline')
    if sizes!=[(192,320)]:raise ValueError('this challenge compares only 192x320 against reviewed 256x384')
    parent=Path(report['reference']['comparison_file'])
    if sha256(parent)!=report['reference']['comparison_sha256']:
        raise ValueError('parent comparison changed')
    original=json.loads(parent.read_text())
    checkpoint=Path(original['source_model'])
    if sha256(checkpoint)!=original['source_model_sha256']:
        raise ValueError('source checkpoint changed')
    return checkpoint,'OFFLINE_192X320_VS_REVIEWED_256X384_ONLY'


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('reference-model','reference-report','evidence-root','ar-intrinsic','stereo-calibration','activation','output-dir'):
        p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--sizes',nargs='+',default=['320x512','256x384'])
    p.add_argument('--repeats',type=int,default=2)
    p.add_argument('--reference-current-256',action='store_true',help='new offline-only 192x320 challenge against reviewed current 256x384')
    args=p.parse_args()
    sizes=[tuple(map(int,s.lower().split('x'))) for s in args.sizes]
    if args.output_dir.exists() or args.repeats<1 or any(len(s)!=2 or any(n<=0 or n%32 for n in s) or s[0]>=416 or s[1]>=640 for s in sizes):
        p.error('new output directory, positive repeats, smaller HxW multiples of 32 required')
    import os
    os.environ['YOLO_AUTOINSTALL']='false'
    import cv2
    import numpy as np
    import onnx
    from ultralytics import YOLO
    from onnx_segmentation_backend import OnnxSegmentationBackend
    from preview_mask_surface import infer_instances
    from analyze_ar0234_stereo_component_ambiguity import load_calibrations
    from benchmark_stereo_sgbm_parallel import runtime_parameters,compute_pair
    from vision_core.person_localization.diagnostic_stereo_resolution import half_disparity
    from benchmark_stereo_resolution import scaled_configs
    cv2.setNumThreads(1)
    baseline=OnnxSegmentationBackend(args.reference_model,args.reference_report,2)
    checkpoint,challenge_status=challenge_reference(args.reference_report,baseline.size,sizes,args.reference_current_256)
    from active_ort_session import active_session_metadata
    active_reference=active_session_metadata(baseline)
    cp=json.loads(args.activation.read_text())['calibration_profile'];ex=cp['ar0234_to_ov9281_physical_left_extrinsic'];extrinsic=Path(ex['path']).expanduser()
    if sha256(args.ar_intrinsic)!=cp['source_sha256']['ar0234_intrinsic'] or sha256(args.stereo_calibration)!=cp['source_sha256']['ov9281_stereo_calibration'] or sha256(extrinsic)!=ex['sha256']:
        raise ValueError('activation/calibration SHA mismatch')
    cal=load_calibrations(args.ar_intrinsic,args.stereo_calibration,extrinsic)
    cases=load_cases(args.evidence_root)
    sources={str(f):sha256(f) for f in (args.reference_model,args.reference_report,checkpoint,args.activation,args.ar_intrinsic,args.stereo_calibration,extrinsic)}
    if args.reference_current_256:
        parent=Path(json.loads(args.reference_report.read_text())['reference']['comparison_file'])
        sources[str(parent)]=sha256(parent)
    for case in cases:sources.update(case['source_sha256'])
    args.output_dir.mkdir(parents=True,exist_ok=False)
    report={'status':challenge_status,'environment':environment(),'rows':[], 'exports':[],
        'reference':baseline.metadata,'active_reference_session':active_reference,'source_sha256':sources,'stereo_scale':.5,'diagnostic_range_m':[.5,4.5],
        'metric_measurement_authorized':False,'actuator_access':False,
        'limitations':['Same saved scene; no physical depth truth or general/dynamic-scene certification.',
        'JPEG stereo recomputed once per cycle and shared by every segmentation size.',
        'Timing: segmentation preprocessing/inference/full-resolution mask extraction, excluding capture/IPC/stereo/depth.',
        'Current-frame selection without manufactured temporal confirmation; no automatic size selection.',
        'Mask comparison against current ONNX, not annotated ground truth.']}
    try:
        prepared=[];configs,code=runtime_parameters();report['stereo_source_sha256']=code
        with ThreadPoolExecutor(max_workers=2) as pool:
            matchers=[cv2.StereoSGBM_create(**c) for c in scaled_configs(configs,.5)]
            for case in cases:
                paths=[Path(case['image']).parent/f'ov9281_physical_{side}_rectified.jpg' for side in ('left','right')]
                for path in paths:sources[str(path)]=sha256(path)
                colors=[cv2.imread(str(path)) for path in paths]
                if any(x is None or x.shape!=(800,1280,3) for x in colors):raise ValueError('invalid stereo images')
                images=[cv2.cvtColor(x,cv2.COLOR_BGR2GRAY) for x in colors]
                if any(x is None or x.shape!=(800,1280) for x in images):raise ValueError('invalid rectified stereo evidence')
                images=[cv2.resize(x,(640,400),interpolation=cv2.INTER_AREA) for x in images]
                maps,_=compute_pair(matchers,*images,executor=pool)
                left,valid=half_disparity(*maps)
                disparity=np.where(valid,left,np.nan)
                prepared.append(disparity)
        first=cv2.imread(cases[0]['image'])
        for size in dict.fromkeys(sizes):
            folder=args.output_dir/('input_'+'x'.join(map(str,size)));folder.mkdir()
            copy=folder/'checkpoint.pt';shutil.copyfile(checkpoint,copy)
            export_args=dict(format='onnx',imgsz=list(size),batch=1,dynamic=False,half=False,simplify=False,opset=17,nms=False,device='cpu')
            exported=Path(YOLO(str(copy),task='segment').export(**export_args))
            if exported.parent.resolve()!=folder.resolve():raise ValueError('export escaped output folder')
            onnx.checker.check_model(str(exported))
            raw,candidate=candidate_backend(exported,size,first)
            active_candidate=active_session_metadata(candidate)
            report.setdefault('active_candidate_sessions',{})[str(size)]=active_candidate
            for key in ('providers','intra_op_num_threads','inter_op_num_threads','execution_mode'):
                if active_reference[key]!=active_candidate[key]:
                    raise ValueError('active session budget/provider differs; do not attribute timing to input size')
            report['exports'].append({'size':list(size),'path':str(exported),'sha256':sha256(exported),'arguments':export_args})
            for repeat in range(args.repeats):
                for index,(case,disparity) in enumerate(zip(cases,prepared)):
                    image=cv2.imread(case['image']);pred={};timings={}
                    order=('reference','candidate') if (index+repeat)%2==0 else ('candidate','reference')
                    for label in order:
                        start=time.perf_counter();pred[label]=infer_instances(image,baseline if label=='reference' else candidate);timings[label]=time.perf_counter()-start
                    a,ma=pred['reference'];b,mb=pred['candidate'];matching=match_instances(a,ma,b,mb)
                    sa,ga,da=depth_for(a,ma,case['timestamp'],disparity,cal)
                    sb,gb,db=depth_for(b,mb,case['timestamp'],disparity,cal)
                    fingerprint=tensor_fingerprint(raw,image)
                    if fingerprint['shape']!=[1,3,*size]:raise ValueError('candidate preprocessing shape mismatch')
                    report['rows'].append({'cycle_id':case['cycle_id'],'size':list(size),'repeat':repeat,'order':order,
                        'reference_s':timings['reference'],'candidate_s':timings['candidate'],'input_tensor':fingerprint,
                        'matching':matching,'reference_selection':sa,'candidate_selection':sb,
                        'selection_comparison':selection_comparison(sa,sb,matching),
                        'reference_geometry':ga,'candidate_geometry':gb,'reference_depth':da,'candidate_depth':db,
                        'depth_delta_m':delta(da,db,'candidate_depth_m'),'mad_delta_m':delta(da,db,'candidate_mad_m'),
                        'sample_count_delta':delta(da,db,'selected_sample_count')})
            subset=[r for r in report['rows'] if r['size']==list(size)]
            print('SIZE',size,json.dumps(summarize(subset)),flush=True)
            del raw,candidate
        report['summaries']={str(size):summarize([r for r in report['rows'] if r['size']==list(size)]) for size in dict.fromkeys(sizes)}
        report['inputs_unchanged']=all(sha256(path)==digest for path,digest in sources.items())
        if not report['inputs_unchanged']:raise ValueError('source evidence changed')
    except Exception as error:
        report.update(status='COMPARISON_FAILED',error_type=type(error).__name__,error_detail=str(error));raise
    finally:
        report['tool_sha256']=sha256(__file__)
        with (args.output_dir/'comparison.json').open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
        print('INPUTS_UNCHANGED:',report.get('inputs_unchanged',False),flush=True)
        print('Saved:',args.output_dir/'comparison.json',flush=True)


if __name__=='__main__':main()
