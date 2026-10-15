#!/usr/bin/env python3
"""Export the same checkpoint and compare segmentation on frozen AR frames.

No cameras, ROS publication, runtime switch, calibration or actuator access.
Uses Ultralytics postprocessing for both engines; verifies identical input tensors.
"""
from __future__ import annotations
import argparse
import gc
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import shutil
import statistics
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def match_instances(reference, masks_a, candidate, masks_b):
    import numpy as np
    from scipy.optimize import linear_sum_assignment
    from segment_saved_ar0234_instances import iou
    if len(reference)!=len(masks_a) or len(candidate)!=len(masks_b):
        raise ValueError('bbox/mask correspondence mismatch')
    scores=np.array([[iou(np.asarray(a['bbox_xyxy_px']),np.asarray(b['bbox_xyxy_px']))
                      for b in candidate] for a in reference],dtype=float).reshape(len(reference),len(candidate))
    pairs=[]
    if scores.size:
        ia,ib=linear_sum_assignment(-scores)
        for a,b in zip(ia,ib):
            ma,mb=masks_a[a],masks_b[b]
            if ma.shape!=mb.shape or ma.dtype!=np.bool_ or mb.dtype!=np.bool_:
                raise ValueError('invalid full-resolution masks')
            union=int(np.count_nonzero(ma|mb));intersection=int(np.count_nonzero(ma&mb))
            best=float(scores[a,b])
            row_tie=int(np.count_nonzero(np.isclose(scores[a],best,rtol=0,atol=1e-9)))>1
            col_tie=int(np.count_nonzero(np.isclose(scores[:,b],best,rtol=0,atol=1e-9)))>1
            pairs.append({'reference_index':int(a),'candidate_index':int(b),
                'bbox_iou':best,'bbox_max_coordinate_delta_px':max(abs(float(x)-float(y)) for x,y in zip(reference[a]['bbox_xyxy_px'],candidate[b]['bbox_xyxy_px'])),
                'confidence_delta':candidate[b]['confidence']-reference[a]['confidence'],
                'mask_iou':intersection/union if union else None,
                'mask_different_pixels':int(np.count_nonzero(ma!=mb)),
                'ambiguous_equal_score':row_tie or col_tie})
    return {'reference_count':len(reference),'candidate_count':len(candidate),'pairs':pairs,
            'unmatched_reference_count':len(reference)-len(pairs),
            'unmatched_candidate_count':len(candidate)-len(pairs),
            'method':'maximum total bbox IoU assignment; no IoU admission threshold; inspect scores/ties'}


def selection(instances, timestamp):
    from vision_core.person_localization.diagnostic_instance_selection import select_instance
    return select_instance(instances,prior_track=None,captured_at_utc=timestamp,max_age_s=5,
                           minimum_iou=.2,maximum_center_displacement_relative=.75,
                           minimum_match_margin=.15,minimum_confidence=.70)


def selection_comparison(first,second,matching):
    a,b=first.get('candidate'),second.get('candidate')
    if a is None or b is None:
        return {'same_disposition':first['disposition']==second['disposition'],
                'both_unavailable':a is None and b is None,'selected_pair':None}
    pair=next((p for p in matching['pairs'] if p['reference_index']==a['instance_index'] and p['candidate_index']==b['instance_index']),None)
    return {'same_disposition':first['disposition']==second['disposition'],
            'both_unavailable':False,'selected_pair':pair,
            'selected_correspondence_unambiguous':pair is not None and pair['bbox_iou']>0 and not pair['ambiguous_equal_score']}


def load_cases(root):
    cases=[]
    for audit_path in sorted(root.rglob('audit.json')):
        audit=json.loads(audit_path.read_text())
        if audit_path.parent.name!=audit['cycle_id']:
            raise ValueError('cycle directory mismatch')
        path=audit_path.parent/'ar0234_source.jpg'
        if not path.is_file():
            raise ValueError(f'missing frozen AR source: {path}')
        cases.append({'cycle_id':audit['cycle_id'],'image':str(path),
                      'timestamp':audit['timestamp'],
                      'source_sha256':{str(audit_path):sha256(audit_path),str(path):sha256(path)}})
    if not cases:
        raise ValueError('no saved full-evidence audit frames; compact preview journals contain no images')
    return cases


def tensor_fingerprint(model, source):
    import numpy as np
    tensor=model.predictor.preprocess([source]).detach().cpu().numpy()
    return {'shape':list(tensor.shape),'dtype':str(tensor.dtype),
            'sha256':hashlib.sha256(np.ascontiguousarray(tensor).tobytes()).hexdigest()}


class FixedInputModel:
    def __init__(self,model,size):self.model,self.size=model,size
    def predict(self,**kwargs):
        return self.model.predict(**{**kwargs,'imgsz':self.size,'rect':False,'dnn':False})


def geometry_checker():
    """Execute the unchanged runtime geometry method and its declared defaults."""
    import ast
    from types import SimpleNamespace
    path=ROOT/'ros2/sie_ros2/sie_ros2/ar0234_ov9281_dynamic_association_audit_node.py'
    tree=ast.parse(path.read_text())
    method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='_dynamic_target_geometry')
    method.returns=None
    for arg in method.args.args:arg.annotation=None
    defaults={}
    for n in ast.walk(tree):
        if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='declare_parameter' and len(n.args)==2 and isinstance(n.args[0],ast.Constant):
            name=n.args[0].value
            if name in ('dynamic_target_min_width_to_height_ratio','dynamic_target_edge_margin_px'):
                defaults[name]=ast.literal_eval(n.args[1])
    if len(defaults)!=2:raise ValueError('cannot bind runtime geometry defaults')
    namespace={'ContractError':ValueError}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),str(path),'exec'),namespace)
    owner=SimpleNamespace(**defaults)
    return lambda candidate: namespace['_dynamic_target_geometry'](owner,candidate,1920) if candidate else None


def environment():
    packages={}
    for name in ('ultralytics','torch','torchvision','numpy','scipy','opencv-python','onnx','onnxruntime','onnxscript'):
        try:packages[name]=importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:packages[name]=None
    return {'python':sys.version,'executable':sys.executable,'machine':platform.machine(),'packages':packages}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model',type=Path,required=True)
    p.add_argument('--evidence-root',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--ort-threads',type=int,nargs='+',default=[1,2,4])
    p.add_argument('--repeats',type=int,default=2)
    args=p.parse_args()
    if args.output_dir.exists() or args.repeats<1 or not args.ort_threads or any(t<1 for t in args.ort_threads):
        p.error('use a new output directory, positive repeats/thread counts')
    # Fail before export rather than permitting an implicit environment update.
    import os
    os.environ['YOLO_AUTOINSTALL']='false'
    import cv2
    import numpy as np
    import torch
    import onnx
    import onnxruntime as ort
    import onnxscript  # explicit preflight for modern PyTorch exporters
    from ultralytics import YOLO
    from preview_mask_surface import infer_instances
    cv2.setNumThreads(1)
    cases=load_cases(args.evidence_root)
    check_geometry=geometry_checker()
    model_sha=sha256(args.model)
    args.output_dir.mkdir(parents=True,exist_ok=False)
    report={'status':'OFFLINE_SEGMENTATION_COMPARISON_ONLY','source_model':str(args.model),
            'source_model_sha256':model_sha,'environment':environment(),'cases':cases,'rows':[],
            'metric_measurement_authorized':False,'actuator_access':False,
            'limitations':['Frozen JPEGs, single recorded scene; not generic/dynamic-scene certification.',
                           'Current-frame selection only; no stereo-confirmed temporal track is manufactured.',
                           'Local process inference/extraction timing, not companion IPC or exposure-to-command age.',
                           'Same Ultralytics/Torch mask reconstruction and NMS remain in both paths.',
                           'No automatic backend switch, acceptance threshold, quantization or motion.']}
    try:
        checkpoint=args.output_dir/'source_checkpoint.pt'
        shutil.copyfile(args.model,checkpoint)
        baseline=YOLO(str(checkpoint),task='segment')
        if baseline.task!='segment' or baseline.names.get(0)!='person':
            raise ValueError('expected the existing COCO person segmentation checkpoint')
        first=cv2.imread(cases[0]['image'])
        if first is None or first.shape!=(1200,1920,3):raise ValueError('unexpected saved AR size')
        # Probe the CURRENT live call, including rectangular auto-letterboxing.
        infer_instances(first,baseline)
        live_tensor=tensor_fingerprint(baseline,first)
        if live_tensor['dtype']!='float32' or live_tensor['shape'][:2]!=[1,3]:
            raise ValueError('expected FP32 batch-1 current live input')
        size=live_tensor['shape'][2:]
        export_args=dict(format='onnx',imgsz=size,batch=1,dynamic=False,half=False,
                         simplify=False,opset=17,nms=False,device='cpu')
        exported=Path(YOLO(str(checkpoint),task='segment').export(**export_args))
        if exported.parent.resolve()!=args.output_dir.resolve() or not exported.is_file():
            raise ValueError('export escaped dedicated output directory')
        onnx.checker.check_model(str(exported))
        report['export']={'file':str(exported),'sha256':sha256(exported),'arguments':export_args,
                          'live_input_tensor':live_tensor}
        wrapped_pt=FixedInputModel(baseline,size)
        infer_instances(first,wrapped_pt)
        fixed_tensor=tensor_fingerprint(baseline,first)
        if fixed_tensor!=live_tensor:raise ValueError('fixed comparison preprocessing differs from current live PyTorch')
        references=[]
        for case in cases:
            image=cv2.imread(case['image'])
            if image is None or image.shape!=(1200,1920,3):raise ValueError('unexpected saved AR size')
            instances,masks=infer_instances(image,wrapped_pt)
            references.append((instances,masks,tensor_fingerprint(baseline,image)))
        report['torch_threads']=torch.get_num_threads()
        report['opencv_threads']=cv2.getNumThreads()
        for threads in dict.fromkeys(args.ort_threads):
            candidate=YOLO(str(exported),task='segment')
            wrapped_ort=FixedInputModel(candidate,size)
            infer_instances(first,wrapped_ort)  # initialize version-specific Ultralytics backend
            backend=candidate.predictor.model
            if not hasattr(backend,'session') or backend.device.type!='cpu':
                raise ValueError('unsupported Ultralytics ONNX backend; refusing silent fallback')
            opts=ort.SessionOptions();opts.intra_op_num_threads=threads;opts.inter_op_num_threads=1
            opts.execution_mode=ort.ExecutionMode.ORT_SEQUENTIAL
            opts.add_session_config_entry('session.intra_op.allow_spinning','0')
            backend.session=ort.InferenceSession(str(exported),sess_options=opts,providers=['CPUExecutionProvider'])
            if backend.session.get_providers()!=['CPUExecutionProvider']:
                raise ValueError('unexpected execution provider')
            if backend.session.get_inputs()[0].shape!=live_tensor['shape']:
                raise ValueError('export/input shape mismatch')
            # Exclude session construction and two warm-ups; retain end-to-end
            # prediction/extraction time and actual preprocessed tensor hashes.
            for _ in range(2):infer_instances(first,wrapped_ort);infer_instances(first,wrapped_pt)
            for repeat in range(args.repeats):
                for index,case in enumerate(cases):
                    image=cv2.imread(case['image'])
                    ref,masks_ref,fingerprint_ref=references[index]
                    order=('pt','onnx') if (repeat+index)%2==0 else ('onnx','pt')
                    predictions={};timings={}
                    for label in order:
                        started=time.perf_counter()
                        predictions[label]=infer_instances(image,wrapped_pt if label=='pt' else wrapped_ort)
                        timings[label]=time.perf_counter()-started
                    instances,masks=predictions['onnx']
                    current_ref,current_masks=predictions['pt']
                    matching=match_instances(current_ref,current_masks,instances,masks)
                    current_tensor=tensor_fingerprint(candidate,image)
                    pt_tensor=tensor_fingerprint(baseline,image)
                    row={'cycle_id':case['cycle_id'],'threads':threads,'repeat':repeat,'order':order,
                         'pytorch_s':timings['pt'],'onnx_s':timings['onnx'],
                         'input_tensors_equal':current_tensor==pt_tensor==fingerprint_ref,
                         'input_tensor':current_tensor,'matching':matching,
                         'pytorch_instances':current_ref,'onnx_instances':instances,
                         'pytorch_repeat_matching':match_instances(ref,masks_ref,current_ref,current_masks),
                         'pytorch_selection':selection(current_ref,case['timestamp']),
                         'onnx_selection':selection(instances,case['timestamp'])}
                    row['pytorch_geometry']=check_geometry(row['pytorch_selection'].get('candidate'))
                    row['onnx_geometry']=check_geometry(row['onnx_selection'].get('candidate'))
                    row['selection_comparison']=selection_comparison(row['pytorch_selection'],row['onnx_selection'],matching)
                    report['rows'].append(row)
            subset=[r for r in report['rows'] if r['threads']==threads]
            overlaps=[p['mask_iou'] for r in subset for p in r['matching']['pairs'] if p['mask_iou'] is not None]
            print('QUALITY', 'threads=',threads,'mask_IoU_median=',statistics.median(overlaps) if overlaps else None,
                  'mask_IoU_min=',min(overlaps) if overlaps else None,
                  'selection_status_changes=',sum(not r['selection_comparison']['same_disposition'] for r in subset),
                  'geometry_status_changes=',sum((r['pytorch_geometry'] or {}).get('accepted')!=(r['onnx_geometry'] or {}).get('accepted') for r in subset),flush=True)
            print('THREADS',threads,'PT_median_s=',round(statistics.median(r['pytorch_s'] for r in subset),4),
                  'ONNX_median_s=',round(statistics.median(r['onnx_s'] for r in subset),4),
                  'tensor_equal=',sum(r['input_tensors_equal'] for r in subset),'/',len(subset),flush=True)
            del backend,wrapped_ort,candidate;gc.collect()
        report['inputs_unchanged']=sha256(args.model)==model_sha and all(sha256(path)==digest for c in cases for path,digest in c['source_sha256'].items())
        report['all_input_tensors_equal']=all(r['input_tensors_equal'] for r in report['rows'])
        report['code_sha256']={str(path):sha256(path) for path in (Path(__file__),
            Path(__file__).with_name('preview_mask_surface.py'),ROOT/'vision_core/person_localization/diagnostic_instance_selection.py',
            ROOT/'ros2/sie_ros2/sie_ros2/ar0234_ov9281_dynamic_association_audit_node.py')}
    except Exception as error:
        report.update(status='COMPARISON_FAILED',error_type=type(error).__name__,error_detail=str(error))
        raise
    finally:
        with (args.output_dir/'comparison.json').open('x') as stream:
            json.dump(report,stream,indent=2,allow_nan=False);stream.write('\n')
        print('Saved:',args.output_dir/'comparison.json',flush=True)
    if not report['inputs_unchanged'] or not report['all_input_tensors_equal']:
        raise SystemExit('input verification failed; do not switch backend')


if __name__=='__main__':main()
