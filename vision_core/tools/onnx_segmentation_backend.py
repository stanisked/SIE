"""Explicit diagnostic ONNX adapter bound to the reviewed offline export/report."""
from __future__ import annotations
import hashlib
import importlib.metadata
import json
from pathlib import Path


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify_report(model, report_path, threads):
    report=json.loads(Path(report_path).read_text())
    if report.get('status')=='OFFLINE_192X320_VS_REVIEWED_256X384_ONLY':
        return verify_192_report(model,report,threads)
    if report.get('status')=='OFFLINE_INPUT_SIZE_CHALLENGE_ONLY':
        return verify_size_report(model,report,threads)
    if (report.get('status')!='OFFLINE_SEGMENTATION_COMPARISON_ONLY'
            or report.get('inputs_unchanged') is not True
            or report.get('all_input_tensors_equal') is not True):
        raise ValueError('ONNX comparison did not complete verified inputs')
    export=report['export']
    if digest(model)!=export['sha256']:
        raise ValueError('ONNX export SHA mismatch')
    if digest(report['source_model'])!=report['source_model_sha256']:
        raise ValueError('source checkpoint changed since comparison')
    shape=export['live_input_tensor']['shape']
    if len(shape)!=4 or shape[:2]!=[1,3] or export['live_input_tensor']['dtype']!='float32':
        raise ValueError('invalid reviewed FP32 input shape')
    rows=[r for r in report['rows'] if r['threads']==threads]
    if not rows or any(not r['input_tensors_equal'] for r in rows):
        raise ValueError('requested thread count has no verified comparison')
    # Bind this rollout to the operator-reviewed exact-mask challenge, not a
    # new approximate-IoU admission threshold or an automatically chosen model.
    for row in rows:
        match=row['matching'];choice=row['selection_comparison']
        if (match['unmatched_reference_count'] or match['unmatched_candidate_count']
                or any(p['mask_iou']!=1. or p['ambiguous_equal_score'] for p in match['pairs'])
                or not choice['same_disposition']
                or (not choice['both_unavailable'] and choice.get('selected_correspondence_unambiguous') is not True)
                or (row['pytorch_geometry'] or {}).get('accepted')!=(row['onnx_geometry'] or {}).get('accepted')):
            raise ValueError('report differs from reviewed exact-mask/selection result')
    return report,shape


def verify_size_report(model,report,threads):
    """Bind only the operator-reviewed 256x384 diagnostic experiment.

    These checks describe recorded evidence, not new per-frame quality gates.
    The exact-mask PyTorch/ONNX reference verifier remains unchanged.
    """
    from benchmark_segmentation_input_size import summarize
    if threads!=2 or report.get('inputs_unchanged') is not True:
        raise ValueError('input-size challenge requires verified sources and two threads')
    reference=report['reference']
    reference_path=reference['comparison_file']
    if digest(reference_path)!=reference['comparison_sha256']:
        raise ValueError('reference comparison SHA mismatch')
    original=json.loads(Path(reference_path).read_text())
    reviewed,_=verify_report(original['export']['file'],reference_path,2)
    if reference['model_sha256']!=reviewed['export']['sha256'] or reference['source_checkpoint_sha256']!=reviewed['source_model_sha256']:
        raise ValueError('reference model/checkpoint mismatch')
    exports=[e for e in report['exports'] if e['size']==[256,384]]
    if len(exports)!=1 or digest(model)!=exports[0]['sha256']:
        raise ValueError('reviewed smaller export SHA mismatch')
    export=exports[0];arguments=export['arguments']
    expected={'format':'onnx','imgsz':[256,384],'batch':1,'dynamic':False,'half':False,
              'simplify':False,'opset':17,'nms':False,'device':'cpu'}
    if arguments!=expected:raise ValueError('smaller export configuration mismatch')
    rows=[r for r in report['rows'] if r['size']==[256,384]]
    from collections import Counter
    counts=Counter(r['cycle_id'] for r in rows)
    if len(rows)!=44 or len(counts)!=22 or set(counts.values())!={2}:
        raise ValueError('expected reviewed 22 cycles, two repetitions')
    for row in rows:
        if row['input_tensor']['shape']!=[1,3,256,384] or row['input_tensor']['dtype']!='float32':
            raise ValueError('smaller preprocessing differs from review')
        if row['depth_delta_m']!=0 or row['reference_depth']['status']!='COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY' or row['candidate_depth']['status']!=row['reference_depth']['status']:
            raise ValueError('depth differs from reviewed smaller-input comparison')
    summary=summarize(rows)
    if (any(summary[k] for k in ('selection_disposition_changes','selected_correspondence_failures','geometry_status_changes','unmatched_instances'))
            or summary['mask_iou_min']!=0.9458133917729704
            or summary['mask_iou_median']!=0.9539935499009282):
        raise ValueError('report differs from operator-reviewed smaller-input result')
    if any(report['environment']['packages'][p]!=reviewed['environment']['packages'][p] for p in ('ultralytics','onnxruntime','torch','numpy')):
        raise ValueError('size/reference environments differ')
    # Common adapter metadata, without relabelling this as exact-mask equality.
    return {**report,'source_model_sha256':reviewed['source_model_sha256']},[1,3,256,384]


def verify_192_report(model,report,threads):
    """Bind the separately reviewed 192x320 diagnostic results, not a new gate."""
    from benchmark_segmentation_input_size import summarize
    from collections import Counter
    if threads!=2 or report.get('inputs_unchanged') is not True:
        raise ValueError('192x320 challenge requires verified inputs and compatibility thread argument 2')
    reference=report['reference'];parent=reference['comparison_file']
    if digest(parent)!=reference['comparison_sha256']:
        raise ValueError('256x384 comparison SHA mismatch')
    original=json.loads(Path(parent).read_text())
    if original.get('status')!='OFFLINE_INPUT_SIZE_CHALLENGE_ONLY':
        raise ValueError('192x320 requires reviewed 256x384 parent')
    exports=[e for e in original['exports'] if e['size']==[256,384]]
    if len(exports)!=1:raise ValueError('missing 256x384 parent export')
    # The parent verifier binds the previous exact ONNX comparison/checkpoint.
    parent_model=exports[0].get('path') or exports[0].get('file')
    if not parent_model:raise ValueError('parent export path required')
    reviewed,shape=verify_report(parent_model,parent,2)
    if shape!=[1,3,256,384] or reference['model_sha256']!=digest(parent_model) or reference['source_checkpoint_sha256']!=reviewed['source_model_sha256']:
        raise ValueError('192x320 reference model/checkpoint mismatch')
    exports=[e for e in report['exports'] if e['size']==[192,320]]
    if len(exports)!=1 or digest(model)!=exports[0]['sha256']:
        raise ValueError('192x320 export SHA mismatch')
    expected={'format':'onnx','imgsz':[192,320],'batch':1,'dynamic':False,'half':False,
        'simplify':False,'opset':17,'nms':False,'device':'cpu'}
    if exports[0]['arguments']!=expected:raise ValueError('192x320 export arguments mismatch')
    rows=report['rows'];counts=Counter(r['cycle_id'] for r in rows)
    if len(rows)!=44 or len(counts)!=22 or set(counts.values())!={2}:
        raise ValueError('expected reviewed 22 cycles, two repetitions')
    for cid in counts:
        if {r['repeat'] for r in rows if r['cycle_id']==cid}!={0,1}:
            raise ValueError('192x320 repeat coverage mismatch')
    for row in rows:
        if row['size']!=[192,320] or row['input_tensor']['shape']!=[1,3,192,320] or row['input_tensor']['dtype']!='float32':
            raise ValueError('192x320 tensor shape mismatch')
        if row['reference_depth']['status']!='COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY' or row['candidate_depth']['status']!=row['reference_depth']['status']:
            raise ValueError('192x320 depth status differs from review')
        m=row['matching'];choice=row['selection_comparison']
        if not choice['same_disposition'] or choice.get('selected_correspondence_unambiguous') is not True:
            raise ValueError('192x320 target correspondence differs from review')
        extra=m['unmatched_reference_count']+m['unmatched_candidate_count']
        if extra:
            if (not row['cycle_id'].endswith(('00000014','00000020')) or
                m['reference_count']!=1 or m['candidate_count']!=2 or
                m['unmatched_reference_count']!=0 or m['unmatched_candidate_count']!=1 or
                row['depth_delta_m']!=0 or
                any(row[k]['current_eligible_count']!=1 or row[k]['candidate']['instance_index']!=0 or
                    row[k]['reason']!='DIRECT_CURRENT_FRAME_SINGLE_TARGET' for k in ('reference_selection','candidate_selection'))):
                raise ValueError('unmatched instance differs from reviewed two non-selected detections')
    summary=summarize(rows)
    observed={'mask_iou_median':0.9721894940444458,'mask_iou_min':0.9652224742582682,
        'selection_disposition_changes':0,'selected_correspondence_failures':0,
        'geometry_status_changes':0,'unmatched_instances':4,
        'absolute_depth_delta_median_m':0.,'absolute_depth_delta_max_m':0.010505437850952148}
    if any(summary[k]!=v for k,v in observed.items()):
        raise ValueError('192x320 results differ from operator-reviewed evidence')
    for package in ('ultralytics','onnxruntime','torch','numpy'):
        if report['environment']['packages'][package]!=reviewed['environment']['packages'][package]:
            raise ValueError('192x320 reference environment mismatch')
    return {**report,'source_model_sha256':reviewed['source_model_sha256']},[1,3,192,320]


class OnnxSegmentationBackend:
    task='segment'
    def __init__(self,model_path,report_path,threads):
        report,shape=verify_report(model_path,report_path,threads)
        for package in ('ultralytics','onnxruntime','torch','numpy'):
            if importlib.metadata.version(package)!=report['environment']['packages'][package]:
                raise ValueError(f'{package} differs from compared environment')
        import numpy as np
        import onnxruntime as ort
        from ultralytics import YOLO
        from preview_mask_surface import infer_instances
        self.model=YOLO(str(model_path),task='segment');self.size=shape[2:]
        # Same wrapper/session replacement as the successful offline challenge.
        source=np.zeros((1200,1920,3),np.uint8)
        infer_instances(source,self)
        backend=self.model.predictor.model
        if not hasattr(backend,'session') or backend.device.type!='cpu':
            raise ValueError('unsupported ONNX wrapper; no fallback')
        opts=ort.SessionOptions();opts.intra_op_num_threads=threads;opts.inter_op_num_threads=1
        opts.execution_mode=ort.ExecutionMode.ORT_SEQUENTIAL
        opts.add_session_config_entry('session.intra_op.allow_spinning','0')
        backend.session=ort.InferenceSession(str(model_path),sess_options=opts,providers=['CPUExecutionProvider'])
        if backend.session.get_providers()!=['CPUExecutionProvider'] or backend.session.get_inputs()[0].shape!=shape:
            raise ValueError('ONNX provider/input differs from compared export')
        if self.model.task!='segment' or self.model.names.get(0)!='person':
            raise ValueError('expected reviewed COCO person segmentation')
        self.names=self.model.names
        infer_instances(source,self)
        tensor=self.model.predictor.preprocess([source]).detach().cpu().numpy()
        if list(tensor.shape)!=shape or str(tensor.dtype)!='float32':
            raise ValueError('ONNX preprocessing input mismatch')
        self.metadata={'engine':'onnxruntime','threads':threads,'providers':backend.session.get_providers(),
                       'review_kind':report['status'],
                       'input_shape':shape,'comparison_sha256':digest(report_path),
                       'comparison_file':str(report_path),'model_sha256':digest(model_path),
                       'source_checkpoint_sha256':report['source_model_sha256'],
                       'environment':{p:importlib.metadata.version(p) for p in ('ultralytics','onnxruntime','torch','numpy')},
                       'rect':False,'dtype':'float32','inter_op_threads':1,'intra_op_spinning':False}

    def predict(self,**kwargs):
        return self.model.predict(**{**kwargs,'imgsz':self.size,'rect':False,'dnn':False})
