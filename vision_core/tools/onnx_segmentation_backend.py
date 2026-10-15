"""Explicit diagnostic ONNX adapter bound to the reviewed offline export/report."""
from __future__ import annotations
import hashlib
import importlib.metadata
import json
from pathlib import Path


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify_report(model, report_path, threads):
    report=json.loads(Path(report_path).read_text())
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
                       'input_shape':shape,'comparison_sha256':digest(report_path),
                       'comparison_file':str(report_path),'model_sha256':digest(model_path),
                       'source_checkpoint_sha256':report['source_model_sha256'],
                       'environment':{p:importlib.metadata.version(p) for p in ('ultralytics','onnxruntime','torch','numpy')},
                       'rect':False,'dtype':'float32','inter_op_threads':1,'intra_op_spinning':False}

    def predict(self,**kwargs):
        return self.model.predict(**{**kwargs,'imgsz':self.size,'rect':False,'dnn':False})
