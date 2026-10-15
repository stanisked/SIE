"""Reviewed artifact binding and fixed-shape CPU adapter wiring without real ML."""
import importlib.metadata
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'vision_core/tools'))
from onnx_segmentation_backend import verify_report, digest, OnnxSegmentationBackend


def fixture(folder):
    model=folder/'model.onnx';model.write_bytes(b'export')
    source=folder/'source.pt';source.write_bytes(b'checkpoint')
    row={'threads':2,'input_tensors_equal':True,
         'matching':{'unmatched_reference_count':0,'unmatched_candidate_count':0,
                     'pairs':[{'mask_iou':1.,'ambiguous_equal_score':False}]},
         'selection_comparison':{'same_disposition':True,'both_unavailable':False,'selected_correspondence_unambiguous':True},
         'pytorch_geometry':{'accepted':True},'onnx_geometry':{'accepted':True}}
    report={'status':'OFFLINE_SEGMENTATION_COMPARISON_ONLY','inputs_unchanged':True,
            'all_input_tensors_equal':True,'source_model':str(source),'source_model_sha256':digest(source),
            'export':{'sha256':digest(model),'live_input_tensor':{'shape':[1,3,416,640],'dtype':'float32'}},
            'environment':{'packages':{p:'fixture' for p in ('ultralytics','onnxruntime','torch','numpy')}},'rows':[row]}
    path=folder/'comparison.json';path.write_text(json.dumps(report))
    return model,path,report


class OnnxBackendTests(unittest.TestCase):
    def test_exact_report_export_threads_and_original_checkpoint_are_bound(self):
        with tempfile.TemporaryDirectory() as directory:
            model,path,report=fixture(Path(directory))
            self.assertEqual(verify_report(model,path,2)[1],[1,3,416,640])
            with self.assertRaisesRegex(ValueError,'thread count'):verify_report(model,path,4)
            model.write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError,'export SHA'):verify_report(model,path,2)
            model.write_bytes(b'export');Path(report['source_model']).write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError,'checkpoint changed'):verify_report(model,path,2)

    def test_mask_selection_and_geometry_disagreement_do_not_roll_out(self):
        with tempfile.TemporaryDirectory() as directory:
            model,path,report=fixture(Path(directory))
            for field in ('mask','selection','geometry','inputs'):
                current=json.loads(json.dumps(report))
                if field=='mask':current['rows'][0]['matching']['pairs'][0]['mask_iou']=.99
                if field=='selection':current['rows'][0]['selection_comparison']['same_disposition']=False
                if field=='geometry':current['rows'][0]['onnx_geometry']['accepted']=False
                if field=='inputs':current['all_input_tensors_equal']=False
                path.write_text(json.dumps(current))
                with self.assertRaises(ValueError):verify_report(model,path,2)

    def test_adapter_uses_reviewed_cpu_session_shape_and_preprocessing(self):
        class Tensor:
            def __init__(self,a):self.a=np.asarray(a)
            def detach(self):return self
            def cpu(self):return self
            def numpy(self):return self.a
        calls=[];sessions=[]
        class Options:
            def add_session_config_entry(self,k,v):self.config=(k,v)
        class Session:
            def __init__(self,path,sess_options,providers):
                self.opts=sess_options;self.providers=providers;sessions.append(self)
            def get_providers(self):return self.providers
            def get_inputs(self):return [SimpleNamespace(shape=[1,3,416,640])]
        class Model:
            task='segment';names={0:'person'}
            def __init__(self,*args,**kwargs):
                self.predictor=SimpleNamespace(model=SimpleNamespace(session=object(),device=SimpleNamespace(type='cpu')),
                    preprocess=lambda source:Tensor(np.zeros((1,3,416,640),np.float32)))
            def predict(self,**kwargs):
                calls.append(kwargs)
                return [SimpleNamespace(boxes=None,masks=None)]
        ort=SimpleNamespace(SessionOptions=Options,InferenceSession=Session,ExecutionMode=SimpleNamespace(ORT_SEQUENTIAL=7))
        with tempfile.TemporaryDirectory() as directory:
            model,path,_=fixture(Path(directory))
            with patch.dict(sys.modules,{'ultralytics':SimpleNamespace(YOLO=Model),'onnxruntime':ort}),patch.object(importlib.metadata,'version',return_value='fixture'):
                backend=OnnxSegmentationBackend(model,path,2)
            self.assertEqual(len(calls),2)
            self.assertTrue(all(c['imgsz']==[416,640] and c['rect'] is False and c['dnn'] is False for c in calls))
            self.assertEqual(sessions[0].opts.intra_op_num_threads,2)
            self.assertEqual(sessions[0].opts.inter_op_num_threads,1)
            self.assertEqual(sessions[0].opts.config,('session.intra_op.allow_spinning','0'))
            self.assertEqual(backend.metadata['comparison_sha256'],digest(path))
            self.assertEqual(backend.metadata['providers'],['CPUExecutionProvider'])

    def test_changed_environment_refuses_before_model_initialization(self):
        with tempfile.TemporaryDirectory() as directory:
            model,path,_=fixture(Path(directory))
            with patch.object(importlib.metadata,'version',return_value='different'):
                with self.assertRaisesRegex(ValueError,'compared environment'):OnnxSegmentationBackend(model,path,2)


if __name__=='__main__':unittest.main()
