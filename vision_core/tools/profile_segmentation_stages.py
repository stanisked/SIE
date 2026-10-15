#!/usr/bin/env python3
"""Time reviewed CPU segmentation stages on existing frames, without changing math."""
from contextlib import ExitStack,contextmanager
from collections import defaultdict
import argparse
import functools
import hashlib
import inspect
import json
from pathlib import Path
import statistics
import sys
import time
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from benchmark_segmentation_onnx import load_cases,sha256,selection,geometry_checker


class Calls:
    def __init__(self):self.seconds=defaultdict(float);self.counts=defaultdict(int)
    def wrap(self,name,function):
        @functools.wraps(function)
        def timed(*args,**kwargs):
            start=time.perf_counter()
            try:return function(*args,**kwargs)
            finally:
                self.seconds[name]+=time.perf_counter()-start;self.counts[name]+=1
        return timed


def active_session_owner(backend):
    """Follow the actual AutoBackend.forward delegation, not shadow attributes."""
    seen=set();owner=backend
    while getattr(owner,'backend',None) is not None:
        if id(owner) in seen:raise ValueError('backend delegation cycle')
        seen.add(id(owner));owner=owner.backend
    session=getattr(owner,'session',None)
    if session is None or not callable(getattr(session,'run',None)):
        raise ValueError('delegated backend has no supported ORT session.run')
    return owner


def active_session_metadata(adapter):
    outer=adapter.model.predictor.model;owner=active_session_owner(outer);session=owner.session
    opts=session.get_session_options()
    return {'owner':type(owner).__module__+'.'+type(owner).__qualname__,
        'delegated':owner is not outer,
        'outer_session_is_active':getattr(outer,'session',None) is session,
        'providers':session.get_providers(),'input_shape':session.get_inputs()[0].shape,
        'intra_op_num_threads':opts.intra_op_num_threads,'inter_op_num_threads':opts.inter_op_num_threads,
        'execution_mode':str(opts.execution_mode),
        'meaning':'Actual session options; zero threads means ORT default, not zero execution threads.'}


@contextmanager
def instrument(adapter):
    from ultralytics.utils import ops
    predictor=adapter.model.predictor
    backend=active_session_owner(predictor.model);session=backend.session
    calls=Calls()
    with ExitStack() as stack:
        for name in ('preprocess','inference','postprocess'):
            stack.enter_context(patch.object(predictor,name,calls.wrap(name,getattr(predictor,name))))
        stack.enter_context(patch.object(session,'run',calls.wrap('ort_run',session.run)))
        # Retina-mask segmentation uses this function through ops. If a future
        # version imports an alias or changes the path, coverage checks refuse.
        for name in ('process_mask_native','scale_masks'):
            stack.enter_context(patch.object(ops,name,calls.wrap(name,getattr(ops,name))))
        yield calls


def signature(prediction):
    instances,masks=prediction
    return {'instances':instances,'mask_sha256':[hashlib.sha256(mask.tobytes()).hexdigest() for mask in masks]}


def derive(total,calls,instance_count):
    seconds=dict(calls.seconds);counts=dict(calls.counts)
    for stage in ('preprocess','inference','postprocess','ort_run'):
        if counts.get(stage)!=1:raise ValueError(f'unexpected/missing profiled stage: {stage} {counts}')
    if instance_count and (not counts.get('process_mask_native') or not counts.get('scale_masks')):
        raise ValueError('full-resolution mask instrumentation not reached; inspect installed source')
    get=lambda key:seconds.get(key,0.)
    return {'total_predict_extract_s':total,**{k+'_s':v for k,v in seconds.items()},
        'outside_predictor_stages_s':total-get('preprocess')-get('inference')-get('postprocess'),
        'inference_outside_ort_s':get('inference')-get('ort_run'),
        'postprocess_outside_mask_native_s':get('postprocess')-get('process_mask_native'),
        'mask_native_outside_scale_s':get('process_mask_native')-get('scale_masks')}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('evidence-root','model','report','output'):p.add_argument('--'+key,type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():p.error('new output required')
    import os
    os.environ['YOLO_AUTOINSTALL']='false'
    import cv2
    import torch
    from ultralytics.utils import ops
    from onnx_segmentation_backend import OnnxSegmentationBackend
    from preview_mask_surface import infer_instances
    adapter=OnnxSegmentationBackend(args.model,args.report,2)
    active=active_session_metadata(adapter)
    print('ACTIVE_ORT:',json.dumps(active),flush=True)
    cases=load_cases(args.evidence_root);sources={str(f):sha256(f) for f in (args.model,args.report)}
    for case in cases:sources.update(case['source_sha256'])
    predictor=adapter.model.predictor
    codepaths={Path(inspect.getsourcefile(function)) for function in (type(predictor),type(active_session_owner(predictor.model)),ops.process_mask_native,ops.scale_masks)}
    codepaths.update((Path(__file__),Path(__file__).with_name('onnx_segmentation_backend.py'),Path(__file__).with_name('preview_mask_surface.py')))
    installed_code={str(path):sha256(path) for path in codepaths}
    rows=[]
    for repeat in range(2):
        for index,case in enumerate(cases):
            image=cv2.imread(case['image'])
            order=('reference','profiled') if (repeat+index)%2==0 else ('profiled','reference')
            predictions={};timings={};stages=None;counts=None
            for label in order:
                if label=='profiled':
                    with instrument(adapter) as calls:
                        started=time.perf_counter();prediction=infer_instances(image,adapter);total=time.perf_counter()-started
                    stages=derive(total,calls,len(prediction[0]));counts=dict(calls.counts)
                else:
                    started=time.perf_counter();prediction=infer_instances(image,adapter);total=time.perf_counter()-started
                predictions[label]=prediction;timings[label]=total
            exact=signature(predictions['reference'])==signature(predictions['profiled'])
            if not exact:raise ValueError('profiling changed instances or masks')
            choices=[selection(predictions[label][0],case['timestamp']) for label in ('reference','profiled')]
            if choices[0]!=choices[1]:raise ValueError('profiling changed target selection')
            rows.append({'cycle_id':case['cycle_id'],'repeat':repeat,'order':order,'exact':exact,
                'selection':choices[0],'geometry':geometry_checker()(choices[0].get('candidate')),
                'stages':stages,'counts':counts,'timing_s':timings})
    values=defaultdict(list)
    for row in rows:
        for key,value in row['stages'].items():values[key].append(value)
    summary={key:{'n':len(v),'median_s':statistics.median(v),'max_s':max(v)} for key,v in values.items()}
    for key,stats in summary.items():print('SEG_STAGE',key,'n=',stats['n'],'median_ms=',round(stats['median_s']*1000,2),'max_ms=',round(stats['max_s']*1000,2))
    unchanged=all(sha256(path)==digest for path,digest in sources.items())
    if not unchanged:raise ValueError('source inputs changed')
    report={'status':'OFFLINE_SEGMENTATION_STAGE_PROFILE_ONLY','rows':rows,'summary':summary,
        'source_sha256':sources,'inputs_unchanged':unchanged,'code_sha256':installed_code,
        'model_declared_metadata':adapter.metadata,'active_ort_session':active,'helper_threads':{'torch':torch.get_num_threads(),'opencv':cv2.getNumThreads()},
        'reference_median_s':statistics.median(r['timing_s']['reference'] for r in rows),
        'actuator_access':False,'limitations':['Isolated process, no concurrent stereo/local fits or live contention.',
            'Timers are nested: never sum ORT with inference, or mask/scale with postprocess.',
            'scale_masks includes resizing/unpadding; process_mask_native also includes prototype reconstruction/cropping/threshold.',
            'Outside-predictor time includes source setup, wrappers, callbacks, result extraction; not pure IPC.',
            'Profiling wrappers add overhead; uninstrumented reference time is recorded on the same frames.',
            'Declared adapter threads may refer to an inactive outer session; actual delegated session settings recorded separately.',
            'No source image, model, inference math, mask, threshold or runtime launch setting is changed.']}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    print('PROFILE_EXACT:',len(rows),'/',len(rows),'INPUTS_UNCHANGED:',unchanged,'Saved:',args.output)


if __name__=='__main__':main()
