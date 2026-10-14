#!/usr/bin/env python3
"""Isolated segmentation/SciPy worker; cycle-bound temporary arrays, no JPEGs."""
import argparse
import contextlib
import hashlib
import json
from pathlib import Path
import sys
import time


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def emit(record):
    print(json.dumps(record,allow_nan=False,separators=(',',':')),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',type=Path,required=True)
    args=parser.parse_args()
    with contextlib.redirect_stdout(sys.stderr):
        import numpy as np
        from ultralytics import YOLO
        from preview_mask_surface import evaluate, segment_array
        model=YOLO(str(args.model),task='segment')
        if model.task!='segment' or model.names.get(0)!='person':
            raise ValueError('expected COCO person segmentation model')
    model_sha=digest(args.model)
    code_sha={str(path.name):digest(path) for path in (
        Path(__file__),Path(__file__).with_name('preview_mask_surface.py'),
        Path(__file__).resolve().parents[1]/'person_localization/local_surface_diagnostic.py',
        Path(__file__).resolve().parents[1]/'person_localization/instance_mask_stereo_fusion.py')}
    emit({'status':'READY','model_sha256':model_sha})
    for line in sys.stdin:
        request={}
        try:
            request=json.loads(line)
            path=Path(request['arrays'])
            if path.name!='current.npz' or digest(path)!=request['arrays_sha256']:
                raise ValueError('array provenance mismatch')
            with np.load(path,allow_pickle=False) as bundle:
                if str(bundle['cycle_id'].item())!=request['cycle_id']:
                    raise ValueError('array cycle mismatch')
                source=bundle['rgb']
                cal={k:bundle[k] for k in ('P1','P2','R1','R','T','K','D','size')}
                started=time.monotonic()
                with contextlib.redirect_stdout(sys.stderr):
                    mask,info=segment_array(source,request['candidate'],model)
                mask_s=time.monotonic()-started
                if mask is None:
                    result={'status':'MASK_UNAVAILABLE','mask':info}
                else:
                    started=time.monotonic()
                    result=evaluate(bundle['disparity'],mask,cal,request['roi'],*request['range'],scale=request.get('scale',1.))
                    result.update(mask=info,surface_s=time.monotonic()-started,
                                  mask_sha256=hashlib.sha256(mask.tobytes()).hexdigest())
                result.update(code_sha256=code_sha,mask_s=mask_s,cycle_id=request['cycle_id'],
                              arrays_sha256=request['arrays_sha256'],model_sha256=model_sha)
                emit(result)
        except Exception as error:
            emit({'cycle_id':request.get('cycle_id'),'status':'MASK_SURFACE_UNAVAILABLE',
                  'reason':type(error).__name__,'error_detail':str(error)[:400]})


if __name__=='__main__':
    main()
