#!/usr/bin/env python3
"""Sparse local-fit worker, with no segmentation model or camera access."""
import hashlib
import json
from pathlib import Path
import sys
import time


def emit(value):print(json.dumps(value,allow_nan=False,separators=(',',':')),flush=True)


def main():
    try:
        import numpy as np
        import scipy
        from preview_mask_surface import local_surface_stats
    except Exception as error:
        emit({'status':'UNAVAILABLE','reason':type(error).__name__,'error_detail':str(error)[:400]})
        return
    emit({'status':'READY','numpy':np.__version__,'scipy':scipy.__version__,
          'blas_threads':1,'code_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (
              Path(__file__),Path(__file__).with_name('preview_mask_surface.py'),
              Path(__file__).resolve().parents[1]/'person_localization/local_surface_diagnostic.py')}})
    for line in sys.stdin:
        request={}
        try:
            request=json.loads(line);path=Path(request['input'])
            sha=hashlib.sha256(path.read_bytes()).hexdigest()
            if sha!=request['input_sha256']:raise ValueError('surface input SHA mismatch')
            started=time.monotonic()
            with np.load(path,allow_pickle=False) as data:
                if str(data['cycle_id'].item())!=request['cycle_id']:raise ValueError('surface cycle mismatch')
                stats=local_surface_stats(data)
            emit({'cycle_id':request['cycle_id'],'status':'LOCAL_SURFACE_DIAGNOSTIC_ONLY',
                  'input_sha256':sha,'local_surface':stats,'local_processing_s':time.monotonic()-started})
        except Exception as error:
            emit({'cycle_id':request.get('cycle_id'),'status':'LOCAL_SURFACE_UNAVAILABLE',
                  'reason':type(error).__name__,'error_detail':str(error)[:400]})


if __name__=='__main__':main()
