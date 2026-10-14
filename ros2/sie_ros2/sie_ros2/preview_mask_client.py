"""Single-owner companion process with bounded, cycle-bound temporary input."""
import hashlib
import json
from pathlib import Path
import select
import subprocess
import tempfile
import time


class PreviewMaskClient:
    def __init__(self, python, script, model, timeout=30.):
        self.timeout=timeout
        self.process=None
        self.model_sha256=hashlib.sha256(Path(model).read_bytes()).hexdigest()
        try:
            self.process=subprocess.Popen([str(python),'-u',str(script),'--model',str(model)],
                stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,
                text=True,bufsize=1)
            ready=self._read(60.)
            if ready.get('status')!='READY' or ready.get('model_sha256')!=self.model_sha256:
                raise ValueError('preview mask worker model mismatch')
        except BaseException:
            self.close()
            raise

    def _read(self, timeout):
        if self.process is None or self.process.stdout is None:
            raise RuntimeError('preview worker unavailable')
        readable,_,_=select.select([self.process.stdout],[],[],timeout)
        if not readable:
            raise TimeoutError('preview mask worker timeout')
        line=self.process.stdout.readline()
        if not line:
            raise RuntimeError('preview mask worker closed')
        response=json.loads(line)
        if not isinstance(response,dict):
            raise ValueError('invalid worker response')
        return response

    def evaluate(self, cycle_id, rgb, disparity, cal, candidate, roi, depth_range, *, scale=1.):
        import numpy as np
        started=time.monotonic()
        try:
            # RAM-backed Linux temp storage; separate random directory per
            # request. No compressed images, persistent arrays or old masks.
            with tempfile.TemporaryDirectory(prefix='sie-preview-',dir='/dev/shm') as directory:
                path=Path(directory)/'current.npz'
                np.savez(path,cycle_id=cycle_id,rgb=rgb,disparity=disparity,**cal)
                sha=hashlib.sha256(path.read_bytes()).hexdigest()
                request={'cycle_id':cycle_id,'arrays':str(path),'arrays_sha256':sha,
                         'candidate':candidate,'roi':list(roi),'range':list(depth_range),'scale':scale}
                if self.process is None or self.process.stdin is None:
                    raise RuntimeError('preview mask worker unavailable')
                self.process.stdin.write(json.dumps(request,separators=(',',':'))+'\n')
                self.process.stdin.flush()
                result=self._read(self.timeout)
                if result.get('cycle_id')!=cycle_id:
                    raise ValueError('preview mask worker cycle mismatch')
                if result.get('status') not in ('MASK_SURFACE_UNAVAILABLE',):
                    if result.get('arrays_sha256')!=sha or result.get('model_sha256')!=self.model_sha256:
                        raise ValueError('preview mask result provenance mismatch')
                result['round_trip_s']=time.monotonic()-started
                return result
        except BaseException:
            # A late response can never be mistaken for a subsequent cycle.
            self.close()
            raise

    def close(self):
        process,self.process=self.process,None
        if process is not None:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill(); process.wait(timeout=2)
            for stream in (process.stdin,process.stdout):
                if stream is not None:
                    stream.close()
