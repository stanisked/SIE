"""Single-owner companion process with bounded, cycle-bound temporary input."""
import hashlib
import json
from pathlib import Path
import select
import subprocess
import tempfile
import time


class PreviewMaskClient:
    def __init__(self, python, script, model, timeout=30., *, engine="pytorch", comparison_report=None, ort_threads=2):
        self.timeout=timeout
        self.process=None
        self.model_sha256=hashlib.sha256(Path(model).read_bytes()).hexdigest()
        command=[str(python),'-u',str(script),'--model',str(model)]
        if engine not in ('pytorch','onnxruntime'):
            raise ValueError('unsupported preview engine')
        report_sha=None
        if engine=='onnxruntime':
            report_sha=hashlib.sha256(Path(comparison_report).read_bytes()).hexdigest()
            command+=['--engine',engine,'--comparison-report',str(comparison_report),'--ort-threads',str(ort_threads)]
        try:
            self.process=subprocess.Popen(command,
                stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,
                text=True,bufsize=1)
            ready=self._read(60.)
            if ready.get('status')!='READY':
                raise RuntimeError(f"preview worker startup failed: {ready.get('reason')}: {ready.get('error_detail')}")
            if ready.get('model_sha256')!=self.model_sha256:
                raise ValueError('preview mask worker model mismatch')
            self.engine_metadata=ready.get('segmentation_engine',{'engine':'pytorch'})
            if engine=='onnxruntime' and (self.engine_metadata.get('engine')!=engine or self.engine_metadata.get('threads')!=ort_threads or self.engine_metadata.get('comparison_sha256')!=report_sha):
                raise ValueError('preview engine/report mismatch')
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

    def _array_request(self, cycle_id, arrays, fields):
        import numpy as np
        started=time.monotonic()
        try:
            with tempfile.TemporaryDirectory(prefix='sie-preview-',dir='/dev/shm') as directory:
                path=Path(directory)/'current.npz'
                np.savez(path,cycle_id=cycle_id,**arrays)
                sha=hashlib.sha256(path.read_bytes()).hexdigest()
                request={**fields,'cycle_id':cycle_id,'arrays':str(path),'arrays_sha256':sha}
                if self.process is None or self.process.stdin is None:
                    raise RuntimeError('preview worker unavailable')
                self.process.stdin.write(json.dumps(request,separators=(',',':'))+'\n')
                self.process.stdin.flush()
                result=self._read(self.timeout)
                if result.get('cycle_id')!=cycle_id:
                    raise ValueError('preview worker cycle mismatch')
                if result.get('status')!='MASK_SURFACE_UNAVAILABLE':
                    if result.get('arrays_sha256')!=sha or result.get('model_sha256')!=self.model_sha256:
                        raise ValueError('preview worker input/model mismatch')
                deferred=result.get('deferred_surface')
                if deferred is not None:
                    job=path.parent/'surface_job.npz'
                    if deferred.get('file')!='surface_job.npz' or hashlib.sha256(job.read_bytes()).hexdigest()!=deferred['sha256']:
                        raise ValueError('deferred surface input SHA mismatch')
                    with np.load(job,allow_pickle=False) as data:
                        if str(data['cycle_id'].item())!=cycle_id:
                            raise ValueError('deferred surface cycle mismatch')
                        result['_surface_job']={k:data[k].copy() for k in data.files if k!='cycle_id'}
                result['round_trip_s']=time.monotonic()-started
                return result
        except BaseException:
            self.close()
            raise

    def infer(self, cycle_id, rgb):
        return self._array_request(cycle_id,{'rgb':rgb},{'operation':'infer_instances'})

    def evaluate_selected(self, cycle_id, disparity, cal, candidate, roi, depth_range, segmentation, *, scale, defer_local=False, prepare_local=True):
        if segmentation.get('cycle_id')!=cycle_id or segmentation.get('model_sha256')!=self.model_sha256:
            raise ValueError('segmentation cycle/model mismatch')
        result=self._array_request(cycle_id,{'disparity':disparity,**cal},{
            'operation':'evaluate_selected_instance','instance_index':candidate['instance_index'],
            'bbox_xyxy_px':candidate['bbox_xyxy_px'],'rgb_arrays_sha256':segmentation['arrays_sha256'],
            'roi':list(roi),'range':list(depth_range),'scale':scale,'defer_local':defer_local,'prepare_local':prepare_local})
        result['mask_s']=segmentation.get('mask_s')
        result['inference_round_trip_s']=segmentation.get('round_trip_s')
        result['round_trip_scope']='FUSION_ONLY_AFTER_SEGMENTATION'
        return result

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
