"""Single-owner companion process with bounded, cycle-bound temporary input."""
import hashlib
import json
from pathlib import Path
import select
import subprocess
import tempfile
import time


class PreviewMaskClient:
    def __init__(self, python, script, model, timeout=30., *, engine="pytorch", comparison_report=None, ort_threads=2, cpu_helper_threads=None, rgb_transport="npz", active_ort_threads=None):
        if rgb_transport not in ("npz","mmap"):raise ValueError("unsupported RGB transport")
        self.rgb_transport=rgb_transport
        self.timeout=timeout
        self.process=None
        self.model_sha256=hashlib.sha256(Path(model).read_bytes()).hexdigest()
        command=[str(python),'-u',str(script),'--model',str(model)]
        if engine not in ('pytorch','onnxruntime'):
            raise ValueError('unsupported preview engine')
        if cpu_helper_threads is not None:
            if cpu_helper_threads not in (1,2):raise ValueError('unsupported helper thread count')
            command+=['--cpu-helper-threads',str(cpu_helper_threads)]
        if active_ort_threads is not None:
            if engine!='onnxruntime' or active_ort_threads not in (1,2):raise ValueError('unsupported active ORT configuration')
            command+=['--active-ort-threads',str(active_ort_threads)]
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
            if active_ort_threads is not None:
                active=self.engine_metadata.get('active_ort_session',{})
                if (self.engine_metadata.get('active_ort_threads_requested')!=active_ort_threads
                        or active.get('intra_op_num_threads')!=active_ort_threads or active.get('inter_op_num_threads')!=1):
                    raise ValueError('preview active ORT setting was not applied')
            if cpu_helper_threads is not None:
                observed=self.engine_metadata.get('cpu_helper_threads',{})
                if observed.get('requested')!=cpu_helper_threads or observed.get('torch')!=cpu_helper_threads or observed.get('opencv')!=cpu_helper_threads:
                    raise ValueError('preview helper thread setting was not applied')
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
                stamps={'setup_s':time.monotonic()-started}
                stage=time.monotonic()
                np.savez(path,cycle_id=cycle_id,**arrays)
                stamps['serialize_npz_s']=time.monotonic()-stage
                stage=time.monotonic()
                sha=hashlib.sha256(path.read_bytes()).hexdigest()
                stamps['input_file_hash_s']=time.monotonic()-stage
                stage=time.monotonic()
                request={**fields,'cycle_id':cycle_id,'arrays':str(path),'arrays_sha256':sha}
                if self.process is None or self.process.stdin is None:
                    raise RuntimeError('preview worker unavailable')
                self.process.stdin.write(json.dumps(request,separators=(',',':'))+'\n')
                self.process.stdin.flush()
                stamps['control_prepare_send_s']=time.monotonic()-stage
                stage=time.monotonic()
                result=self._read(self.timeout)
                stamps['response_wait_decode_s']=time.monotonic()-stage
                stage=time.monotonic()
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
                stamps['validate_response_load_local_s']=time.monotonic()-stage
                result['rpc_timing_s']=stamps
                result['round_trip_s']=time.monotonic()-started
                return result
        except BaseException:
            self.close()
            raise

    def _shared_rgb_request(self,cycle_id,rgb,diagnostic_mask_hashes):
        from vision_core.person_localization.rgb_shared_buffer import publish_rgb
        started=time.monotonic()
        with publish_rgb(cycle_id,rgb) as (descriptor,sha):
            stamps={'shared_publish_s':time.monotonic()-started}
            try:
                stage=time.monotonic()
                request={'operation':'infer_instances','rgb_transport':'mmap','rgb_shared':descriptor,
                    'cycle_id':cycle_id,'arrays_sha256':sha,'diagnostic_mask_hashes':diagnostic_mask_hashes}
                if self.process is None:raise RuntimeError('preview worker unavailable')
                self.process.stdin.write(json.dumps(request,separators=(',',':'))+'\n');self.process.stdin.flush()
                stamps['control_prepare_send_s']=time.monotonic()-stage
                stage=time.monotonic()
                result=self._read(self.timeout)
                stamps['response_wait_decode_s']=time.monotonic()-stage
                stage=time.monotonic()
                if result.get('cycle_id')!=cycle_id:raise ValueError('shared RGB result cycle mismatch')
                if result.get('status')!='MASK_SURFACE_UNAVAILABLE' and (result.get('arrays_sha256')!=sha or result.get('model_sha256')!=self.model_sha256):
                    raise ValueError('shared RGB input/model mismatch')
                result['round_trip_s']=time.monotonic()-started
                stamps['validate_response_s']=time.monotonic()-stage
                result['rpc_timing_s']=stamps
                result['rgb_transport']='mmap'
                return result
            except BaseException:
                self.close()  # Stop reader before owner unmaps/unlinks on failure.
                raise

    def infer(self, cycle_id, rgb, *, diagnostic_mask_hashes=False):
        if getattr(self,'rgb_transport','npz')=='mmap':
            return self._shared_rgb_request(cycle_id,rgb,diagnostic_mask_hashes)
        return self._array_request(cycle_id,{'rgb':rgb},{'operation':'infer_instances',
            'diagnostic_mask_hashes':diagnostic_mask_hashes})

    def evaluate_selected(self, cycle_id, disparity, cal, candidate, roi, depth_range, segmentation, *, scale, defer_local=False, prepare_local=True, prepared_geometry=None):
        if segmentation.get('cycle_id')!=cycle_id or segmentation.get('model_sha256')!=self.model_sha256:
            raise ValueError('segmentation cycle/model mismatch')
        arrays={'disparity':disparity,**cal} if prepared_geometry is None else {'geometry_'+k:v for k,v in prepared_geometry.items()}
        if prepared_geometry is not None and str(prepared_geometry['source_cycle_id'].item())!=cycle_id:
            raise ValueError('prepared geometry source cycle mismatch')
        result=self._array_request(cycle_id,arrays,{
            'operation':'evaluate_selected_instance','prepared_geometry':prepared_geometry is not None,'instance_index':candidate['instance_index'],
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
