"""Single-owner companion process with bounded, cycle-bound temporary input."""
import hashlib
import json
import math
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

    def infer_jpeg(self,cycle_id,payload,*,diagnostic_mask_hashes=False):
        from vision_core.person_localization.jpeg_frame_transport import publish_jpeg
        started=time.monotonic()
        with publish_jpeg(cycle_id,payload) as (descriptor,sha):
            stamps={'jpeg_publish_s':time.monotonic()-started}
            try:
                stage=time.monotonic()
                request={'operation':'infer_instances','rgb_transport':'jpeg','jpeg_frame':descriptor,
                    'cycle_id':cycle_id,'arrays_sha256':sha,'diagnostic_mask_hashes':diagnostic_mask_hashes}
                if self.process is None:raise RuntimeError('preview worker unavailable')
                self.process.stdin.write(json.dumps(request,separators=(',',':'))+'\n');self.process.stdin.flush()
                stamps['control_prepare_send_s']=time.monotonic()-stage
                stage=time.monotonic();result=self._read(self.timeout)
                stamps['response_wait_decode_s']=time.monotonic()-stage
                if result.get('cycle_id')!=cycle_id:raise ValueError('JPEG result cycle mismatch')
                if result.get('status')!='MASK_SURFACE_UNAVAILABLE' and (result.get('arrays_sha256')!=sha or result.get('model_sha256')!=self.model_sha256):
                    raise ValueError('JPEG input/model mismatch')
                result['rpc_timing_s']=stamps;result['round_trip_s']=time.monotonic()-started
                result['rgb_transport']='jpeg'
                return result
            except BaseException:
                self.close();raise

    def infer(self, cycle_id, rgb, *, diagnostic_mask_hashes=False):
        if getattr(self,'rgb_transport','npz')=='mmap':
            return self._shared_rgb_request(cycle_id,rgb,diagnostic_mask_hashes)
        return self._array_request(cycle_id,{'rgb':rgb},{'operation':'infer_instances',
            'diagnostic_mask_hashes':diagnostic_mask_hashes})

    def evaluate_selected(self, cycle_id, disparity, cal, candidate, roi, depth_range, segmentation, *, scale, defer_local=False, prepare_local=True, prepared_geometry=None, fast_local=False, front_profile=None, source_snapshot=False, snapshot_periodic=False, snapshot_reacquire=False):
        if segmentation.get('cycle_id')!=cycle_id or segmentation.get('model_sha256')!=self.model_sha256:
            raise ValueError('segmentation cycle/model mismatch')
        arrays={'disparity':disparity,**cal} if prepared_geometry is None else {'geometry_'+k:v for k,v in prepared_geometry.items()}
        if prepared_geometry is not None and str(prepared_geometry['source_cycle_id'].item())!=cycle_id:
            raise ValueError('prepared geometry source cycle mismatch')
        front_args={}
        if front_profile is not None:
            from vision_core.person_localization.robot_front_geometry import validate_profile
            validate_profile(front_profile)
            arrays.update(front_P1=cal['P1'],front_R1=cal['R1'])
            front_args={'front_mount_profile':front_profile}
        result=self._array_request(cycle_id,arrays,{
            **front_args,'operation':'evaluate_selected_instance','prepared_geometry':prepared_geometry is not None,'instance_index':candidate['instance_index'],
            'bbox_xyxy_px':candidate['bbox_xyxy_px'],'rgb_arrays_sha256':segmentation['arrays_sha256'],
            'roi':list(roi),'range':list(depth_range),'scale':scale,'defer_local':defer_local,'prepare_local':prepare_local,
            **({'shadow_source_snapshot':True,'snapshot_periodic':snapshot_periodic,
                'snapshot_reacquire':snapshot_reacquire} if source_snapshot else {}),
            **({'fast_local':True} if fast_local else {})})
        payload=result.get('_shadow_source_mask_snapshot')
        if payload is not None and (not source_snapshot or
                payload.get('cycle_id') != cycle_id or
                payload.get('mask_sha256') != result.get('mask_sha256')):
            raise ValueError('source snapshot mask binding mismatch')
        if fast_local and result.get('status')!='MASK_SURFACE_UNAVAILABLE':
            fast=result.get('fast_local_consistency') or {}
            expected={'source_cycle_id':cycle_id,'mask_sha256':result.get('mask_sha256'),
                'instance_index':candidate['instance_index'],'model_sha256':self.model_sha256,
                'rgb_arrays_sha256':segmentation['arrays_sha256'],'fusion_arrays_sha256':result.get('arrays_sha256')}
            if fast.get('provenance')!=expected or not expected['mask_sha256'] or fast.get('status')!='LOCAL_CROSSFIT_DIAGNOSTIC_ONLY':
                raise ValueError('fast local current-frame binding mismatch')
        if front_profile is not None and result.get('status')!='MASK_SURFACE_UNAVAILABLE':
            from vision_core.person_localization.robot_front_geometry import profile_hash
            front=result.get('front_geometry') or {}
            expected={'source_cycle_id':cycle_id,'mask_sha256':result.get('mask_sha256'),
                'instance_index':candidate['instance_index'],'model_sha256':self.model_sha256,
                'rgb_arrays_sha256':segmentation['arrays_sha256'],'fusion_arrays_sha256':result.get('arrays_sha256')}
            if (front.get('provenance')!=expected or not expected['mask_sha256'] or
                    front.get('profile_sha256')!=profile_hash(front_profile) or
                    front.get('distance_from_front_m') is not None or front.get('actuator_access') is not False):
                raise ValueError('front geometry current-frame/profile binding mismatch')
            layer=result.get('candidate_layer_front_geometry')
            if layer is not None and (layer.get('provenance')!=expected or
                    layer.get('profile_sha256')!=front['profile_sha256'] or
                    layer.get('point_set')!='NUMERIC_CANDIDATE_DEPTH_LAYER_DIAGNOSTIC_ONLY' or
                    layer.get('semantic_ownership_verified') is not False or
                    layer.get('point_count')!=(result.get('mask_depth') or {}).get('candidate_sample_count') or
                    layer.get('distance_from_front_m') is not None or
                    layer.get('actuator_access') is not False):
                raise ValueError('candidate layer current-frame/profile binding mismatch')
            if fast_local and layer is not None:
                local_front=result.get('local_candidate_front_geometry') or {}
                residual=local_front.get('local_fit_residual') or {}
                if (local_front.get('provenance')!=expected or
                        local_front.get('profile_sha256')!=front['profile_sha256'] or
                        local_front.get('point_set')!='LOCAL_FITTED_NUMERIC_CANDIDATE_LAYER_DIAGNOSTIC_ONLY' or
                        local_front.get('candidate_sample_count')!=layer['point_count'] or
                        type(local_front.get('point_count')) is not int or
                        not 0<=local_front['point_count']<=layer['point_count'] or
                        residual.get('sample_count')!=local_front['point_count'] or
                        local_front.get('source_depth_status')!=(result.get('mask_depth') or {}).get('status') or
                        local_front.get('semantic_ownership_verified') is not False or
                        local_front.get('distance_from_front_m') is not None or
                        local_front.get('actuator_access') is not False or
                        local_front.get('status') not in (
                            'LOCAL_FITTED_CANDIDATE_FRONT_DIAGNOSTIC_ONLY',
                            'INSUFFICIENT_LOCAL_FITTED_CANDIDATE_POINTS') or
                        (local_front['status']=='LOCAL_FITTED_CANDIDATE_FRONT_DIAGNOSTIC_ONLY'
                         and (local_front['point_count']<100 or
                              type((local_front.get('known_term_m') or {}).get('p05')) not in (float,int) or
                              not math.isfinite(local_front['known_term_m']['p05']) or
                              type(residual.get('absolute_p90_m')) not in (float,int) or
                              not math.isfinite(residual['absolute_p90_m']))) or
                        (local_front['status']=='INSUFFICIENT_LOCAL_FITTED_CANDIDATE_POINTS'
                         and (local_front['point_count']>=100 or
                              local_front.get('known_term_m') is not None))):
                    raise ValueError('local candidate front current-frame/profile binding mismatch')
                from vision_core.person_localization.fast_local_depth_consistency import STABLE_LOCAL_PATCH_POLICY
                from vision_core.person_localization.torso_core_geometry import (
                    TORSO_CORE_POLICY, torso_core_roi)
                stable=result.get('stable_candidate_front_geometry') or {}
                patches=stable.get('stable_local_patches') or {}
                stable_count=stable.get('point_count')
                if (stable.get('provenance')!=expected or
                        stable.get('profile_sha256')!=front['profile_sha256'] or
                        stable.get('point_set')!='STABLE_LOCAL_PATCHES_NUMERIC_CANDIDATE_LAYER_DIAGNOSTIC_ONLY' or
                        stable.get('candidate_sample_count')!=layer['point_count'] or
                        stable.get('source_depth_status')!=(result.get('mask_depth') or {}).get('status') or
                        stable.get('semantic_ownership_verified') is not False or
                        stable.get('distance_from_front_m') is not None or
                        stable.get('actuator_access') is not False or
                        patches.get('policy')!=STABLE_LOCAL_PATCH_POLICY or
                        type(stable_count) is not int or
                        not 0<=stable_count<=layer['point_count'] or
                        patches.get('selected_point_count')!=stable_count or
                        (patches.get('selected_held_out_residual') or {}).get('sample_count')!=stable_count or
                        stable.get('status') not in ('STABLE_CANDIDATE_FRONT_DIAGNOSTIC_ONLY',
                            'INSUFFICIENT_STABLE_LOCAL_PATCH_POINTS') or
                        (stable['status']=='STABLE_CANDIDATE_FRONT_DIAGNOSTIC_ONLY' and
                         (stable_count<STABLE_LOCAL_PATCH_POLICY['minimum_selected_points'] or
                          type((stable.get('known_term_m') or {}).get('p05')) not in (float,int) or
                          not math.isfinite(stable['known_term_m']['p05']))) or
                        (stable['status']=='INSUFFICIENT_STABLE_LOCAL_PATCH_POINTS' and
                         (stable_count>=STABLE_LOCAL_PATCH_POLICY['minimum_selected_points'] or
                          stable.get('known_term_m') is not None))):
                    raise ValueError('stable candidate front current-frame/profile binding mismatch')
                torso=result.get('stable_torso_core_front_geometry') or {}
                torso_count=torso.get('point_count')
                torso_candidates=torso.get('torso_core_candidate_point_count')
                expected_torso_roi=torso_core_roi(candidate['bbox_xyxy_px'])
                if (torso.get('provenance')!=expected or
                        torso.get('profile_sha256')!=front['profile_sha256'] or
                        torso.get('point_set')!='STABLE_LOCAL_PATCHES_WITHIN_CURRENT_RGB_TORSO_CORE_DIAGNOSTIC_ONLY' or
                        torso.get('candidate_sample_count')!=layer['point_count'] or
                        torso.get('source_depth_status')!=(result.get('mask_depth') or {}).get('status') or
                        torso.get('semantic_ownership_verified') is not False or
                        torso.get('physical_body_part_verified') is not False or
                        torso.get('distance_from_front_m') is not None or
                        torso.get('actuator_access') is not False or
                        torso.get('torso_core_policy')!=TORSO_CORE_POLICY or
                        torso.get('stable_local_patch_policy')!=STABLE_LOCAL_PATCH_POLICY or
                        torso.get('torso_core_rgb_roi_xyxy_px')!=expected_torso_roi or
                        type(torso_count) is not int or type(torso_candidates) is not int or
                        not 0<=torso_count<=torso_candidates<=layer['point_count'] or
                        torso.get('status') not in ('STABLE_TORSO_CORE_FRONT_DIAGNOSTIC_ONLY',
                            'INSUFFICIENT_STABLE_TORSO_CORE_POINTS') or
                        (torso['status']=='STABLE_TORSO_CORE_FRONT_DIAGNOSTIC_ONLY' and
                         (torso_count<TORSO_CORE_POLICY['minimum_stable_points'] or
                          type((torso.get('known_term_m') or {}).get('median')) not in (float,int) or
                          not math.isfinite(torso['known_term_m']['median']) or
                          torso.get('selected_known_term_m')!=torso['known_term_m']['median'])) or
                        (torso['status']=='INSUFFICIENT_STABLE_TORSO_CORE_POINTS' and
                         (torso_count>=TORSO_CORE_POLICY['minimum_stable_points'] or
                          torso.get('known_term_m') is not None or
                          torso.get('selected_known_term_m') is not None))):
                    raise ValueError('stable torso core current-frame/profile binding mismatch')
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

