"""One active fit and one latest pending fit, independent of inference RPC."""
from copy import deepcopy
from vision_core.person_localization.control_depth_quality_shadow import assess_control_depth
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import queue
import select
import subprocess
import tempfile
import threading
import time


class LocalSurfaceClient:
    def __init__(self,python,script,timeout=30.):
        self.timeout=timeout;self.process=None
        env={**os.environ,'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','MKL_NUM_THREADS':'1'}
        self.process=subprocess.Popen([str(python),'-u',str(script)],stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True,bufsize=1,env=env)
        try:
            self.metadata=self._read(60.)
            if self.metadata.get('status')!='READY':raise RuntimeError(f"local worker not ready: {self.metadata}")
        except BaseException:
            self.close();raise

    def _read(self,timeout):
        if self.process is None:raise RuntimeError('local worker unavailable')
        ready,_,_=select.select([self.process.stdout],[],[],timeout)
        if not ready:raise TimeoutError('local worker timeout')
        line=self.process.stdout.readline()
        if not line:raise RuntimeError('local worker closed')
        response=json.loads(line)
        if not isinstance(response,dict):raise ValueError("invalid local worker response")
        return response

    def compute(self,job):
        import numpy as np
        try:
            with tempfile.TemporaryDirectory(prefix='sie-local-',dir='/dev/shm') as folder:
                path=Path(folder)/'samples.npz'
                np.savez(path,cycle_id=job['cycle_id'],**job['samples'])
                sha=hashlib.sha256(path.read_bytes()).hexdigest()
                request={'cycle_id':job['cycle_id'],'input':str(path),'input_sha256':sha}
                if self.process is None:raise RuntimeError('local worker unavailable')
                self.process.stdin.write(json.dumps(request)+'\n');self.process.stdin.flush()
                result=self._read(self.timeout)
                if result.get('status') not in ('LOCAL_SURFACE_DIAGNOSTIC_ONLY','LOCAL_SURFACE_UNAVAILABLE'):
                    raise ValueError('unexpected local worker status')
                if result.get('cycle_id')!=job['cycle_id']:raise ValueError('local worker cycle mismatch')
                if result.get('status')=='LOCAL_SURFACE_DIAGNOSTIC_ONLY' and result.get('input_sha256')!=sha:
                    raise ValueError('local worker input mismatch')
                return result
        except BaseException:
            self.close();raise

    def close(self):
        process,self.process=self.process,None
        if process is not None:
            if process.poll() is None:
                process.terminate()
                try:process.wait(timeout=2)
                except subprocess.TimeoutExpired:process.kill();process.wait(timeout=2)
            for stream in (process.stdin,process.stdout):
                if stream is not None:stream.close()


class LatestSurfaceDiagnostics:
    def __init__(self,client,directory,on_result=None):
        self.client=client;self.directory=Path(directory);self.directory.mkdir(parents=True,exist_ok=False)
        self.on_result=on_result
        self.pending=queue.Queue(maxsize=1);self.closed=False;self.dropped=0;self.errors=0
        self.completed=0;self.write_errors=0;self.last_error=None
        self.thread=threading.Thread(target=self._run,name='sie-local-diagnostic',daemon=True);self.thread.start()

    def submit(self,cycle_id,samples,provenance):
        if self.closed:return False
        job={'cycle_id':cycle_id,'samples':deepcopy(samples),'provenance':deepcopy(provenance),
             'submitted_monotonic':time.monotonic()}
        try:self.pending.put_nowait(job)
        except queue.Full:
            try:self.pending.get_nowait();self.pending.task_done();self.dropped+=1
            except queue.Empty:pass
            self.pending.put_nowait(job)
        return True

    def _run(self):
        while True:
            job=self.pending.get()
            try:
                if job is None:return
                started=time.monotonic()
                try:
                    result=self.client.compute(job)
                except Exception as error:
                    result={'cycle_id':job['cycle_id'],'status':'LOCAL_SURFACE_UNAVAILABLE',
                            'reason':type(error).__name__,'error_detail':str(error)[:400]}
                if result.get('status')!='LOCAL_SURFACE_DIAGNOSTIC_ONLY':
                    self.errors+=1;self.last_error=result.get('reason',result.get('status'))
                result.update(schema_version='sie.local_surface_async_diagnostic.v1',
                    completed_at_utc=datetime.now(timezone.utc).isoformat(),
                    local_worker=getattr(self.client,'metadata',None),
                    provenance=job['provenance'],reference_frame='rectified_left_optical_frame',unit='m',
                    confidence=None,confidence_status='NOT_VALIDATED',metric_measurement_authorized=False,
                    navigation_eligible=False,actuator_access=False,
                    queue_wait_s=started-job['submitted_monotonic'],
                    submit_to_result_s=time.monotonic()-job['submitted_monotonic'],
                    replaced_pending_count=self.dropped)
                source=job["provenance"].get("control_quality_input")
                if source is not None:
                    result["control_depth_quality_shadow"]=assess_control_depth(source,result)
                if self.on_result is not None:
                    try:self.on_result(result)
                    except Exception as error:result['publication_error']=type(error).__name__
                try:
                    with (self.directory/('local-surface-'+job['cycle_id']+'.json')).open('x') as stream:
                        json.dump(result,stream,allow_nan=False,separators=(',',':'));stream.write('\n')
                except Exception as error:
                    self.write_errors+=1;self.last_error=type(error).__name__
                self.completed+=1
            finally:self.pending.task_done()

    def close(self):
        if not self.closed:
            self.closed=True;self.pending.put(None);self.thread.join();self.client.close()
