"""Continuously drain native V4L2 MJPEG stream, retain one immutable latest packet."""
import json
import math
import subprocess
import tempfile
import threading
import time


def read_exact(stream,size):
    chunks=[]
    while size:
        chunk=stream.read(size)
        if not chunk:raise EOFError('truncated native frame payload')
        chunks.append(chunk);size-=len(chunk)
    return b''.join(chunks)


def validate_packet(metadata,width,height,previous_sequence=None):
    if metadata.get('kind')!='frame':raise ValueError('unexpected native record')
    size=metadata.get('payload_bytes');sequence=metadata.get('sequence');flags=metadata.get('flags')
    if type(size) is not int or not 0<size<=width*height*3+65536:raise ValueError('invalid native payload size')
    if type(sequence) is not int or not 0<=sequence<=0xffffffff:raise ValueError('invalid frame sequence')
    if previous_sequence is not None and sequence<=previous_sequence:raise ValueError('repeated/reversed driver sequence')
    if type(flags) is not int or flags&0x40:raise ValueError('driver buffer error')
    if flags&0xe000!=0x2000 or flags&0x70000!=0x10000:
        raise ValueError('expected MONOTONIC START_OF_EXPOSURE driver flags')
    stamp=metadata.get('driver_timestamp_s');host=metadata.get('host_dequeue_monotonic_s')
    if any(type(v) not in (int,float) or not math.isfinite(v) for v in (stamp,host)) or not 0<stamp<=host:
        raise ValueError('invalid driver/host timestamp')


class LatestV4L2Capture:
    def __init__(self,executable,device,width,height,timeout=5.):
        self.width,self.height,self.timeout=width,height,timeout
        self.condition=threading.Condition();self.latest=None;self.error=None;self.closed=False
        self.format=None;self.buffers=None;self.received=0;self.replaced=0
        self.stderr=tempfile.TemporaryFile()
        try:
            self.process=subprocess.Popen([str(executable),device,str(width),str(height)],
                stdout=subprocess.PIPE,stderr=self.stderr,start_new_session=True)
        except BaseException:
            self.stderr.close();raise
        self.thread=threading.Thread(target=self._drain,name='sie-latest-camera',daemon=True);self.thread.start()

    def _drain(self):
        previous=None
        try:
            while not self.closed:
                line=self.process.stdout.readline(16385)
                if not line or len(line)>16384 or not line.endswith(b'\n'):raise EOFError('native camera header unavailable')
                item=json.loads(line)
                if item.get('kind')=='format':
                    if self.format is not None or (item.get('width'),item.get('height'),item.get('fourcc'))!=(self.width,self.height,1196444237):
                        raise ValueError('native camera format mismatch')
                    self.format=item;continue
                if item.get('kind')=='buffers':
                    if self.format is None or self.buffers is not None or item.get('allocated',0)<1:raise ValueError('invalid native buffers')
                    self.buffers=item;continue
                if self.format is None or self.buffers is None:raise ValueError('native frame before configuration')
                validate_packet(item,self.width,self.height,previous);previous=item['sequence']
                payload=read_exact(self.process.stdout,item['payload_bytes'])
                if item.get('warmup'):continue
                item={**item,'host_packet_received_monotonic_s':time.monotonic()}
                with self.condition:
                    self.received+=1
                    if self.latest is not None:self.replaced+=1
                    self.latest=(payload,item);self.condition.notify_all()
        except Exception as error:
            with self.condition:
                if not self.closed:self.error=f'{type(error).__name__}: {error}'
                self.latest=None;self.condition.notify_all()

    def snapshot(self,after_sequence=None):
        deadline=time.monotonic()+self.timeout
        with self.condition:
            while True:
                if self.closed:raise RuntimeError('camera closed')
                if self.error:raise RuntimeError(self.error)
                if self.latest is not None and (after_sequence is None or self.latest[1]['sequence']>after_sequence):
                    payload,metadata=self.latest
                    return payload,{**metadata,'received_count':self.received,'replaced_latest_count':self.replaced,
                                    'driver_format':self.format,'driver_buffers':self.buffers}
                left=deadline-time.monotonic()
                if left<=0:raise TimeoutError('no new native camera frame')
                self.condition.wait(left)

    def release(self):
        with self.condition:self.closed=True;self.condition.notify_all()
        if self.process.poll() is None:
            self.process.terminate()
            try:self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=2)
        self.thread.join(timeout=2)
        if self.process.stdout:self.process.stdout.close()
        self.stderr.close()


def freshness_report(metadata,ready,executable_sha256):
    ages={name:ready-data['driver_timestamp_s'] for name,data in metadata.items() if 'driver_timestamp_s' in data}
    paired=set(ages)=={'ar0234','ov9281'}
    return {"capture_backend":"latest_v4l2","frames":metadata,"native_executable_sha256":executable_sha256,
        "driver_declared_soe_to_result_s":ages,
        "driver_declared_soe_to_pair_selection_s":{name:data['pair_selected_monotonic_s']-data['driver_timestamp_s'] for name,data in metadata.items() if 'pair_selected_monotonic_s' in data and 'driver_timestamp_s' in data},
        "pair_selection_to_result_s":{name:ready-data['pair_selected_monotonic_s'] for name,data in metadata.items() if 'pair_selected_monotonic_s' in data},
        "maximum_driver_declared_age_s":max(ages.values()) if paired else None,
        "driver_declared_pair_skew_s":abs(metadata['ar0234']['driver_timestamp_s']-metadata['ov9281']['driver_timestamp_s']) if paired else None,
        "within_250ms_by_driver_metadata":paired and all(math.isfinite(v) and 0<=v<=.25 for v in ages.values()),
        "observation_timestamp_basis":"HOST_CYCLE_START_UTC; driver stamps provided separately",
        "physical_exposure_timing_independently_verified":False,"command_issued":False,
        "endpoint":"RESULT_READY_BEFORE_PUBLICATION; not a motor command"}


def select_latest_pair(ar_capture,stereo_capture,last_sequences):
    """Freeze both compressed snapshots before any image decode; no history search."""
    ar=ar_capture.snapshot(last_sequences.get('ar0234'))
    stereo=stereo_capture.snapshot(last_sequences.get('ov9281'))
    # Return only a complete pair. Metadata/payload snapshots are independently owned.
    selected=time.monotonic()
    return {name:(packet[0],{**packet[1],'pair_selected_monotonic_s':selected}) for name,packet in (('ar0234',ar),('ov9281',stereo))}


def diagnostic_freshness_groups(rows):
    """Keep fast refusals out of coherent-depth age/latency success counts."""
    import statistics
    groups={}
    for label in ('COHERENT_DEPTH','OTHER_RESULTS'):
        selected=[]
        for row in rows:
            coherent=(row.get('status') in ('MASKED_DEPTH_SHADOW_DIAGNOSTIC_ONLY','SHAPE_AWARE_SHADOW_DIAGNOSTIC_ONLY')
                and row.get('shape_diagnostic',{}).get('mask_depth',{}).get('status')=='COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY')
            if coherent==(label=='COHERENT_DEPTH'):selected.append(row)
        fresh=[row['frame_freshness'] for row in selected if row.get('frame_freshness')]
        durations=[row.get('processing_timing',{}).get('host_capture_start_to_result_ready_s') for row in selected]
        durations=[v for v in durations if type(v) in (float,int) and math.isfinite(v) and v>=0]
        groups[label]={'cycles':len(selected),'with_driver_metadata':len(fresh),
            'driver_age_within_250ms':sum(f.get('within_250ms_by_driver_metadata') is True for f in fresh),
            'host_median_s':statistics.median(durations) if durations else None,
            'host_max_s':max(durations) if durations else None,
            'host_within_250ms':sum(v<=.25 for v in durations),'host_timed_cycles':len(durations)}
    return groups
