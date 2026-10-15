#!/usr/bin/env python3
"""Probe driver frame clocks and one-buffer queuing; no controls/calibration changes."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import platform
import statistics
import subprocess
import tempfile

DEVICES=(('AR0234','/dev/v4l/by-id/usb-DECXIN_CAMERA_DECXIN_CAMERA_01.00.00-video-index0',1920,1200),
         ('OV9281','/dev/v4l/by-id/usb-TSTC_Web_Camera_TSTC_Web_Camera-video-index0',2560,800))


def summarize(records):
    rows=[r for r in records if r.get('kind')=='frame' and not r['warmup']]
    if not rows:return {'status':'NO_POST_WARMUP_FRAMES','frame_count':0}
    clocks=Counter();sources=Counter();normal=[];paused=[];errors=0
    for r in rows:
        flags=r['flags'];clock=flags&0xe000;source=flags&0x70000
        clocks[{0x2000:'MONOTONIC',0x4000:'COPY',0:'UNKNOWN'}.get(clock,'UNRECOGNIZED')]+=1
        sources[{0:'END_OF_FRAME',0x10000:'START_OF_EXPOSURE'}.get(source,'UNRECOGNIZED')]+=1
        if flags&0x40:errors+=1
        stamp=r['driver_timestamp_s'];host=r['host_dequeue_monotonic_s']
        # Compare only clock domains explicitly declared identical by V4L2.
        if clock==0x2000 and stamp>0 and math.isfinite(stamp) and math.isfinite(host):
            (paused if r['after_pause'] else normal).append(host-stamp)
    def stats(values):return {'n':len(values),'median_s':statistics.median(values) if values else None,'max_s':max(values) if values else None,'minimum_s':min(values) if values else None}
    return {'status':'DRIVER_METADATA_DIAGNOSTIC_ONLY','frame_count':len(rows),'clock_flags':dict(clocks),'timestamp_sources':dict(sources),
        'buffer_error_count':errors,'normal_driver_stamp_to_dequeue':stats(normal),
        'after_250ms_read_pause_driver_stamp_to_dequeue':stats(paused),
        'sequence_deltas':[b['sequence']-a['sequence'] for a,b in zip(rows,rows[1:])],
        'exposure_age_established':False,
        'reason':'Driver flags describe timestamp semantics; no independent sensor timing validation or live pipeline binding.'}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    if args.output.exists():p.error('new output required')
    source=Path(__file__).with_name('probe_v4l2_frame_clock.c');results=[]
    with tempfile.TemporaryDirectory(prefix='sie-frame-clock-') as folder:
        executable=Path(folder)/'probe'
        subprocess.run(['cc','-std=c11','-O2','-Wall','-Wextra','-Werror',str(source),'-o',str(executable)],check=True)
        for name,device,w,h in DEVICES:
            result=subprocess.run([str(executable),device,str(w),str(h)],capture_output=True,text=True,timeout=60)
            records=[json.loads(line) for line in result.stdout.splitlines() if line.strip()]
            summary=summarize(records)
            if result.returncode:summary['status']='PROBE_FAILED'
            item={'camera':name,'device':device,'returncode':result.returncode,'stderr':result.stderr,'records':records,'summary':summary}
            results.append(item);print('FRAME_CLOCK',name,json.dumps(summary),flush=True)
    report={'status':'CAMERA_FRAME_CLOCK_PROBE_ONLY','results':results,'platform':platform.platform(),
        'source_sha256':{str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in (source,Path(__file__))},
        'actuator_access':False,'camera_controls_changed':False,'format_changed':False,
        'limitations':['Run with ROS/capture applications stopped. Cameras opened sequentially, no pairing/synchronization assessment.',
            'One buffer requested; driver allocation recorded. Deliberate 250 ms pause demonstrates this probe queue, not OpenCV queue behavior.',
            'No MJPEG decoding/inference. Driver timestamp-to-dequeue age is not full exposure-to-command latency.',
            'EOF cannot be relabeled SOE. SOE flags alone are not independent physical timing validation.',
            'STREAMON/OFF and buffer allocation are temporary; no gain/exposure/format/FPS controls are written.']}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    print('Saved:',args.output)


if __name__=='__main__':main()
