#!/usr/bin/env python3
"""Camera-only continuous Shadow v2 presence and reacquisition diagnostic.

The camera is drained at 30 fps by a native process. Every processed snapshot
is new and captured after the preceding inference; up to one original JPEG per
second is retained for later operator review. No ROS, UART, network or motors.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import secrets
import shutil
import statistics
import sys
import tempfile
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'ros2/sie_ros2'))

from sie_ros2.latest_v4l2_capture import LatestV4L2Capture  # noqa: E402
from vision_core.person_localization import yolo11_person_upper_body as yolo  # noqa: E402
from vision_core.tools.capture_shadow_rgb_presence import (  # noqa: E402
    DEVICE, WIDTH, HEIGHT, configure_camera, compile_capture,
)


V2_SHA256 = '5583d645afebd3d52706b76655662cb4ce16110b2a0b224af2cc806d0292d07b'
MAX_AGE_S = 0.5
CONFIDENCE = 0.40
NMS_IOU = 0.45
DISK_RESERVE_BYTES = 1024 * 1024 * 1024
MAX_SAVED_BYTES = 1024 * 1024 * 1024


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def result_state(age_s: float, count: int) -> str:
    if not math.isfinite(age_s) or age_s < 0 or age_s > MAX_AGE_S:
        return 'STALE'
    return 'NO_TARGET' if count == 0 else 'SINGLE_TARGET' if count == 1 else 'MULTIPLE_TARGETS'


def summarize(rows: list[dict]) -> dict:
    states = {name: sum(r['state'] == name for r in rows) for name in
              ('STALE', 'NO_TARGET', 'SINGLE_TARGET', 'MULTIPLE_TARGETS')}
    fresh_ages = [r['driver_soe_to_result_s'] for r in rows if r['state'] != 'STALE']
    all_ages = [r['driver_soe_to_result_s'] for r in rows]
    gaps = [b['host_result_monotonic_s']-a['host_result_monotonic_s']
            for a,b in zip(rows, rows[1:])]
    return {'processed_frames': len(rows), 'states': states,
            'first_acquisitions': sum(r['transition'] == 'ACQUIRED' for r in rows),
            'loss_events': sum(r['transition'] == 'LOST' for r in rows),
            'reacquisition_events': sum(r['transition'] == 'REACQUIRED' for r in rows),
            'ambiguous_continuous_switches': sum(r['transition'] == 'UNRESOLVED_SWITCH' for r in rows),
            'fresh_fraction': len(fresh_ages)/len(rows) if rows else None,
            'driver_soe_to_result_median_ms': round(statistics.median(all_ages)*1000,2) if rows else None,
            'driver_soe_to_result_p95_ms': round(sorted(all_ages)[int(.95*(len(rows)-1))]*1000,2) if rows else None,
            'driver_soe_to_result_max_ms': round(max(all_ages)*1000,2) if rows else None,
            'result_gap_median_ms': round(statistics.median(gaps)*1000,2) if gaps else None,
            'result_gap_max_ms': round(max(gaps)*1000,2) if gaps else None}


def session(*, output: Path, model: Path, duration_s: float,
            scenario: str, projector_state: str, motor_power_state: str,
            sample_interval_s: float = 1.0) -> dict:
    if (not 5 <= duration_s <= 600 or not .5 <= sample_interval_s <= 10
            or not scenario.strip() or projector_state != 'off' or motor_power_state != 'off'):
        raise ValueError('INVALID_DIAGNOSTIC_SETTINGS')
    if sha256(model) != V2_SHA256:
        raise ValueError('SHADOW_V2_MODEL_SHA256_MISMATCH')
    if yolo.MODEL_SHA256 != 'dde42238b5742f9c0b79c29863c44bc97b678aa75b86a4db5bb602a2d72c259c':
        raise ValueError('PREPROCESS_CONTRACT_MISMATCH')
    if output.exists():
        raise FileExistsError(output)
    os.umask(0o077)
    output.mkdir(parents=True, mode=0o700)
    (output/'images').mkdir(mode=0o700)
    report = {'schema_version': 'sie.shadow.rgb_v2_live_presence_diagnostic.v1',
              'status': 'FAILED', 'started_at_utc': utc_now(),
              'scenario_operator_description': scenario,
              'model_sha256': V2_SHA256, 'preprocess_sha256': sha256(Path(yolo.__file__)),
              'tool_sha256': sha256(Path(__file__)),
              'camera_device': DEVICE, 'camera': None,
              'duration_requested_s': duration_s,
              'sample_interval_s': sample_interval_s,
              'confidence_threshold': CONFIDENCE, 'nms_iou_threshold': NMS_IOU,
              'freshness_max_driver_soe_to_result_s': MAX_AGE_S,
              'projector_state_operator_reported': projector_state,
              'motor_power_state_operator_reported': motor_power_state,
              'actuator_access': False, 'network_requests': 0,
              'motor_posts': 0, 'metric_measurement_authorized': False,
              'navigation_eligible': False,
              'operator_truth_labels': 'NOT_COLLECTED',
              'identity_recognition': False, 'images_saved': 0, 'bytes_saved': 0}
    capture = None
    rows = []
    started = None
    try:
        if shutil.disk_usage(output).free < DISK_RESERVE_BYTES + MAX_SAVED_BYTES:
            raise RuntimeError('INSUFFICIENT_DISK_SPACE')
        report['camera'] = configure_camera(DEVICE)
        binary, capture_sha = compile_capture(output)
        report['capture_source_sha256'] = capture_sha
        report['capture_binary_sha256'] = sha256(binary)
        import onnxruntime as ort
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 2
        ort_session = ort.InferenceSession(str(model), sess_options=opts,
                                            providers=['CPUExecutionProvider'])
        inputs = ort_session.get_inputs()
        if len(inputs) != 1 or tuple(inputs[0].shape) != (1,3,640,640):
            raise RuntimeError('INVALID_ONNX_INPUT_SHAPE')
        report['onnxruntime_version'] = ort.__version__
        report['opencv_version'] = cv2.__version__
        previous_tempdir = tempfile.tempdir
        tempfile.tempdir = str(output)
        try:
            capture = LatestV4L2Capture(binary, DEVICE, WIDTH, HEIGHT, timeout=5.0)
        finally:
            tempfile.tempdir = previous_tempdir
        capture.snapshot()  # Wait for a valid post-warmup packet.
        started = time.monotonic()
        previous_sequence = None
        previous_state = 'NO_TARGET'
        previous_box = None
        saw_target = False
        segment_id = 0
        last_saved_at = -math.inf
        next_progress = started + 15
        print(f'LIVE_RGB_V2_STARTED {output} duration_s={duration_s}',flush=True)
        with (output/'frames.jsonl').open('x') as journal:
            while time.monotonic() - started < duration_s:
                requested_at = time.monotonic()
                payload, meta = capture.snapshot(previous_sequence,
                    not_before_received_s=requested_at)
                previous_sequence = meta['sequence']
                frame = cv2.imdecode(np.frombuffer(payload,np.uint8),cv2.IMREAD_COLOR)
                if frame is None or frame.shape != (HEIGHT,WIDTH,3):
                    raise RuntimeError('INVALID_LIVE_JPEG')
                tensor, transform = yolo.letterbox_bgr(frame)
                outputs = ort_session.run(None, {inputs[0].name:tensor})
                if len(outputs) != 1:
                    raise RuntimeError('INVALID_ONNX_OUTPUT')
                detections = yolo.decode_yolo11_one_class_output(
                    outputs[0],transform=transform,
                    confidence_threshold=CONFIDENCE,nms_iou_threshold=NMS_IOU)
                ready = time.monotonic()
                age = ready - meta['driver_timestamp_s']
                state = result_state(age,len(detections))
                transition = None
                if state == 'SINGLE_TARGET':
                    current_box = detections[0].bbox_xyxy_px
                    if previous_state != 'SINGLE_TARGET':
                        segment_id += 1
                        transition = 'REACQUIRED' if saw_target else 'ACQUIRED'
                    elif previous_box is not None:
                        # An abrupt non-overlap is unresolved identity, never
                        # evidence that the same person is still tracked.
                        a,b = previous_box,current_box
                        ix = max(0,min(a[2],b[2])-max(a[0],b[0]))
                        iy = max(0,min(a[3],b[3])-max(a[1],b[1]))
                        intersection = ix*iy
                        area_a=(a[2]-a[0])*(a[3]-a[1])
                        area_b=(b[2]-b[0])*(b[3]-b[1])
                        if intersection / (area_a+area_b-intersection) < .1:
                            segment_id += 1
                            transition = 'UNRESOLVED_SWITCH'
                    previous_box = current_box
                    saw_target = True
                else:
                    if previous_state == 'SINGLE_TARGET':
                        transition = 'LOST'
                    previous_box = None
                previous_state = state
                saved = None
                if ready - last_saved_at >= sample_interval_s:
                    if (report['bytes_saved']+len(payload)>MAX_SAVED_BYTES or
                            shutil.disk_usage(output).free-len(payload)<DISK_RESERVE_BYTES):
                        raise RuntimeError('EVIDENCE_DISK_LIMIT_EXCEEDED')
                    saved = f'images/ar0234_{len(rows)+1:06d}.jpg'
                    (output/saved).write_bytes(payload)
                    report['images_saved'] += 1
                    report['bytes_saved'] += len(payload)
                    last_saved_at = ready
                row = {'ordinal':len(rows)+1,'driver_sequence':meta['sequence'],
                       'driver_soe_monotonic_s':meta['driver_timestamp_s'],
                       'host_packet_received_monotonic_s':meta['host_packet_received_monotonic_s'],
                       'host_result_monotonic_s':ready,
                       'elapsed_s':ready-started,
                       'driver_soe_to_result_s':age,
                       'packet_received_to_result_s':ready-meta['host_packet_received_monotonic_s'],
                       'state':state,'transition':transition,
                       'visual_segment_id':segment_id if state=='SINGLE_TARGET' else None,
                       'detections':[{'bbox_xyxy_px':list(d.bbox_xyxy_px),
                                      'confidence':d.confidence} for d in detections],
                       'saved_jpeg':saved,
                       'saved_jpeg_sha256':hashlib.sha256(payload).hexdigest() if saved else None}
                journal.write(json.dumps(row,allow_nan=False)+'\n')
                journal.flush()
                rows.append(row)
                if ready >= next_progress:
                    print(f'LIVE_RGB_V2 elapsed_s={ready-started:.0f} frames={len(rows)} '
                          f'state={state} transition={transition}',flush=True)
                    next_progress = ready + 15
        report['status'] = 'COMPLETE'
    except KeyboardInterrupt:
        report['status'] = 'INTERRUPTED'
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
        print('LIVE_RGB_V2_FAILED',report['error'],file=sys.stderr,flush=True)
    finally:
        if capture is not None:
            try:
                capture.release()
            except Exception as error:
                report['status'] = 'FAILED'
                report['capture_cleanup_error'] = f'{type(error).__name__}: {error}'
        report['finished_at_utc'] = utc_now()
        report['elapsed_monotonic_s'] = time.monotonic()-started if started is not None else None
        report['summary'] = summarize(rows)
        journal = output/'frames.jsonl'
        report['frames_jsonl_sha256'] = sha256(journal) if journal.exists() else None
        (output/'session.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        print('SUMMARY',json.dumps({'status':report['status'],**report['summary']}),flush=True)
        print('Saved:',output/'session.json',flush=True)
    return report


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model',required=True,type=Path)
    p.add_argument('--scenario',required=True)
    p.add_argument('--projector-state',required=True,choices=('off',))
    p.add_argument('--motor-power-state',required=True,choices=('off',))
    p.add_argument('--duration-s',type=float,default=600.)
    p.add_argument('--sample-interval-s',type=float,default=1.)
    p.add_argument('--output',type=Path,default=Path.home()/'.local/state/sie/evidence'/
                   ('shadow_rgb_v2_live_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+secrets.token_hex(3)))
    args=p.parse_args()
    report=session(output=args.output,model=args.model,duration_s=args.duration_s,
                   scenario=args.scenario,projector_state=args.projector_state,
                   motor_power_state=args.motor_power_state,
                   sample_interval_s=args.sample_interval_s)
    return 0 if report['status']=='COMPLETE' else 1


if __name__=='__main__':
    raise SystemExit(main())
