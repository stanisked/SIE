#!/usr/bin/env python3
"""Offline evaluation of the v1/v2 AR0234 upper-body ONNX on reviewed development JPEGs.

Reads files only. Does not open a camera, serial port, ROS node or motor API.
The fixed 0.40 confidence is the existing YOLO11 upper-body preview policy.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import platform
import statistics
import sys
import tarfile
import time

MODEL_SHA256 = {
    'v1': 'dde42238b5742f9c0b79c29863c44bc97b678aa75b86a4db5bb602a2d72c259c',
    'v2': '5583d645afebd3d52706b76655662cb4ce16110b2a0b224af2cc806d0292d07b',
}
EXPECTED_LABEL_ARCHIVE_SHA256 = '795511dd6d3e863902630550af347152bfb9a0c99b863c130c699b6d18c6dc44'
CONFIDENCE_THRESHOLD = 0.40
NMS_IOU_THRESHOLD = 0.45
MATCH_IOU_THRESHOLD = 0.50


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def box_iou(a: list[float] | tuple, b: list[float] | tuple) -> float:
    ax1,ay1,ax2,ay2 = a
    bx1,by1,bx2,by2 = b
    area_a = max(0,ax2-ax1)*max(0,ay2-ay1)
    area_b = max(0,bx2-bx1)*max(0,by2-by1)
    intersection = max(0,min(ax2,bx2)-max(ax1,bx1))*max(0,min(ay2,by2)-max(ay1,by1))
    union = area_a + area_b - intersection
    return intersection / union if union else 0.0


def summarize(rows: list[dict]) -> dict:
    positive = [r for r in rows if r['person_visible']]
    negative = [r for r in rows if not r['person_visible']]
    presence_tp = sum(bool(r['detections']) for r in positive)
    localization_tp = sum(r['best_iou'] >= MATCH_IOU_THRESHOLD for r in positive)
    false_positive_frames = sum(bool(r['detections']) for r in negative)
    unmatched_positive_frames = sum(bool(r['detections']) and r['best_iou'] < MATCH_IOU_THRESHOLD for r in positive)
    return {
        'frames':len(rows),'positive':len(positive),'negative':len(negative),
        'presence':{'tp':presence_tp,'fn':len(positive)-presence_tp,
                    'fp':false_positive_frames,'tn':len(negative)-false_positive_frames,
                    'recall':presence_tp/len(positive),
                    'precision':presence_tp/(presence_tp+false_positive_frames) if presence_tp+false_positive_frames else None,
                    'negative_frame_false_positive_rate':false_positive_frames/len(negative)},
        'localization':{'positive_frames_matched_iou_0p50':localization_tp,
                        'positive_frames_unmatched':len(positive)-localization_tp,
                        'positive_frames_with_detection_but_wrong_box':unmatched_positive_frames,
                        'recall':localization_tp/len(positive)},
    }


def verified_labels(archive: Path) -> tuple[dict,list[dict]]:
    if sha256(archive) != EXPECTED_LABEL_ARCHIVE_SHA256:
        raise ValueError('LABEL_ARCHIVE_SHA256_MISMATCH')
    with tarfile.open(archive) as tf:
        manifest = json.load(tf.extractfile('manifest.json'))
        if manifest.get('schema_version') != 'sie.shadow.rgb_upper_body_yolo_validation.v1':
            raise ValueError('INVALID_VALIDATION_MANIFEST')
        if manifest.get('checked_labelme_archive_sha256') != '9ce54b4abf210fefde816e051bbe6c4ed3ecd35953f720d4d6dcf948834ea5e1':
            raise ValueError('CHECKED_LABELME_BINDING_MISMATCH')
        rows = manifest['rows']
        if len(rows)!=120 or len({r['image'] for r in rows})!=120:
            raise ValueError('INVALID_VALIDATION_ROWS')
        for row in rows:
            label = tf.extractfile(row['yolo_label']).read()
            if hashlib.sha256(label).hexdigest()!=row['yolo_label_sha256']:
                raise ValueError('YOLO_LABEL_HASH_MISMATCH:' + row['image'])
    return manifest,rows


def main() -> None:
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--repo-root',required=True,type=Path)
    p.add_argument('--model-version',required=True,choices=sorted(MODEL_SHA256))
    p.add_argument('--model',required=True,type=Path)
    p.add_argument('--images-dir',required=True,type=Path,
                   help='Directory with the original 120 ar0234_*.jpg images')
    p.add_argument('--labels-archive',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path,help='New JSON path; existing file is not overwritten')
    args=p.parse_args()
    if args.output.exists():p.error('--output exists; choose a fresh path')
    if not args.output.parent.is_dir():p.error('--output parent directory missing')
    manifest,labels=verified_labels(args.labels_archive)
    expected_model_sha256=MODEL_SHA256[args.model_version]
    if sha256(args.model)!=expected_model_sha256:
        raise ValueError('MODEL_SHA256_MISMATCH')
    # Verify every source byte before loading ONNX or evaluating anything.
    for row in labels:
        if sha256(args.images_dir/Path(row['image']).name)!=row['source_sha256']:
            raise ValueError('IMAGE_SHA256_MISMATCH:' + row['image'])
    sys.path.insert(0,str(args.repo_root.resolve()))
    import cv2
    import onnxruntime as ort
    from vision_core.person_localization import yolo11_person_upper_body as yolo
    # Keep the production v1 binding intact. This diagnostic explicitly loads
    # its hashed v2 file, without changing the active camera runner's model.
    if yolo.MODEL_SHA256 != MODEL_SHA256['v1']:
        raise ValueError('REPOSITORY_PREPROCESS_CONTRACT_MISMATCH')
    module_sha256=sha256(Path(yolo.__file__))
    session_options=ort.SessionOptions()
    session_options.intra_op_num_threads=2
    session=ort.InferenceSession(str(args.model),sess_options=session_options,
                                 providers=['CPUExecutionProvider'])
    inputs=session.get_inputs()
    if len(inputs)!=1 or tuple(inputs[0].shape)!=(1,3,640,640):
        raise ValueError('UNEXPECTED_MODEL_INPUT')
    evaluated=[]
    for i,row in enumerate(labels,1):
        frame_started=time.perf_counter()
        frame=cv2.imread(str(args.images_dir/Path(row['image']).name),cv2.IMREAD_COLOR)
        if frame is None or frame.shape!=(1200,1920,3):
            raise ValueError('INVALID_JPEG:' + row['image'])
        tensor,transform=yolo.letterbox_bgr(frame)
        infer_started=time.perf_counter()
        outputs=session.run(None,{inputs[0].name:tensor})
        infer_finished=time.perf_counter()
        if len(outputs)!=1:raise ValueError('UNEXPECTED_MODEL_OUTPUT')
        detections=yolo.decode_yolo11_one_class_output(outputs[0],transform=transform,
                         confidence_threshold=CONFIDENCE_THRESHOLD,
                         nms_iou_threshold=NMS_IOU_THRESHOLD)
        frame_finished=time.perf_counter()
        predicted=[{'box_xyxy_px':list(d.bbox_xyxy_px),'confidence':d.confidence}
                   for d in detections]
        best=max((box_iou(row['box_xyxy_px'],d['box_xyxy_px']) for d in predicted),default=0.0) if row['box_xyxy_px'] else None
        evaluated.append({'image':row['image'],'source_sha256':row['source_sha256'],
                          'person_visible':row['person_visible'],
                          'ground_truth_box_xyxy_px':row['box_xyxy_px'],
                          'detections':predicted,'best_iou':best,
                          'inference_ms':round((infer_finished-infer_started)*1000,3),
                          'decode_to_detections_ms':round((frame_finished-frame_started)*1000,3)})
        if i%20==0:print(f'EVALUATED {i}/{len(labels)}',flush=True)
    report={'schema_version':'sie.shadow.rgb_reviewed_dev_onnx.v1',
            'status':'OFFLINE_DIAGNOSTIC_ONLY',
            'model_version':args.model_version,
            'model_sha256':expected_model_sha256,
            'runtime':{'machine':platform.machine(), 'onnxruntime':ort.__version__,
                       'opencv':cv2.__version__, 'provider':'CPUExecutionProvider'},
            'preprocess_postprocess_module_sha256':module_sha256,
            'labels_archive_sha256':sha256(args.labels_archive),
            'source_archive_sha256':manifest['source_archive_sha256'],
            'thresholds':{'confidence':CONFIDENCE_THRESHOLD,
                          'nms_iou':NMS_IOU_THRESHOLD,'ground_truth_iou':MATCH_IOU_THRESHOLD},
            'summary':summarize(evaluated),'frames':evaluated,
            'limitations':['Validation set was manually inspected during development; not a blind test.',
                           'Only this set and exact labeled frames are evaluated.',
                           'Individual 5-second samples are not a continuous tracking or motion test.',
                           'No depth, safe clearance or motor control was evaluated.']}
    report['timing_ms']={name:{'median':round(statistics.median(values),3),
                              'p95':round(sorted(values)[int(.95*(len(values)-1))],3),
                              'max':round(max(values),3)}
                         for name,values in ((field,[r[field] for r in evaluated]) for field in
                             ('inference_ms','decode_to_detections_ms'))}
    with args.output.open('x',encoding='utf-8') as f:
        json.dump(report,f,indent=2,allow_nan=False)
        f.write('\n')
    print('SUMMARY',json.dumps(report['summary'],ensure_ascii=False))
    print('TIMING_MS',json.dumps(report['timing_ms']))
    print('Saved:',args.output)


if __name__=='__main__':main()
