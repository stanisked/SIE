#!/usr/bin/env python3
"""Evaluate a frozen, operator-reviewed Shadow RGB subset offline.

Never opens a camera, serial port, ROS node, or motor interface. This program
reads the exact source JPEGs and reviewed LabelMe JSONs, then runs one ONNX
model using the existing production preprocessing and detection decoder.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import platform
import statistics
import sys
import tarfile
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from vision_core.tools.evaluate_shadow_rgb_dev_onnx import (
    MODEL_SHA256, CONFIDENCE_THRESHOLD, NMS_IOU_THRESHOLD,
    MATCH_IOU_THRESHOLD, box_iou, sha256, summarize,
)

SOURCE_ARCHIVE_SHA256 = 'd7e6ffce46a105204c628b950767e66114d73a1fb4cc467637db5b662580836b'
REVIEWED_ARCHIVE_SHA256 = '2bb99b701c733872cc51e2f6e21e4c66c86438281bee8ae1070fc0418a941fa0'


def verified_subset(subset_dir: Path, reviewed_archive: Path) -> list[dict]:
    if sha256(reviewed_archive) != REVIEWED_ARCHIVE_SHA256:
        raise ValueError('REVIEWED_ARCHIVE_SHA256_MISMATCH')
    manifest = json.loads((subset_dir / 'selection_manifest.json').read_text())
    if manifest.get('selected_count') != 120 or len(manifest.get('frames', [])) != 120:
        raise ValueError('INVALID_SOURCE_MANIFEST')
    expected = {Path(row['file']).stem + '.json': row for row in manifest['frames']}
    if len(expected) != 120:
        raise ValueError('DUPLICATE_SOURCE_IMAGES')
    rows = []
    with tarfile.open(reviewed_archive, 'r:gz') as tf:
        members = tf.getmembers()
        if len(members) != 120 or any(
                not m.isfile() or m.size > 1_000_000 or
                m.name != 'images/' + Path(m.name).name or
                Path(m.name).suffix != '.json' for m in members):
            raise ValueError('INVALID_REVIEWED_ARCHIVE_MEMBERS')
        if {Path(m.name).name for m in members} != set(expected):
            raise ValueError('REVIEWED_LABEL_SET_MISMATCH')
        for member in sorted(members, key=lambda m: m.name):
            source = expected[Path(member.name).name]
            image = subset_dir / source['file']
            if sha256(image) != source['sha256']:
                raise ValueError('SOURCE_IMAGE_HASH_MISMATCH:' + str(image))
            label = json.load(tf.extractfile(member))
            if (label.get('imagePath') != image.name or
                    label.get('imageWidth') != 1920 or label.get('imageHeight') != 1200):
                raise ValueError('LABEL_IMAGE_BINDING_MISMATCH:' + member.name)
            shapes = label.get('shapes')
            if not isinstance(shapes, list) or len(shapes) > 1:
                raise ValueError('INVALID_PERSON_COUNT:' + member.name)
            box = None
            if shapes:
                shape = shapes[0]
                if (shape.get('label') != 'person_upper_body' or
                        shape.get('shape_type') != 'rectangle' or
                        len(shape.get('points', [])) != 2):
                    raise ValueError('INVALID_LABEL_SHAPE:' + member.name)
                (x1, y1), (x2, y2) = shape['points']
                box = [x1, y1, x2, y2]
                if (not all(type(x) in (float, int) and math.isfinite(x) for x in box)
                        or not (0 <= x1 < x2 <= 1920 and 0 <= y1 < y2 <= 1200)):
                    raise ValueError('INVALID_LABEL_BOX:' + member.name)
            if label.get('flags', {}).get('sie_person_visible') != bool(box):
                raise ValueError('LABEL_VISIBILITY_FLAG_MISMATCH:' + member.name)
            rows.append({'image': image.name, 'source_sha256': source['sha256'],
                         'person_visible': bool(box), 'box_xyxy_px': box})
    if sum(row['person_visible'] for row in rows) != 100:
        raise ValueError('UNEXPECTED_LABEL_COUNTS')
    return rows


def evaluate(args: argparse.Namespace) -> dict:
    rows = verified_subset(args.subset_dir, args.reviewed_archive)
    expected_hash = MODEL_SHA256[args.model_version]
    if sha256(args.model) != expected_hash:
        raise ValueError('MODEL_SHA256_MISMATCH')
    import cv2
    import onnxruntime as ort
    from vision_core.person_localization import yolo11_person_upper_body as yolo
    if yolo.MODEL_SHA256 != MODEL_SHA256['v1']:
        raise ValueError('REPOSITORY_PREPROCESS_CONTRACT_MISMATCH')
    options = ort.SessionOptions()
    options.intra_op_num_threads = 2
    session = ort.InferenceSession(str(args.model), sess_options=options,
                                   providers=['CPUExecutionProvider'])
    inputs = session.get_inputs()
    if len(inputs) != 1 or tuple(inputs[0].shape) != (1, 3, 640, 640):
        raise ValueError('UNEXPECTED_MODEL_INPUT')
    evaluated = []
    for i, row in enumerate(rows, 1):
        started = time.perf_counter()
        image = cv2.imread(str(args.subset_dir / 'images' / row['image']), cv2.IMREAD_COLOR)
        if image is None or image.shape != (1200, 1920, 3):
            raise ValueError('INVALID_SOURCE_IMAGE:' + row['image'])
        tensor, transform = yolo.letterbox_bgr(image)
        infer_started = time.perf_counter()
        outputs = session.run(None, {inputs[0].name: tensor})
        infer_end = time.perf_counter()
        if len(outputs) != 1:
            raise ValueError('UNEXPECTED_MODEL_OUTPUT')
        detections = yolo.decode_yolo11_one_class_output(
            outputs[0], transform=transform,
            confidence_threshold=CONFIDENCE_THRESHOLD,
            nms_iou_threshold=NMS_IOU_THRESHOLD)
        completed = time.perf_counter()
        boxes = [{'box_xyxy_px': list(d.bbox_xyxy_px), 'confidence': d.confidence}
                 for d in detections]
        best = max((box_iou(row['box_xyxy_px'], d['box_xyxy_px'])
                    for d in boxes), default=0.0) if row['person_visible'] else None
        evaluated.append({**row, 'ground_truth_box_xyxy_px': row['box_xyxy_px'],
                          'detections': boxes, 'best_iou': best,
                          'inference_ms': round((infer_end - infer_started)*1000, 3),
                          'decode_to_detections_ms': round((completed - started)*1000, 3)})
        if i % 20 == 0:
            print(f'EVALUATED {i}/120', flush=True)
    timings = {name: {'median': round(statistics.median(values), 3),
                      'p95': round(sorted(values)[int(.95 * (len(values)-1))], 3),
                      'max': round(max(values), 3)}
               for name, values in ((field, [r[field] for r in evaluated])
                                    for field in ('inference_ms', 'decode_to_detections_ms'))}
    return {'schema_version': 'sie.shadow.rgb_blind_reviewed_onnx.v1',
            'status': 'OFFLINE_DIAGNOSTIC_ONLY',
            'human_review_attestation': 'Operator stated all 120 frames checked; 65 JSON files changed from visual draft.',
            'model_version': args.model_version, 'model_sha256': expected_hash,
            'runtime': {'machine': platform.machine(), 'onnxruntime': ort.__version__,
                        'opencv': cv2.__version__, 'provider': 'CPUExecutionProvider'},
            'preprocess_postprocess_module_sha256': sha256(Path(yolo.__file__)),
            'reviewed_archive_sha256': sha256(args.reviewed_archive),
            'source_archive_sha256': SOURCE_ARCHIVE_SHA256,
            'thresholds': {'confidence': CONFIDENCE_THRESHOLD, 'nms_iou': NMS_IOU_THRESHOLD,
                           'ground_truth_iou': MATCH_IOU_THRESHOLD},
            'summary': summarize(evaluated), 'timing_ms': timings, 'frames': evaluated,
            'limitations': ['One 5-second sample per image; no continuous tracking test.',
                            'No stereo depth, clearance, or motor behavior evaluated.']}


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model-version', required=True, choices=sorted(MODEL_SHA256))
    p.add_argument('--model', required=True, type=Path)
    p.add_argument('--subset-dir', required=True, type=Path)
    p.add_argument('--reviewed-archive', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    args = p.parse_args()
    if args.output.exists():
        p.error('Output already exists')
    report = evaluate(args)
    with args.output.open('x') as f:
        json.dump(report, f, indent=2, allow_nan=False)
        f.write('\n')
    print('SUMMARY', json.dumps(report['summary']))
    print('Saved:', args.output)


if __name__ == '__main__':
    main()
