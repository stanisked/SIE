"""Experimental, non-production OpenCV MediaPipe person-detector adapter."""
from __future__ import annotations

import hashlib
import importlib.util
import json
import math
from dataclasses import dataclass
from pathlib import Path
from time import perf_counter_ns

import cv2
import numpy as np

MODEL_SHA256 = "47fd5599d6fa17608f03e0eb0ae230baa6e597d7e8a2c8199fe00abea55a701f"
INPUT_SIZE = 224
SCORE_THRESHOLD = 0.5
NMS_THRESHOLD = 0.3


class ChallengeError(RuntimeError): pass
@dataclass(frozen=True)
class Transform:
    ratio: float; left: int; top: int; width: int; height: int

def checked_model_buffer(path: Path, expected_sha256: str = MODEL_SHA256) -> np.ndarray:
    data = path.read_bytes()
    if hashlib.sha256(data).hexdigest() != expected_sha256: raise ChallengeError("MODEL_SHA256_MISMATCH")
    return np.frombuffer(data, dtype=np.uint8)
def anchors(reference_path: Path) -> np.ndarray:
    spec = importlib.util.spec_from_file_location("pinned_mp_persondet_reference", reference_path)
    if spec is None or spec.loader is None: raise ChallengeError("REFERENCE_ADAPTER_UNAVAILABLE")
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    value = module.MPPersonDet._load_anchors(object())
    if value.shape != (2254, 2): raise ChallengeError("ANCHOR_CONTRACT_MISMATCH")
    return value.astype(np.float32, copy=False)
def preprocess(image_bgr: np.ndarray) -> tuple[np.ndarray, Transform]:
    if image_bgr.dtype != np.uint8 or image_bgr.ndim != 3 or image_bgr.shape[2] != 3: raise ChallengeError("INVALID_BGR_IMAGE")
    height, width = image_bgr.shape[:2]; ratio = min(INPUT_SIZE / height, INPUT_SIZE / width)
    resized_h, resized_w = int(height * ratio), int(width * ratio)
    rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    rgb = (rgb - .5) * 2.0
    rgb = cv2.resize(rgb, (resized_w, resized_h))
    top, left = (INPUT_SIZE - resized_h) // 2, (INPUT_SIZE - resized_w) // 2
    padded = cv2.copyMakeBorder(rgb, top, INPUT_SIZE-resized_h-top, left, INPUT_SIZE-resized_w-left, cv2.BORDER_CONSTANT, value=(0,0,0))
    return np.transpose(padded, (2,0,1))[None].astype(np.float32), Transform(ratio, left, top, width, height)
def _scores(outputs: list[np.ndarray]) -> np.ndarray:
    if outputs[1].shape != (1,2254,1): raise ChallengeError("OUTPUT_CONTRACT_MISMATCH")
    logits = np.clip(outputs[1][0,:,0].astype(np.float64), -100, 100)
    return 1/(1+np.exp(-logits))
def decode(outputs: list[np.ndarray], priors: np.ndarray, transform: Transform, threshold: float = SCORE_THRESHOLD) -> list[dict[str, object]]:
    boxes_blob, scores_blob = outputs
    if boxes_blob.shape != (1,2254,12) or scores_blob.shape != (1,2254,1): raise ChallengeError("OUTPUT_CONTRACT_MISMATCH")
    scores = _scores(outputs)
    delta = boxes_blob[0,:,:4].astype(np.float64); center, wh = delta[:,:2]/INPUT_SIZE, delta[:,2:]/INPUT_SIZE
    xy1=(center-wh/2+priors)*INPUT_SIZE; xy2=(center+wh/2+priors)*INPUT_SIZE
    boxes=np.c_[xy1,xy2]; boxes[:,[0,2]]=(boxes[:,[0,2]]-transform.left)/transform.ratio; boxes[:,[1,3]]=(boxes[:,[1,3]]-transform.top)/transform.ratio
    boxes[:,[0,2]]=np.clip(boxes[:,[0,2]],0,transform.width); boxes[:,[1,3]]=np.clip(boxes[:,[1,3]],0,transform.height)
    keep=cv2.dnn.NMSBoxes(boxes.tolist(), scores.tolist(), threshold, NMS_THRESHOLD, top_k=5000)
    result=[]
    for index in np.asarray(keep).reshape(-1):
        x1,y1,x2,y2=boxes[int(index)]; score=float(scores[int(index)])
        if math.isfinite(score) and 0<=score<=1 and x2>x1 and y2>y1: result.append({"bbox":[float(x1),float(y1),float(x2),float(y2)],"confidence":score})
    return result
def status_for_count(count: int) -> str: return "PERSON_LOST" if count == 0 else "SINGLE_PERSON" if count == 1 else "MULTIPLE_PERSONS"
def load_net(model_path: Path):
    return cv2.dnn.readNetFromONNX(checked_model_buffer(model_path))
def validate_dataset_status(status: object) -> str:
    if type(status) is not str or status not in {"PHASE_A_COMPLETE_PHASE_B_PENDING", "COMPLETE"}:
        raise ChallengeError("DATASET_NOT_PHASE_A_FINAL")
    return status
def run_challenge(dataset_root: Path, model_path: Path, reference_path: Path) -> dict[str, object]:
    manifest_path=dataset_root/'dataset_manifest.json'; manifest=json.loads(manifest_path.read_text())
    validate_dataset_status(manifest.get('status'))
    records=manifest['records']
    if len(records)!=16: raise ChallengeError('EXPECTED_16_PHASE_A_RECORDS')
    net=load_net(model_path); net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV); net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU); priors=anchors(reference_path)
    rows=[]
    for record in records:
        payload=(dataset_root/'frames'/record['image_filename']).read_bytes(); image=cv2.imdecode(np.frombuffer(payload,np.uint8),cv2.IMREAD_COLOR)
        blob,transform=preprocess(image); net.setInput(blob); start=perf_counter_ns(); outputs=net.forward(['Identity:0','Identity_1:0']); latency=(perf_counter_ns()-start)/1e6
        scores = _scores(outputs); sweep = []
        for threshold in (0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50):
            found = decode(outputs, priors, transform, threshold); sweep.append({'threshold': threshold, 'candidates_above_threshold': int(np.count_nonzero(scores >= threshold)), 'detected_count': len(found), 'status': status_for_count(len(found))})
        found=decode(outputs,priors,transform); count=len(found); rows.append({'scenario_id':record['scenario_id'],'expected_person_count':record['expected_person_count'],'max_sigmoid_score':float(np.max(scores)),'threshold_sweep':sweep,'detected_count':count,'status':status_for_count(count),'detections':found,'inference_latency_ms':latency})
    expected=[x['expected_person_count'] for x in rows]; detected=[x['detected_count'] for x in rows]; lat=np.array([x['inference_latency_ms'] for x in rows])
    threshold_metrics = []
    for threshold in (0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50):
        outcomes = [next(x for x in row['threshold_sweep'] if x['threshold'] == threshold) for row in rows]
        counts = [x['detected_count'] for x in outcomes]
        threshold_metrics.append({'threshold': threshold, 'exact_count_accuracy': sum(a == b for a,b in zip(expected, counts))/len(rows), 'empty_false_positives': sum(x['detected_count'] > 0 for x in outcomes[:3]), 'single_person_detections': sum(x['detected_count'] == 1 for x in outcomes[3:]), 'multiple_persons_on_single_expected': sum(x['detected_count'] > 1 for x in outcomes[3:]), 'median_latency_ms': float(np.median(lat)), 'p95_latency_ms': float(np.percentile(lat,95))})
    return {'schema_version':'sie_mp_persondet_ar0234_challenge_v1','challenge_status':'EXPERIMENTAL_THRESHOLD_DIAGNOSTIC_NOT_PRODUCTION_APPROVAL','model_sha256':MODEL_SHA256,'dataset_manifest_sha256':hashlib.sha256(manifest_path.read_bytes()).hexdigest(),'opencv_version':cv2.__version__,'numpy_version':np.__version__,'python_version':__import__('sys').version,'preprocessing':{'channels':'BGR_TO_RGB','input':'NCHW float32 224x224','normalization':'[0,255]->[0,1]->[-1,1]','padding':'centered aspect preserving'},'thresholds': [0.05,0.10,0.15,0.20,0.25,0.30,0.40,0.50], 'nms_threshold':NMS_THRESHOLD,'frames':rows,'threshold_metrics':threshold_metrics,'metrics':{'default_threshold':SCORE_THRESHOLD,'exact_count_accuracy':sum(a==b for a,b in zip(expected,detected))/len(rows),'empty_false_positives':sum(x['detected_count']>0 for x in rows if x['expected_person_count']==0),'single_person_successes':sum(x['detected_count']==1 for x in rows if x['expected_person_count']==1),'single_person_total':13,'median_latency_ms':float(np.median(lat)),'p95_latency_ms':float(np.percentile(lat,95))},'limitation':'Exploratory threshold sweep on 16 frames; do not select a production threshold. Multiple-person behavior is not tested.'}
