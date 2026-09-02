from __future__ import annotations
import hashlib
from pathlib import Path
import numpy as np
import pytest
from vision_core.mp_persondet_challenge import ChallengeError, checked_model_buffer, decode, preprocess, status_for_count, Transform
def test_preprocess_shape_range_and_bgr_to_rgb():
 image=np.zeros((100,200,3),np.uint8); image[0,0]=[0,0,255]; blob,_=preprocess(image); assert blob.shape==(1,3,224,224) and blob.dtype==np.float32 and blob.min()>=-1 and blob.max()<=1 and blob[0,0,56,0]==1
def test_anchor_contract():
 from vision_core.mp_persondet_challenge import anchors
 assert anchors(Path('/home/stanislav/dev_ws/model_artifacts/opencv_mp_persondet_2023mar/mp_persondet.reference.py')).shape==(2254,2)
def test_decode_and_reverse_mapping():
 boxes=np.zeros((1,2254,12),np.float32); scores=np.full((1,2254,1),-100,np.float32); boxes[0,0,:4]=[112,112,100,100]; scores[0,0,0]=10; result=decode([boxes,scores],np.zeros((2254,2),np.float32),Transform(1,0,0,224,224)); assert len(result)==1 and 0<=result[0]['confidence']<=1
@pytest.mark.parametrize(('count','status'),[(0,'PERSON_LOST'),(1,'SINGLE_PERSON'),(2,'MULTIPLE_PERSONS'),(3,'MULTIPLE_PERSONS')])
def test_count_policy(count,status): assert status_for_count(count)==status
@pytest.mark.parametrize('status',['PHASE_A_COMPLETE_PHASE_B_PENDING','COMPLETE'])
def test_finalized_dataset_statuses_are_accepted(status):
 from vision_core.mp_persondet_challenge import validate_dataset_status
 assert validate_dataset_status(status) == status
@pytest.mark.parametrize('status',[None, '', 'IN_PROGRESS', 1, True])
def test_unfinalized_or_invalid_dataset_statuses_block(status):
 from vision_core.mp_persondet_challenge import validate_dataset_status
 with pytest.raises(ChallengeError, match='DATASET_NOT_PHASE_A_FINAL'): validate_dataset_status(status)
def test_model_mismatch_blocks_loader(tmp_path):
 p=tmp_path/'x.onnx'; p.write_bytes(b'x')
 with pytest.raises(ChallengeError,match='MODEL_SHA256_MISMATCH'): checked_model_buffer(p)
def test_threshold_sweep_count_metrics_are_deterministic():
 from vision_core.mp_persondet_challenge import _scores
 scores=_scores([np.zeros((1,2254,12),np.float32), np.array([[[0.0]]*2254],np.float32)])
 assert scores.shape == (2254,)
 assert int(np.count_nonzero(scores >= .5)) == 2254
def _rows(expected, detected=None):
 detected = expected if detected is None else detected
 return [{'expected_person_count': e, 'detected_count': d, 'inference_latency_ms': float(i+1), 'threshold_sweep': [{'threshold': t, 'detected_count': d} for t in (0.05,0.10,0.15,0.20,0.25,0.30,0.40,0.50)]} for i,(e,d) in enumerate(zip(expected,detected))]
def test_dataset_metrics_generalize_to_four_single_records():
 from vision_core.mp_persondet_challenge import calculate_threshold_metrics
 assert calculate_threshold_metrics(_rows([1,1,1,1]), (0.5,))[0]['exact_count_accuracy'] == 1.0
def test_dataset_metrics_ignore_record_order_and_compute_totals():
 from vision_core.mp_persondet_challenge import calculate_threshold_metrics
 result=calculate_threshold_metrics(_rows([1,0,1,0], [1,2,1,1]), (0.5,))[0]
 assert result['empty_false_positives'] == 2 and result['single_person_detections'] == 2
def test_empty_records_and_invalid_expected_count_block():
 from vision_core.mp_persondet_challenge import calculate_threshold_metrics
 with pytest.raises(ChallengeError, match='EMPTY_DATASET_RECORDS'): calculate_threshold_metrics([], (0.5,))
 for value in (True, None, 3):
  with pytest.raises(ChallengeError, match='INVALID_EXPECTED_PERSON_COUNT'): calculate_threshold_metrics(_rows([value]), (0.5,))
def test_original_three_empty_thirteen_single_shape_remains_supported():
 from vision_core.mp_persondet_challenge import calculate_threshold_metrics
 result=calculate_threshold_metrics(_rows([0,0,0]+[1]*13, [1,1,1]+[1]*13), (0.5,))[0]
 assert result['empty_false_positives'] == 3 and result['single_person_detections'] == 13
