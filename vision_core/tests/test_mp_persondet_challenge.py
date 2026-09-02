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
def test_model_mismatch_blocks_loader(tmp_path):
 p=tmp_path/'x.onnx'; p.write_bytes(b'x')
 with pytest.raises(ChallengeError,match='MODEL_SHA256_MISMATCH'): checked_model_buffer(p)
def test_threshold_sweep_count_metrics_are_deterministic():
 from vision_core.mp_persondet_challenge import _scores
 scores=_scores([np.zeros((1,2254,12),np.float32), np.array([[[0.0]]*2254],np.float32)])
 assert scores.shape == (2254,)
 assert int(np.count_nonzero(scores >= .5)) == 2254
