"""Exact-cycle source snapshot packaging, bounded writer, no cameras or ROS."""
import hashlib
import json
from pathlib import Path
import tempfile

import numpy as np
import cv2
import pytest

from vision_core.person_localization.shadow_source_snapshot import (
    LatestSourceSnapshotWriter, capture_reason, pack_mask, unpack_mask)
from vision_core.tools.summarize_shadow_source_snapshots import summarize


def test_snapshot_triggers_keep_point_sets_separate():
    depth={'status':'COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY'}
    all_points={'known_term_m':{'p05':1.05}}
    candidate={'known_term_m':{'p05':2.1}}
    assert capture_reason(depth,all_points,candidate)=='FRONT_POINT_SET_DISAGREEMENT'
    assert capture_reason(depth,all_points,{'known_term_m':{'p05':1.10}}) is None
    assert capture_reason(depth,all_points,{'known_term_m':{'p05':1.10}},
                          reacquire=True)=='FIRST_COHERENT_AFTER_TRACK_LOSS'
    assert capture_reason({'status':'DEPTH_MAD_EXCEEDED'},all_points,candidate,
                          periodic=True)=='PERIODIC_SELECTED_INSTANCE'
    assert capture_reason(depth,all_points,{},periodic=True)=='PERIODIC_SELECTED_INSTANCE'


def test_source_bundle_preserves_original_jpegs_mask_and_stereo():
    mask=np.zeros((1200,1920),dtype=bool)
    mask[100:250,300:480]=True
    payload=pack_mask(mask)
    np.testing.assert_array_equal(unpack_mask(payload),mask)
    with pytest.raises(ValueError,match='hash mismatch'):
        unpack_mask({**payload,'mask_sha256':'0'*64})
    rgb_frame=np.zeros((1200,1920,3),dtype=np.uint8)
    stereo_frame=np.zeros((800,2560,3),dtype=np.uint8)
    rgb=cv2.imencode('.jpg',rgb_frame)[1].tobytes()
    stereo=cv2.imencode('.jpg',stereo_frame)[1].tobytes()
    digest_rgb=hashlib.sha256(cv2.imdecode(
        np.frombuffer(rgb,dtype=np.uint8),cv2.IMREAD_COLOR).tobytes()).hexdigest()
    disparity=np.full((400,640),12.,np.float32)
    lr=np.ones((400,640),dtype=bool)
    cal={'P1':np.eye(3,4),'P2':np.eye(3,4),'R1':np.eye(3),
         'R':np.eye(3),'T':np.zeros(3),'K':np.eye(3),
         'D':np.zeros(5),'size':np.asarray([1280,800])}
    with tempfile.TemporaryDirectory() as directory:
        writer=LatestSourceSnapshotWriter(Path(directory)/'snapshots',max_records=1)
        stamps={'ar0234':{'driver_timestamp_s':9.75},
                'ov9281':{'driver_timestamp_s':9.76}}
        assert writer.submit('cycle-00000001',rgb,stereo,disparity,lr,cal,payload,
                             {'frame_freshness':{'frames':stamps},
                              'track_id':'track-1','selected_instance_index':0,
                              'segmentation_model_sha256':'model-sha',
                              'segmentation_rgb_payload_sha256':digest_rgb})
        disparity[:]=0
        lr[:]=False
        assert not writer.submit('cycle-00000002',rgb,stereo,disparity,lr,cal,payload,{})
        writer.close()
        assert (writer.accepted,writer.completed,writer.skipped,writer.errors)==(1,1,1,0)
        base=Path(directory)/'snapshots'/'cycle-00000001'
        manifest=json.loads(base.with_suffix('.json').read_text())
        blob=base.with_suffix('.npz')
        assert manifest['npz_sha256']==hashlib.sha256(blob.read_bytes()).hexdigest()
        assert manifest['instance_mask_sha256']==payload['mask_sha256']
        assert manifest['frame_freshness']['frames']['ar0234']['driver_timestamp_s']==9.75
        with np.load(blob,allow_pickle=False) as bundle:
            assert bundle['rgb_original_jpeg'].tobytes()==rgb
            assert bundle['stereo_original_jpeg'].tobytes()==stereo
            np.testing.assert_array_equal(bundle['instance_mask'],mask)
            assert np.all(bundle['disparity_matching_px']==12.)
            assert bundle['lr_consistent'].all()
            np.testing.assert_array_equal(bundle['cal_P1'],cal['P1'])
        journal=Path(directory)/'bridge.jsonl'
        journal.write_text(json.dumps({'cycle_id':'cycle-00000001',
            'source_snapshot_evidence':{'mask_sha256':payload['mask_sha256']},
            'track_id':'track-1','selected_mask_instance_index':0,
            'segmentation_model_sha256':'model-sha',
            'driver_timestamp_s':{'ar0234':9.75,'ov9281':9.76}})+'\n')
        result=summarize(journal,Path(directory)/'snapshots')
        assert (result['requested_after_publish'],result['validated_bundles'])==(1,1)
        assert not result['errors'] and not result['missing_requested_cycles']
        journal.write_text(json.dumps({'cycle_id':'cycle-00000001',
            'source_snapshot_evidence':{'mask_sha256':payload['mask_sha256']},
            'track_id':'track-1','selected_mask_instance_index':0,
            'segmentation_model_sha256':'model-sha',
            'driver_timestamp_s':{'ar0234':9.75,'ov9281':9.77}})+'\n')
        assert summarize(journal,Path(directory)/'snapshots')['errors'][0]['detail']=='driver timestamp mismatch'
