import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ros2/sie_ros2/sie_ros2'))
from vision_core.person_localization.rgb_shared_buffer import publish_rgb,receive_rgb,binding
from preview_mask_client import PreviewMaskClient


class SharedRgbTests(unittest.TestCase):
    def test_exact_strided_input_cycle_hash_cleanup_and_owned_lifetime(self):
        rgb=np.arange(1200*1920*3,dtype=np.uint8).reshape(1200,1920,3)[:,::-1]
        with publish_rgb('one',rgb) as (descriptor,sha):
            path=Path(descriptor['path']);self.assertTrue(path.exists())
            owned=receive_rgb(descriptor,'one',sha)
            np.testing.assert_array_equal(owned,rgb)
            self.assertTrue(owned.flags.owndata)
            with self.assertRaises(ValueError):receive_rgb(descriptor,'old',sha)
            with self.assertRaises(ValueError):receive_rgb(descriptor,'one','wrong')
            with path.open('r+b') as stream:stream.write(b'changed')
            with self.assertRaisesRegex(ValueError,'content SHA'):receive_rgb(descriptor,'one',sha)
        self.assertFalse(path.exists());np.testing.assert_array_equal(owned,rgb)

    def test_invalid_shape_and_truncated_buffer_refuse(self):
        with self.assertRaises(ValueError):
            with publish_rgb('one',np.zeros((1,2,3),'uint8')):pass
        with publish_rgb('one',np.zeros((1200,1920,3),'uint8')) as (descriptor,sha):
            with open(descriptor['path'],'r+b') as stream:stream.truncate(1)
            with self.assertRaisesRegex(ValueError,'size/type'):receive_rgb(descriptor,'one',sha)

    def test_client_validates_ack_and_records_shared_transport_timing(self):
        client=PreviewMaskClient.__new__(PreviewMaskClient);client.model_sha256='model';client.timeout=1
        sent=[]
        client.process=SimpleNamespace(stdin=SimpleNamespace(write=lambda line:sent.append(json.loads(line)),flush=lambda:None))
        client._read=lambda timeout:{'cycle_id':'one','arrays_sha256':sent[-1]['arrays_sha256'],
            'model_sha256':'model','status':'CURRENT_INSTANCES_DIAGNOSTIC_ONLY'}
        result=client._shared_rgb_request('one',np.zeros((1200,1920,3),'uint8'),False)
        self.assertEqual(result['rgb_transport'],'mmap')
        self.assertIn('shared_publish_s',result['rpc_timing_s'])
        self.assertFalse(Path(sent[-1]['rgb_shared']['path']).exists())
        client._read=lambda timeout:{'cycle_id':'old'}
        closed=[];client.close=lambda:closed.append(True)
        with self.assertRaisesRegex(ValueError,'cycle mismatch'):
            client._shared_rgb_request('one',np.zeros((1200,1920,3),'uint8'),False)
        self.assertEqual(closed,[True])

    def test_client_failure_stops_worker_before_owner_cleanup(self):
        client=PreviewMaskClient.__new__(PreviewMaskClient);client.model_sha256='model'
        client.process=SimpleNamespace(stdin=SimpleNamespace(write=lambda line:None,flush=lambda:None))
        client.timeout=1
        def read(timeout):raise TimeoutError('test')
        client._read=read;closed=[]
        client.close=lambda:closed.append(True)
        before=set(Path('/dev/shm').glob('sie-rgb-*'))
        with self.assertRaises(TimeoutError):client._shared_rgb_request('one',np.zeros((1200,1920,3),'uint8'),False)
        self.assertEqual(closed,[True]);self.assertEqual(set(Path('/dev/shm').glob('sie-rgb-*')),before)


if __name__=='__main__':unittest.main()
