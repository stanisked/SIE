import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ros2/sie_ros2/sie_ros2'));sys.path.insert(0,str(ROOT/'vision_core/tools'))
from vision_core.person_localization.jpeg_frame_transport import publish_jpeg,receive_jpeg,binding
from preview_mask_client import PreviewMaskClient
from control_preview_mask_worker import request_bundle

class JpegTransportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        image=np.random.default_rng(3).integers(0,256,(1200,1920,3),dtype=np.uint8)
        ok,encoded=cv2.imencode('.jpg',image)
        if not ok:raise RuntimeError('test JPEG encoding failed')
        cls.payload=encoded.tobytes();cls.reference=cv2.imdecode(encoded,cv2.IMREAD_COLOR)

    def test_exact_pixels_cycle_binding_worker_bundle_and_cleanup(self):
        with publish_jpeg('current',self.payload) as (descriptor,sha):
            path=Path(descriptor['path'])
            self.assertEqual(path.read_bytes(),self.payload)
            image,elapsed=receive_jpeg(descriptor,'current',sha)
            np.testing.assert_array_equal(image,self.reference)
            self.assertGreaterEqual(elapsed,0)
            with request_bundle({'operation':'infer_instances','rgb_transport':'jpeg','cycle_id':'current',
                    'jpeg_frame':descriptor,'arrays_sha256':sha}) as bundle:
                np.testing.assert_array_equal(bundle['rgb'],self.reference)
            with self.assertRaisesRegex(ValueError,'cycle'):receive_jpeg(descriptor,'old',sha)
            with self.assertRaisesRegex(ValueError,'descriptor'):receive_jpeg(descriptor,'current','bad')
        self.assertFalse(path.exists())
        np.testing.assert_array_equal(image,self.reference)

    def test_corruption_size_invalid_image_and_decode_shape_refuse(self):
        with publish_jpeg('current',self.payload) as (descriptor,sha):
            path=Path(descriptor['path']);raw=bytearray(path.read_bytes());raw[10]^=1;path.write_bytes(raw)
            with self.assertRaisesRegex(ValueError,'content SHA'):receive_jpeg(descriptor,'current',sha)
            path.write_bytes(b'x')
            with self.assertRaisesRegex(ValueError,'size/type'):receive_jpeg(descriptor,'current',sha)
        for payload in (b'\xff\xd8bad',cv2.imencode('.jpg',np.zeros((10,10,3),np.uint8))[1].tobytes()):
            with publish_jpeg('current',payload) as (descriptor,sha):
                with self.assertRaisesRegex(ValueError,'decoded frame'):receive_jpeg(descriptor,'current',sha)
        with self.assertRaises(ValueError):
            with publish_jpeg('current',b'not JPEG'):pass

    def test_client_ack_and_failure_stop_before_file_cleanup(self):
        client=PreviewMaskClient.__new__(PreviewMaskClient);client.model_sha256='model';client.timeout=1
        sent=[];closed=[]
        client.process=SimpleNamespace(stdin=SimpleNamespace(write=lambda s:sent.append(json.loads(s)),flush=lambda:None))
        client._read=lambda timeout:{'cycle_id':'current','status':'CURRENT_INSTANCES_DIAGNOSTIC_ONLY',
            'arrays_sha256':sent[-1]['arrays_sha256'],'model_sha256':'model'}
        result=client.infer_jpeg('current',self.payload)
        self.assertEqual(result['rgb_transport'],'jpeg')
        self.assertIn('jpeg_publish_s',result['rpc_timing_s'])
        self.assertFalse(Path(sent[-1]['jpeg_frame']['path']).exists())
        def close():
            self.assertTrue(Path(sent[-1]['jpeg_frame']['path']).exists());closed.append(True)
        client.close=close;client._read=lambda timeout:{'cycle_id':'wrong'}
        with self.assertRaisesRegex(ValueError,'cycle mismatch'):client.infer_jpeg('current',self.payload)
        self.assertEqual(closed,[True]);self.assertFalse(Path(sent[-1]['jpeg_frame']['path']).exists())

if __name__=='__main__':unittest.main()
