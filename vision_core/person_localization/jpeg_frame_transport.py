"""Cycle-bound RAM JPEG transport. Original bytes, no encoding or image resize."""
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path
import secrets
import stat
import tempfile
import time

MAX_BYTES=1200*1920*3+65536


def binding(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':')).encode()).hexdigest()


@contextmanager
def publish_jpeg(cycle_id,payload):
    if not isinstance(cycle_id,str) or not cycle_id or type(payload) is not bytes or not 2<len(payload)<=MAX_BYTES or not payload.startswith(b'\xff\xd8'):
        raise ValueError('invalid current JPEG input')
    with tempfile.TemporaryDirectory(prefix='sie-jpeg-',dir='/dev/shm') as folder:
        path=Path(folder)/'current.jpg';path.write_bytes(payload)
        descriptor={'protocol':'sie.jpeg_frame.v1','cycle_id':cycle_id,'path':str(path),'nbytes':len(payload),
            'payload_sha256':hashlib.sha256(payload).hexdigest(),'token':secrets.token_hex(16),'decoded_shape':[1200,1920,3]}
        yield descriptor,binding(descriptor)


def receive_jpeg(descriptor,cycle_id,expected_binding):
    import cv2
    import numpy as np
    size=descriptor.get('nbytes')
    if (binding(descriptor)!=expected_binding or descriptor.get('protocol')!='sie.jpeg_frame.v1'
            or descriptor.get('cycle_id')!=cycle_id or descriptor.get('decoded_shape')!=[1200,1920,3]
            or type(size) is not int or not 2<size<=MAX_BYTES or not isinstance(descriptor.get('token'),str)
            or len(descriptor['token'])!=32):raise ValueError('JPEG descriptor/cycle mismatch')
    path=Path(descriptor['path'])
    if path.name!='current.jpg' or path.parent.parent!=Path('/dev/shm') or not path.parent.name.startswith('sie-jpeg-'):
        raise ValueError('unexpected JPEG transport path')
    fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW)
    try:
        info=os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_size!=size:raise ValueError('JPEG size/type mismatch')
        with os.fdopen(fd,'rb',closefd=False) as stream:payload=stream.read(size+1)
    finally:os.close(fd)
    if len(payload)!=size or hashlib.sha256(payload).hexdigest()!=descriptor['payload_sha256']:
        raise ValueError('JPEG content SHA mismatch')
    started=time.monotonic()
    rgb=cv2.imdecode(np.frombuffer(payload,dtype=np.uint8),cv2.IMREAD_COLOR)
    elapsed=time.monotonic()-started
    if rgb is None or rgb.shape!=(1200,1920,3) or rgb.dtype!=np.uint8:
        raise ValueError('JPEG decoded frame mismatch')
    return rgb,elapsed
