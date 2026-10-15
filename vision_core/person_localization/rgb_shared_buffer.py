"""Single-request RGB mmap transport, parent-owned RAM file, no persistent cache."""
from contextlib import contextmanager
import hashlib
import json
import mmap
import os
from pathlib import Path
import secrets
import stat
import tempfile

SHAPE=(1200,1920,3)
NBYTES=1200*1920*3


def binding(descriptor):
    return hashlib.sha256(json.dumps(descriptor,sort_keys=True,separators=(',',':')).encode()).hexdigest()


@contextmanager
def publish_rgb(cycle_id,rgb):
    import numpy as np
    if not isinstance(cycle_id,str) or not cycle_id or rgb.shape!=SHAPE or rgb.dtype!=np.uint8:
        raise ValueError('invalid current RGB shared input')
    with tempfile.TemporaryDirectory(prefix='sie-rgb-',dir='/dev/shm') as folder:
        path=Path(folder)/'current.rgb'
        with path.open('x+b') as stream:
            stream.truncate(NBYTES)
            with mmap.mmap(stream.fileno(),NBYTES,access=mmap.ACCESS_WRITE) as region:
                view=np.ndarray(SHAPE,dtype=np.uint8,buffer=region)
                view[:]=rgb
                sha=hashlib.sha256(memoryview(region)).hexdigest()
                del view
                descriptor={'protocol':'sie.rgb_mmap.v1','cycle_id':cycle_id,'path':str(path),
                    'shape':list(SHAPE),'dtype':'uint8','nbytes':NBYTES,
                    'token':secrets.token_hex(16),'rgb_payload_sha256':sha}
                # Owner keeps the mapping alive until validated response or worker stop.
                yield descriptor,binding(descriptor)


def receive_rgb(descriptor,cycle_id,expected_binding):
    import numpy as np
    if (binding(descriptor)!=expected_binding or descriptor.get('protocol')!='sie.rgb_mmap.v1'
            or descriptor.get('cycle_id')!=cycle_id or descriptor.get('shape')!=list(SHAPE)
            or descriptor.get('dtype')!='uint8' or descriptor.get('nbytes')!=NBYTES
            or not isinstance(descriptor.get('token'),str) or len(descriptor['token'])!=32):
        raise ValueError('shared RGB descriptor/cycle mismatch')
    path=Path(descriptor['path'])
    if path.name!='current.rgb' or path.parent.parent!=Path('/dev/shm') or not path.parent.name.startswith('sie-rgb-'):
        raise ValueError('unexpected shared RGB path')
    fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW)
    try:
        info=os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_size!=NBYTES:raise ValueError('shared RGB size/type mismatch')
        with mmap.mmap(fd,NBYTES,access=mmap.ACCESS_READ) as region:
            if hashlib.sha256(memoryview(region)).hexdigest()!=descriptor['rgb_payload_sha256']:
                raise ValueError('shared RGB content SHA mismatch')
            view=np.ndarray(SHAPE,dtype=np.uint8,buffer=region)
            # Ultralytics may retain orig_img after prediction. An owned copy
            # prevents stale mapped pointers after acknowledgement/unlink.
            result=view.copy();del view
            return result
    finally:os.close(fd)
