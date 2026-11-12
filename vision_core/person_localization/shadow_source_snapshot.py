"""Bounded, opt-in source evidence for motor-off Fast Shadow diagnostics."""
from __future__ import annotations

import base64
import hashlib
import json
import os
from pathlib import Path
import queue
import re
import threading

import numpy as np


def pack_mask(mask):
    if mask.shape != (1200, 1920) or mask.dtype != np.bool_:
        raise ValueError('expected full-resolution current instance mask')
    return {'shape': [1200, 1920],
            'packed_b64': base64.b64encode(np.packbits(mask.reshape(-1)).tobytes()).decode('ascii'),
            'mask_sha256': hashlib.sha256(mask.tobytes()).hexdigest()}


def unpack_mask(value):
    if value.get('shape') != [1200, 1920]:
        raise ValueError('snapshot mask shape mismatch')
    packed = base64.b64decode(value['packed_b64'], validate=True)
    if len(packed) != 1200 * 1920 // 8:
        raise ValueError('snapshot mask byte count mismatch')
    mask = np.unpackbits(np.frombuffer(packed, dtype=np.uint8)).reshape(1200, 1920).astype(bool)
    if hashlib.sha256(mask.tobytes()).hexdigest() != value['mask_sha256']:
        raise ValueError('snapshot mask hash mismatch')
    return mask


def capture_reason(depth, front, candidate, *, periodic=False, reacquire=False):
    all_p05 = (front.get('known_term_m') or {}).get('p05')
    layer_p05 = (candidate.get('known_term_m') or {}).get('p05')
    if (depth.get('status') == 'COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY' and
            all_p05 is not None and layer_p05 is not None and
            abs(float(layer_p05) - float(all_p05)) >= .15):
        return 'FRONT_POINT_SET_DISAGREEMENT'
    if reacquire and candidate and depth.get('status') == 'COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY':
        return 'FIRST_COHERENT_AFTER_TRACK_LOSS'
    if periodic:
        return 'PERIODIC_SELECTED_INSTANCE'
    return None


class LatestSourceSnapshotWriter:
    """One queued source bundle, a finite file budget, and no foreground disk IO."""
    def __init__(self, directory, *, max_records=24):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.pending = queue.Queue(maxsize=1)
        self.max_records = max_records
        self.accepted = self.completed = self.skipped = self.errors = 0
        self.last_error = None
        self.closed = False
        self.worker = threading.Thread(target=self._run, name='sie-shadow-source-writer', daemon=True)
        self.worker.start()

    def submit(self, cycle_id, rgb_jpeg, stereo_jpeg, disparity, lr,
               calibration_arrays, mask_payload, metadata):
        if self.closed or self.accepted >= self.max_records or self.pending.full():
            self.skipped += 1
            return False
        if (not re.fullmatch(r'[A-Za-z0-9_-]+', cycle_id) or
                not isinstance(rgb_jpeg, bytes) or not rgb_jpeg.startswith(b'\xff\xd8') or
                not isinstance(stereo_jpeg, bytes) or not stereo_jpeg.startswith(b'\xff\xd8')):
            raise ValueError('missing same-cycle original JPEG packets')
        mask = unpack_mask(mask_payload)
        disparity = np.asarray(disparity)
        lr = np.asarray(lr)
        if disparity.shape != (400, 640) or lr.shape != disparity.shape or lr.dtype != np.bool_:
            raise ValueError('expected half-resolution current stereo arrays')
        keys=('P1','P2','R1','R','T','K','D','size')
        if not isinstance(calibration_arrays,dict) or any(k not in calibration_arrays for k in keys):
            raise ValueError('missing current calibration arrays')
        calibration={k:np.asarray(calibration_arrays[k]).copy() for k in keys}
        item = (cycle_id, rgb_jpeg, stereo_jpeg, disparity.copy(), lr.copy(), mask,
                calibration, dict(metadata))
        try:
            self.pending.put_nowait(item)
        except queue.Full:
            self.skipped += 1
            return False
        self.accepted += 1
        return True

    def _run(self):
        while True:
            item = self.pending.get()
            try:
                if item is None:
                    return
                self._write(*item)
                self.completed += 1
            except Exception as error:
                self.errors += 1
                self.last_error = type(error).__name__
            finally:
                self.pending.task_done()

    def _write(self, cycle_id, rgb_jpeg, stereo_jpeg, disparity, lr, mask,
               calibration, metadata):
        path = self.directory / (cycle_id + '.npz')
        temporary = self.directory / (cycle_id + '.npz.tmp')
        with temporary.open('xb') as stream:
            np.savez_compressed(stream, rgb_original_jpeg=np.frombuffer(rgb_jpeg, dtype=np.uint8),
                stereo_original_jpeg=np.frombuffer(stereo_jpeg, dtype=np.uint8),
                disparity_matching_px=disparity, lr_consistent=lr, instance_mask=mask,
                **{'cal_'+key:value for key,value in calibration.items()})
        os.replace(temporary, path)
        manifest = {'cycle_id': cycle_id, 'status': 'SOURCE_SNAPSHOT_DIAGNOSTIC_ONLY',
                    'npz_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                    'rgb_original_jpeg_sha256': hashlib.sha256(rgb_jpeg).hexdigest(),
                    'stereo_original_jpeg_sha256': hashlib.sha256(stereo_jpeg).hexdigest(),
                    'instance_mask_sha256': hashlib.sha256(mask.tobytes()).hexdigest(),
                    **metadata}
        with (self.directory / (cycle_id + '.json')).open('x') as stream:
            json.dump(manifest, stream, allow_nan=False, separators=(',', ':'))
            stream.write('\n')

    def close(self):
        if not self.closed:
            self.closed = True
            self.pending.put(None)
            self.worker.join()
