#!/usr/bin/env python3
"""Verify exact-cycle source bundles from a motor-off Fast Shadow run."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import cv2


def digest(data):
    return hashlib.sha256(data).hexdigest()


def summarize(journal, directory):
    rows={}
    with Path(journal).open() as stream:
        for line in stream:
            row=json.loads(line)
            if row.get('cycle_id') is not None:
                rows[row['cycle_id']]=row
    requested={cid:row for cid,row in rows.items() if row.get('source_snapshot_evidence')}
    validated=[];errors=[]
    for manifest_path in sorted(Path(directory).glob('*.json')):
        try:
            manifest=json.loads(manifest_path.read_text())
            cid=manifest['cycle_id']
            row=requested[cid]
            if manifest_path.stem!=cid or manifest['instance_mask_sha256']!=row['source_snapshot_evidence']['mask_sha256']:
                raise ValueError('cycle/mask mismatch')
            track_id=row['track_id']
            if (track_id is None and row.get('mask_status')=='DEPTH_MAD_EXCEEDED' and
                    row.get('rgb_continuity_status')=='MATCHED_CURRENT_RGB_DIAGNOSTIC_ONLY'):
                track_id=row.get('rgb_continuity_track_id')
            if (manifest['track_id']!=track_id or
                    manifest['selected_instance_index']!=row['selected_mask_instance_index'] or
                    manifest['segmentation_model_sha256']!=row['segmentation_model_sha256']):
                raise ValueError('track/instance/model mismatch')
            for sensor in ('ar0234','ov9281'):
                if manifest['frame_freshness']['frames'][sensor]['driver_timestamp_s']!=row['driver_timestamp_s'][sensor]:
                    raise ValueError('driver timestamp mismatch')
            path=manifest_path.with_suffix('.npz')
            if digest(path.read_bytes())!=manifest['npz_sha256']:
                raise ValueError('bundle hash mismatch')
            with np.load(path,allow_pickle=False) as bundle:
                for field,sensor in (('rgb_original_jpeg','rgb_original_jpeg_sha256'),
                                     ('stereo_original_jpeg','stereo_original_jpeg_sha256')):
                    if digest(bundle[field].tobytes())!=manifest[sensor]:
                        raise ValueError('original JPEG hash mismatch')
                rgb=cv2.imdecode(bundle['rgb_original_jpeg'],cv2.IMREAD_COLOR)
                stereo=cv2.imdecode(bundle['stereo_original_jpeg'],cv2.IMREAD_COLOR)
                if (rgb is None or rgb.shape!=(1200,1920,3) or
                        stereo is None or stereo.shape!=(800,2560,3) or
                        digest(rgb.tobytes())!=manifest['segmentation_rgb_payload_sha256']):
                    raise ValueError('decoded original RGB/stereo binding mismatch')
                if digest(bundle['instance_mask'].tobytes())!=manifest['instance_mask_sha256']:
                    raise ValueError('instance mask hash mismatch')
                if (bundle['disparity_matching_px'].shape!=(400,640) or
                        bundle['lr_consistent'].shape!=(400,640)):
                    raise ValueError('stereo array shape mismatch')
                if any('cal_'+key not in bundle for key in ('P1','P2','R1','R','T','K','D','size')):
                    raise ValueError('calibration array missing')
            validated.append(cid)
        except (ValueError,KeyError,OSError,TypeError) as error:
            errors.append({'file':manifest_path.name,'reason':type(error).__name__,
                           'detail':str(error)[:100]})
    missing=sorted(set(requested)-set(validated))
    return {'scope':'MOTOR_OFF_SOURCE_EVIDENCE; not Measurement or movement admission',
            'requested_after_publish':len(requested),'validated_bundles':len(validated),
            'validated_cycle_ids':validated,'missing_requested_cycles':missing,
            'errors':errors}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('journal',type=Path)
    parser.add_argument('directory',type=Path)
    args=parser.parse_args()
    result=summarize(args.journal,args.directory)
    print(json.dumps(result,indent=2,ensure_ascii=False))
    if result['errors'] or not result['validated_bundles']:
        raise SystemExit(2)


if __name__=='__main__':
    main()
