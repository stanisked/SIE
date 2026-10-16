#!/usr/bin/env python3
"""One physical front-plane comparison. Diagnostic only, no correction or motors."""
from __future__ import annotations
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from vision_core.person_localization.robot_front_geometry import load_profile, profile_hash, forward_terms

EXPECTED_CALIBRATION_SHA='39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52'

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def stats(values):
    a=np.asarray(values,dtype=float)
    if a.ndim!=1 or not a.size or not np.isfinite(a).all():raise ValueError('invalid plane samples')
    return {'count':int(a.size),'median_m':float(np.median(a)),
            'mad_m':float(np.median(np.abs(a-np.median(a)))),
            'p05_m':float(np.percentile(a,5)),'p95_m':float(np.percentile(a,95))}

def comparison(frame_rows,reference_m):
    if not np.isfinite(reference_m) or reference_m<=0:raise ValueError('invalid physical reference')
    medians=[r['dense_known_term']['median_m'] for r in frame_rows]
    if not medians:return {'status':'NO_ACCEPTED_DENSE_BOARD_FRAMES'}
    median=float(np.median(medians));delta=median-reference_m
    return {'status':'SINGLE_POINT_COMPARISON_ONLY','frame_count':len(medians),
            'median_known_term_m':median,'frame_median_span_m':max(medians)-min(medians),
            'corner_comparison_statuses':dict(Counter(r.get('corner_comparison',{}).get('status','NOT_RECORDED') for r in frame_rows)),
            'corner_geometry_warning':any(r.get('corner_comparison',{}).get('status')!='CORNER_COMPARISON_DIAGNOSTIC_ONLY' for r in frame_rows),
            'known_term_minus_physical_reference_m':delta,'absolute_difference_m':abs(delta),
            'within_declared_100mm_numeric_screen':bool(abs(delta)<=.1),
            'screen_is_accuracy_validation':False,'optical_offset_estimated':False,
            'correction_applied':False,'physical_depth_error_bound_established':False}

def pair_corner_order(left,right):
    """Choose only original versus 180-degree corner order using epipolar evidence."""
    candidates=[]
    for name,c in [('original',right),('reverse_180',right[::-1])]:
        disp=left[:,0]-c[:,0]
        vertical=np.abs(left[:,1]-c[:,1])
        if np.count_nonzero(disp>.5)>=math.ceil(.9*len(disp)):
            candidates.append((float(np.median(vertical)),name,c))
    if not candidates:raise ValueError('NO_POSITIVE_CORNER_DISPARITY_ORDER')
    candidates.sort(key=lambda x:x[0])
    if candidates[0][0]>1.5:raise ValueError('CORNER_EPIPOLAR_ERROR_EXCEEDED')
    if len(candidates)>1 and abs(candidates[0][0]-candidates[1][0])<.1:
        raise ValueError('CORNER_ORDER_AMBIGUOUS')
    return candidates[0][1],candidates[0][2],candidates[0][0]


def corner_distance_diagnostic(left,right,p1,p2,r1,mount):
    if right is None:
        return {'status':'CORNER_COMPARISON_UNAVAILABLE','reason':'BOARD_NOT_FOUND_RIGHT'},np.empty((0,2)),np.empty(0),np.empty(0)
    metrics=[]
    for name,c in [('original',right),('reverse_180',right[::-1])]:
        d=left[:,0]-c[:,0];vertical=left[:,1]-c[:,1]
        metrics.append({'order':name,'positive_disparity_fraction':float(np.mean(d>.5)),
                        'median_signed_vertical_px':float(np.median(vertical)),
                        'median_absolute_vertical_px':float(np.median(np.abs(vertical))),
                        'p95_absolute_vertical_px':float(np.percentile(np.abs(vertical),95))})
    try:
        order,paired,epipolar=pair_corner_order(left,right)
    except ValueError as error:
        return {'status':'CORNER_COMPARISON_REFUSED','reason':str(error),'order_diagnostics':metrics},right,np.empty(0),np.empty(0)
    disparity=left[:,0]-paired[:,0];good=disparity>.5
    depth=abs(float(p2[0,3]))/disparity[good]/1000.
    known=forward_terms(left[good,0],left[good,1],depth,p1,r1,mount)
    return {'status':'CORNER_COMPARISON_DIAGNOSTIC_ONLY','right_corner_order':order,
            'median_absolute_vertical_px':epipolar,'order_diagnostics':metrics,
            'known_term':stats(known)},paired,depth,known


def collect(args,report):
    import cv2
    from concurrent.futures import ThreadPoolExecutor
    from validate_ov9281_pi_depth_v7_pose_aware import (find_corners,corner_detection_flags,
        object_points,board_pose_in_rectified_left,set_auto_exposure)
    from benchmark_stereo_sgbm_parallel import runtime_parameters,compute_pair
    from vision_core.person_localization.diagnostic_stereo_resolution import half_matchers,half_disparity
    if sha(args.stereo_calibration)!=EXPECTED_CALIBRATION_SHA:
        raise ValueError('Expected active v7 calibration hash does not match')
    mount=load_profile(args.mount_profile)
    with np.load(args.stereo_calibration,allow_pickle=False) as c:
        if str(c['camera_1_semantics'].item())!='physical_left':raise ValueError('wrong stereo camera semantics')
        size=tuple(map(int,c['size']))
        k1,d1,k2,d2,r1,r2,p1,p2=(c[k].copy() for k in ('K1','D1','K2','D2','R1','R2','P1','P2'))
    if size!=(1280,800):raise ValueError('expected v7 1280x800 per-camera calibration')
    configs,config_sources=runtime_parameters()
    cv2.setNumThreads(1)
    matchers,half_configs=half_matchers(cv2,*[cv2.StereoSGBM_create(**c) for c in configs])
    maps=[cv2.initUndistortRectifyMap(k,d,r,p,size,cv2.CV_32FC1)
          for k,d,r,p in ((k1,d1,r1,p1),(k2,d2,r2,p2))]
    report['helper_sha256']={name:sha(ROOT/name) for name in (
        'vision_core/tools/validate_ov9281_pi_depth_v7_pose_aware.py',
        'vision_core/tools/benchmark_stereo_sgbm_parallel.py',
        'vision_core/person_localization/diagnostic_stereo_resolution.py',
        'vision_core/person_localization/robot_front_geometry.py')}
    board=(9,6);obj=object_points(board,.025)
    flags=corner_detection_flags('sb_exhaustive')
    report.update(calibration_sha256=EXPECTED_CALIBRATION_SHA,mount_profile_sha256=profile_hash(mount),
                  mount_profile=mount,matcher_configs=half_configs,matcher_source_sha256=config_sources,
                  matching_scope='full half-resolution frame; board polygon instead of RGB person/frustum crop',
                  depth_range_m=[.5,4.5],minimum_dense_owned_samples=100,
                  capture={'device':args.device,'requested_mode':'V4L2 MJPG 2560x800@60 auto_exposure=3',
                           'flush_frames':60,'thermal_warmup_qualification':False},opencv_version=cv2.__version__)
    set_auto_exposure(args.device)
    cap=cv2.VideoCapture(args.device,cv2.CAP_V4L2)
    rejected=Counter()
    report['attempt_records']=[]
    try:
        cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,2560);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,800)
        cap.set(cv2.CAP_PROP_FPS,60);cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
        if not cap.isOpened():raise RuntimeError('Cannot open stereo camera; stop other camera processes')
        for _ in range(60):cap.read()
        with ThreadPoolExecutor(max_workers=2) as pool:
            for attempt in range(1,9):
                if len(report['frames'])>=3:break
                report['attempts']=attempt
                ok,combined=cap.read()
                if not ok or combined is None or combined.shape[:2]!=(800,2560):
                    rejected['INVALID_COMBINED_FRAME']+=1;continue
                image=args.output/f'attempt_{attempt:02d}_combined_decoded.png'
                if not cv2.imwrite(str(image),combined):raise OSError('Cannot save decoded source image')
                attempt_row={'attempt':attempt,'status':'CAPTURED','source_files':{image.name:sha(image)}}
                report['attempt_records'].append(attempt_row)
                right_raw,left_raw=combined[:,:1280],combined[:,1280:]
                lc=find_corners(left_raw,board,flags);rc=find_corners(right_raw,board,flags)
                if lc is None:
                    rejected['BOARD_NOT_FOUND_LEFT']+=1
                    attempt_row.update(status='DENSE_UNAVAILABLE',reason='BOARD_NOT_FOUND_LEFT')
                    print('SEARCH',attempt,'board not found left',flush=True);continue
                left_c=cv2.undistortPoints(lc,k1,d1,R=r1,P=p1).reshape(-1,2)
                right_c=None if rc is None else cv2.undistortPoints(rc,k2,d2,R=r2,P=p2).reshape(-1,2)
                corner,right_c,cz,corner_known=corner_distance_diagnostic(left_c,right_c,p1,p2,r1,mount)
                attempt_row['corner_comparison']=corner
                rect=[]
                for raw,m in zip((left_raw,right_raw),maps):
                    im=cv2.remap(raw,*m,cv2.INTER_LINEAR)
                    rect.append(cv2.resize(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),(640,400),interpolation=cv2.INTER_AREA))
                raw_maps,match_s=compute_pair(matchers,*rect,executor=pool)
                disparity,lr=half_disparity(*raw_maps)
                # Interior of the detected inner-corner hull stays on the board plane.
                hull=cv2.convexHull(np.rint((left_c+.5)*.5-.5).astype(np.int32))
                mask=np.zeros((400,640),np.uint8);cv2.fillConvexPoly(mask,hull,1)
                mask=cv2.erode(mask,np.ones((3,3),np.uint8),borderType=cv2.BORDER_CONSTANT,borderValue=0)
                v,u=np.mgrid[0:400:2,0:640:2];sample_d=disparity[::2,::2]
                good=lr[::2,::2]&(mask[::2,::2]>0)&np.isfinite(sample_d)&(sample_d>0)
                u,v,d=u[good],v[good],sample_d[good]
                z=abs(float(p2[0,3]))*.5/d/1000.
                valid=(z>=.5)&(z<=4.5);u,v,d,z=u[valid],v[valid],d[valid],z[valid]
                known=forward_terms(u,v,z,p1,r1,mount,scale=.5)
                arrays=args.output/f'attempt_{attempt:02d}_geometry.npz'
                np.savez_compressed(arrays,u_half=u,v_half=v,depth_m=z,disparity_half_px=d,
                                    disparity_left_raw=raw_maps[0],disparity_right_raw=raw_maps[1],
                                    lr_consistency_mask=lr,board_interior_mask=mask,known_term_m=known,
                                    left_rectified_corners=left_c,right_rectified_corners=right_c,
                                    corner_depth_m=cz,corner_known_term_m=corner_known,
                                    left_raw_corners=lc,right_raw_corners=rc if rc is not None else np.empty((0,1,2),np.float32))
                attempt_row['source_files'][arrays.name]=sha(arrays)
                attempt_row['dense_board_support']=len(z)
                if len(z)<100:
                    rejected['INSUFFICIENT_DENSE_BOARD_SUPPORT']+=1
                    attempt_row.update(status='DENSE_UNAVAILABLE',reason='INSUFFICIENT_DENSE_BOARD_SUPPORT')
                    print('SEARCH',attempt,'dense board support=',len(z),'corner=',corner['status'],flush=True);continue
                pose={};normal_angle=None
                try:
                    pose=board_pose_in_rectified_left(lc,obj,k1,d1,r1)
                    normal=np.asarray(pose['board_normal_unit'])@r1
                    theta=math.radians(mount['block_pitch_up_deg'])
                    forward_axis=np.array([0,math.sin(theta),math.cos(theta)])
                    normal_angle=math.degrees(math.acos(float(np.clip(abs(normal@forward_axis),0,1))))
                except Exception as error:
                    pose={'error':type(error).__name__+': '+str(error)}
                index=len(report['frames'])+1
                attempt_row['status']='DENSE_BOARD_DIAGNOSTIC_ONLY'
                row={'captured_host_utc':datetime.now(timezone.utc).isoformat(),
                     'dense_optical_depth':stats(z),'dense_known_term':stats(known),
                     'corner_comparison':corner,'corner_known_term':corner.get('known_term'),
                     'pnp_diagnostic':pose,
                     'pnp_board_normal_to_robot_forward_deg':normal_angle,'stereo_match_pair_s':match_s,
                     'source_files':{image.name:sha(image),arrays.name:sha(arrays)}}
                report['frames'].append(row)
                delta=row['dense_known_term']['median_m']-args.reference_front_distance_mm/1000.
                print(f'FRAME {index}/3 n={len(z)} known_term={np.median(known):.4f} m difference={1000*delta:+.1f} mm corner={corner['status']}',flush=True)
    finally:
        cap.release()
        report['rejections']=dict(rejected)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--stereo-calibration',type=Path,default=Path('/home/elwis/dev_ws/runtime_artifacts/ov9281_pi_stereo_v7/ov9281_pi_stereo_v7.npz'))
    p.add_argument('--mount-profile',type=Path,default=ROOT/'vision_core/config/diagnostics/robot_front_mount_operator_v1.json')
    p.add_argument('--device',default='/dev/v4l/by-id/usb-TSTC_Web_Camera_TSTC_Web_Camera-video-index0')
    p.add_argument('--reference-front-distance-mm',type=float,required=True)
    p.add_argument('--reference-uncertainty-mm',type=float)
    p.add_argument('--output',type=Path)
    args=p.parse_args()
    if not np.isfinite(args.reference_front_distance_mm) or args.reference_front_distance_mm<=0:p.error('positive finite reference required')
    if args.reference_uncertainty_mm is not None and (not np.isfinite(args.reference_uncertainty_mm) or args.reference_uncertainty_mm<0):p.error('invalid reference uncertainty')
    args.output=args.output or Path.home()/'.local/state/sie/evidence'/('front_board_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    args.output.mkdir(parents=True,exist_ok=False)
    report={'status':'DIAGNOSTIC_ONLY','reference':{'distance_m':args.reference_front_distance_mm/1000.,
        'standard_uncertainty_m':None if args.reference_uncertainty_mm is None else args.reference_uncertainty_mm/1000.,
        'reference_frame':'robot_longitudinal_axis_from_front_plane',
        'method':'operator tape/ruler horizontal longitudinal distance to stationary vertical board plane'},
        'board':{'inner_corners':[9,6],'square_size_mm':25.},'frames':[],
        'measurement_authorized':False,'actuator_access':False,'correction_applied':False,
        'limitations':['Single-point nominal known-term comparison, not an optical-offset calibration.',
                       'Optical datum, mount angular errors and depth bias are not separable here.',
                       'Board polygon replaces person association; full half-frame matching differs from target crop.',
                       'No timing, thermal, dynamic-scene or whole-range accuracy qualification.'],
        'code_sha256':sha(__file__)}
    code=0
    try:
        collect(args,report)
        report['comparison']=comparison(report['frames'],args.reference_front_distance_mm/1000.)
        if len(report['frames'])<3:report['status']='INCOMPLETE_DIAGNOSTIC';code=1
    except (Exception,KeyboardInterrupt) as error:
        report['status']='CAPTURE_INCOMPLETE';report['error']=type(error).__name__+': '+str(error);code=1
    finally:
        (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(report.get('comparison',{'error':report.get('error'),'rejections':report.get('rejections')}),indent=2))
    print('Saved:',args.output/'report.json')
    return code

if __name__=='__main__':raise SystemExit(main())
