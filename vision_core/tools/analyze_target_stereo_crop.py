#!/usr/bin/env python3
"""Estimate target stereo crop area/support; never run a cropped matcher."""
import argparse
import ast
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import statistics
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
import cv2
import numpy as np


def runtime_frustum_margin():
    path=ROOT/'ros2/sie_ros2/sie_ros2/ar0234_ov9281_dynamic_association_audit_node.py'
    values=[]
    for node in ast.walk(ast.parse(path.read_text())):
        if (isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute)
                and node.func.attr=='declare_parameter' and len(node.args)==2
                and isinstance(node.args[0],ast.Constant) and node.args[0].value=='frustum_search_margin_px'):
            values.append(ast.literal_eval(node.args[1]))
    if len(values)!=1 or type(values[0]) is not int or values[0]<0:
        raise ValueError('cannot bind existing diagnostic frustum margin')
    return values[0]


def clip(rect,size):
    w,h=size;x1,y1,x2,y2=rect
    return [max(0,min(w,int(x1))),max(0,min(h,int(y1))),max(0,min(w,int(x2))),max(0,min(h,int(y2)))]


def contains(rect,x,y):
    a,b,c,d=rect
    return (x>=a)&(x<c)&(y>=b)&(y<d)


def frustum_core(roi,cal,scale,lower=.5,upper=4.5):
    """Dense <=1 RGB-pixel ray sampling; z is rectified-left optical z.

    Sampling plus existing diagnostic padding is not a formal continuous
    distortion/calibration-uncertainty bound. Observed support is checked below.
    """
    x1,y1,x2,y2=map(float,roi)
    if not (0<=x1<x2<=1920 and 0<=y1<y2<=1200):raise ValueError('invalid RGB target region')
    x=np.linspace(x1,x2,int(math.ceil(x2-x1))+1)
    y=np.linspace(y1,y2,int(math.ceil(y2-y1))+1)
    xx,yy=np.meshgrid(x,y)
    ar=cv2.undistortPoints(np.column_stack((xx.ravel(),yy.ravel())).reshape(-1,1,2),cal['K'],cal['D']).reshape(-1,2)
    rays=np.column_stack((ar,np.ones(len(ar))))
    q=(cal['R1']@cal['R']@rays.T).T;t=cal['R1']@cal['T']
    size=tuple(int(v*scale) for v in cal['size'])
    if not np.isfinite(q).all() or np.any(q[:,2]<=0):return [0,0,*size],'FULL_FRAME_NON_FORWARD_RAYS'
    projected=[]
    for z in (lower,upper):
        distance=(z-t[2])/q[:,2]
        if np.any(distance<=0):return [0,0,*size],'FULL_FRAME_NON_FORWARD_FRUSTUM'
        points=q*distance[:,None]+t
        homogeneous=(cal['P1']@np.column_stack((points,np.ones(len(points)))).T).T
        xy=homogeneous[:,:2]/homogeneous[:,2,None]
        projected.append((xy+.5)*scale-.5)
    pixels=np.concatenate(projected)
    if not np.isfinite(pixels).all():return [0,0,*size],'FULL_FRAME_NONFINITE_PROJECTION'
    raw=[math.floor(pixels[:,0].min()),math.floor(pixels[:,1].min()),
         math.ceil(pixels[:,0].max())+1,math.ceil(pixels[:,1].max())+1]
    core=clip(raw,size)
    if core[2]<=core[0] or core[3]<=core[1]:raise ValueError('RGB target has no stereo overlap')
    return core,'DENSE_SAMPLED_DIAGNOSTIC_FRUSTUM'


def crop_plan(roi,cal,configs,scale=.5,diagnostic_margin_full_px=32):
    core,method=frustum_core(roi,cal,scale)
    size=tuple(int(v*scale) for v in cal['size'])
    # Same x origin in both images. Include all disparity hypotheses in both
    # directions and block/LR-rounding support; do not narrow disparity search.
    disparity=max(max(abs(c['minDisparity']),abs(c['minDisparity']+c['numDisparities']-1)) for c in configs)
    block_radius=max(c['blockSize']//2 for c in configs)
    diagnostic=int(math.ceil(diagnostic_margin_full_px*scale))
    rounding=int(math.ceil(1.5*scale))
    pad_x=disparity+block_radius+rounding+diagnostic
    pad_y=block_radius+diagnostic
    a,b,c,d=core;crop=clip([a-pad_x,b-pad_y,c+pad_x,d+pad_y],size)
    width,height=crop[2]-crop[0],crop[3]-crop[1]
    valid=[];full_valid=[]
    for config in configs:
        x,y,w,h=cv2.getValidDisparityROI((0,0,width,height),(0,0,width,height),
            config['minDisparity'],config['numDisparities'],config['blockSize'])
        valid.append([crop[0]+x,crop[1]+y,crop[0]+x+w,crop[1]+y+h])
        x,y,w,h=cv2.getValidDisparityROI((0,0,*size),(0,0,*size),config['minDisparity'],config['numDisparities'],config['blockSize'])
        full_valid.append([x,y,x+w,y+h])
    return {'core_xyxy_px':core,'common_crop_xyxy_px':crop,'crop_size_px':[width,height],
        'area_fraction':width*height/(size[0]*size[1]),'left_valid_xyxy_px':valid[0],
        'right_valid_xyxy_px':valid[1],'full_left_valid_xyxy_px':full_valid[0],
        'full_right_valid_xyxy_px':full_valid[1],'frustum_method':method,
        'padding':{'max_disparity_search_px':disparity,'block_radius_px':block_radius,
            'lr_rounding_allowance_px':rounding,'existing_diagnostic_margin_px':diagnostic,
            'total_x_px':pad_x,'total_y_px':pad_y}}


def support_loss(plan,u,v,d):
    left=contains(plan['left_valid_xyxy_px'],u,v)
    xr=np.rint(u-d)
    right=contains(plan['right_valid_xyxy_px'],xr,v)
    full=contains(plan['full_left_valid_xyxy_px'],u,v)&contains(plan['full_right_valid_xyxy_px'],xr,v)
    count=len(u)
    return {'target_sample_count':count,
        'outside_crop_count':int((~contains(plan['common_crop_xyxy_px'],u,v)).sum()),
        'outside_left_valid_count':int((~left).sum()),
        'right_lookup_outside_right_valid_count':int((~right).sum()),
        'unsupported_either_direction_count':int((~(left&right)).sum()),
        'already_outside_full_frame_support_count':int((~full).sum()),
        'additional_crop_support_loss_count':int((full&~(left&right)).sum()),
        'unsupported_fraction':float((~(left&right)).mean()) if count else None,
        'meaning':'Observed full-half-frame LR-valid mask-owned samples only; not cropped-matcher equivalence.'}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('evidence-root','ar-intrinsic','stereo-calibration','activation','output'):
        p.add_argument('--'+key,type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():p.error('new output required')
    from analyze_ar0234_stereo_component_ambiguity import load_calibrations,sha256
    from replay_shadow_mask_depth_range import replay
    from preview_mask_surface import prepare_stereo_points
    from compare_shadow_mask_depth_median import owned
    from benchmark_stereo_sgbm_parallel import runtime_parameters,compute_pair
    from benchmark_stereo_resolution import scaled_configs
    from vision_core.person_localization.diagnostic_stereo_resolution import half_disparity
    cp=json.loads(args.activation.read_text())['calibration_profile'];ex=cp['ar0234_to_ov9281_physical_left_extrinsic'];extrinsic=Path(ex['path']).expanduser()
    if sha256(args.ar_intrinsic)!=cp['source_sha256']['ar0234_intrinsic'] or sha256(args.stereo_calibration)!=cp['source_sha256']['ov9281_stereo_calibration'] or sha256(extrinsic)!=ex['sha256']:
        raise ValueError('calibration/activation mismatch')
    cal=load_calibrations(args.ar_intrinsic,args.stereo_calibration,extrinsic)
    configs,code=runtime_parameters();configs=scaled_configs(configs,.5)
    margin=runtime_frustum_margin()
    cv2.setNumThreads(1);matchers=[cv2.StereoSGBM_create(**c) for c in configs]
    sources={str(f):sha256(f) for f in (args.ar_intrinsic,args.stereo_calibration,args.activation,extrinsic)};rows=[]
    with ThreadPoolExecutor(max_workers=2) as pool:
        for path in sorted(args.evidence_root.rglob('audit.json')):
            verified=replay(path,cal,.5,4.5)
            if 'mask_sha256' not in verified:raise ValueError('expected verified target mask')
            audit=json.loads(path.read_text());cid=audit['cycle_id'];folder=path.parent
            mp=folder/f'{cid}.png';meta=folder/f'{cid}.json'
            metadata=json.loads(meta.read_text());mask=cv2.imread(str(mp),cv2.IMREAD_GRAYSCALE)==255
            interior=cv2.erode(mask.astype('uint8'),np.ones((5,5),'uint8'),borderType=cv2.BORDER_CONSTANT,borderValue=0)
            images=[]
            for side in ('left','right'):
                f=folder/f'ov9281_physical_{side}_rectified.jpg';sources[str(f)]=sha256(f)
                color=cv2.imread(str(f))
                if color is None or color.shape!=(800,1280,3):raise ValueError('invalid stereo evidence')
                images.append(cv2.resize(cv2.cvtColor(color,cv2.COLOR_BGR2GRAY),(640,400),interpolation=cv2.INTER_AREA))
            for f in (path,mp,meta,folder/'ar0234_source.jpg',folder/'disparity_lr_consistent_px.npy'):sources[str(f)]=sha256(f)
            maps,_=compute_pair(matchers,*images,executor=pool)
            disparity,lr=half_disparity(*maps);disparity=np.where(lr,disparity,np.nan)
            points=prepare_stereo_points(disparity,cal,scale=.5,cycle_id=cid)
            regions={'saved_inner_roi':audit['stereo_geometry']['ar_inner_roi_xyxy_px'],
                     'saved_model_bbox':metadata['model_bbox_xyxy_px']}
            for label,roi in regions.items():
                plan=crop_plan(roi,cal,configs,diagnostic_margin_full_px=margin)
                selected=owned(points['xy'],interior,roi)
                loss=support_loss(plan,points['u'][selected],points['v'][selected],points['d'][selected])
                rows.append({'cycle_id':cid,'region':label,'rgb_roi_xyxy_px':roi,'plan':plan,'observed_support':loss})
                print('CROP',cid[-8:],label,'size=',plan['crop_size_px'],'area_fraction=',round(plan['area_fraction'],4),
                    'target_samples=',loss['target_sample_count'],'additional_support_loss=',loss['additional_crop_support_loss_count'],'outside_crop=',loss['outside_crop_count'],flush=True)
    if not rows:raise ValueError('no saved cycles')
    unchanged=all(sha256(Path(f))==digest for f,digest in sources.items())
    if not unchanged:raise ValueError('inputs changed')
    summary={}
    for label in sorted({r['region'] for r in rows}):
        group=[r for r in rows if r['region']==label]
        summary[label]={'cycles':len(group),'area_fraction_median':statistics.median(r['plan']['area_fraction'] for r in group),
            'area_fraction_max':max(r['plan']['area_fraction'] for r in group),
            'target_samples_total':sum(r['observed_support']['target_sample_count'] for r in group),
            'additional_support_loss_total':sum(r['observed_support']['additional_crop_support_loss_count'] for r in group),
            'outside_crop_total':sum(r['observed_support']['outside_crop_count'] for r in group),
            'already_outside_full_frame_support_total':sum(r['observed_support']['already_outside_full_frame_support_count'] for r in group),
            'cycles_with_additional_support_loss':sum(r['observed_support']['additional_crop_support_loss_count']>0 for r in group),
            'cycles_without_target_samples':sum(r['observed_support']['target_sample_count']==0 for r in group)}
        print('CROP_SUMMARY',label,json.dumps(summary[label]),flush=True)
    report={'status':'OFFLINE_CROP_AREA_SUPPORT_ESTIMATE_ONLY','rows':rows,'summary':summary,
        'inputs_unchanged':unchanged,'source_sha256':sources,'calibration_sha256':cal['sha'],
        'runtime_source_sha256':code,'matcher_configs':configs,'existing_diagnostic_margin_full_px':margin,'scale':.5,'diagnostic_range_m':[.5,4.5],
        'tool_sha256':sha256(Path(__file__)),'metric_measurement_authorized':False,'actuator_access':False,
        'limitations':['No cropped matcher executed; zero observed support loss does not prove disparity equality.',
            'SGBM semi-global paths and speckle behavior can change on cropping despite local search/block padding.',
            'Dense ray samples/existing padding do not bound unknown calibration or target-motion error.',
            'ROI comes from saved RGB targets/masks, not a fresh 256x384 inference.',
            'Current-frame crop is available only after RGB selection; present full-frame stereo overlaps RGB, so area reduction alone is not latency gain.',
            'No measured stereo area-to-runtime scaling or universal scene/motion validation.']}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    print('INPUTS_UNCHANGED:',unchanged,'Saved:',args.output)


if __name__=='__main__':main()
