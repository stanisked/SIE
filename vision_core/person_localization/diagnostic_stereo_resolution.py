"""Explicit half-resolution diagnostic stereo geometry. Never an activation."""
from __future__ import annotations
import numpy as np


def matcher_parameters(matcher):
    keys=('MinDisparity','NumDisparities','BlockSize','P1','P2','Disp12MaxDiff',
          'PreFilterCap','UniquenessRatio','SpeckleWindowSize','SpeckleRange','Mode')
    return {(key if key in ('P1','P2') else key[0].lower()+key[1:]):getattr(matcher,'get'+key)() for key in keys}


def half_matchers(cv2, left, right):
    configs=[matcher_parameters(m) for m in (left,right)]
    if configs[0]['minDisparity']!=0 or configs[0]['numDisparities']!=128 or configs[1]['minDisparity']!=-128 or configs[1]['numDisparities']!=128:
        raise ValueError('half diagnostic requires the challenged SGBM 128 profile')
    for config in configs:
        config['numDisparities']=64
    configs[1]['minDisparity']=-64
    return [cv2.StereoSGBM_create(**c) for c in configs],configs


def half_disparity(left_raw, right_raw):
    """Small-pixel disparity and LR mask; 0.75 small px = 1.5 original px."""
    left,right=(a.astype(np.float32)/16. for a in (left_raw,right_raw))
    if left.shape!=right.shape or left.ndim!=2:
        raise ValueError('stereo map shapes disagree')
    y,x=np.indices(left.shape)
    xr=np.rint(x-left).astype(np.int32)
    valid=np.isfinite(left)&(left>.5)&(xr>=0)&(xr<left.shape[1])
    matched=np.full(left.shape,np.nan,np.float32)
    matched[valid]=right[y[valid],xr[valid]]
    consistent=valid&np.isfinite(matched)&(matched<-.5)&(np.abs(left+matched)<=.75)
    return left,consistent


def restore_crop(raw_maps,box,full_shape,configs):
    """Common crop origin: disparity unchanged; restore global LR rounding/grid."""
    x1,y1,x2,y2=box;h,w=full_shape
    if not 0<=x1<x2<=w or not 0<=y1<y2<=h:raise ValueError('invalid crop box')
    restored=[]
    for raw,config in zip(raw_maps,configs):
        if raw.shape!=(y2-y1,x2-x1) or raw.dtype!=np.int16:raise ValueError('unexpected cropped map')
        invalid=(config['minDisparity']-1)*16
        full=np.full(full_shape,invalid,np.int16);full[y1:y2,x1:x2]=raw;restored.append(full)
    if len(restored)!=2:raise ValueError('expected both matching directions')
    return restored

