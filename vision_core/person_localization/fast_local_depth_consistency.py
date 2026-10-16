"""Batched two-fold local disparity-plane diagnostics. No admission or filtering."""
from __future__ import annotations
import numpy as np


def stats(values):
    values=np.asarray(values,dtype=np.float64)
    if not values.size:return {'sample_count':0}
    median=float(np.median(values))
    return {'sample_count':int(values.size),'median_m':median,
        'mad_m':float(np.median(np.abs(values-median))),
        'absolute_p90_m':float(np.percentile(np.abs(values),90)),
        'absolute_max_m':float(np.max(np.abs(values)))}


def analyze_fast_local_depth(u,v,disparity,fb,*,stride=4,tile_px=32,minimum_points=24):
    """Fit each grid parity, evaluate the other, then swap; no in-sample residuals.

    Local affine disparity describes a plane, not an independently known surface.
    Ordinary least squares deliberately retains all training outliers. Numerical
    degeneracy checks and minimum fit support are not physical quality thresholds.
    """
    u,v,d=(np.asarray(x,dtype=np.float64) for x in (u,v,disparity))
    if (u.ndim!=1 or u.shape!=v.shape or u.shape!=d.shape or
        not all(np.isfinite(x).all() for x in (u,v,d)) or np.any(d<=0) or
        np.any(u<0) or np.any(v<0) or not np.isfinite(fb) or fb<=0 or
        type(stride) is not int or type(tile_px) is not int or stride<1 or
        tile_px<stride or tile_px%stride or type(minimum_points) is not int or minimum_points<12):
        raise ValueError('invalid positive finite disparity grid')
    if np.any(u%stride) or np.any(v%stride):raise ValueError('stride-aligned coordinates required')
    cells,ids=np.unique(np.column_stack((u.astype(np.int64)//tile_px,v.astype(np.int64)//tile_px)),axis=0,return_inverse=True)
    n=len(cells);count=np.bincount(ids,minlength=n)
    residual=np.full(len(d),np.nan);fold=((u/stride+v/stride).astype(np.int64)%2)
    # Fixed tile-local coordinates keep batched normal matrices well scaled.
    design=np.column_stack(((u%tile_px)/tile_px-.5,(v%tile_px)/tile_px-.5,np.ones(len(d))))
    valid_folds=np.zeros((2,n),bool)
    insufficient=np.zeros(n,bool);degenerate=np.zeros(n,bool);nonpositive=np.zeros(n,bool)
    for parity in (0,1):
        train=fold==parity;test=~train
        train_n=np.bincount(ids[train],minlength=n)
        test_n=np.bincount(ids[test],minlength=n)
        gram=np.zeros((n,3,3));rhs=np.zeros((n,3))
        for j in range(3):
            rhs[:,j]=np.bincount(ids[train],weights=design[train,j]*d[train],minlength=n)
            for k in range(j,3):
                gram[:,j,k]=gram[:,k,j]=np.bincount(ids[train],weights=design[train,j]*design[train,k],minlength=n)
        eig=np.linalg.eigvalsh(gram)
        enough=(count>=minimum_points)&(train_n>=6)&(test_n>=6)
        stable=eig[:,0]>1e-10*np.maximum(eig[:,-1],1.)
        insufficient|=(count>=minimum_points)&((train_n<6)|(test_n<6))
        degenerate|=enough & ~stable
        good=enough & stable
        valid_folds[parity]=good
        coefficients=np.zeros((n,3))
        coefficients[good]=np.linalg.solve(gram[good],rhs[good,:,None])[...,0]
        chosen=np.flatnonzero(test & good[ids])
        prediction=np.einsum('ij,ij->i',design[chosen],coefficients[ids[chosen]])
        finite=np.isfinite(prediction)&(prediction>0)
        failed=np.unique(ids[chosen[~finite]])
        valid_folds[parity,failed]=False
        nonpositive[failed]=True
        chosen=chosen[finite];prediction=prediction[finite]
        residual[chosen]=fb/d[chosen]-fb/prediction
    valid_patch=valid_folds.all(axis=0)
    residual[~valid_patch[ids]]=np.nan
    evaluated=np.isfinite(residual)
    report={'status':'LOCAL_CROSSFIT_DIAGNOSTIC_ONLY','algorithm':'BATCHED_OLS_DISPARITY_PLANE_TWO_FOLD',
        'sample_count':len(d),'patch_count':n,'fitted_patch_count':int(valid_patch.sum()),
        'fitted_sample_fraction':float(evaluated.mean()) if len(d) else 0.,
        'unfitted_patch_fraction':float((~valid_patch).mean()) if n else None,
        'patch_status_counts':{'TOO_FEW_POINTS':int((count<minimum_points).sum()),
            'INSUFFICIENT_FOLD_SUPPORT':int(insufficient.sum()),
            'DEGENERATE_TRAINING_SUPPORT':int(degenerate.sum()),
            'NONPOSITIVE_OR_NONFINITE_PREDICTION':int(nonpositive.sum()),
            'FITTED_BOTH_FOLDS':int(valid_patch.sum())},
        'physical_depth_error_bound_m':None,'confidence':None,'confidence_status':'NOT_VALIDATED',
        'tile_size_px':tile_px,'minimum_points_per_tile':minimum_points,
        'raw_depth':stats(fb/d),'pooled_held_out_axial_residual':stats(residual[evaluated]),
        'metric_measurement_authorized':False,'suitable_for_control':False,'actuator_access':False,
        'quality_thresholds_added':False,'reference_frame':'rectified_left_optical_frame','unit':'m',
        'limitations':['Local residual is not physical depth accuracy or a bound on median depth error.',
            'A smooth biased or incorrectly associated surface can have zero residual.',
            'Both folds use spatially correlated stereo samples; this is not independent truth.',
            'No support, layer ambiguity, freshness or calibration gate is replaced.']}
    return report,{'axial_residual_m':residual,'patch_id':ids,'evaluated':evaluated}
