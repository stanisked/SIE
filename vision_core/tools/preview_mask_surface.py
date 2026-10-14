"""Array-only current-frame mask/shape diagnostic, shared by live and replay.

No camera, image writes, smoothing, acceptance policy or movement interface.
"""
from __future__ import annotations
import cv2
import numpy as np
from compare_shadow_mask_depth_median import project, owned, in_roi
from analyze_shadow_local_surfaces import coarse_coverage
from vision_core.person_localization.instance_mask_stereo_fusion import masked_depth_hypothesis
from vision_core.person_localization.local_surface_diagnostic import analyze_local_surfaces


def evaluate(disparity, mask, cal, roi, lower=.5, upper=4.5, *, scale=1.):
    if scale not in (1., .5, .25) or not 0 < lower < upper <= 4.5:
        raise ValueError('invalid diagnostic scale/range')
    width, height = cal['size']
    if disparity.shape != (int(height*scale), int(width*scale)):
        raise ValueError('disparity/calibration size mismatch')
    if mask.shape != (1200,1920) or mask.dtype != np.bool_:
        raise ValueError('expected current AR boolean mask')
    # A fixed original-image sampling density, without upsampling/duplicating
    # depth. Resize maps pixel centres: u_full=(u_small+.5)/scale-.5.
    stride = int(4*scale)
    v, u = np.mgrid[0:disparity.shape[0]:stride, 0:disparity.shape[1]:stride]
    d = disparity[::stride, ::stride]
    good = np.isfinite(d) & (d > .5*scale)
    u, v, d = u[good], v[good], d[good]
    fb = abs(float(cal['P2'][0,3]))/1000.*scale
    z = abs(float(cal['P2'][0,3]))*scale/d/1000.
    valid = (z >= lower) & (z <= upper)
    u, v, d, z = u[valid], v[valid], d[valid], z[valid]
    xy = project((u.astype(float)+.5)/scale-.5,
                 (v.astype(float)+.5)/scale-.5, z, cal)
    in_box = in_roi(xy, roi)
    depth = masked_depth_hypothesis(xy[in_box], z[in_box], mask)
    interior = cv2.erode(mask.astype(np.uint8), np.ones((5,5),np.uint8),
                         borderType=cv2.BORDER_CONSTANT, borderValue=0)
    selected = owned(xy, interior, roi)
    u, v, d, xy = u[selected], v[selected], d[selected], xy[selected]
    local, data = analyze_local_surfaces(u, v, d, fb, stride=stride,
                                        tile_px=int(32*scale))
    local['rgb_coverage'] = coarse_coverage(interior, roi, xy,
                                          np.isfinite(data['axial_residual_m']))
    count = local['patch_count']
    local['unfitted_patch_fraction'] = 1-local['fitted_patch_count']/count if count else None
    # Compact result preserves maxima and all failed/sparse support counters.
    return {'status':'SHAPE_AWARE_SHADOW_DIAGNOSTIC_ONLY',
            'mask_depth':depth, 'local_surface':{k:v for k,v in local.items() if k!='patches'},
            'reference_frame':'rectified_left_optical_frame', 'unit':'m',
            'confidence':None, 'confidence_status':'NOT_VALIDATED',
            'metric_measurement_authorized':False, 'navigation_eligible':False,
            'actuator_access':False, 'acceptance_rule_added':False,
            'diagnostic_depth_range_m':[lower,upper], 'stereo_scale':scale,
            'sample_stride_original_px':4,
            'limitations':['Local fit residuals are not physical distance error.',
                          'Global MAD remains a comparison metric, not shape-aware admission.']}


def segment_array(source, candidate, model):
    from segment_saved_ar0234_instances import iou
    if source.shape != (1200,1920,3) or source.dtype != np.uint8:
        raise ValueError('invalid current RGB frame')
    results = model.predict(source=source, imgsz=640, conf=.25, classes=[0],
                            retina_masks=True, device='cpu', verbose=False)
    if len(results)!=1:
        raise ValueError('unexpected segmentation count')
    result=results[0]
    if result.boxes is None or result.masks is None or not len(result.boxes):
        return None, {'status':'MASK_UNAVAILABLE','reason':'NO_PERSON_SEGMENTATION'}
    boxes=result.boxes.xyxy.cpu().numpy()
    if len(result.masks.data)!=len(boxes) or np.any(result.boxes.cls.cpu().numpy()!=0):
        raise ValueError('inconsistent person masks')
    scores=np.array([iou(np.asarray(candidate['bbox_xyxy_px']),box) for box in boxes])
    order=np.argsort(scores)[::-1]
    best=int(order[0]); runner=float(scores[order[1]]) if len(order)>1 else 0.
    info={'best_bbox_iou':float(scores[best]),'runner_up_bbox_iou':runner,
          'person_instance_count':len(boxes)}
    if scores[best]<.50 or scores[best]-runner<.15:
        return None, {**info,'status':'MASK_UNAVAILABLE','reason':'PERSON_INSTANCE_MATCH_AMBIGUOUS'}
    mask=result.masks.data[best].cpu().numpy()>.5
    if mask.shape!=source.shape[:2] or not mask.any():
        raise ValueError('invalid full resolution mask')
    return mask, {**info,'status':'CURRENT_FRAME_MASK_DIAGNOSTIC_ONLY'}
