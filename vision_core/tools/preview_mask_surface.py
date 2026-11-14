"""Array-only current-frame mask/shape diagnostic, shared by live and replay.

No camera, image writes, smoothing, acceptance policy or movement interface.
"""
from __future__ import annotations
import cv2
import numpy as np
from compare_shadow_mask_depth_median import project, owned, in_roi
from analyze_shadow_local_surfaces import coarse_coverage
from vision_core.person_localization.instance_mask_stereo_fusion import (
    masked_depth_hypothesis,
    depth_from_owned_samples_with_candidate_indices)
from vision_core.person_localization.local_surface_diagnostic import analyze_local_surfaces
from vision_core.person_localization.torso_core_geometry import (
    TORSO_CORE_POLICY, projected_in_torso_core)


def evaluate(disparity, mask, cal, roi, lower=.5, upper=4.5, *, scale=1., defer_local=False, prepare_local=True, ownership_backend="opencv",fast_local=False,front_geometry=None,instance_bbox=None):
    points=prepare_stereo_points(disparity,cal,lower,upper,scale=scale)
    return evaluate_prepared(points,mask,roi,defer_local=defer_local,
        prepare_local=prepare_local,ownership_backend=ownership_backend,fast_local=fast_local,
        front_geometry=front_geometry,instance_bbox=instance_bbox)


def prepare_stereo_points(disparity,cal,lower=.5,upper=4.5,*,scale=1.,cycle_id=""):
    if scale not in (1., .5, .25) or not 0 < lower < upper <= 4.5:
        raise ValueError('invalid diagnostic scale/range')
    width, height = cal['size']
    if disparity.shape != (int(height*scale), int(width*scale)):
        raise ValueError('disparity/calibration size mismatch')
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
    return {'u':u,'v':v,'d':d,'z':z,'xy':xy,'fb':np.asarray(fb),
        'stride':np.asarray(stride),'scale':np.asarray(scale),'lower':np.asarray(lower),
        'upper':np.asarray(upper),'source_cycle_id':np.asarray(cycle_id)}


GEOMETRY_KEYS=('u','v','d','z','xy','fb','stride','scale','lower','upper','source_cycle_id')


def evaluate_prepared(points,mask,roi,*,defer_local=False,prepare_local=True,ownership_backend="opencv",fast_local=False,front_geometry=None,instance_bbox=None):
    if ownership_backend not in ('opencv','reference_numpy'):raise ValueError('unknown mask ownership backend')
    if mask.shape!=(1200,1920) or mask.dtype!=np.bool_:raise ValueError('expected current AR boolean mask')
    scale=float(points['scale']);lower=float(points['lower']);upper=float(points['upper'])
    if scale not in (1.,.5,.25) or not 0<lower<upper<=4.5:raise ValueError('invalid prepared scale/range')
    u,v,d,z,xy=(points[key] for key in ('u','v','d','z','xy'))
    if any(a.ndim!=1 or len(a)!=len(u) for a in (u,v,d,z)) or xy.shape!=(len(u),2):
        raise ValueError('prepared point shape mismatch')
    fb=float(points['fb']);stride=int(points['stride'])
    if stride!=int(4*scale) or not np.isfinite(fb) or fb<=0:raise ValueError('invalid prepared geometry metadata')
    # One equivalent 5x5 constant-zero erosion and one ownership selection.
    # Reuse them for depth statistics and (when scheduled) local-fit inputs.
    selected = interior = candidate_indices = None
    if ownership_backend == "reference_numpy":
        in_box = in_roi(xy, roi)
        depth = masked_depth_hypothesis(xy[in_box], z[in_box], mask)
    else:
        interior = cv2.erode(mask.astype(np.uint8), np.ones((5,5),np.uint8),
                             borderType=cv2.BORDER_CONSTANT, borderValue=0)
        selected = owned(xy, interior, roi)
        depth, candidate_indices = depth_from_owned_samples_with_candidate_indices(
            z[selected], int(np.count_nonzero(mask)), int(np.count_nonzero(interior)))
    fast=fast_data=None
    if fast_local:
        import time
        from vision_core.person_localization.fast_local_depth_consistency import analyze_fast_local_depth
        if selected is None:
            interior=cv2.erode(mask.astype(np.uint8),np.ones((5,5),np.uint8),borderType=cv2.BORDER_CONSTANT,borderValue=0)
            selected=owned(xy,interior,roi)
        started=time.monotonic()
        fast,fast_data=analyze_fast_local_depth(u[selected],v[selected],d[selected],fb,
            stride=stride,tile_px=int(32*scale))
        fast['fit_s']=time.monotonic()-started
        fast['rgb_coverage_status']='NOT_COMPUTED_IN_FAST_CURRENT_FRAME_PATH'
    front=layer_front=local_candidate_front=stable_candidate_front=stable_torso_core_front=None
    if front_geometry is not None:
        import time
        from vision_core.person_localization.robot_front_geometry import describe_front
        if selected is None:
            interior=cv2.erode(mask.astype(np.uint8),np.ones((5,5),np.uint8),borderType=cv2.BORDER_CONSTANT,borderValue=0)
            selected=owned(xy,interior,roi)
        started=time.monotonic()
        front=describe_front(u[selected],v[selected],z[selected],front_geometry['P1'],front_geometry['R1'],
            front_geometry['profile'],scale=scale,source_depth_status=depth.get('status'))
        front['point_set']='ALL_CURRENT_MASK_OWNED_STEREO_POINTS'
        front['processing_s']=time.monotonic()-started
        if candidate_indices is None:
            check, candidate_indices = depth_from_owned_samples_with_candidate_indices(
                z[selected], int(np.count_nonzero(mask)), int(np.count_nonzero(interior)))
            if check != depth:
                raise ValueError('reference and current owned depth layers disagree')
        if len(candidate_indices):
            owned_u,owned_v,owned_z=u[selected],v[selected],z[selected]
            layer_started=time.monotonic()
            layer_front=describe_front(owned_u[candidate_indices],owned_v[candidate_indices],
                owned_z[candidate_indices],front_geometry['P1'],front_geometry['R1'],
                front_geometry['profile'],scale=scale,source_depth_status=depth.get('status'))
            # Numerical depth grouping alone cannot identify the physical person.
            layer_front['point_set']='NUMERIC_CANDIDATE_DEPTH_LAYER_DIAGNOSTIC_ONLY'
            layer_front['semantic_ownership_verified']=False
            layer_front['processing_s']=time.monotonic()-layer_started
            if fast is not None:
                # A secondary depth layer must not participate in the local
                # fit of the numerical candidate. Reuse the existing fit when
                # every owned point belongs to that candidate.
                if len(candidate_indices)==len(owned_u):
                    candidate_fast,candidate_data=fast,fast_data
                    candidate_fit_s=0.
                else:
                    from vision_core.person_localization.fast_local_depth_consistency import analyze_fast_local_depth
                    fit_started=time.monotonic()
                    candidate_fast,candidate_data=analyze_fast_local_depth(
                        owned_u[candidate_indices],owned_v[candidate_indices],
                        d[selected][candidate_indices],fb,
                        stride=stride,tile_px=int(32*scale))
                    candidate_fit_s=time.monotonic()-fit_started
                fitted=candidate_data['evaluated']
                fitted_count=int(np.count_nonzero(fitted))
                local_candidate_front={
                    'status':'INSUFFICIENT_LOCAL_FITTED_CANDIDATE_POINTS',
                    'point_set':'LOCAL_FITTED_NUMERIC_CANDIDATE_LAYER_DIAGNOSTIC_ONLY',
                    'candidate_sample_count':len(candidate_indices),
                    'point_count':fitted_count,
                    'local_fit_residual':candidate_fast['pooled_held_out_axial_residual'],
                    'local_fit_fitted_fraction':candidate_fast['fitted_sample_fraction'],
                    'candidate_refit_s':candidate_fit_s,
                    'source_depth_status':depth.get('status'),
                    'reference_frame':'robot_longitudinal_axis_from_front_plane',
                    'unit':'m','confidence':None,'confidence_status':'NOT_VALIDATED',
                    'profile_sha256':front['profile_sha256'],
                    'known_term_m':None,'semantic_ownership_verified':False,
                    'distance_from_front_m':None,
                    'measurement_authorized':False,'navigation_eligible':False,
                    'actuator_access':False}
                if fitted_count>=100:
                    local_candidate_front.update(describe_front(
                        owned_u[candidate_indices][fitted],
                        owned_v[candidate_indices][fitted],
                        owned_z[candidate_indices][fitted],
                        front_geometry['P1'],front_geometry['R1'],
                        front_geometry['profile'],scale=scale,
                        source_depth_status=depth.get('status')))
                    local_candidate_front['status']='LOCAL_FITTED_CANDIDATE_FRONT_DIAGNOSTIC_ONLY'
                    local_candidate_front['point_set']='LOCAL_FITTED_NUMERIC_CANDIDATE_LAYER_DIAGNOSTIC_ONLY'
                    local_candidate_front['candidate_sample_count']=len(candidate_indices)
                    local_candidate_front['local_fit_residual']=candidate_fast['pooled_held_out_axial_residual']
                    local_candidate_front['local_fit_fitted_fraction']=candidate_fast['fitted_sample_fraction']
                    local_candidate_front['candidate_refit_s']=candidate_fit_s
                    local_candidate_front['semantic_ownership_verified']=False
                    local_candidate_front['limitations'].append(
                        'Local fit availability does not validate physical target ownership or depth error.')
                    local_candidate_front['limitations'].append(
                        'Filtering points can move p05 nearer or farther; it is not a conservative clearance bound.')
                from vision_core.person_localization.fast_local_depth_consistency import select_stable_local_patches
                stable_report,stable=select_stable_local_patches(candidate_data)
                stable_candidate_front={
                    'status':'INSUFFICIENT_STABLE_LOCAL_PATCH_POINTS',
                    'point_set':'STABLE_LOCAL_PATCHES_NUMERIC_CANDIDATE_LAYER_DIAGNOSTIC_ONLY',
                    'candidate_sample_count':len(candidate_indices),
                    'point_count':stable_report['selected_point_count'],
                    'stable_local_patches':stable_report,
                    'source_depth_status':depth.get('status'),
                    'reference_frame':'robot_longitudinal_axis_from_front_plane',
                    'unit':'m','confidence':None,'confidence_status':'NOT_VALIDATED',
                    'profile_sha256':front['profile_sha256'],
                    'known_term_m':None,'semantic_ownership_verified':False,
                    'distance_from_front_m':None,'measurement_authorized':False,
                    'navigation_eligible':False,'actuator_access':False}
                if stable_report['status']=='STABLE_LOCAL_PATCHES_DIAGNOSTIC_ONLY':
                    stable_candidate_front.update(describe_front(
                        owned_u[candidate_indices][stable],
                        owned_v[candidate_indices][stable],
                        owned_z[candidate_indices][stable],
                        front_geometry['P1'],front_geometry['R1'],
                        front_geometry['profile'],scale=scale,
                        source_depth_status=depth.get('status')))
                    stable_candidate_front['status']='STABLE_CANDIDATE_FRONT_DIAGNOSTIC_ONLY'
                    stable_candidate_front['point_set']='STABLE_LOCAL_PATCHES_NUMERIC_CANDIDATE_LAYER_DIAGNOSTIC_ONLY'
                    stable_candidate_front['candidate_sample_count']=len(candidate_indices)
                    stable_candidate_front['stable_local_patches']=stable_report
                    stable_candidate_front['semantic_ownership_verified']=False
                    stable_candidate_front['limitations'].append(
                        'Provisional held-out residual limits do not establish physical distance accuracy or clearance.')
                if instance_bbox is not None:
                    candidate_xy=xy[selected][candidate_indices]
                    in_core,core_roi=projected_in_torso_core(candidate_xy,instance_bbox)
                    stable_core=stable & in_core
                    core_count=int(np.count_nonzero(stable_core))
                    stable_torso_core_front={
                        'status':'INSUFFICIENT_STABLE_TORSO_CORE_POINTS',
                        'point_set':'STABLE_LOCAL_PATCHES_WITHIN_CURRENT_RGB_TORSO_CORE_DIAGNOSTIC_ONLY',
                        'candidate_sample_count':len(candidate_indices),
                        'torso_core_candidate_point_count':int(np.count_nonzero(in_core)),
                        'point_count':core_count,
                        'torso_core_rgb_roi_xyxy_px':core_roi,
                        'torso_core_policy':dict(TORSO_CORE_POLICY),
                        'stable_local_patch_policy':dict(stable_report['policy']),
                        'source_depth_status':depth.get('status'),
                        'reference_frame':'robot_longitudinal_axis_from_front_plane',
                        'unit':'m','confidence':None,'confidence_status':'NOT_VALIDATED',
                        'profile_sha256':front['profile_sha256'],
                        'known_term_m':None,'selected_known_term_m':None,
                        'semantic_ownership_verified':False,
                        'physical_body_part_verified':False,
                        'distance_from_front_m':None,'measurement_authorized':False,
                        'navigation_eligible':False,'actuator_access':False,
                        'limitations':[
                            'The fixed RGB box fractions are a torso-core heuristic, not body-part recognition.',
                            'Stable local disparity does not establish physical distance accuracy or target clearance.',
                            'The median is a following-range diagnostic and not a nearest-obstacle statistic.']}
                    if core_count>=TORSO_CORE_POLICY['minimum_stable_points']:
                        stable_torso_core_front.update(describe_front(
                            owned_u[candidate_indices][stable_core],
                            owned_v[candidate_indices][stable_core],
                            owned_z[candidate_indices][stable_core],
                            front_geometry['P1'],front_geometry['R1'],
                            front_geometry['profile'],scale=scale,
                            source_depth_status=depth.get('status')))
                        stable_torso_core_front['status']='STABLE_TORSO_CORE_FRONT_DIAGNOSTIC_ONLY'
                        stable_torso_core_front['point_set']='STABLE_LOCAL_PATCHES_WITHIN_CURRENT_RGB_TORSO_CORE_DIAGNOSTIC_ONLY'
                        stable_torso_core_front['candidate_sample_count']=len(candidate_indices)
                        stable_torso_core_front['torso_core_candidate_point_count']=int(np.count_nonzero(in_core))
                        stable_torso_core_front['point_count']=core_count
                        stable_torso_core_front['torso_core_rgb_roi_xyxy_px']=core_roi
                        stable_torso_core_front['torso_core_policy']=dict(TORSO_CORE_POLICY)
                        stable_torso_core_front['stable_local_patch_policy']=dict(stable_report['policy'])
                        stable_torso_core_front['selected_known_term_m']=stable_torso_core_front['known_term_m']['median']
                        stable_torso_core_front['semantic_ownership_verified']=False
                        stable_torso_core_front['physical_body_part_verified']=False
                        stable_torso_core_front['limitations'].extend([
                            'The fixed RGB box fractions are a torso-core heuristic, not body-part recognition.',
                            'The median is a following-range diagnostic and not a nearest-obstacle statistic.'])
    def attach(result):
        if fast is not None:result['fast_local_consistency']=fast
        if front is not None:result['front_geometry']=front
        if layer_front is not None:result['candidate_layer_front_geometry']=layer_front
        if local_candidate_front is not None:result['local_candidate_front_geometry']=local_candidate_front
        if stable_candidate_front is not None:result['stable_candidate_front_geometry']=stable_candidate_front
        if stable_torso_core_front is not None:result['stable_torso_core_front_geometry']=stable_torso_core_front
        return result
    if defer_local and not prepare_local:
        return attach(diagnostic_result(depth,scale,lower,upper,
            {'status':'SKIPPED_BY_DIAGNOSTIC_SAMPLING'},True)),None
    if selected is None:
        interior = cv2.erode(mask.astype(np.uint8), np.ones((5,5),np.uint8),
                             borderType=cv2.BORDER_CONSTANT, borderValue=0)
        selected = owned(xy, interior, roi)
    u, v, d, xy = u[selected], v[selected], d[selected], xy[selected]
    samples={'u':u,'v':v,'d':d,'xy':xy,'interior':interior,
             'roi':np.asarray(roi),'fb':np.asarray(fb),'stride':np.asarray(stride),'scale':np.asarray(scale)}
    local = {'status':'DEFERRED_NOT_YET_COMPUTED'} if defer_local else local_surface_stats(samples)
    result=attach(diagnostic_result(depth,scale,lower,upper,local,defer_local))
    return (result,samples) if defer_local else result


def diagnostic_result(depth,scale,lower,upper,local,defer_local):
    result={'status':'MASKED_DEPTH_SHADOW_DIAGNOSTIC_ONLY' if defer_local else 'SHAPE_AWARE_SHADOW_DIAGNOSTIC_ONLY',
            'mask_depth':depth, 'local_surface':local,
            'reference_frame':'rectified_left_optical_frame', 'unit':'m',
            'confidence':None, 'confidence_status':'NOT_VALIDATED',
            'metric_measurement_authorized':False, 'navigation_eligible':False,
            'actuator_access':False, 'acceptance_rule_added':False,
            'diagnostic_depth_range_m':[lower,upper], 'stereo_scale':scale,
            'sample_stride_original_px':4,
            'limitations':['Local fit residuals are not physical distance error.',
                          'Global MAD remains a comparison metric, not shape-aware admission.']}
    return result


def local_surface_stats(samples):
    u,v,d,xy=(samples[k] for k in ('u','v','d','xy'))
    fb=float(samples['fb']);stride=int(samples['stride']);scale=float(samples['scale'])
    interior=samples['interior'];roi=samples['roi']
    local, data = analyze_local_surfaces(u, v, d, fb, stride=stride,
                                        tile_px=int(32*scale))
    local['rgb_coverage'] = coarse_coverage(interior, roi, xy,
                                          np.isfinite(data['axial_residual_m']))
    count = local['patch_count']
    local['unfitted_patch_fraction'] = 1-local['fitted_patch_count']/count if count else None
    return {k:v for k,v in local.items() if k!='patches'}


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


def infer_instances(source, model):
    """One model invocation returns matching boxes/masks with stable result indices."""
    if source.shape!=(1200,1920,3) or source.dtype!=np.uint8:
        raise ValueError('invalid current RGB frame')
    results=model.predict(source=source,imgsz=640,conf=.25,classes=[0],
                          retina_masks=True,device='cpu',verbose=False)
    if len(results)!=1:
        raise ValueError('unexpected segmentation batch')
    result=results[0]
    if result.boxes is None or not len(result.boxes):
        return [],[]
    if result.masks is None or len(result.masks.data)!=len(result.boxes):
        raise ValueError('missing box/mask correspondence')
    boxes=result.boxes.xyxy.cpu().numpy();scores=result.boxes.conf.cpu().numpy()
    labels=result.boxes.cls.cpu().numpy()
    if np.any(labels!=0):
        raise ValueError('unexpected semantic class')
    instances=[];masks=[]
    for index,(box,confidence) in enumerate(zip(boxes,scores)):
        mask=result.masks.data[index].cpu().numpy()>.5
        if mask.shape!=source.shape[:2] or not mask.any():
            raise ValueError('invalid instance mask')
        x1,y1,x2,y2=map(float,box)
        instances.append({'instance_index':index,'bbox_xyxy_px':[x1,y1,x2,y2],
                          'center_x_px':(x1+x2)/2,'confidence':float(confidence),
                          'truncated_left':x1<=3,'truncated_right':x2>=1917,
                          'truncated_top':y1<=3,'truncated_bottom':y2>=1197})
        masks.append(mask)
    return instances,masks


class CurrentInstanceMasks:
    """One inference result, consumed once for exactly its RGB cycle and index."""
    def __init__(self):
        self.current=None

    def infer(self, cycle_id, rgb_sha, source, model):
        self.current=None  # clear even if the next inference fails
        instances,masks=infer_instances(source,model)
        self.current=(cycle_id,rgb_sha,instances,masks)
        return instances

    def take(self, cycle_id, rgb_sha, index, bbox):
        current,self.current=self.current,None
        if current is None or current[0]!=cycle_id or current[1]!=rgb_sha:
            raise ValueError('no matching current RGB segmentation')
        instances,masks=current[2:]
        if type(index) is not int or not 0<=index<len(instances) or instances[index]['bbox_xyxy_px']!=bbox:
            raise ValueError('selected box and mask index disagree')
        return masks[index]

