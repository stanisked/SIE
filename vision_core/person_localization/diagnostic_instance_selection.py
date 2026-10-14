"""Current-frame segmentation target selection; diagnostic, no frozen contract."""
from datetime import datetime
import math


def select_instance(instances, *, prior_track, captured_at_utc, max_age_s,
                    minimum_iou, maximum_center_displacement_relative, minimum_match_margin, minimum_confidence=.70):
    candidates=[]
    for item in instances:
        box=item.get('bbox_xyxy_px',[])
        if len(box)!=4 or not all(math.isfinite(float(v)) for v in box):
            continue
        x1,y1,x2,y2=box
        if not 0<=x1<x2<=1920 or not 0<=y1<y2<=1200 or not minimum_confidence<=item.get('confidence',-1)<=1:
            continue
        # Preserve the existing current-frame top/left/right truncation rule.
        # Bottom truncation is recorded but may proceed to geometry checks.
        if x1<=3 or x2>=1917 or y1<=3:
            continue
        candidates.append(item)
    base={'disposition':'UNAVAILABLE','candidate':None,'current_eligible_count':len(candidates),
          'metric_measurement_authorized':False,'selection_source':'CURRENT_FRAME_SEGMENTATION',
          'policy':{'minimum_confidence':minimum_confidence,'max_age_s':max_age_s,
                    'minimum_iou':minimum_iou,'maximum_center_displacement_relative':maximum_center_displacement_relative,
                    'minimum_match_margin':minimum_match_margin}}
    if len(candidates)==1:
        return {**base,'disposition':'OBSERVED','reason':'DIRECT_CURRENT_FRAME_SINGLE_TARGET','candidate':candidates[0]}
    if not candidates:
        return {**base,'reason':'NO_CURRENT_NON_EDGE_TARGET'}
    if not isinstance(prior_track,dict):
        return {**base,'reason':'MULTIPLE_TARGETS_NO_CONFIRMED_TRACK'}
    try:
        now=datetime.fromisoformat(captured_at_utc)
        then=datetime.fromisoformat(prior_track['confirmed_at_utc'])
        age=(now-then).total_seconds()
        box=list(prior_track['bbox_xyxy_px'])
        if not 0<=age<=max_age_s:
            return {**base,'reason':'TEMPORAL_TRACK_STALE'}
        if len(box)!=4 or not all(math.isfinite(float(v)) for v in box) or box[2]<=box[0] or box[3]<=box[1]:
            raise ValueError('invalid track box')
        older=prior_track.get('previous_bbox_xyxy_px')
        older_at=prior_track.get('previous_confirmed_at_utc')
        if older is not None and older_at is not None:
            interval=(then-datetime.fromisoformat(older_at)).total_seconds()
            if interval>0 and len(older)==4 and all(math.isfinite(float(v)) for v in older):
                box=[v+(v-o)*age/interval for v,o in zip(box,older)]
        if box[2]<=box[0] or box[3]<=box[1]:
            raise ValueError('invalid predicted box')
    except (KeyError,TypeError,ValueError):
        return {**base,'reason':'TEMPORAL_TRACK_INVALID'}
    diagonal=math.hypot(box[2]-box[0],box[3]-box[1]);matches=[]
    for item in candidates:
        b=item['bbox_xyxy_px']
        intersection=max(0,min(box[2],b[2])-max(box[0],b[0]))*max(0,min(box[3],b[3])-max(box[1],b[1]))
        union=(box[2]-box[0])*(box[3]-box[1])+(b[2]-b[0])*(b[3]-b[1])-intersection
        iou=intersection/union
        delta=math.hypot((b[0]+b[2]-box[0]-box[2])/2,(b[1]+b[3]-box[1]-box[3])/2)/diagonal
        if iou>=minimum_iou and delta<=maximum_center_displacement_relative:
            score=iou+.25*max(0,1-delta/maximum_center_displacement_relative)
            matches.append((score,item))
    matches.sort(key=lambda pair:pair[0],reverse=True)
    if not matches or (len(matches)>1 and matches[0][0]-matches[1][0]<minimum_match_margin):
        return {**base,'reason':'TEMPORAL_CURRENT_FRAME_MATCH_AMBIGUOUS_OR_ABSENT'}
    return {**base,'disposition':'OBSERVED','reason':'TEMPORAL_CURRENT_FRAME_MATCH','candidate':matches[0][1]}
