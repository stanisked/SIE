"""Partial robot-front geometry, diagnostic only. Never supplies admission."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import numpy as np


def profile_hash(profile):
    return hashlib.sha256(json.dumps(profile,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def validate_profile(profile):
    if (profile.get('schema_version')!='sie.robot_front_mount_operator.v1' or
            profile.get('status')!='OPERATOR_REPORTED_PARTIAL_GEOMETRY_NOT_ACTIVATED' or
            profile.get('source_frame')!='physical_left_optical_frame' or
            profile.get('measurement_authorized') is not False or profile.get('actuator_access') is not False):
        raise ValueError('invalid diagnostic mount profile')
    for key in ('front_rim_behind_robot_front_m','block_pitch_up_deg'):
        if type(profile.get(key)) not in (int,float) or not np.isfinite(profile[key]):
            raise ValueError('invalid mount value: '+key)
    if not 0<=profile['front_rim_behind_robot_front_m']<2 or not -89<profile['block_pitch_up_deg']<89:
        raise ValueError('unsupported mount geometry')
    a=profile.get('alignment_assumptions',{})
    if a!={'block_yaw_deg':0.0,'block_roll_deg':0.0,'status':'UNSURVEYED_DIAGNOSTIC_ASSUMPTION'}:
        raise ValueError('only declared zero roll/yaw diagnostic hypothesis is supported')
    if profile.get('rim_forward_of_optical_center_m') is not None:
        raise ValueError('a surveyed optical datum requires a separate reviewed profile')
    if not profile.get('operator_provenance',{}).get('method'):
        raise ValueError('missing operator provenance')
    profile_hash(profile)  # reject non-JSON/nonfinite metadata as well
    return profile


def load_profile(path):
    return validate_profile(json.loads(Path(path).read_text()))


def forward_terms(u, v, z, p1, r1, profile, *, scale=1.):
    """Return known term K of front distance K - unknown optical-to-rim offset.

    R1 maps physical-left raw optical coordinates into rectified-left coordinates.
    Undo R1 first; the reported physical block pitch does not describe rectification.
    Optical axes: x right, y down, z forward. Robot longitudinal axis: forward.
    """
    validate_profile(profile)
    u,v,z=(np.asarray(x,dtype=np.float64) for x in (u,v,z))
    if any(x.ndim!=1 or x.shape!=z.shape or not np.isfinite(x).all() for x in (u,v,z)) or (z<=0).any():
        raise ValueError('expected equal finite 1-D pixel/depth arrays in metres')
    if scale not in (1.,.5,.25):raise ValueError('unsupported stereo scale')
    p1=np.asarray(p1,dtype=np.float64);r1=np.asarray(r1,dtype=np.float64)
    if p1.shape!=(3,4) or not np.isfinite(p1).all() or not np.allclose(p1[:,3],0,atol=1e-9,rtol=0):
        raise ValueError('expected physical-left pinhole P1 with zero translation')
    if p1[0,0]<=0 or p1[1,1]<=0 or not np.allclose(p1[2,:3],[0,0,1],atol=1e-9,rtol=0):
        raise ValueError('invalid projection matrix')
    if (r1.shape!=(3,3) or not np.isfinite(r1).all() or
            not np.allclose(r1.T@r1,np.eye(3),atol=1e-6,rtol=0) or
            not np.isclose(np.linalg.det(r1),1.,atol=1e-6,rtol=0)):
        raise ValueError('R1 must be a proper raw-left to rectified-left rotation')
    # Same full-resolution pixel-centre convention as the stereo/RGB projection.
    pixels=np.column_stack(((u+.5)/scale-.5,(v+.5)/scale-.5,np.ones(len(z))))
    rays=np.linalg.solve(p1[:,:3],pixels.T).T
    rect=rays*(z/rays[:,2])[:,None]
    raw=rect@r1
    theta=np.deg2rad(profile['block_pitch_up_deg'])
    forward=raw[:,1]*np.sin(theta)+raw[:,2]*np.cos(theta)
    return forward-profile['front_rim_behind_robot_front_m']


def describe_front(u,v,z,p1,r1,profile,*,scale=1.,source_depth_status=None):
    known=forward_terms(u,v,z,p1,r1,profile,scale=scale)
    stats=None
    if known.size:
        q=np.percentile(known,[0,5,50,95,100])
        stats=dict(zip(('minimum','p05','median','p95','maximum'),map(float,q)))
    return {'status':'PARTIAL_FRONT_GEOMETRY_DIAGNOSTIC_ONLY' if known.size else 'NO_OWNED_POINTS',
            'source_frame':'rectified_left_optical_frame',
            'reference_frame':'robot_longitudinal_axis_from_front_plane',
            'unit':'m','confidence':None,'confidence_status':'NOT_VALIDATED',
            'point_count':int(known.size),'known_term_m':stats,
            'distance_from_front_m':None,'rim_forward_of_optical_center_m':None,
            'formula':'distance_from_front_m = known_term_m - rim_forward_of_optical_center_m',
            'profile_sha256':profile_hash(profile),'source_depth_status':source_depth_status,
            'measurement_authorized':False,'navigation_eligible':False,'actuator_access':False,
            'limitations':['Optical-to-rim forward offset remains unknown.',
                           'Mount roll/yaw alignment and survey uncertainty remain unverified.',
                           'Owned visible target points are not complete swept-path obstacle coverage.',
                           'No physical distance accuracy or movement admission is established.']}
