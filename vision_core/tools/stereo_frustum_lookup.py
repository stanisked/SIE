"""Calibration-bound startup lookup for diagnostic RGB-to-stereo crop bounds.

Integer RGB lattice envelope, not a certified bound on continuous distortion.
No prior-frame bbox, model inference, depth correction or motion admission.
"""
import hashlib
import math
import cv2
import numpy as np


def calibration_binding(cal):
    digest=hashlib.sha256()
    for key in ('K','D','R1','R','T','P1','size'):
        value=np.ascontiguousarray(cal[key])
        digest.update(key.encode());digest.update(str(value.dtype).encode())
        digest.update(str(value.shape).encode());digest.update(value.tobytes())
    return digest.hexdigest()


class FrustumLookup:
    def __init__(self,cal,*,scale=.5,lower=.5,upper=4.5,rgb_size=(1920,1200),chunk_rows=64):
        if scale not in (1.,.5,.25) or not 0<lower<upper<=4.5 or chunk_rows<1:raise ValueError('invalid lookup settings')
        self.binding=calibration_binding(cal);self.scale=scale;self.lower=lower;self.upper=upper
        self.rgb_size=tuple(rgb_size);w,h=self.rgb_size
        self.stereo_size=tuple(int(n*scale) for n in cal['size'])
        # Include the image boundary coordinate accepted by the reference ROI API.
        self.table=np.full((2,h+1,w+1,2),np.nan,dtype=np.float64)
        transform=cal['R1']@cal['R'];translation=cal['R1']@cal['T']
        for start in range(0,h+1,chunk_rows):
            stop=min(h+1,start+chunk_rows)
            xx,yy=np.meshgrid(np.arange(w+1,dtype=float),np.arange(start,stop,dtype=float))
            normalized=cv2.undistortPoints(np.column_stack((xx.ravel(),yy.ravel())).reshape(-1,1,2),cal['K'],cal['D']).reshape(-1,2)
            rays=np.column_stack((normalized,np.ones(len(normalized))))
            q=(transform@rays.T).T
            for index,z in enumerate((lower,upper)):
                valid=np.isfinite(q).all(axis=1)&(q[:,2]>0)
                distance=np.full(len(q),np.nan)
                distance[valid]=(z-translation[2])/q[valid,2]
                valid&=distance>0
                points=q*distance[:,None]+translation
                homogeneous=(cal['P1']@np.column_stack((points,np.ones(len(points)))).T).T
                projected=(homogeneous[:,:2]/homogeneous[:,2,None]+.5)*scale-.5
                projected[~valid]=np.nan
                self.table[index,start:stop]=projected.reshape(stop-start,w+1,2)
        self.table.setflags(write=False)

    def core(self,roi,cal,scale,lower=.5,upper=4.5):
        if scale!=self.scale or lower!=self.lower or upper!=self.upper or calibration_binding(cal)!=self.binding:raise ValueError('lookup calibration/scale mismatch')
        x1,y1,x2,y2=map(float,roi);w,h=self.rgb_size
        if not (0<=x1<x2<=w and 0<=y1<y2<=h):raise ValueError('invalid lookup ROI')
        # Outward integer coverage includes all cells touched by this ROI.
        bounds=self._extrema(math.floor(x1),math.floor(y1),math.ceil(x2)+1,math.ceil(y2)+1)
        if bounds is None:return [0,0,*self.stereo_size],'FULL_FRAME_LOOKUP_NON_FORWARD'
        low,high=bounds
        sw,sh=self.stereo_size
        core=[max(0,min(sw,math.floor(low[0]))),max(0,min(sh,math.floor(low[1]))),
              max(0,min(sw,math.ceil(high[0])+1)),max(0,min(sh,math.ceil(high[1])+1))]
        if core[2]<=core[0] or core[3]<=core[1]:raise ValueError('lookup ROI has no stereo overlap')
        return core,'STARTUP_INTEGER_GRID_ENVELOPE_DIAGNOSTIC_ONLY'


    def _extrema(self,x1,y1,x2,y2):
        region=self.table[:,y1:y2,x1:x2,:]
        if not np.isfinite(region).all():return None
        return region.min(axis=(0,1,2)),region.max(axis=(0,1,2))


class BlockFrustumLookup(FrustumLookup):
    """Exact integer-grid extrema via full tiles plus unrounded edge strips."""
    def __init__(self,reference,block_size=32):
        if type(block_size) is not int or block_size<1:raise ValueError('invalid block size')
        self.__dict__.update(reference.__dict__)
        self.block_size=block_size
        h,w=self.table.shape[1:3];b=block_size
        self.block_min=np.empty(((h+b-1)//b,(w+b-1)//b,2))
        self.block_max=np.empty_like(self.block_min)
        self.block_valid=np.empty(self.block_min.shape[:2],dtype=bool)
        for by in range(self.block_min.shape[0]):
            for bx in range(self.block_min.shape[1]):
                region=self.table[:,by*b:min((by+1)*b,h),bx*b:min((bx+1)*b,w),:]
                self.block_valid[by,bx]=np.isfinite(region).all()
                self.block_min[by,bx]=region.min(axis=(0,1,2))
                self.block_max[by,bx]=region.max(axis=(0,1,2))
        for array in (self.block_min,self.block_max,self.block_valid):array.setflags(write=False)
        self.index_bytes=sum(a.nbytes for a in (self.block_min,self.block_max,self.block_valid))

    def _extrema(self,x1,y1,x2,y2):
        b=self.block_size
        ix1=(x1+b-1)//b*b;ix2=x2//b*b
        iy1=(y1+b-1)//b*b;iy2=y2//b*b
        if ix1>=ix2 or iy1>=iy2:return super()._extrema(x1,y1,x2,y2)
        selection=(slice(iy1//b,iy2//b),slice(ix1//b,ix2//b))
        if not self.block_valid[selection].all():return None
        lows=[self.block_min[selection].min(axis=(0,1))]
        highs=[self.block_max[selection].max(axis=(0,1))]
        # Disjoint strips: do not round the queried rectangle to tile boundaries.
        for box in ((x1,y1,x2,iy1),(x1,iy2,x2,y2),(x1,iy1,ix1,iy2),(ix2,iy1,x2,iy2)):
            if box[0]>=box[2] or box[1]>=box[3]:continue
            bounds=super()._extrema(*box)
            if bounds is None:return None
            lows.append(bounds[0]);highs.append(bounds[1])
        return np.min(lows,axis=0),np.max(highs,axis=0)
