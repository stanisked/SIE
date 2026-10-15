import sys
from pathlib import Path
import unittest
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from preview_mask_surface import evaluate
from compare_shadow_mask_depth_median import owned,in_roi
from vision_core.person_localization.instance_mask_stereo_fusion import masked_depth_hypothesis,depth_from_owned_samples
from vision_core.tests.test_preview_mask_surface import calibration


class OwnershipEquivalenceTests(unittest.TestCase):
    def test_random_borders_holes_nan_and_rounding_match_reference(self):
        rng=np.random.default_rng(19)
        for kind in range(5):
            mask=np.ones((70,90),bool)
            if kind==1:mask[:]=False
            if kind==2:mask[10:30,20:60]=False
            if kind==3:mask=rng.random(mask.shape)>.07
            if kind==4:mask[:,40:]=False
            xy=rng.uniform([-3,-3],[93,73],(5000,2));xy[:4]=[[np.nan,2],[2,np.inf],[1.5,2.5],[89.5,69.5]]
            z=rng.uniform(2.,2.3,len(xy));roi=[0.,0.,90.,70.]
            old=masked_depth_hypothesis(xy[in_roi(xy,roi)],z[in_roi(xy,roi)],mask)
            interior=cv2.erode(mask.astype('uint8'),np.ones((5,5),'uint8'),borderType=cv2.BORDER_CONSTANT,borderValue=0)
            selected=owned(xy,interior,roi)
            new=depth_from_owned_samples(z[selected],np.count_nonzero(mask),np.count_nonzero(interior))
            self.assertEqual(old,new)

    def test_every_scale_and_local_sample_arrays_unchanged(self):
        rng=np.random.default_rng(4);cal=calibration();mask=np.ones((1200,1920),bool);mask[20:30,40:50]=False
        for scale in (1.,.5,.25):
            d=rng.uniform(15*scale,40*scale,(int(96*scale),int(128*scale))).astype('float32')
            d[::7,::3]=np.nan
            for prepare in (False,True):
                args=(d,mask,cal,[0,0,128,96])
                a,sa=evaluate(*args,scale=scale,defer_local=True,prepare_local=prepare,ownership_backend='reference_numpy')
                b,sb=evaluate(*args,scale=scale,defer_local=True,prepare_local=prepare)
                self.assertEqual(a,b)
                if prepare:
                    for key in sa:np.testing.assert_array_equal(sa[key],sb[key])
                else:self.assertIsNone(sb)


if __name__=='__main__':unittest.main()
