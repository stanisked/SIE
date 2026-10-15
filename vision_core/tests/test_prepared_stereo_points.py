import sys
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from preview_mask_surface import prepare_stereo_points,evaluate_prepared,evaluate
from vision_core.tests.test_preview_mask_surface import calibration


class PreparedPointsTests(unittest.TestCase):
    def test_split_matches_all_scales_roi_masks_and_local_inputs(self):
        rng=np.random.default_rng(72);cal=calibration()
        for scale in (1.,.5,.25):
            d=rng.uniform(12*scale,45*scale,(int(96*scale),int(128*scale))).astype('float32');d[::3,::5]=np.nan
            points=prepare_stereo_points(d,cal,scale=scale,cycle_id='one')
            for empty in (False,True):
                mask=np.ones((1200,1920),bool);mask[20:35,40:90]=False
                if empty:mask[:]=False
                for roi in ([0,0,128,96],[10,12,100,80]):
                    reference,samples=evaluate(d,mask,cal,roi,scale=scale,defer_local=True)
                    with patch('preview_mask_surface.project',side_effect=AssertionError('reprojection')):
                        prepared,other=evaluate_prepared(points,mask,roi,defer_local=True)
                    self.assertEqual(reference,prepared)
                    for key in samples:np.testing.assert_array_equal(samples[key],other[key])
            self.assertEqual(points['source_cycle_id'].item(),'one')


if __name__=='__main__':unittest.main()
