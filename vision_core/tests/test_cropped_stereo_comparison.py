import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from benchmark_target_stereo_crop import restore_crop,sample_comparison
from vision_core.person_localization.diagnostic_stereo_resolution import half_disparity


class CroppedStereoTests(unittest.TestCase):
    def test_global_origin_and_lr_bankers_rounding_are_preserved(self):
        left=np.full((2,6),-16,np.int16);right=np.full((2,6),-80,np.int16)
        left[0,2]=24;right[0,1]=-24;right[0,0]=-48
        _,local_lr=half_disparity(left,right)
        self.assertFalse(local_lr[0,2])
        raw=restore_crop([left,right],[3,1,9,3],(4,12),[{'minDisparity':0},{'minDisparity':-4}])
        disparity,global_lr=half_disparity(*raw)
        self.assertTrue(global_lr[1,5]);self.assertEqual(disparity[1,5],1.5)
        self.assertFalse(global_lr[:,0].any())
        self.assertEqual(raw[0][0,0],-16)
        np.testing.assert_array_equal(raw[0][1:3,3:9],left)

    def test_invalid_crop_or_shape_is_rejected(self):
        raw=np.zeros((2,3),np.int16);configs=[{'minDisparity':0},{'minDisparity':-64}]
        with self.assertRaises(ValueError):restore_crop([raw,raw],[1,0,6,2],(3,6),configs)
        with self.assertRaises(ValueError):restore_crop([raw,raw],[-1,0,2,2],(3,6),configs)

    def test_lost_gained_and_changed_samples_are_not_hidden_by_same_counts(self):
        a={'u':np.array([1,2,3]),'v':np.array([1,1,1]),'d':np.array([10.,12.,14.])}
        b={'u':np.array([2,3,4]),'v':np.array([1,1,1]),'d':np.array([12.,15.,16.])}
        report=sample_comparison(a,b,20,38.)
        self.assertEqual(report['lost_sample_count'],1);self.assertEqual(report['gained_sample_count'],1)
        self.assertEqual(report['changed_common_disparities'],1)
        self.assertEqual(report['lost_xy_px'],[[1,1]])
        self.assertEqual(report['gained_xy_px'],[[4,1]])
        empty={k:np.array([]) for k in ('u','v','d')}
        self.assertIsNone(sample_comparison(empty,empty,20,38.)['depth_delta_m']['absolute_max'])


if __name__=='__main__':unittest.main()
