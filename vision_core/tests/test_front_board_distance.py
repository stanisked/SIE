"""Offline checks for board correspondence and single-point reporting."""
import unittest
import numpy as np
from vision_core.tools.check_robot_front_board_distance import pair_corner_order,comparison,stats

class BoardDistanceTests(unittest.TestCase):
    def test_order_recovery_uses_both_views_and_epipolar_evidence(self):
        y,x=np.mgrid[0:6,0:9];left=np.column_stack((x.ravel()*10+200,y.ravel()*10+100)).astype(float)
        right=left-np.array([100.,0.])
        name,chosen,error=pair_corner_order(left,right[::-1])
        self.assertEqual(name,'reverse_180');self.assertEqual(error,0.)
        np.testing.assert_array_equal(chosen,right)
    def test_bad_vertical_or_nonpositive_disparity_rejected(self):
        left=np.array([[100.,0.],[110.,20.],[120.,40.]])
        with self.assertRaisesRegex(ValueError,'EPIPOLAR'):
            pair_corner_order(left,left-np.array([20.,-10.]))
        with self.assertRaisesRegex(ValueError,'NO_POSITIVE'):
            pair_corner_order(left,left+np.array([100.,0.]))
    def test_ambiguous_order_is_not_guessed(self):
        a=np.array([[100.,20.],[110.,20.]])
        with self.assertRaisesRegex(ValueError,'AMBIGUOUS'):pair_corner_order(a,a-[50.,0.])
    def test_numeric_screen_never_estimates_optical_offset_or_physical_bound(self):
        rows=[{'dense_known_term':stats([1.48,1.49,1.50])} for _ in range(3)]
        r=comparison(rows,1.5)
        self.assertAlmostEqual(r['known_term_minus_physical_reference_m'],-.01)
        self.assertTrue(r['within_declared_100mm_numeric_screen'])
        self.assertFalse(r['screen_is_accuracy_validation'])
        self.assertFalse(r['optical_offset_estimated'])
        self.assertFalse(r['correction_applied'])
        self.assertFalse(r['physical_depth_error_bound_established'])
    def test_missing_samples_and_invalid_values(self):
        self.assertEqual(comparison([],1.5)['status'],'NO_ACCEPTED_DENSE_BOARD_FRAMES')
        for a in ([],[np.nan],[np.inf]):
            with self.assertRaises(ValueError):stats(a)
        with self.assertRaises(ValueError):comparison([],0.)

if __name__=='__main__':unittest.main()
