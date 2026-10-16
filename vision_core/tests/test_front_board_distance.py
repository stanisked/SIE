"""Offline checks for board correspondence and single-point reporting."""
import unittest
import numpy as np
from vision_core.tools.check_robot_front_board_distance import pair_corner_order,comparison,stats,corner_distance_diagnostic

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
    def test_corner_refusal_is_separate_from_dense_comparison(self):
        from vision_core.person_localization.robot_front_geometry import load_profile
        from pathlib import Path
        profile=load_profile(Path(__file__).resolve().parents[2]/'vision_core/config/diagnostics/robot_front_mount_operator_v1.json')
        p1=np.array([[100.,0,0,0],[0,100.,0,0],[0,0,1,0]])
        p2=p1.copy();p2[0,3]=-75000
        left=np.array([[100.,0.],[110.,20.],[120.,40.]])
        right=left-np.array([20.,-4.])
        corner,_,depth,known=corner_distance_diagnostic(left,right,p1,p2,np.eye(3),profile)
        self.assertEqual(corner['status'],'CORNER_COMPARISON_REFUSED')
        self.assertEqual(corner['reason'],'CORNER_EPIPOLAR_ERROR_EXCEEDED')
        self.assertEqual(corner['order_diagnostics'][0]['median_absolute_vertical_px'],4.)
        self.assertEqual(len(depth),0);self.assertEqual(len(known),0)
        report=comparison([{'dense_known_term':stats([1.49,1.50,1.51]),'corner_comparison':corner}],1.5)
        self.assertEqual(report['median_known_term_m'],1.5)
        self.assertTrue(report['corner_geometry_warning'])
        self.assertFalse(report['physical_depth_error_bound_established'])
    def test_absent_right_board_is_supplementary_only(self):
        result,_,_,_=corner_distance_diagnostic(np.zeros((54,2)),None,None,None,None,None)
        self.assertEqual(result['reason'],'BOARD_NOT_FOUND_RIGHT')
    def test_missing_samples_and_invalid_values(self):
        self.assertEqual(comparison([],1.5)['status'],'NO_ACCEPTED_DENSE_BOARD_FRAMES')
        for a in ([],[np.nan],[np.inf]):
            with self.assertRaises(ValueError):stats(a)
        with self.assertRaises(ValueError):comparison([],0.)

if __name__=='__main__':unittest.main()
