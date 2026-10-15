import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from analyze_target_stereo_crop import frustum_core,crop_plan,support_loss


def calibration():
    p=np.array([[500.,0,640,0],[0,500.,400,0],[0,0,1,0]])
    return {'P1':p,'R1':np.eye(3),'R':np.eye(3),'T':np.zeros(3),'K':p[:,:3].copy(),'D':np.zeros(5),'size':(1280,800)}


class CropAreaTests(unittest.TestCase):
    def test_identity_frustum_and_bidirectional_support_padding(self):
        cal=calibration();configs=[{'minDisparity':0,'numDisparities':64,'blockSize':7},{'minDisparity':-64,'numDisparities':64,'blockSize':7}]
        plan=crop_plan([500,300,780,500],cal,configs)
        a,b,c,d=plan['core_xyxy_px']
        self.assertLessEqual(a,250);self.assertGreaterEqual(c,390)
        self.assertLess(plan['area_fraction'],1)
        loss=support_loss(plan,np.array([250.,320.,390.]),np.array([200.,200.,200.]),np.array([63.,32.,1.]))
        self.assertEqual(loss['outside_crop_count'],0)
        self.assertEqual(loss['additional_crop_support_loss_count'],0)

    def test_outside_point_and_no_optimistic_empty_coverage(self):
        configs=[{'minDisparity':0,'numDisparities':64,'blockSize':7},{'minDisparity':-64,'numDisparities':64,'blockSize':7}]
        plan=crop_plan([500,300,780,500],calibration(),configs)
        lost=support_loss(plan,np.array([500.]),np.array([200.]),np.array([20.]))
        self.assertEqual(lost['outside_crop_count'],1)
        self.assertEqual(lost['additional_crop_support_loss_count'],1)
        empty=support_loss(plan,np.array([]),np.array([]),np.array([]))
        self.assertIsNone(empty['unsupported_fraction'])
        with self.assertRaises(ValueError):frustum_core([20,20,10,10],calibration(),.5)
        far=calibration();far['T']=np.array([100.,0,0])
        with self.assertRaisesRegex(ValueError,'no stereo overlap'):frustum_core([500,300,780,500],far,.5)

    def test_rectified_z_translation_and_nonforward_fallback(self):
        cal=calibration();cal['T']=np.array([.1,0,.2])
        core,method=frustum_core([640,400,642,402],cal,.5)
        # At z=.5 m, x shift from t_x is fx*.1/.5=100 original px.
        self.assertGreaterEqual(core[2],370)
        cal['R']=np.diag([-1.,1.,-1.])
        core,method=frustum_core([500,300,780,500],cal,.5)
        self.assertEqual(core,[0,0,640,400]);self.assertIn('FULL_FRAME',method)


if __name__=='__main__':unittest.main()
