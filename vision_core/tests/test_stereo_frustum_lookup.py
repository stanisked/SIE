import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from stereo_frustum_lookup import FrustumLookup
from analyze_target_stereo_crop import frustum_core,crop_plan
from vision_core.tests.test_preview_mask_surface import calibration


class FrustumLookupTests(unittest.TestCase):
    def test_integer_reference_bounds_and_fractional_outward_coverage(self):
        cal=calibration();lookup=FrustumLookup(cal,rgb_size=(128,96),chunk_rows=17)
        roi=[10,10,80,60]
        self.assertEqual(lookup.core(roi,cal,.5)[0],frustum_core(roi,cal,.5)[0])
        roi=[10.2,11.3,70.8,60.4]
        a=frustum_core(roi,cal,.5)[0];b=lookup.core(roi,cal,.5)[0]
        self.assertTrue(b[0]<=a[0] and b[1]<=a[1] and b[2]>=a[2] and b[3]>=a[3])
        self.assertFalse(lookup.table.flags.writeable)
        configs=[{'minDisparity':0,'numDisparities':64,'blockSize':7},{'minDisparity':-64,'numDisparities':64,'blockSize':7}]
        plan=crop_plan(roi,cal,configs,frustum_lookup=lookup)
        self.assertIn('STARTUP',plan['frustum_method'])

    def test_calibration_range_scale_mismatch_and_invalid_roi_refuse(self):
        cal=calibration();lookup=FrustumLookup(cal,rgb_size=(128,96))
        other={**cal,'K':cal['K'].copy()};other['K'][0,0]+=1
        with self.assertRaises(ValueError):lookup.core([1,1,10,10],other,.5)
        with self.assertRaises(ValueError):lookup.core([1,1,10,10],cal,1.)
        with self.assertRaises(ValueError):lookup.core([1,1,10,10],cal,.5,lower=1.)
        with self.assertRaises(ValueError):lookup.core([10,10,1,1],cal,.5)

    def test_nonforward_lookup_falls_back_and_chunking_preserves_values(self):
        cal=calibration()
        a=FrustumLookup(cal,rgb_size=(128,96),chunk_rows=11)
        b=FrustumLookup(cal,rgb_size=(128,96),chunk_rows=32)
        np.testing.assert_allclose(a.table,b.table,rtol=0,atol=1e-12)
        cal['R']=np.diag([-1.,1.,-1.])
        backward=FrustumLookup(cal,rgb_size=(128,96))
        self.assertEqual(backward.core([1,1,10,10],cal,.5)[0],[0,0,64,48])


if __name__=='__main__':unittest.main()
