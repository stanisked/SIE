import unittest
import hashlib
from pathlib import Path
import tempfile
from vision_core.tools.benchmark_fast_local_depth import sources_unchanged
import numpy as np
from vision_core.person_localization.fast_local_depth_consistency import analyze_fast_local_depth


class FastLocalTests(unittest.TestCase):
    def test_final_manifest_accepts_string_paths_and_detects_mutation(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'evidence.bin';path.write_bytes(b'original')
            manifest={str(path):hashlib.sha256(b'original').hexdigest()}
            self.assertTrue(sources_unchanged(manifest))
            self.assertTrue(sources_unchanged({path:manifest[str(path)]}))
            path.write_bytes(b'changed')
            self.assertFalse(sources_unchanged(manifest))

    def grid(self):
        v,u=np.mgrid[0:64:4,0:64:4]
        return u.ravel(),v.ravel()

    def test_sloped_surface_is_not_noise_and_input_unchanged(self):
        u,v=self.grid();d=20+.05*u+.08*v;before=d.copy()
        r,data=analyze_fast_local_depth(u,v,d,75.)
        self.assertTrue(np.array_equal(d,before))
        self.assertEqual(r['fitted_sample_fraction'],1.)
        self.assertGreater(r['raw_depth']['mad_m'],.05)
        self.assertLess(r['pooled_held_out_axial_residual']['absolute_max_m'],1e-12)
        self.assertTrue(data['evaluated'].all())
        self.assertFalse(r['suitable_for_control'])
        # Constant disparity bias remains undetectable by local consistency.
        biased,_=analyze_fast_local_depth(u,v,d+2,75.)
        self.assertLess(biased['pooled_held_out_axial_residual']['absolute_max_m'],1e-12)

    def test_crossfit_matches_individual_lstsq_and_keeps_outliers(self):
        u,v=self.grid();rng=np.random.default_rng(4)
        d=30+.01*u+.02*v+rng.normal(0,.3,len(u));d[::17]+=3
        r,data=analyze_fast_local_depth(u,v,d,75.)
        fold=((u//4+v//4)%2)
        reference=np.full(len(d),np.nan)
        for patch in np.unique(data['patch_id']):
            for parity in (0,1):
                train=(data['patch_id']==patch)&(fold==parity)
                test=(data['patch_id']==patch)&(fold!=parity)
                a=np.column_stack((u,v,np.ones(len(u))))
                coefficients=np.linalg.lstsq(a[train],d[train],rcond=None)[0]
                reference[test]=75/d[test]-75/(a[test]@coefficients)
        np.testing.assert_allclose(data['axial_residual_m'],reference,atol=1e-12)
        self.assertGreater(r['pooled_held_out_axial_residual']['absolute_max_m'],.1)
        self.assertEqual(r['pooled_held_out_axial_residual']['sample_count'],len(d))

    def test_sparse_degenerate_missing_and_invalid_never_become_good_depth(self):
        for u,v in ((np.array([]),np.array([])),(np.arange(0,32,4),np.zeros(8)),
                    (np.arange(0,128,4),np.zeros(32))):
            r,data=analyze_fast_local_depth(u,v,np.full(len(u),30.),75.)
            self.assertEqual(r['pooled_held_out_axial_residual']['sample_count'],0)
            self.assertFalse(r['suitable_for_control'])
        u=np.arange(0,128,4);v=np.zeros(32)
        degenerate,_=analyze_fast_local_depth(u,v,np.full(32,30.),75.,tile_px=128)
        self.assertEqual(degenerate['patch_status_counts']['DEGENERATE_TRAINING_SUPPORT'],1)
        u,v=self.grid()
        for d in (np.zeros(len(u)),np.full(len(u),np.nan)):
            with self.assertRaises(ValueError):analyze_fast_local_depth(u,v,d,75.)
        with self.assertRaises(ValueError):analyze_fast_local_depth(u+.5,v,np.ones(len(u)),75.)


if __name__=='__main__':unittest.main()
