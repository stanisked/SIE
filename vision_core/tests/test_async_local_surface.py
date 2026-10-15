"""No foreground fit, numerical parity, bounded latest queue and real fit RPC."""
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'vision_core/tools'))
sys.path.insert(0,str(ROOT/'ros2/sie_ros2/sie_ros2'))
from preview_mask_surface import evaluate,local_surface_stats
from async_local_surface import LocalSurfaceClient,LatestSurfaceDiagnostics


def inputs():
    p=np.array([[100.,0,0,0],[0,100.,0,0],[0,0,1,0]])
    p2=p.copy();p2[0,3]=-75954.18816496264
    cal=dict(P1=p,P2=p2,R1=np.eye(3),R=np.eye(3),T=np.zeros(3),K=p[:,:3],D=np.zeros(5),size=(128,96))
    _,u=np.indices((48,64));d=(15+.04*u).astype(np.float32)
    return d,np.ones((1200,1920),bool),cal,[4,4,120,88]


class AsyncSurfaceTests(unittest.TestCase):
    def test_fast_mask_depth_does_not_fit_and_matches_synchronous_result(self):
        d,mask,cal,roi=inputs()
        original=evaluate(d,mask,cal,roi,scale=.5)
        with patch('preview_mask_surface.analyze_local_surfaces',side_effect=AssertionError('foreground fit')):
            fast,samples=evaluate(d,mask,cal,roi,scale=.5,defer_local=True)
        self.assertEqual(fast['mask_depth'],original['mask_depth'])
        self.assertEqual(fast['local_surface']['status'],'DEFERRED_NOT_YET_COMPUTED')
        self.assertEqual(local_surface_stats(samples),original['local_surface'])
        self.assertEqual(fast['status'],'MASKED_DEPTH_SHADOW_DIAGNOSTIC_ONLY')

    def test_unscheduled_cycle_keeps_depth_without_preparing_a_local_job(self):
        d,mask,cal,roi=inputs()
        reference=evaluate(d,mask,cal,roi,scale=.5)
        with patch('preview_mask_surface.local_surface_stats',side_effect=AssertionError('unneeded local fit')):
            skipped,job=evaluate(d,mask,cal,roi,scale=.5,defer_local=True,prepare_local=False)
        self.assertIsNone(job)
        self.assertEqual(skipped['mask_depth'],reference['mask_depth'])
        self.assertEqual(skipped['local_surface']['status'],'SKIPPED_BY_DIAGNOSTIC_SAMPLING')

    def test_one_pending_replaces_old_job_and_preserves_identity_and_array_copy(self):
        started,release=threading.Event(),threading.Event();seen=[]
        class Client:
            closed=False
            def compute(self,job):
                if job['cycle_id']=='one':
                    started.set()
                    if not release.wait(3):raise RuntimeError('test timeout')
                seen.append((job['cycle_id'],job['samples']['value'].copy()))
                return {'cycle_id':job['cycle_id'],'status':'LOCAL_SURFACE_DIAGNOSTIC_ONLY','local_surface':{}}
            def close(self):self.closed=True
        client=Client()
        with tempfile.TemporaryDirectory() as folder:
            worker=LatestSurfaceDiagnostics(client,Path(folder)/'results')
            try:
                worker.submit('one',{'value':np.array([1])},{'source_cycle_id':'one'})
                self.assertTrue(started.wait(2))
                worker.submit('two',{'value':np.array([2])},{'source_cycle_id':'two'})
                samples={'value':np.array([3])}
                worker.submit('three',samples,{'source_cycle_id':'three'});samples['value'][0]=99
                self.assertEqual(worker.pending.qsize(),1);self.assertEqual(worker.dropped,1)
            finally:release.set();worker.close()
            self.assertEqual([cid for cid,_ in seen],['one','three'])
            self.assertEqual(seen[-1][1][0],3)
            files=sorted(worker.directory.glob('local-surface-*.json'))
            self.assertEqual(len(files),2)
            for path in files:
                row=json.loads(path.read_text());self.assertEqual(row['cycle_id'],row['provenance']['source_cycle_id'])
                self.assertFalse(row['actuator_access'])
            self.assertTrue(client.closed);self.assertFalse(worker.submit('late',{},{}))

    def test_real_separate_fit_worker_matches_local_stats_and_rejects_wrong_cycle(self):
        d,mask,cal,roi=inputs();_,samples=evaluate(d,mask,cal,roi,scale=.5,defer_local=True)
        expected=local_surface_stats(samples)
        client=LocalSurfaceClient(sys.executable,ROOT/'vision_core/tools/local_surface_async_worker.py')
        before=set(Path('/dev/shm').glob('sie-local-*'))
        try:
            result=client.compute({'cycle_id':'one','samples':samples})
            self.assertEqual(result['cycle_id'],'one')
            self.assertEqual(result['local_surface']['sample_count'],expected['sample_count'])
            for key,value in expected['pooled_held_out_axial_residual'].items():
                self.assertAlmostEqual(result['local_surface']['pooled_held_out_axial_residual'][key],value,places=10)
            client._read=lambda timeout:{'status':'LOCAL_SURFACE_DIAGNOSTIC_ONLY','cycle_id':'WRONG'}
            with self.assertRaisesRegex(ValueError,'cycle mismatch'):client.compute({'cycle_id':'two','samples':samples})
            self.assertIsNone(client.process)
        finally:client.close()
        self.assertEqual(set(Path('/dev/shm').glob('sie-local-*')),before)


if __name__=='__main__':unittest.main()
