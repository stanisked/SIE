import unittest
from vision_core.tools.analyze_local_rgb_overlap import analyze,overlap

class LocalOverlapTests(unittest.TestCase):
    def test_exact_boundary_and_source_grouping(self):
        def row(n,start):return {'cycle_id':f'run-{n:08d}','timestamp':f'2026-10-15T00:00:0{start}+00:00',
            'perception_timeline_s':{'rgb_call_start_after_cycle_start_s':.1,'rgb_call_end_after_cycle_start_s':.3},
            'processing_timing':{'host_capture_start_to_result_ready_s':.24}}
        primary=[row(1,0),row(2,1),row(3,2)]
        local=[{'cycle_id':'run-00000001','provenance':{'source_cycle_id':'run-00000001'},
            'completed_at_utc':'2026-10-15T00:00:01.200000+00:00','submit_to_result_s':.3,'queue_wait_s':.01}]
        result=analyze(primary,local)
        self.assertEqual([r['group'] for r in result['rows']],['LOCAL_REQUEST_OVERLAP','NO_RECORDED_LOCAL_OVERLAP'])
        self.assertEqual(result['excluded'],{'first_cycle':1})
        self.assertEqual(overlap((1,2),(2,3)),0)
        local[0]['provenance']['source_cycle_id']='old'
        with self.assertRaises(ValueError):analyze(primary,local)

if __name__=='__main__':unittest.main()
