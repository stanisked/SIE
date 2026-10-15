"""Real subprocess groups: terminal SIGINT, drain, and unexpected exit evidence."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ros2/sie_ros2/sie_ros2'))
from async_local_surface import LocalSurfaceClient,LatestSurfaceDiagnostics


class LocalShutdownTests(unittest.TestCase):
    def test_group_sigint_does_not_interrupt_active_local_job(self):
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory);worker=folder/'worker.py'
            worker.write_text('''import json,os,sys,time
print(json.dumps({'status':'READY'}),flush=True)
for line in sys.stdin:
    request=json.loads(line)
    print('ACCEPTED',file=sys.stderr,flush=True)
    time.sleep(.15)
    print(json.dumps({'cycle_id':request['cycle_id'],'status':'LOCAL_SURFACE_DIAGNOSTIC_ONLY','input_sha256':request['input_sha256'],'local_surface':{},'worker_group':os.getpgrp()}),flush=True)
''')
            parent=folder/'parent.py'
            parent.write_text('''import json,os,signal,sys
from pathlib import Path
sys.path.insert(0,sys.argv[1])
sys.path.insert(0,str(Path(sys.argv[1]).parents[2]))
from async_local_surface import LocalSurfaceClient
signal.signal(signal.SIGINT,lambda *args:None)
client=LocalSurfaceClient(sys.executable,Path(sys.argv[2]))
print('ARMED',flush=True)
r=client.compute({'cycle_id':'one','samples':{}})
client.close()
print(json.dumps({'result':r,'parent_group':os.getpgrp(),'exit':client.last_exit}),flush=True)
''')
            p=subprocess.Popen([sys.executable,str(parent),str(ROOT/'ros2/sie_ros2/sie_ros2'),str(worker)],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,start_new_session=True)
            try:
                self.assertEqual(p.stdout.readline().strip(),'ARMED')
                # Signal the whole parent's foreground-style process group.
                os.killpg(p.pid,signal.SIGINT)
                stdout,stderr=p.communicate(timeout=10)
                self.assertEqual(p.returncode,0,stderr)
                result=json.loads(stdout.strip())
                self.assertEqual(result['result']['cycle_id'],'one')
                self.assertNotEqual(result['result']['worker_group'],result['parent_group'])
                self.assertEqual(result['exit']['returncode'],0)
                self.assertEqual(result['exit']['method'],'STDIN_EOF')
            finally:
                if p.poll() is None:p.kill();p.wait()
                for stream in (p.stdout,p.stderr):stream.close()

    def test_crash_exit_code_is_preserved_in_saved_result(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);script=root/'crash.py'
            script.write_text("import os,sys\nprint('{\"status\":\"READY\"}',flush=True)\nsys.stdin.readline()\nos._exit(7)\n")
            client=LocalSurfaceClient(sys.executable,script)
            queue=LatestSurfaceDiagnostics(client,root/'results')
            queue.submit('one',{'x':np.array([1])},{'source_cycle_id':'one'})
            queue.close()
            result=json.loads(next((root/'results').glob('*.json')).read_text())
            self.assertEqual(result['status'],'LOCAL_SURFACE_UNAVAILABLE')
            self.assertEqual(result['worker_failure']['returncode'],7)
            self.assertEqual(result['worker_failure']['stage'],'UNEXPECTED_STDOUT_EOF')
            self.assertEqual(queue.errors,1)
            self.assertIsNone(client.process)


if __name__=='__main__':unittest.main()
