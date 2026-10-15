import sys
from pathlib import Path
from types import SimpleNamespace
import tempfile
import hashlib
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from active_ort_session import configure_active_ort,verify_active_ort


class ActiveOrtTests(unittest.TestCase):
    def test_session_replacement_targets_executed_owner_and_verifies_options(self):
        class Options:
            def __init__(self):self.settings={}
            def add_session_config_entry(self,k,v):self.settings[k]=v
            def get_session_config_entry(self,k):return self.settings[k]
        class Session:
            def __init__(self,*args,sess_options=None,providers=None):self.options=sess_options;self.providers=providers
            def get_inputs(self):return [SimpleNamespace(shape=[1,3,256,384])]
            def get_providers(self):return self.providers
            def get_session_options(self):return self.options
            def run(self):return 'actual'
        ort=SimpleNamespace(SessionOptions=Options,InferenceSession=Session,ExecutionMode=SimpleNamespace(ORT_SEQUENTIAL='seq'))
        old=Session(providers=['CPUExecutionProvider']);shadow=object()
        owner=SimpleNamespace(session=old);outer=SimpleNamespace(backend=owner,session=shadow)
        with tempfile.TemporaryDirectory() as directory:
            model=Path(directory)/'model.onnx';model.write_bytes(b'model')
            adapter=SimpleNamespace(model=SimpleNamespace(predictor=SimpleNamespace(model=outer)),metadata={'model_sha256':hashlib.sha256(b'model').hexdigest()})
            with patch.dict(sys.modules,{'onnxruntime':ort}):
                metadata=configure_active_ort(adapter,model,2)
                self.assertIsNot(owner.session,old);self.assertIs(outer.session,shadow)
                self.assertEqual(metadata['intra_op_num_threads'],2)
                verify_active_ort(adapter,2)
                owner.session.options.intra_op_num_threads=0
                with self.assertRaises(ValueError):verify_active_ort(adapter,2)
                model.write_bytes(b'changed')
                with self.assertRaisesRegex(ValueError,'SHA'):configure_active_ort(adapter,model,1)


if __name__=='__main__':unittest.main()
