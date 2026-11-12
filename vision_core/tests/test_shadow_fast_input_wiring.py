"""Check the Shadow capture path requests fingerprints in both semantic modes."""
import ast
from pathlib import Path
import time
from types import SimpleNamespace
import unittest


class ShadowFastInputWiringTests(unittest.TestCase):
    def test_mask_and_v3_modes_request_current_instance_fingerprints(self):
        path = Path(__file__).resolve().parents[2] / (
            "ros2/sie_ros2/sie_ros2/ar0234_ov9281_control_preview_node.py")
        tree = ast.parse(path.read_text())
        node = next(item for item in tree.body if isinstance(item, ast.ClassDef)
                    and item.name == "Ar0234Ov9281ControlPreviewNode")
        method = next(item for item in node.body if isinstance(item, ast.FunctionDef)
                      and item.name == "_infer_current_segmentation")
        namespace = {"time": time}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[])),
                     str(path), "exec"), namespace)
        infer = namespace[method.name]
        for active in (False, True):
            with self.subTest(shadow_fast_input=active):
                calls = []
                worker = SimpleNamespace(infer=lambda *args, **kwargs:
                    calls.append((args, kwargs)) or {"status": "ok"})
                instance = SimpleNamespace(_native_rgb_jpeg=False,
                                           _shadow_fast_input=active, _mask_client=worker)
                self.assertEqual(infer(instance, "cycle", object(), {}), {"status": "ok"})
                self.assertEqual(len(calls), 1)
                self.assertIs(calls[0][1]["diagnostic_mask_hashes"], active)


if __name__ == "__main__":
    unittest.main()
