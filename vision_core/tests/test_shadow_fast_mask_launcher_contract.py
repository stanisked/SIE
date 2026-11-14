from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_segmentation_mask_source_disables_unused_v3_detector() -> None:
    launcher = (ROOT / "vision_core/tools/run_shadow_fast_preview.sh").read_text()

    segmentation_case = launcher.split("segmentation_mask)", 1)[1].split(";;", 1)[0]
    assert "ar_detector_enabled=false" in segmentation_case
    assert "detector_ros_args" not in segmentation_case
    assert 'detector_ros_args=()' in launcher
    assert '"${detector_ros_args[@]}"' in launcher
    assert 'model_path:="$startup_model_path"' not in launcher
    assert 'if [[ "$semantic_source" = v3_mask_binding ]]' in launcher
    assert '-p ar_person_detector_enabled:="$ar_detector_enabled"' in launcher


def test_v3_binding_keeps_exact_artifact_gate() -> None:
    launcher = (ROOT / "vision_core/tools/run_shadow_fast_preview.sh").read_text()

    v3_case = launcher.split("v3_mask_binding)", 1)[1].split(";;", 1)[0]
    assert "ar_detector_enabled=true" in v3_case
    assert 'detector_ros_args=(-p "model_path:=$v3" -p \'model_version:=v3\')' in v3_case
    assert "fccd45fbcf490134eb786ab3b38cf115dd03466edd85801ad44c8353f38549f5" in launcher
