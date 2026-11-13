from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_segmentation_mask_source_disables_unused_v3_detector() -> None:
    launcher = (ROOT / "vision_core/tools/run_shadow_fast_preview.sh").read_text()

    segmentation_case = launcher.split("segmentation_mask)", 1)[1].split(";;", 1)[0]
    assert "ar_detector_enabled=false" in segmentation_case
    assert "startup_model_path=''" in segmentation_case
    assert "startup_model_version=disabled" in segmentation_case
    assert 'if [[ "$semantic_source" = v3_mask_binding ]]' in launcher
    assert '-p ar_person_detector_enabled:="$ar_detector_enabled"' in launcher


def test_v3_binding_keeps_exact_artifact_gate() -> None:
    launcher = (ROOT / "vision_core/tools/run_shadow_fast_preview.sh").read_text()

    v3_case = launcher.split("v3_mask_binding)", 1)[1].split(";;", 1)[0]
    assert "ar_detector_enabled=true" in v3_case
    assert 'startup_model_path="$v3"' in v3_case
    assert "startup_model_version=v3" in v3_case
    assert "fccd45fbcf490134eb786ab3b38cf115dd03466edd85801ad44c8353f38549f5" in launcher
