"""Offline-only AR0234 person ROI to Stereo V6 depth fusion."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path

import cv2

from vision_core.person_depth_fusion.offline import (
    AR_INTRINSIC, DEFAULT_POLICY_PATH, EXTRINSIC_CANDIDATE_PATH,
    EXTRINSIC_VALIDATION_PATH, STEREO_CALIBRATION, PersonDepthFusionError,
    PersonDepthFusionOffline, build_offline_stereo_kernel, load_fusion_calibration,
    write_offline_report,
)
from vision_core.person_localization.mp_persondet import MPPersonDetOpenCV
from vision_core.person_localization.pipeline import PersonLocalizationPipeline


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ar-image", type=Path, required=True)
    parser.add_argument("--stereo-combined-image", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True, help="absolute external MP-PersonDet ONNX path")
    parser.add_argument("--reference", type=Path, required=True, help="absolute pinned MP-PersonDet reference adapter path")
    parser.add_argument("--ar-intrinsic", type=Path, default=AR_INTRINSIC)
    parser.add_argument("--stereo-calibration", type=Path, default=STEREO_CALIBRATION)
    parser.add_argument("--stereo-policy", type=Path, default=DEFAULT_POLICY_PATH)
    parser.add_argument("--candidate-extrinsic", type=Path, default=EXTRINSIC_CANDIDATE_PATH)
    parser.add_argument("--physical-validation", type=Path, default=EXTRINSIC_VALIDATION_PATH)
    parser.add_argument("--project-root", type=Path, default=Path.cwd())
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    try:
        ar = cv2.imread(str(args.ar_image), cv2.IMREAD_COLOR)
        combined = cv2.imread(str(args.stereo_combined_image), cv2.IMREAD_COLOR)
        if ar is None or combined is None:
            raise PersonDepthFusionError("input PNG decode failed")
        calibration = load_fusion_calibration(
            candidate_path=args.candidate_extrinsic,
            validation_path=args.physical_validation,
            ar_intrinsic_path=args.ar_intrinsic,
            stereo_calibration_path=args.stereo_calibration,
            stereo_policy_path=args.stereo_policy,
        )
        detector = MPPersonDetOpenCV(args.model, args.reference)
        fusion = PersonDepthFusionOffline(
            PersonLocalizationPipeline(detector),
            build_offline_stereo_kernel(calibration, project_root=args.project_root),
            calibration,
        )
        measurement = fusion.process(
            ar, combined, captured_at_utc=datetime.now(timezone.utc), cycle_id="offline-png"
        )
        write_offline_report(
            args.report, measurement, calibration,
            ar_image_path=args.ar_image, combined_image_path=args.stereo_combined_image,
        )
    except (PersonDepthFusionError, ValueError) as error:
        parser.error(str(error))
    print(measurement.status.value)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
