"""Solve an offline AR0234-to-raw-physical-left candidate extrinsic."""
from __future__ import annotations

import argparse
from pathlib import Path

from vision_core.rgb_stereo_extrinsic.solve import (
    AR_INTRINSIC,
    ARTIFACT_PATH,
    DATASET_ROOT,
    STEREO_CALIBRATION,
    ExtrinsicSolveError,
    solve_candidate,
    VALIDATION_ARTIFACT_PATH,
    VALIDATION_DATASET_ROOT,
    validate_candidate,
    write_candidate_artifact,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, default=DATASET_ROOT)
    parser.add_argument("--ar-intrinsic", type=Path, default=AR_INTRINSIC)
    parser.add_argument("--stereo-calibration", type=Path, default=STEREO_CALIBRATION)
    parser.add_argument("--artifact", type=Path, default=ARTIFACT_PATH)
    parser.add_argument("--validate-candidate", action="store_true", help="validate the immutable candidate against the independent three-pair dataset")
    parser.add_argument("--validation-dataset-root", type=Path, default=VALIDATION_DATASET_ROOT)
    parser.add_argument("--validation-artifact", type=Path, default=VALIDATION_ARTIFACT_PATH)
    arguments = parser.parse_args()
    try:
        if arguments.validate_candidate:
            artifact = validate_candidate(arguments.validation_dataset_root, arguments.artifact)
            write_candidate_artifact(artifact, arguments.validation_artifact)
        else:
            artifact = solve_candidate(arguments.dataset_root, arguments.ar_intrinsic, arguments.stereo_calibration)
            write_candidate_artifact(artifact, arguments.artifact)
    except ExtrinsicSolveError as error:
        parser.error(str(error))
    print(f"{artifact['status']} {arguments.validation_artifact if arguments.validate_candidate else arguments.artifact}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
