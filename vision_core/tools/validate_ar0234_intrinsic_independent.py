#!/usr/bin/env python3
"""Validate immutable AR0234 checkerboard evidence against an existing intrinsic."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from vision_core.ar0234_intrinsic_independent_validation import (
    IntrinsicValidationError,
    validate_independent_dataset,
    write_validation_outputs,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-manifest", type=Path, required=True)
    parser.add_argument("--intrinsic", type=Path, required=True)
    arguments = parser.parse_args()
    try:
        report = validate_independent_dataset(dataset_manifest=arguments.dataset_manifest, intrinsic=arguments.intrinsic)
        write_validation_outputs(output_root=arguments.dataset_manifest.parent, report=report)
    except IntrinsicValidationError as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
