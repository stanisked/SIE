#!/usr/bin/env python3
"""Subject-source-labelled local AR0234 RGB dataset capture. No detector or model is run."""
from __future__ import annotations
import argparse
import subprocess
import sys
from pathlib import Path

from vision_core.person_dataset.capture import AR0234_BY_ID, DEFAULT_DATASET_ROOT, DatasetCaptureError, capture_runtime, finalize_dataset, write_default_plan

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--write-default-plan", type=Path, metavar="PATH")
    modes.add_argument("--finalize", action="store_true")
    modes.add_argument("--capture", action="store_true")
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--device", type=Path)
    parser.add_argument("--subject-source", choices=("SELF_CAPTURE", "CONSENTED_VOLUNTEER", "NO_PERSON"), help="dataset-level subject source; required for human capture")
    return parser.parse_args()

def main() -> int:
    args = parse_args()
    if args.write_default_plan is not None:
        write_default_plan(args.write_default_plan); print(f"WROTE_DEFAULT_PLAN {args.write_default_plan}"); return 0
    if args.finalize:
        print(finalize_dataset(args.dataset_root)["status"]); return 0
    if args.device != AR0234_BY_ID: raise DatasetCaptureError("--capture requires exact --device /dev/v4l/by-id/...-video-index0")
    commit = subprocess.run(["git", "rev-parse", "HEAD"], text=True, stdout=subprocess.PIPE, check=True).stdout.strip()
    print("Capture only yourself or a volunteer who agreed to participate.\nKeep this dataset local and outside Git.", file=sys.stderr)
    capture_runtime(args.dataset_root, device=args.device, subject_source=args.subject_source, git_commit=commit); return 0

if __name__ == "__main__":
    try: raise SystemExit(main())
    except (KeyboardInterrupt, SystemExit): raise
    except (DatasetCaptureError, OSError, subprocess.SubprocessError) as error:
        print(f"ERROR: {error}", file=sys.stderr); raise SystemExit(2) from error
