#!/usr/bin/env python3

"""Validate an AR0234 detector manifest without starting a backend.

No detector is bundled with this MVP. This command validates manifest syntax
and then refuses to open the camera until a separately approved backend adapter
is supplied. It never starts stereo, emits a Measurement, or sends a network
command to the mobile base.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from vision_core.person_localization import AR0234_BY_ID, DetectorArtifact


_ARTIFACT_MANIFEST_KEYS = frozenset(
    {
        "schema_version",
        "backend_id",
        "backend_version",
        "model_id",
        "model_version",
        "artifact_sha256",
        "output_contract_version",
        "person_label",
        "confidence_semantics",
        "confidence_threshold",
        "inference_parameters",
    }
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", type=Path, default=AR0234_BY_ID)
    parser.add_argument(
        "--detector-artifact",
        type=Path,
        required=True,
        help="JSON manifest for a separately reviewed detector artifact",
    )
    return parser.parse_args()


def _load_artifact(path: Path) -> DetectorArtifact:
    with path.open(encoding="utf-8") as handle:
        payload = json.load(handle)
    if not isinstance(payload, dict):
        raise ValueError("detector artifact manifest must be a JSON object")
    payload_keys = frozenset(payload)
    if payload_keys != _ARTIFACT_MANIFEST_KEYS:
        missing = sorted(_ARTIFACT_MANIFEST_KEYS - payload_keys)
        unknown = sorted(payload_keys - _ARTIFACT_MANIFEST_KEYS)
        raise ValueError(f"detector artifact manifest keys mismatch: missing={missing}, unknown={unknown}")
    return DetectorArtifact(
        schema_version=payload["schema_version"],
        backend_id=payload["backend_id"],
        backend_version=payload["backend_version"],
        model_id=payload["model_id"],
        model_version=payload["model_version"],
        artifact_sha256=payload["artifact_sha256"],
        output_contract_version=payload["output_contract_version"],
        person_label=payload["person_label"],
        confidence_semantics=payload["confidence_semantics"],
        confidence_threshold=payload["confidence_threshold"],
        inference_parameters=payload["inference_parameters"],
    )


def main() -> int:
    args = parse_args()
    if args.device != AR0234_BY_ID:
        raise ValueError("only the approved AR0234 /dev/v4l/by-id/...-video-index0 path is allowed")
    artifact = _load_artifact(args.detector_artifact)
    print(
        json.dumps(
            {
                "status": "MANIFEST_SYNTACTICALLY_VALID_BACKEND_ADAPTER_NOT_CONFIGURED",
                "device": str(AR0234_BY_ID),
                "detector": artifact.metadata(),
            },
            sort_keys=True,
        ),
        flush=True,
    )
    return 3


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(2) from error
