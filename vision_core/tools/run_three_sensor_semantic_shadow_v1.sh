#!/usr/bin/env bash
# Capture only the existing isolated sensor diagnostics for Step 71 offline replay.
set -euo pipefail
if [[ $# -ne 0 ]]; then
  echo 'This launcher accepts no passthrough arguments.' >&2
  exit 2
fi
if [[ ! "${SIE_TEMPORAL_LD19_CLOCK_EVIDENCE:-}" =~ ^sha256:[0-9a-f]{64}$ ]]; then
  echo 'Set SIE_TEMPORAL_LD19_CLOCK_EVIDENCE to current same-host LD19 clock evidence.' >&2
  exit 2
fi
export SIE_STEP71_SOURCE_EVIDENCE=1
export SIE_TEMPORAL_CAMERA_SELECTION=latest_pair
export SIE_TEMPORAL_DURATION_S=30
exec bash "$(dirname "${BASH_SOURCE[0]}")/run_live_temporal_shadow_v1.sh"
