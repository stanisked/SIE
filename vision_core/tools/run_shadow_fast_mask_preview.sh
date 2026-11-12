#!/usr/bin/env bash
# Motor-off preview with the reviewed COCO instance-mask model as RGB semantics.
set -eo pipefail
export SHADOW_FAST_SEMANTIC_SOURCE=segmentation_mask
exec bash "$(dirname "${BASH_SOURCE[0]}")/run_shadow_fast_preview.sh" "$@"
