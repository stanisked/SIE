#!/usr/bin/env bash
# Use the reviewed sensor-only launcher and a separate diagnostic observer.
set -eo pipefail
if [[ $# -eq 1 && "$1" == --help ]]; then
  echo 'Usage: run_live_entity_centric_v1.sh baseline|enabled'
  echo 'Runs the existing 30-second sensor-only preview with an independent observer.'
  exit 0
fi
if [[ $# -ne 1 || ( "$1" != baseline && "$1" != enabled ) ]]; then
  echo 'Choose baseline or enabled. No passthrough arguments are accepted.' >&2
  exit 2
fi
mode=$1
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/ros2/sie_ros2:$PWD/vision_core/tools:$PWD${PYTHONPATH:+:$PYTHONPATH}"
export OPENBLAS_NUM_THREADS=1
unset SIE_STEP71_SOURCE_EVIDENCE SIE_SHADOW_SOURCE_SNAPSHOTS
export SIE_TEMPORAL_CAMERA_SELECTION=latest_pair SIE_TEMPORAL_DURATION_S=30
umask 077

runtime=/home/elwis/dev_ws/runtime_envs/sie_mvp/bin/python
evidence_root="${XDG_STATE_HOME:-$HOME/.local/state}/sie/evidence"
mkdir -p "$evidence_root"
evidence=$(mktemp -d "$evidence_root/live_entity_${mode}_XXXXXXXX")
echo "Step 74 evidence: $evidence"

"$runtime" -m sie_ros2.live_entity_centric_node --mode "$mode" --output "$evidence/preview" \
  >"$evidence/observer.log" 2>&1 &
observer=$!
stop_observer() {
  if [[ -n "$observer" ]]; then
    kill -TERM "$observer" 2>/dev/null || true
    wait "$observer"
    observer=
  fi
}
trap 'stop_observer' EXIT
for ((attempt=0; attempt<100; attempt++)); do
  [[ -f "$evidence/preview/ready" ]] && break
  kill -0 "$observer" 2>/dev/null || break
  sleep .1
done
[[ -f "$evidence/preview/ready" ]] || { echo 'Step 74 observer did not become ready.' >&2; exit 2; }

bash vision_core/tools/run_live_temporal_shadow_v1.sh | tee "$evidence/camera-launch.log"
stop_observer
trap - EXIT
(
  cd "$evidence"
  camera_evidence=$(sed -n 's/^Evidence: //p' camera-launch.log)
  journal=("$camera_evidence"/control-preview-*/temporal-association-shadow.jsonl)
  sha256sum -- "${journal[@]}" > camera-temporal.sha256
  sha256sum -- preview/summary.json preview/camera/events.jsonl preview/ld19/events.jsonl \
    camera-launch.log camera-temporal.sha256 > SHA256SUMS.txt
  "$runtime" -m json.tool preview/summary.json
  cat SHA256SUMS.txt
)
