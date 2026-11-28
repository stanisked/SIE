#!/usr/bin/env bash
# Use the reviewed commit on the laptop and the unchanged Pi baseline checkout.
set -euo pipefail
mode="${1:-}"
boot="${2:-}"
if [[ "$mode" == status && $# -eq 1 ]]; then
  :
elif [[ "$mode" != live || $# -ne 2 || ! "$boot" =~ ^[0-9A-F]{16}$ ]]; then
  echo "Usage: $0 status | live BOOT_ID" >&2
  exit 2
fi
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo="$(git -C "$here" rev-parse --show-toplevel)"
head="$(git -C "$repo" rev-parse HEAD)"
audit=tools/diagnostics/step75_5a_tf_audit
git -C "$repo" diff --exit-code "$head" -- "$audit" tools/diagnostics/step75_5a_slam.py \
  ros2/sie_ros2/config/diagnostics/slam_toolbox_async_v1.yaml
stage="$(mktemp -d)"
trap 'rm -rf -- "$stage"' EXIT
git -C "$repo" archive --format=tar.gz -o "$stage/bundle.tar.gz" "$head" -- \
  "$audit/CMakeLists.txt" "$audit/tf_receipts.cpp" "$audit/live-on-pi.sh" \
  tools/diagnostics/step75_5a_slam.py ros2/sie_ros2/config/diagnostics/slam_toolbox_async_v1.yaml
read -r digest _ < <(sha256sum "$stage/bundle.tar.gz")
host=elwis@100.70.100.29
remote_name=".sie_step75_5b_${mode}_$(date +%s)_$$.tar.gz"
scp -- "$stage/bundle.tar.gz" "$host:$remote_name"

remote_script='set -e -o pipefail
umask 077
archive="$HOME/$1"
printf "%s  %s\n" "$2" "$archive" | sha256sum --check --strict
root="$HOME/.local/state/sie/deployments"
mkdir -p "$root"
stage="$(mktemp -d "$root/step75_5b_$3.XXXXXX")"
mv -- "$archive" "$stage/bundle.tar.gz"
printf "%s\n" "$5" > "$stage/diagnostic-commit.txt"
tar --no-same-owner --no-same-permissions -xzf "$stage/bundle.tar.gz" -C "$stage"
printf "Step 75.5b staging directory: %s\n" "$stage"
exec bash "$stage/tools/diagnostics/step75_5a_tf_audit/live-on-pi.sh" "$3" "$4"'
ssh -t "$host" "bash -c '$remote_script' -- '$remote_name' '$digest' '$mode' '$boot' '$head'"
