#!/usr/bin/env bash
# Synthetic-only retry after copying this diagnostic into its own directory.
set -eo pipefail
umask 077
audit_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
bundle="$(cd -- "$audit_dir/../../.." && pwd)"
source_checkout=/home/elwis/dev_ws/sie_step75_5a_baseline
approved=2a00bd80348e8d10d4577e886f354d30e24057c1
evidence_root="$HOME/.local/state/sie/evidence"
positive=step75_5a_5t0dgdzt
negative=step75_5a_niqj29yf

test "$(git -C "$source_checkout" rev-parse HEAD)" = "$approved"
test -z "$(git -C "$source_checkout" status --porcelain --untracked-files=no)"
source /opt/ros/jazzy/setup.bash
test "$ROS_DISTRO" = jazzy
export PYTHONDONTWRITEBYTECODE=1
dpkg-query -W ros-jazzy-rclpy ros-jazzy-rclcpp ros-jazzy-slam-toolbox | tee "$bundle/packages.txt"
command -v cmake c++ >/dev/null

for trial in "$positive" "$negative"; do
  (cd "$evidence_root/$trial" && sha256sum --check --strict --status SHA256SUMS.txt)
done
python3 - "$evidence_root/$positive" "$evidence_root/$negative" <<'PY' | tee "$bundle/prior_evidence_readback.json"
import json
from pathlib import Path
import sys
import yaml

positive, negative = (Path(path) for path in sys.argv[1:])
p = json.loads((positive / "qualification.json").read_text())
n = json.loads((negative / "qualification.json").read_text())
assert p["result"] == "FAIL" and n["result"] == "PASS"
assert p["refusal_reasons"] == ["MAP_TF_PUBLISHER_GID_REQUIRED_FOR_MULTIPLE_ENDPOINTS"]
parameters = yaml.safe_load((positive / "effective_mapper.yaml").read_text())["slam_toolbox"]["ros__parameters"]
rows = [json.loads(line) for line in (positive / "scan_timing.jsonl").read_text().splitlines()]
first_scan = rows[0]["header_stamp_ns"]
predicted_first_map = first_scan + round(parameters["transform_timeout"] * 1e9)
print(json.dumps({
    "positive_original_result": p["result"],
    "negative_original_result": n["result"],
    "both_original_manifests_verified": True,
    "original_first_map_tf_stamp_recorded": False,
    "predicted_first_map_tf_stamp_ns_if_first_scan_drove_publication": predicted_first_map,
    "original_map_chain_misses": [
        {
            "scan": i,
            "scan_stamp_ns": row["header_stamp_ns"],
            "input_tf": row["input_tf"],
            "matched_slam_pose": row["matched_slam_pose"],
            "relative_to_predicted_first_map_tf_ns": row["header_stamp_ns"] - predicted_first_map,
        }
        for i, row in enumerate(rows, 1) if not row["map_tf"]
    ],
}, indent=2))
PY

cmake -S "$audit_dir" -B "$bundle/build" 2>&1 | tee "$bundle/tf-audit-configure.log"
cmake --build "$bundle/build" --parallel 2 2>&1 | tee "$bundle/tf-audit-build.log"

exit_code=0
python3 "$bundle/tools/diagnostics/step75_5a_slam.py" synthetic --domain 179 \
  --source "$source_checkout" --tf-audit-executable "$bundle/build/sie_tf_receipts" \
  2>&1 | tee "$bundle/positive-retry.log" || exit_code=1
python3 "$bundle/tools/diagnostics/step75_5a_slam.py" missing-tf --domain 180 \
  --source "$source_checkout" --tf-audit-executable "$bundle/build/sie_tf_receipts" \
  2>&1 | tee "$bundle/negative-retry.log" || exit_code=1
exit "$exit_code"
