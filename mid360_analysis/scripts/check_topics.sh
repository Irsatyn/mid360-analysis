#!/usr/bin/env bash
# Every potentially blocking ROS command has a deadline; errors are preserved as evidence.
set -euo pipefail
if [[ $# -gt 1 || ${1:-} == -h || ${1:-} == --help ]]; then
  echo 'Usage: check_topics.sh [evidence_directory] (default: docs/evidence)'
  if [[ $# -gt 1 ]]; then exit 2; fi
  exit 0
fi
command -v ros2 >/dev/null || { echo 'Source ROS2 and your workspace first.' >&2; exit 1; }
out_dir=${1:-docs/evidence}
mkdir -p "$out_dir"
out_dir=$(cd "$out_dir" && pwd)
run_dir=$(mktemp -d "$out_dir/frames_$(date +%Y%m%d_%H%M%S)_XXXXXX")
log="$out_dir/topics_$(date +%Y%m%d_%H%M%S).txt"
exec > >(tee "$log") 2>&1
run_check() {
  printf '\n$'; printf ' %q' "$@"; printf '\n'
  local status=0
  "$@" || status=$?
  printf '[exit=%s]\n' "$status"
}
run_check timeout --signal=INT --kill-after=2s 10s ros2 topic list -t
for topic in /livox/lidar /livox/imu /Odometry; do
  run_check timeout --signal=INT --kill-after=2s 10s ros2 topic info -v "$topic"
  run_check timeout --signal=INT --kill-after=2s 5s ros2 topic hz "$topic"
done
for topic in /livox/imu /Odometry /analysis/imu/stats /analysis/odom/tf_error; do
  run_check timeout --signal=INT --kill-after=2s 5s ros2 topic echo --once "$topic"
done
run_check timeout --signal=INT --kill-after=2s 3s ros2 run tf2_ros tf2_echo camera_init body
(
  cd "$run_dir"
  run_check timeout --signal=INT --kill-after=2s 15s ros2 run tf2_tools view_frames
)
echo "Evidence saved: $log"
echo "TF artifacts: $run_dir"
