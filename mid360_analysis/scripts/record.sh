#!/usr/bin/env bash
set -euo pipefail
if [[ $# -gt 1 || ${1:-} == -h || ${1:-} == --help ]]; then
  echo "Usage: record.sh [output_directory]"
  echo "Default: bags/mid360_<timestamp>; run while bringup is active."
  if [[ $# -gt 1 ]]; then exit 2; fi
  exit 0
fi
command -v ros2 >/dev/null || { echo 'Source ROS2 and your workspace first.' >&2; exit 1; }
out=${1:-bags/mid360_$(date +%Y%m%d_%H%M%S)}
[[ ! -e "$out" ]] || { echo "Output already exists: $out" >&2; exit 2; }
mkdir -p "$(dirname "$out")"
share=$(ros2 pkg prefix --share mid360_analysis)
exec ros2 bag record -s sqlite3 -o "$out" --qos-profile-overrides-path "$share/config/bag_qos.yaml" \
  /livox/lidar /livox/imu /Odometry /path /cloud_registered /tf /tf_static
