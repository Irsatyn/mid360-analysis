#!/usr/bin/env bash
set -euo pipefail
if [[ ${1:-} == -h || ${1:-} == --help ]]; then
  echo 'Usage: play.sh <bag_directory> [rate]'; exit 0
fi
[[ $# -ge 1 && $# -le 2 ]] || { echo 'Usage: play.sh <bag_directory> [rate]' >&2; exit 2; }
[[ -e "$1" ]] || { echo "Bag does not exist: $1" >&2; exit 2; }
rate=${2:-1.0}
python3 - "$rate" <<'PY'
import math
import sys
try:
    rate = float(sys.argv[1])
    if not math.isfinite(rate) or rate <= 0:
        raise ValueError()
except ValueError:
    sys.exit('rate must be a finite positive number')
PY
command -v ros2 >/dev/null || { echo 'Source ROS2 and your workspace first.' >&2; exit 1; }
share=$(ros2 pkg prefix --share mid360_analysis)
exec ros2 bag play "$1" --clock --rate "$rate" --delay 2.0 \
  --qos-profile-overrides-path "$share/config/bag_qos.yaml" \
  --topics /livox/lidar /livox/imu /Odometry /path /cloud_registered /cloud_registered_body /Laser_map /tf /tf_static
