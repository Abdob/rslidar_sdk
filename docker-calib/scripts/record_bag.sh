#!/usr/bin/env bash
# Record /rslidar_points, /rslidar_imu_data, /image_raw to a rosbag2.
#
# Usage (from inside the docker-calib container; see sensors launch first):
#   ./record_bag.sh                       # writes /data/bags/<timestamp>/
#   ./record_bag.sh my_scene              # writes /data/bags/my_scene/
#
# To process with FAST-LIVO2 later: convert with `rosbags-convert` to a
# rosbag1, drop it in docker-livo2/bags/, run.
set -e

NAME="${1:-$(date +%Y%m%d_%H%M%S)}"
OUT="/opt/calib/bags/$NAME"
mkdir -p "$(dirname "$OUT")"

echo "Recording to $OUT  (Ctrl-C to stop)"
exec ros2 bag record \
    -o "$OUT" \
    /rslidar_points \
    /rslidar_imu_data \
    /image_raw
