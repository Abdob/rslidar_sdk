#!/usr/bin/env bash
# Record the three FAST-LIVO2 input topics to a ros2 bag, all on the AIRY
# lidar hardware clock (lidar_config.yaml: use_lidar_clock: true).
#
# Clocks:
#   /rslidar_points         lidar hardware clock        (rslidar_sdk)
#   /rslidar_imu_data_fixed lidar hardware clock, m/s^2 (imu_bridge.py)
#   /image_raw_synced       lidar hardware clock        (image_restamp.py)
#
# We do NOT record the raw /image_raw (host wall clock) -- image_restamp.py
# maps it onto the lidar clock by removing the live coarse host<->lidar offset
# and adding the fine Kalibr shift from config/time_sync.yaml. See that node
# and run_kalibr.sh. Recording the raw /rslidar_imu_data is also skipped; the
# _fixed stream is what FAST-LIVO2 consumes.
#
# PREREQUISITE: the sensor stack must already be publishing, BEFORE this script:
#   ./docker-gst-camera/docker_run.sh        # /image_raw
#   ./docker-calib/docker_run.sh sensors     # /rslidar_points + /rslidar_imu_data[_fixed]
# This script pre-flights those topics and aborts if any is silent.
#
# Usage (typically via docker_exec.sh from the host):
#   ./docker-calib/docker_exec.sh bash /opt/calib/scripts/ros_bag_record.sh ezoffice
#   -> /opt/calib/bags/ezoffice/
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/_record_common.sh"

FILE_NAME="${1:?Usage: ros_bag_record.sh <bag_name>}"

echo "Pre-flight: confirming sensor topics are publishing..."
if ! preflight_topics 10 /image_raw /rslidar_points /rslidar_imu_data /rslidar_imu_data_fixed; then
    echo "ABORT: not all inputs are live. Start them first:" >&2
    echo "  /image_raw                       -> ./docker-gst-camera/docker_run.sh   (camera)" >&2
    echo "  /rslidar_points, /rslidar_imu_data[_fixed] -> ./docker-calib/docker_run.sh sensors" >&2
    exit 1
fi

echo "Starting image_restamp (/image_raw -> /image_raw_synced)..."
python3 "$SCRIPT_DIR/image_restamp.py" &
RESTAMP_PID=$!
trap 'kill "$RESTAMP_PID" 2>/dev/null' EXIT

echo "Waiting for /image_raw_synced to start flowing..."
if ! wait_for_topic /image_raw_synced 15; then
    echo "ABORT: image_restamp produced no /image_raw_synced." >&2
    echo "  It needs /rslidar_imu_data (clock reference) AND /image_raw flowing." >&2
    exit 1
fi

echo "Recording ROS bag named ${FILE_NAME}... Ctrl-C to stop."
set +e
ros2 bag record -o /opt/calib/bags/${FILE_NAME} \
    /rslidar_points \
    /rslidar_imu_data_fixed \
    /image_raw_synced
set -e

assert_bag_nonempty /opt/calib/bags/${FILE_NAME} || exit 1
