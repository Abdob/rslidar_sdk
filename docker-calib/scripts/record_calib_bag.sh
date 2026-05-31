#!/usr/bin/env bash
# Record a camera+IMU bag for Kalibr temporal calibration.
#
# Kalibr's camera-IMU calibration only converges when the camera and IMU are
# already on roughly the same clock (it solves for a SUB-SECOND time_shift). So
# we record /image_raw_synced -- the COARSELY aligned image from
# image_restamp.py, NOT the raw /image_raw which is ~1.78e9 s away on the host
# clock. With cam_lidar_time_shift still 0.0 in time_sync.yaml, /image_raw_synced
# is the camera on the lidar clock minus only the unknown fine shift, which is
# exactly what Kalibr will recover.
#
# PREREQUISITE: the sensor stack must already be publishing. In separate
# terminals, BEFORE this script:
#   ./docker-gst-camera/docker_run.sh        # /image_raw
#   ./docker-calib/docker_run.sh sensors     # /rslidar_points + /rslidar_imu_data[_fixed]
# This script pre-flights those topics and aborts if any is silent (that was the
# cause of the earlier empty 0-message bag).
#
# How to record:
#   1. Print an AprilGrid (matching kalibr_aprilgrid.yaml) and fix it to a wall.
#   2. Run this, then move the rig in front of the grid exciting all 6 axes
#      (3 rotations, 3 translations), staying in frame, for ~60-90 s.
#   3. Ctrl-C to stop.
#
# Usage (typically via docker_exec.sh from the host):
#   ./docker-calib/docker_exec.sh bash /opt/calib/scripts/record_calib_bag.sh kalibr_run1
#   -> /opt/calib/bags/kalibr_run1/   (feed to run_kalibr.sh)
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/_record_common.sh"

FILE_NAME="${1:?Usage: record_calib_bag.sh <bag_name>}"

echo "Pre-flight: confirming sensor topics are publishing..."
if ! preflight_topics 10 /image_raw /rslidar_imu_data /rslidar_imu_data_fixed; then
    echo "ABORT: not all inputs are live. Start them first:" >&2
    echo "  /image_raw                -> ./docker-gst-camera/docker_run.sh   (camera)" >&2
    echo "  /rslidar_imu_data[_fixed] -> ./docker-calib/docker_run.sh sensors (lidar + imu_bridge)" >&2
    exit 1
fi

echo "Starting image_restamp (coarse alignment, fine shift from time_sync.yaml)..."
python3 "$SCRIPT_DIR/image_restamp.py" &
RESTAMP_PID=$!
trap 'kill "$RESTAMP_PID" 2>/dev/null' EXIT

echo "Waiting for /image_raw_synced to start flowing..."
if ! wait_for_topic /image_raw_synced 15; then
    echo "ABORT: image_restamp produced no /image_raw_synced." >&2
    echo "  It needs /rslidar_imu_data (clock reference) AND /image_raw flowing." >&2
    exit 1
fi

echo "Recording Kalibr calib bag '${FILE_NAME}' -- wave the rig at the AprilGrid. Ctrl-C to stop."
set +e
ros2 bag record -o /opt/calib/bags/${FILE_NAME} \
    /image_raw_synced \
    /rslidar_imu_data_fixed
set -e

assert_bag_nonempty /opt/calib/bags/${FILE_NAME} || exit 1
