#!/usr/bin/env bash
# Record a timing-sync bag: camera + LiDAR + IMU while you translate the rig
# back-and-forth in one dimension in front of a checkerboard on a wall.
#
# PREREQUISITE: the sensor stack must already be publishing. In separate
# terminals, BEFORE this script:
#   ./docker-gst-camera/docker_run.sh        # /image_raw[/compressed], /camera_info
#   ./docker-calib/docker_run.sh sensors     # /rslidar_points + /rslidar_imu_data[_fixed]
#
# How to record:
#   1. Put a checkerboard on a wall; aim the rig at it.
#   2. Run this, then smoothly translate the rig toward/away (or left/right)
#      several times -- distinct reversals are what we align -- for ~30-60 s.
#   3. Ctrl-C to stop.
#
# Usage (via docker_run.sh / docker_exec.sh):
#   ./docker-custom/docker_run.sh record run1
#   -> /opt/custom/bags/run1/   (then: extract run1, plot run1)
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/_record_common.sh"

FILE_NAME="${1:?Usage: record_sync_bag.sh <bag_name>}"
BAG_DIR="/opt/custom/bags/${FILE_NAME}"

# Topic names come from config/sync.yaml so this stays in sync with extraction.
CFG=/opt/custom/config/sync.yaml
read_cfg() { grep -E "^\s*$1:" "$CFG" | head -1 | sed -E "s/^[^:]*:\s*//; s/\s*(#.*)?$//"; }
IMAGE_TOPIC="$(read_cfg image)";        IMAGE_TOPIC="${IMAGE_TOPIC:-/image_raw/compressed}"
CAMINFO_TOPIC="$(read_cfg camera_info)"; CAMINFO_TOPIC="${CAMINFO_TOPIC:-/camera_info}"
CLOUD_TOPIC="$(read_cfg cloud)";        CLOUD_TOPIC="${CLOUD_TOPIC:-/rslidar_points}"
IMU_TOPIC="$(read_cfg imu)";            IMU_TOPIC="${IMU_TOPIC:-/rslidar_imu_data_fixed}"

echo "Pre-flight: confirming sensor topics are publishing..."
if ! preflight_topics 10 "$IMAGE_TOPIC" "$CLOUD_TOPIC" "$IMU_TOPIC"; then
    echo "ABORT: not all inputs are live. Start them first:" >&2
    echo "  $IMAGE_TOPIC -> ./docker-gst-camera/docker_run.sh   (camera)" >&2
    echo "  $CLOUD_TOPIC / $IMU_TOPIC -> ./docker-calib/docker_run.sh sensors" >&2
    exit 1
fi

echo "Recording '${FILE_NAME}' -- translate the rig back-and-forth. Ctrl-C to stop."
set +e
ros2 bag record -o "$BAG_DIR" \
    "$IMAGE_TOPIC" \
    "$CAMINFO_TOPIC" \
    "$CLOUD_TOPIC" \
    "$IMU_TOPIC"
set -e

assert_bag_nonempty "$BAG_DIR" || exit 1
