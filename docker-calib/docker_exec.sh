#!/usr/bin/env bash
# Exec into the running docker-calib container with ROS 2 Humble + the
# rslidar_sdk workspace already sourced. The container must already be
# running (./docker_run.sh sensors, intrinsic, etc.) in another terminal.
#
# Usage:
#   ./docker_exec.sh                              # interactive bash
#   ./docker_exec.sh ros2 topic hz /image_raw     # one-shot command
#   ./docker_exec.sh bash /opt/calib/scripts/record_bag.sh office
set -e

CONTAINER="rslidar-airy-calib"
SETUP='source /opt/ros/humble/setup.bash; [ -f /opt/ros_ws/install/setup.bash ] && source /opt/ros_ws/install/setup.bash'

if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}\$"; then
    echo "Container '${CONTAINER}' is not running. Start it first with ./docker_run.sh <mode>" >&2
    exit 1
fi

if [ $# -eq 0 ]; then
    exec docker exec -it "$CONTAINER" bash -c "$SETUP; exec bash"
else
    exec docker exec -it "$CONTAINER" bash -c "$SETUP; exec \"\$@\"" _ "$@"
fi
