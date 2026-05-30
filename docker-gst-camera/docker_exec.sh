#!/usr/bin/env bash
# Exec into the running docker-gst-camera container with ROS 2 Jazzy + the
# camera workspace already sourced.
#
# Usage:
#   ./docker_exec.sh                              # interactive bash
#   ./docker_exec.sh ros2 topic hz /image_raw     # one-shot command
set -e

CONTAINER="rslidar-airy-gst-camera"
SETUP='source /opt/ros/jazzy/setup.bash; [ -f /opt/ros_ws/install/setup.bash ] && source /opt/ros_ws/install/setup.bash'

if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}\$"; then
    echo "Container '${CONTAINER}' is not running. Start it first with ./docker_run.sh" >&2
    exit 1
fi

if [ $# -eq 0 ]; then
    exec docker exec -it "$CONTAINER" bash -c "$SETUP; exec bash"
else
    exec docker exec -it "$CONTAINER" bash -c "$SETUP; exec \"\$@\"" _ "$@"
fi
