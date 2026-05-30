#!/usr/bin/env bash
# Exec into the running docker-livo2 container with ROS 1 Noetic + the
# FAST-LIVO2 catkin workspace already sourced. The container must already
# be running (./docker_run.sh avia, rsairy, etc.) in another terminal.
#
# Usage:
#   ./docker_exec.sh                                  # interactive bash
#   ./docker_exec.sh rostopic hz /livox/lidar         # one-shot command
#   ./docker_exec.sh rosbag play /data/foo.bag --clock
set -e

CONTAINER="rslidar-airy-livo2"
SETUP='source /opt/ros/noetic/setup.bash; [ -f /opt/catkin_ws/devel/setup.bash ] && source /opt/catkin_ws/devel/setup.bash'

if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}\$"; then
    echo "Container '${CONTAINER}' is not running. Start it first with ./docker_run.sh <mode>" >&2
    exit 1
fi

if [ $# -eq 0 ]; then
    exec docker exec -it "$CONTAINER" bash -c "$SETUP; exec bash"
else
    exec docker exec -it "$CONTAINER" bash -c "$SETUP; exec \"\$@\"" _ "$@"
fi
