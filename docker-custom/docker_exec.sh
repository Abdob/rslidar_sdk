#!/usr/bin/env bash
# Exec into the running docker-custom container with ROS 2 + the rslidar_sdk
# workspace + CUDA env already sourced (the image entrypoint handles sourcing).
#
# Usage:
#   ./docker_exec.sh                                          # interactive bash
#   ./docker_exec.sh bash /opt/custom/scripts/record_sync_bag.sh run1
#   ./docker_exec.sh ros2 topic list
set -e

CONTAINER="rslidar-airy-custom"

if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}\$"; then
    echo "Container '${CONTAINER}' is not running. Start it first with ./docker_run.sh" >&2
    exit 1
fi

if [ $# -eq 0 ]; then
    exec docker exec -it "$CONTAINER" /entrypoint.sh bash
else
    exec docker exec -it "$CONTAINER" /entrypoint.sh "$@"
fi
