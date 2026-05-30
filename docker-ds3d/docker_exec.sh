#!/usr/bin/env bash
# Exec into the running docker-ds3d container. This image is DeepStream 9.0
# (no ROS), so nothing extra is sourced — just opens a bash inside the
# container.
#
# Usage:
#   ./docker_exec.sh                              # interactive bash
#   ./docker_exec.sh gst-inspect-1.0 nvds3dfilter # one-shot command
set -e

CONTAINER="rslidar-airy-ds3d"

if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}\$"; then
    echo "Container '${CONTAINER}' is not running. Start it first with ./docker_run.sh" >&2
    exit 1
fi

if [ $# -eq 0 ]; then
    exec docker exec -it "$CONTAINER" bash
else
    exec docker exec -it "$CONTAINER" "$@"
fi
