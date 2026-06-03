#!/usr/bin/env bash
# Build the timing-sync experiment image (tag: rslidar-airy-custom).
#
# This image is FROM rslidar-airy-calib, so that image must exist first.
set -e

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

if ! docker image inspect rslidar-airy-calib >/dev/null 2>&1; then
    echo "ERROR: base image 'rslidar-airy-calib' not found." >&2
    echo "       Build it first:  ./docker-calib/docker_build.sh" >&2
    exit 1
fi

docker build \
    -t rslidar-airy-custom \
    -f "$REPO_ROOT/docker-custom/Dockerfile" \
    "$REPO_ROOT"
