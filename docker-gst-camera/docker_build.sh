#!/usr/bin/env bash
# Build the GPU-accelerated MJPEG camera publisher container.
# Tags: rslidar-airy-gst-camera
set -e

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

docker build \
    -t rslidar-airy-gst-camera \
    -f "$REPO_ROOT/docker-gst-camera/Dockerfile" \
    "$REPO_ROOT"
