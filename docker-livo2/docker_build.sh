#!/usr/bin/env bash
# Build the FAST-LIVO2 container.
# Tags: rslidar-airy-livo2
set -e

HOST_DIR="$(cd "$(dirname "$0")" && pwd)"

docker build -t rslidar-airy-livo2 -f "$HOST_DIR/Dockerfile" "$HOST_DIR"
