#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
docker build -f docker-ros2/Dockerfile -t rslidar-airy-ros2 .
