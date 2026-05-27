#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
docker build -f docker-ds3d/Dockerfile -t rslidar-airy-ds3d .
