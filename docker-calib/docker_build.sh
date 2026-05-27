#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
docker build -f docker-calib/Dockerfile -t rslidar-airy-calib .
