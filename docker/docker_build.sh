#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
docker build -f docker/Dockerfile -t rslidar-airy-demo .
