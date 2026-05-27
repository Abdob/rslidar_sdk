#!/usr/bin/env bash
# Run the Ds3D AIRY render pipeline.
# Needs:
#   --gpus all                  CUDA / NVENC / GL
#   --net=host                  receive UDP from the LiDAR
#   X11 forwarding              nveglglessink renders to a host window
set -e

xhost +local:docker >/dev/null 2>&1 || true

exec docker run --rm -it \
    --gpus all \
    --net=host \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}" \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --device /dev/dri \
    --name rslidar-airy-ds3d \
    rslidar-airy-ds3d "$@"
