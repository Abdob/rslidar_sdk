#!/usr/bin/env bash
# Launch the rslidar_sdk ROS 2 node + RViz2 against the AIRY at 192.168.1.200.
#   --net=host   : receive UDP from the LiDAR on host ports 6699/7788
#   X11 forward  : RViz2 opens on your desktop
set -e

xhost +local:docker >/dev/null 2>&1 || true

exec docker run --rm -it \
    --net=host \
    --privileged \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}" \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --device /dev/dri \
    --name rslidar-airy-ros2 \
    rslidar-airy-ros2 "$@"
