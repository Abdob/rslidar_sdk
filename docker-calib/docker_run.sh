#!/usr/bin/env bash
# Run the calibration container.
#
# Usage:
#   ./docker_run.sh                            # interactive shell inside /opt/calib
#   ./docker_run.sh intrinsic                  # ros2 launch calib/intrinsic
#   ./docker_run.sh test-intrinsic             # ros2 launch calib/test_intrinsic
#   ./docker_run.sh extrinsic                  # ros2 launch calib/extrinsic
#   ./docker_run.sh colorize                   # ros2 launch calib/colorize
#   ./docker_run.sh -- python3 …               # arbitrary command
#
# The config/, scripts/, launch/, rviz/ dirs are bind-mounted from this host
# directory so you can edit YAML/Python and rerun without rebuilding.
set -e

HOST_DIR="$(cd "$(dirname "$0")" && pwd)"

xhost +local:docker >/dev/null 2>&1 || true

# Dispatch convenience aliases.
ARGS=("$@")
case "${1:-}" in
    intrinsic)
        ARGS=(ros2 launch /opt/calib/launch/intrinsic.launch.py)
        ;;
    test-intrinsic)
        ARGS=(ros2 launch /opt/calib/launch/test_intrinsic.launch.py)
        ;;
    extrinsic)
        ARGS=(ros2 launch /opt/calib/launch/extrinsic.launch.py)
        ;;
    colorize)
        ARGS=(ros2 launch /opt/calib/launch/colorize.launch.py)
        ;;
    sensors)
        ARGS=(ros2 launch /opt/calib/launch/sensors.launch.py)
        ;;
    --)
        shift
        ARGS=("$@")
        ;;
    "")
        ARGS=(bash)
        ;;
esac

exec docker run --rm -it \
    --net=host \
    --privileged \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}" \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev:/dev \
    -v "$HOST_DIR/config":/opt/calib/config:rw \
    -v "$HOST_DIR/config/lidar_config.yaml":/opt/ros_ws/src/rslidar_sdk/config/config.yaml:ro \
    -v "$HOST_DIR/scripts":/opt/calib/scripts:ro \
    -v "$HOST_DIR/launch":/opt/calib/launch:ro \
    -v "$HOST_DIR/rviz":/opt/calib/rviz:ro \
    --name rslidar-airy-calib \
    rslidar-airy-calib "${ARGS[@]}"
