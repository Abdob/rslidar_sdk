#!/usr/bin/env bash
# Run the GPU-accelerated MJPEG camera publisher.
#
# Usage:
#   ./docker_run.sh                            # GPU pipeline, default 1280x720@30
#   ./docker_run.sh --use_gpu false            # CPU fallback
#   ./docker_run.sh --width 1920 --height 1080 # other resolution
#   ./docker_run.sh -- bash                    # shell inside the container
#
# Requires --gpus all on the host: install nvidia-container-toolkit if you
# haven't.  Sources the host's intrinsics.yaml from docker-calib so the
# published /camera_info is correct without copying anything.
set -e

HOST_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$HOST_DIR/.." && pwd)"

xhost +local:docker >/dev/null 2>&1 || true

ARGS=("$@")
case "${1:-}" in
    --)
        shift
        ARGS=("$@")
        ;;
    "")
        ARGS=(ros2 launch /opt/camera/launch/camera.launch.py)
        ;;
    *)
        # Pass launch args through to ros2 launch.
        ARGS=(ros2 launch /opt/camera/launch/camera.launch.py "$@")
        ;;
esac

exec docker run --rm -it \
    --net=host \
    --privileged \
    --gpus all \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}" \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev:/dev \
    -v "$REPO_ROOT/docker-calib/config":/opt/calib/config:ro \
    -v "$HOST_DIR/config":/opt/camera/config:rw \
    -v "$HOST_DIR/launch":/opt/camera/launch:ro \
    --name rslidar-airy-gst-camera \
    rslidar-airy-gst-camera "${ARGS[@]}"
