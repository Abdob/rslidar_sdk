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

# Headless container: it only publishes /image_raw to ROS (fpsdisplaysink uses
# fakesink). Deliberately NO DISPLAY / X11 mount -- on the DGX Spark, exposing an
# X display routes nvvideoconvert's EGL init to Mesa's software Vulkan (ZINK),
# which fails with VK_ERROR_INCOMPATIBLE_DRIVER and kills the pipeline (no
# /image_raw). Without a display, EGL init no-ops and the GPU path runs fine.

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
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /dev:/dev \
    -v "$REPO_ROOT/docker-calib/config":/opt/calib/config:ro \
    -v "$HOST_DIR/config":/opt/camera/config:rw \
    -v "$HOST_DIR/launch":/opt/camera/launch:ro \
    --name rslidar-airy-gst-camera \
    rslidar-airy-gst-camera "${ARGS[@]}"
