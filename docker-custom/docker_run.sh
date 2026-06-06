#!/usr/bin/env bash
# Run the timing-sync experiment container.
#
# Usage:
#   ./docker_run.sh                              # interactive shell in /opt/custom
#   ./docker_run.sh record run1                  # record_sync_bag.sh run1
#   ./docker_run.sh extract run1                 # extract_signals.py run1
#   ./docker_run.sh plot run1                    # plot_sync.py run1
#   ./docker_run.sh -- python3 ...               # arbitrary command
#
# config/, scripts/, bags/ are bind-mounted from this dir so you can edit
# YAML/Python and re-run without rebuilding. docker-calib/config is mounted
# read-only for the camera intrinsics.
#
# --gpus all gives the container the GPU (driver + CUDA). DISPLAY/X11 is for
# matplotlib's interactive window (CPU/X rendering -- it does NOT touch the
# NVIDIA EGL path, so unlike the camera container it's safe here). Plot scripts
# default to saving a PNG, so a display is optional.
set -e

HOST_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$HOST_DIR/.." && pwd)"

mkdir -p "$HOST_DIR/bags"

xhost +local:docker >/dev/null 2>&1 || true

# Dispatch convenience aliases.
case "${1:-}" in
    record)  shift; ARGS=(bash /opt/custom/scripts/record_sync_bag.sh "$@") ;;
    extract) shift; ARGS=(python3 /opt/custom/scripts/extract_signals.py "$@") ;;
    axes)    shift; ARGS=(python3 /opt/custom/scripts/inspect_axes.py "$@") ;;
    measure) shift; ARGS=(python3 /opt/custom/scripts/plot_measurements.py "$@") ;;
    plot)    shift; ARGS=(python3 /opt/custom/scripts/plot_sync.py "$@") ;;
    --)      shift; ARGS=("$@") ;;
    "")      ARGS=(bash) ;;
    *)       ARGS=("$@") ;;
esac

exec docker run --rm -it \
    --net=host \
    --privileged \
    --gpus all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev:/dev \
    -v "$HOST_DIR/config":/opt/custom/config:rw \
    -v "$HOST_DIR/scripts":/opt/custom/scripts:ro \
    -v "$HOST_DIR/bags":/opt/custom/bags:rw \
    -v "$REPO_ROOT/docker-calib/config":/opt/calib/config:ro \
    --name rslidar-airy-custom \
    rslidar-airy-custom "${ARGS[@]}"
