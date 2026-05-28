#!/usr/bin/env bash
# Run the FAST-LIVO2 container.
#
# Usage:
#   ./docker_run.sh                            # interactive shell
#   ./docker_run.sh avia                       # roslaunch fast_livo mapping_avia.launch
#   ./docker_run.sh -- rosbag play /data/x.bag # arbitrary command (after --)
#
# Bind-mounts:
#   ./bags     -> /data       (drop downloaded bags here)
#   ./config   -> /opt/calib  (your intrinsics.yaml + extrinsic.yaml, for later)
set -e

HOST_DIR="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$HOST_DIR/bags" "$HOST_DIR/config"

xhost +local:docker >/dev/null 2>&1 || true

ARGS=("$@")
case "${1:-}" in
    avia)
        ARGS=(roslaunch fast_livo mapping_avia.launch)
        ;;
    avia-marslvig)
        ARGS=(roslaunch fast_livo mapping_avia_marslvig.launch)
        ;;
    mid360)
        ARGS=(roslaunch fast_livo mapping_avia.launch)   # placeholder; add when we have a mid360 launch
        ;;
    hesai)
        ARGS=(roslaunch fast_livo mapping_hesaixt32_hilti22.launch)
        ;;
    ouster)
        ARGS=(roslaunch fast_livo mapping_ouster_ntu.launch)
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
    -e DISPLAY="${DISPLAY:-:0}" \
    -e XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$HOST_DIR/bags":/data:rw \
    -v "$HOST_DIR/config":/opt/calib:rw \
    --name rslidar-airy-livo2 \
    rslidar-airy-livo2 "${ARGS[@]}"
