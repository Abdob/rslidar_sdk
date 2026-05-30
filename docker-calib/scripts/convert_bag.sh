#!/usr/bin/env bash
# Convert a ROS 2 bag (directory with metadata.yaml + *.db3) into a ROS 1
# bag (single .bag file) using the `rosbags` Python package. Output is
# written next to the source under /opt/calib/bags/.
#
# Usage (from inside the docker-calib container):
#   bash /opt/calib/scripts/convert_bag.sh ezoffice
#     -> /opt/calib/bags/ezoffice/  ->  /opt/calib/bags/ezoffice.bag
set -e

NAME="${1:?Usage: convert_bag.sh <bag_name>}"
SRC="/opt/calib/bags/$NAME"
DST="/opt/calib/bags/$NAME.bag"

if [ ! -d "$SRC" ]; then
    echo "Source bag directory not found: $SRC" >&2
    exit 1
fi
if [ ! -f "$SRC/metadata.yaml" ]; then
    echo "Not a ROS 2 bag (no metadata.yaml inside $SRC)" >&2
    exit 1
fi
if [ -e "$DST" ]; then
    echo "Destination already exists: $DST (delete it first to re-convert)" >&2
    exit 1
fi

if ! command -v rosbags-convert >/dev/null 2>&1; then
    echo "Installing rosbags (one-time)..."
    pip3 install --quiet --user rosbags
    export PATH="$HOME/.local/bin:$PATH"
fi

echo "Converting $SRC -> $DST"
rosbags-convert --src "$SRC" --dst "$DST"
echo "Done. To use with FAST-LIVO2, move/copy to docker-livo2/bags/:"
echo "  mv $DST /path/to/docker-livo2/bags/"
