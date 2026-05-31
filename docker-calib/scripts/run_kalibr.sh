#!/usr/bin/env bash
# Run Kalibr camera-IMU calibration to recover the fine camera<->lidar-clock
# time shift, then tell you what to paste into config/time_sync.yaml.
#
# RUN THIS ON THE HOST, not inside the docker-calib container -- it launches
# the Kalibr Docker image, so it needs the host's `docker`. The container has
# no Docker daemon (that's the "docker: command not found" you hit).
#
# Two-step flow (Kalibr only reads ROS 1 bags):
#   1. INSIDE the docker-calib container, convert the bag:
#        bash /opt/calib/scripts/convert_bag.sh kalibr_run1
#        -> writes bags/kalibr_run1.bag
#   2. ON THE HOST, run this:
#        bash docker-calib/scripts/run_kalibr.sh kalibr_run1
#
# It runs kalibr_calibrate_imu_camera in $KALIBR_IMG against:
#        --cam    config/kalibr_camchain.yaml   (/image_raw_synced)
#        --imu    config/kalibr_imu.yaml         (/rslidar_imu_data_fixed)
#        --target config/kalibr_aprilgrid.yaml
# and prints time_shift_cam_imu from the results file.
#
# Override the Kalibr image if you built your own:
#   KALIBR_IMG=myrepo/kalibr:latest bash run_kalibr.sh kalibr_run1
set -e

NAME="${1:?Usage: run_kalibr.sh <bag_name>}"
# stereolabs/kalibr is NOT a Stereolabs algorithm -- it's a prebuilt Docker
# image of the standard ethz-asl/kalibr toolbox (same kalibr_calibrate_imu_camera).
# Override with any Kalibr image (e.g. christianbrommer/kalibr) or your own build.
KALIBR_IMG="${KALIBR_IMG:-stereolabs/kalibr:latest}"

# Resolve repo paths from this script's own location (docker-calib/scripts/),
# so it works from any host cwd. bags/ and docker-calib/config/ are the same
# dirs the container sees as /opt/calib/bags and /opt/calib/config.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BAGS="$REPO_ROOT/bags"
CFG="$REPO_ROOT/docker-calib/config"
ROS1_BAG="$BAGS/$NAME.bag"

if ! command -v docker >/dev/null 2>&1; then
    echo "ERROR: 'docker' not found." >&2
    echo "Run run_kalibr.sh ON THE HOST, not inside the docker-calib container." >&2
    exit 1
fi

if [ ! -f "$ROS1_BAG" ]; then
    echo "ROS 1 bag not found: $ROS1_BAG" >&2
    echo "Convert it first, INSIDE the docker-calib container:" >&2
    echo "    bash /opt/calib/scripts/convert_bag.sh $NAME" >&2
    exit 1
fi

# -w /data/bags so Kalibr writes results-imucam-*.txt into the host bags dir.
# MPLBACKEND=Agg: Kalibr's report step builds matplotlib figures even with
# --dont-show-report; the default Tk backend then dies headless ("no $DISPLAY").
# Agg renders the PDF without a window. We also don't let a report-only failure
# abort the script -- the .txt/.yaml results are written before the report.
echo "Running kalibr_calibrate_imu_camera in $KALIBR_IMG ..."
set +e
docker run --rm -t \
    -v "$BAGS":/data/bags \
    -v "$CFG":/data/cfg \
    -w /data/bags \
    -e MPLBACKEND=Agg \
    "$KALIBR_IMG" \
    kalibr_calibrate_imu_camera \
        --bag    "/data/bags/$NAME.bag" \
        --cam    /data/cfg/kalibr_camchain.yaml \
        --imu    /data/cfg/kalibr_imu.yaml \
        --target /data/cfg/kalibr_aprilgrid.yaml \
        --dont-show-report
set -e

# Surface the result. Kalibr writes results-imucam-<bag>.txt in the work dir.
RESULT=$(ls -t "$BAGS"/results-imucam-*.txt 2>/dev/null | head -1 || true)
echo
if [ -n "$RESULT" ] && grep -qiE 'timeshift' "$RESULT"; then
    echo "=== Kalibr result: $RESULT ==="
    grep -iE 'reprojection error \(cam0\) \[px\]' "$RESULT" || true
    # The timeshift VALUE is on the line BELOW the label, so print both (-A1).
    # (Do not confuse this with "time offset with respect to IMU0: 0.0", which
    #  is IMU0 vs itself and is always 0 in a single-IMU rig.)
    grep -iE -A1 'timeshift cam0 to imu0' "$RESULT" || true
    echo
    echo ">>> Paste the number BELOW 'timeshift cam0 to imu0' (t_imu = t_cam + shift) into:"
    echo "    docker-calib/config/time_sync.yaml -> cam_lidar_time_shift"
    echo "    then re-record with ros_bag_record.sh (image_restamp.py reloads it)."
else
    echo "No results-imucam-*.txt found. Check the Kalibr output above for errors"
    echo "(common: too few AprilGrid detections -> re-record with more grid coverage)."
fi
