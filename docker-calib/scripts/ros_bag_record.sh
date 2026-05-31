#!/usr/bin/env bash
# Record the three FAST-LIVO2 input topics to a ros2 bag.
#
# IMPORTANT: we record /rslidar_imu_data_fixed (m/s^2, ROS clock) and NOT
# the raw /rslidar_imu_data (g, sensor uptime). The raw stream is unusable
# for FAST-LIVO2 — see docker-calib/scripts/imu_bridge.py.
#
# Usage:
#   bash /opt/calib/scripts/ros_bag_record.sh ezoffice
#   -> /opt/calib/bags/ezoffice/
FILE_NAME="${1:?Usage: ros_bag_record.sh <bag_name>}"
echo "Recording ROS bag named ${FILE_NAME}..."
ros2 bag record -o /opt/calib/bags/${FILE_NAME} \
    /rslidar_points \
    /rslidar_imu_data_fixed \
    /image_raw
