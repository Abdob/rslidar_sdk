FILE_NAME=$1
echo "Recording ROS bag named ${FILE_NAME}..."
ros2 bag record -o /opt/calib/bags/${FILE_NAME} \
    /rslidar_points \
    /rslidar_imu_data \
    /image_raw
