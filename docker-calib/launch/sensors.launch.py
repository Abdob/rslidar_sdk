"""Start the AIRY LiDAR publisher + the IMU bridge.

Reused by intrinsic / extrinsic / colorize launches. Camera publishing is
moved out of this container — start docker-gst-camera in parallel so that
`/image_raw` arrives over the host's DDS network.

  Terminal 1:   ./docker-gst-camera/docker_run.sh
  Terminal 2:   ./docker-calib/docker_run.sh <mode>

Topics expected after both are up:
   /rslidar_points          ~10 Hz   (this container, rslidar_sdk)
   /rslidar_imu_data        ~200 Hz  (this container, rslidar_sdk; in g,
                                       sensor uptime stamps — raw)
   /rslidar_imu_data_fixed  ~200 Hz  (this container, imu_bridge.py; in
                                       m/s^2, ROS clock stamps — feed this
                                       to FAST-LIVO2 / FAST-LIO)
   /image_raw               ~30 Hz   (docker-gst-camera)
   /image_raw/compressed    ~30 Hz   (docker-gst-camera, native MJPEG bytes)
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rslidar_sdk",
            executable="rslidar_sdk_node",
            name="rslidar_sdk_node",
            output="screen",
        ),
        ExecuteProcess(
            cmd=["python3", "/opt/calib/scripts/imu_bridge.py"],
            name="imu_bridge",
            output="screen",
        ),
    ])
