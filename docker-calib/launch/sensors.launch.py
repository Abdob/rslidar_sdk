"""Start the AIRY LiDAR publisher.

Reused by intrinsic / extrinsic / colorize launches. Camera publishing is
moved out of this container — start docker-gst-camera in parallel so that
`/image_raw` arrives over the host's DDS network.

  Terminal 1:   ./docker-gst-camera/docker_run.sh
  Terminal 2:   ./docker-calib/docker_run.sh <mode>

Topics expected after both are up:
   /rslidar_points          ~10 Hz   (this container)
   /rslidar_imu_data        ~200 Hz  (this container, if LiDAR-side enabled)
   /image_raw               ~30 Hz   (docker-gst-camera)
   /image_raw/compressed    ~30 Hz   (docker-gst-camera, native MJPEG bytes)
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rslidar_sdk",
            executable="rslidar_sdk_node",
            name="rslidar_sdk_node",
            output="screen",
        ),
    ])
