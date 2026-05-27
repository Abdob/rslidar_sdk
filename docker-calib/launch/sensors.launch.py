"""Start the AIRY LiDAR publisher + USB camera node together.

Reused by intrinsic / extrinsic / colorize launches.
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
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="usb_cam",
            output="screen",
            parameters=["/opt/calib/config/usb_cam.yaml"],
        ),
    ])
