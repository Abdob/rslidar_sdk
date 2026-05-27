"""Launch usb_cam + the fisheye chessboard intrinsic calibrator."""
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="usb_cam",
            output="screen",
            parameters=["/opt/calib/config/usb_cam.yaml"],
        ),
        ExecuteProcess(
            cmd=["python3", "/opt/calib/scripts/calibrate_intrinsic.py"],
            output="screen",
        ),
    ])
