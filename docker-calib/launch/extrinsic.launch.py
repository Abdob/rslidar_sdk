"""Launch sensors + extrinsic calibrator + RViz."""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource("/opt/calib/launch/sensors.launch.py")),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", "/opt/calib/rviz/extrinsic.rviz"],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["python3", "/opt/calib/scripts/calibrate_extrinsic.py"],
            output="screen",
        ),
    ])
