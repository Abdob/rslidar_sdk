"""Launch sensors + colorizer + RViz2 (colored cloud preview)."""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource("/opt/calib/launch/sensors.launch.py")),
        ExecuteProcess(
            cmd=["python3", "/opt/calib/scripts/colorize_node.py"],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", "/opt/calib/rviz/colorize.rviz"],
            output="screen",
        ),
    ])
