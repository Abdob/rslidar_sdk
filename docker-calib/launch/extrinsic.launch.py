"""Launch sensors + extrinsic calibrator."""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource("/opt/calib/launch/sensors.launch.py")),
        ExecuteProcess(
            cmd=["python3", "/opt/calib/scripts/calibrate_extrinsic.py"],
            output="screen",
        ),
    ])
