"""Launch the integrated hardware_main IMU receiver in IMU-only mode."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import FindExecutable, LaunchConfiguration


def generate_launch_description():
    """Start hardware_main without opening motor CAN or sending motor commands."""
    return LaunchDescription([
        DeclareLaunchArgument(
            "port",
            default_value="auto",
            description="DM IMU serial device path, or auto to use /dev/serial/by-id",
        ),
        ExecuteProcess(
            cmd=[
                FindExecutable(name="ros2"),
                "run",
                "mymit_robot",
                "hardware_main",
                "--imu-only",
                "--imu",
                LaunchConfiguration("port"),
            ],
            output="screen",
        ),
    ])
