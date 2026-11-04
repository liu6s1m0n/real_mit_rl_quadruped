"""以 IMU-only 模式启动 hardware_main 内置的 IMU 接收链路."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import FindExecutable, LaunchConfiguration


def generate_launch_description():
    """启动 hardware_main，不打开电机 CAN，也不发送任何电机命令."""
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
                "dm1_robot",
                "hardware_main",
                "--imu-only",
                "--imu",
                LaunchConfiguration("port"),
            ],
            output="screen",
        ),
    ])
