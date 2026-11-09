"""启动 DM1 MuJoCo 仿真控制器（sim_main），窗口带原生方向按钮."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _desktop_session_environment():
    """
    补齐 MuJoCo GLFW 窗口需要、但当前环境缺失的 X11 会话变量.

    从 SSH、IDE 终端等非桌面会话启动时，XDG_RUNTIME_DIR、DBUS_SESSION_BUS_ADDRESS
    和 XAUTHORITY 可能未设置；这里只在变量缺失且对应文件存在时补上。
    """
    actions = []
    runtime_dir = Path(f"/run/user/{os.getuid()}")
    if not os.environ.get("XDG_RUNTIME_DIR") and runtime_dir.is_dir():
        actions.append(SetEnvironmentVariable("XDG_RUNTIME_DIR", str(runtime_dir)))

    bus_path = runtime_dir / "bus"
    if not os.environ.get("DBUS_SESSION_BUS_ADDRESS") and bus_path.exists():
        actions.append(SetEnvironmentVariable(
            "DBUS_SESSION_BUS_ADDRESS", f"unix:path={bus_path}"
        ))

    authority = Path.home() / ".Xauthority"
    if not os.environ.get("XAUTHORITY") and authority.is_file():
        actions.append(SetEnvironmentVariable("XAUTHORITY", str(authority)))
    return actions


def generate_launch_description():
    """在当前桌面会话中启动 DM1 控制器窗口."""
    return LaunchDescription([
        DeclareLaunchArgument(
            "render_gpu",
            default_value="auto",
            description="OpenGL renderer selection: auto or nvidia",
        ),
        DeclareLaunchArgument(
            "hardware_sim",
            default_value="false",
            description=(
                "Run the complete hardware driver/controller path "
                "against simulated devices"
            ),
        ),
        DeclareLaunchArgument(
            "algorithm_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("dm1_robot"), "config", "opendog"
            ]),
            description="OpenDog algorithm configuration directory",
        ),
        DeclareLaunchArgument(
            "torque_limit",
            default_value="30",
            description="Hardware simulation torque limit; match the hardware_main run",
        ),
        *_desktop_session_environment(),
        OpaqueFunction(function=_controller_node),
    ])


def _controller_node(context):
    """把 launch 参数解析为 sim_main 的命令行参数并创建节点."""
    arguments = [
        "--render-gpu", LaunchConfiguration("render_gpu").perform(context),
        "--algorithm-config", LaunchConfiguration("algorithm_config").perform(context),
    ]
    hardware_sim = LaunchConfiguration("hardware_sim").perform(context).lower()
    if hardware_sim not in ("true", "false", "1", "0"):
        raise ValueError("hardware_sim must be true or false")
    # 硬件在环模式额外传入力矩上限，与真机 hardware_main 保持一致。
    if hardware_sim in ("true", "1"):
        arguments += [
            "--hardware-sim", "--torque-limit",
            LaunchConfiguration("torque_limit").perform(context),
        ]
    return [Node(
        package="dm1_robot", executable="sim_main",
        arguments=arguments, output="screen",
    )]
