"""Launch DM1 MuJoCo controller with native direction buttons."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _desktop_session_environment():
    """Provide missing X11 session variables required by MuJoCo's GLFW window."""
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
    """Start the DM1 controller window in the current desktop session."""
    return LaunchDescription([
        DeclareLaunchArgument(
            "render_gpu",
            default_value="auto",
            description="OpenGL renderer selection: auto or nvidia",
        ),
        DeclareLaunchArgument(
            "hardware_sim",
            default_value="false",
            description="Run the complete hardware driver/controller path against simulated devices",
        ),
        DeclareLaunchArgument(
            "torque_limit",
            default_value="88",
            description="Hardware simulation torque limit; match the hardware_main run",
        ),
        DeclareLaunchArgument(
            "walk_mode",
            default_value="mpc",
            # 换模型要修改的地方：launch 参数说明中的模型编号
            description="Walking controller selection: mpc or rl (model_4210/model_4245)",
        ),
        *_desktop_session_environment(),
        OpaqueFunction(function=_controller_node),
    ])


def _controller_node(context):
    arguments = [
        "--walk-mode", LaunchConfiguration("walk_mode").perform(context),
        "--render-gpu", LaunchConfiguration("render_gpu").perform(context),
    ]
    hardware_sim = LaunchConfiguration("hardware_sim").perform(context).lower()
    if hardware_sim not in ("true", "false", "1", "0"):
        raise ValueError("hardware_sim must be true or false")
    if hardware_sim in ("true", "1"):
        arguments += [
            "--hardware-sim", "--torque-limit",
            LaunchConfiguration("torque_limit").perform(context),
        ]
    return [Node(
        package="mymit_robot", executable="mymit_robot_user",
        arguments=arguments, output="screen",
    )]
