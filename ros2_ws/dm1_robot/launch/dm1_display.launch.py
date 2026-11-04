"""用原生 MuJoCo simulate 查看 DM1 模型，不启动控制器."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import FindExecutable, PathJoinSubstitution
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
    """启动 MuJoCo simulate 并加载 DM1 的 scene.xml."""
    scene_file = PathJoinSubstitution([
        FindPackageShare("dm1_robot"), "description", "scene.xml"
    ])
    return LaunchDescription(_desktop_session_environment() + [
        ExecuteProcess(
            cmd=[FindExecutable(name="simulate"), scene_file],
            output="screen",
        ),
    ])
