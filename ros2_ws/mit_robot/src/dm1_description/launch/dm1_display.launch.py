"""Display the DM1 model in native MuJoCo Simulate without the controller."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


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
    scene_file = PathJoinSubstitution([
        FindPackageShare("mymit_robot"), "dm1", "scene.xml"
    ])
    return LaunchDescription(_desktop_session_environment() + [
        ExecuteProcess(
            cmd=[FindExecutable(name="simulate"), scene_file],
            output="screen",
        ),
    ])
