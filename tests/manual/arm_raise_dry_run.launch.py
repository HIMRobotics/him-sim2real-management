# ruff: noqa: N999
"""Start five isolated safety processes, with no motor publisher or automatic arm."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Allow initial DDS discovery, then enforce normal feedback deadlines."""
    interface = LaunchConfiguration("sdk_interface")
    sdk_domain = LaunchConfiguration("sdk_domain")
    discovery_timeout = LaunchConfiguration("startup_timeout_secs")
    roles = ("feedback", "checker", "motor_gate", "controller_stop", "arm_raise")
    parameters = {
        "feedback": {"sdk_interface": interface, "sdk_domain": sdk_domain},
        "controller_stop": {"startup_timeout_secs": discovery_timeout},
    }
    processes = [
        Node(
            package="him_safety_nodes",
            executable="safety_node",
            arguments=[role],
            parameters=[parameters.get(role, {})],
            output="screen",
            respawn=False,
        )
        for role in roles[:-1]
    ]
    processes.append(
        ExecuteProcess(
            cmd=["/usr/bin/python3", "-m", "tests.manual.arm_raise_node"],
            cwd=str(Path(__file__).resolve().parents[2]),
            output="screen",
            respawn=False,
        )
    )
    # A partially surviving test graph must not look like a healthy new run.
    exit_handlers = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=process,
                on_exit=[
                    EmitEvent(event=Shutdown(reason=f"Safety process exited: {role}"))
                ],
            )
        )
        for role, process in zip(roles, processes)
    ]
    return LaunchDescription(
        [
            DeclareLaunchArgument("sdk_interface", default_value="eth0"),
            DeclareLaunchArgument("sdk_domain", default_value="0"),
            DeclareLaunchArgument("startup_timeout_secs", default_value="5.0"),
            *exit_handlers,
            *processes,
        ]
    )
