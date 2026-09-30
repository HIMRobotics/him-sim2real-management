# ruff: noqa: N999
"""Launch the arm test with diagnostic output unless hardware is explicitly enabled."""

from pathlib import Path
import subprocess

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start_processes(context):
    """Use the running systemd gate for hardware or start a diagnostic gate."""
    hardware = LaunchConfiguration("enable_hardware_output").perform(context)
    if hardware not in ("true", "false"):
        raise ValueError("enable_hardware_output must be true or false")
    if hardware == "true":
        service = subprocess.run(
            ["systemctl", "--user", "is-active", "--quiet", "g1-safety-stops.service"],
            capture_output=True, timeout=5,
        )
        if service.returncode != 0:
            raise RuntimeError(
                "Safety service is not active. Start it and wait for warmup: "
                "systemctl --user start g1-safety-stops.service"
            )
    interface = LaunchConfiguration("sdk_interface")
    sdk_domain = LaunchConfiguration("sdk_domain")
    discovery_timeout = LaunchConfiguration("startup_timeout_secs")
    gate_roles = () if hardware == "true" else ("motor_gate",)
    roles = ("feedback", "checker", *gate_roles, "controller_stop", "arm_raise")
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
            cmd=[
                "/usr/bin/python3",
                "-m",
                "tests.manual.arm_raise_node",
                "--ros-args",
                "-p",
                ["test_policy:=", LaunchConfiguration("test_policy")],
                "-p",
                ["handover_timeout_secs:=", LaunchConfiguration("handover_timeout_secs")],
            ],
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
    return [*exit_handlers, *processes]


def generate_launch_description():
    """Expose explicit output selection without automatically arming the robot."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("sdk_interface", default_value="eth0"),
            DeclareLaunchArgument("test_policy", default_value="slow_raise"),
            DeclareLaunchArgument("sdk_domain", default_value="0"),
            DeclareLaunchArgument("startup_timeout_secs", default_value="5.0"),
            DeclareLaunchArgument("enable_hardware_output", default_value="false"),
            DeclareLaunchArgument("handover_timeout_secs", default_value="5.0"),
            OpaqueFunction(function=start_processes),
        ]
    )
