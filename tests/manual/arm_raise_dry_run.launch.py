# ruff: noqa: N999
"""Launch the arm test with diagnostic output unless hardware is explicitly enabled."""

from pathlib import Path

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
    """Select one motor gate and validate hardware settings before starting nodes."""
    hardware = LaunchConfiguration("enable_hardware_output").perform(context)
    if hardware not in ("true", "false"):
        raise ValueError("enable_hardware_output must be true or false")
    gate_role = "hardware_gate" if hardware == "true" else "motor_gate"
    profile_path = LaunchConfiguration("hardware_profile_path").perform(context)
    handover_timeout_secs = 5.0
    if hardware == "true":
        from robot_safety.hardware_profile import HardwareProfile

        profile = HardwareProfile.load(profile_path)
        handover_timeout_secs = profile.handover_secs + 1.0
    interface = LaunchConfiguration("sdk_interface")
    sdk_domain = LaunchConfiguration("sdk_domain")
    discovery_timeout = LaunchConfiguration("startup_timeout_secs")
    roles = ("feedback", "checker", gate_role, "controller_stop", "arm_raise")
    parameters = {
        "feedback": {"sdk_interface": interface, "sdk_domain": sdk_domain},
        "hardware_gate": {
            "enable_hardware_output": True,
            "hardware_profile_path": profile_path,
            "sdk_interface": interface,
            "sdk_domain": sdk_domain,
        },
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
                f"handover_timeout_secs:={handover_timeout_secs}",
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
            DeclareLaunchArgument("hardware_profile_path", default_value=""),
            OpaqueFunction(function=start_processes),
        ]
    )
