"""Check motor-gate selection without starting processes or connecting to a robot."""

import importlib.util
import unittest
from pathlib import Path

from launch import LaunchContext
from launch_ros.actions import Node

REPO = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "manual_arm_launch", REPO / "tests/manual/arm_raise_dry_run.launch.py"
)
LAUNCH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(LAUNCH)


class ManualLaunchTests(unittest.TestCase):
    def context(self, hardware, profile=""):
        context = LaunchContext()
        context.launch_configurations.update(
            enable_hardware_output=hardware,
            hardware_profile_path=profile,
            sdk_interface="eth0",
            sdk_domain="0",
            startup_timeout_secs="5.0",
            test_policy="unitree_arm7",
        )
        return context

    def node_roles(self, context):
        nodes = [
            action
            for action in LAUNCH.start_processes(context)
            if isinstance(action, Node)
        ]
        self.assertEqual(len(nodes), 4)
        # Roles are literal arguments, executable lookup is deliberately not run.
        return {
            substitution.text
            for node in nodes
            for argument in node.cmd
            for substitution in argument
            if hasattr(substitution, "text")
        }

    def test_diagnostic_output_selects_only_mock_gate(self):
        roles = self.node_roles(self.context("false"))
        self.assertIn("motor_gate", roles)
        self.assertNotIn("hardware_gate", roles)

    def test_hardware_output_replaces_mock_gate(self):
        roles = self.node_roles(
            self.context(
                "true", str(REPO / "robot_safety/profiles/g1_type5.template.json")
            )
        )
        self.assertIn("hardware_gate", roles)
        self.assertNotIn("motor_gate", roles)

    def test_missing_hardware_profile_rejected_before_process_creation(self):
        with self.assertRaises((OSError, ValueError)):
            LAUNCH.start_processes(self.context("true"))

    def test_invalid_enable_value_rejected(self):
        with self.assertRaises(ValueError):
            LAUNCH.start_processes(self.context("yes"))
