"""Check motor-gate selection without starting processes or connecting to a robot."""

import importlib.util
import unittest
from pathlib import Path
from unittest.mock import patch

from launch import LaunchContext
from launch_ros.actions import Node

REPO = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "manual_arm_launch", REPO / "tests/manual/arm_raise_dry_run.launch.py"
)
LAUNCH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(LAUNCH)


class ManualLaunchTests(unittest.TestCase):
    def context(self, hardware):
        context = LaunchContext()
        context.launch_configurations.update(
            enable_hardware_output=hardware,
            handover_timeout_secs="5.0",
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
        hardware = context.launch_configurations["enable_hardware_output"] == "true"
        self.assertEqual(len(nodes), 3 if hardware else 4)
        # Roles are literal arguments, executable lookup is deliberately not run.
        return {
            substitution.text
            for node in nodes
            for argument in node.cmd
            for substitution in argument
            if hasattr(substitution, "text")
        }

    def test_diagnostic_output_selects_only_mock_gate(self):
        with patch.object(LAUNCH.subprocess, "run") as service:
            roles = self.node_roles(self.context("false"))
            service.assert_not_called()
        self.assertIn("motor_gate", roles)
        self.assertNotIn("hardware_gate", roles)

    def test_hardware_output_reuses_active_systemd_gate(self):
        with patch.object(LAUNCH.subprocess, "run") as service:
            service.return_value.returncode = 0
            roles = self.node_roles(self.context("true"))
            service.assert_called_once_with(
                ["systemctl", "--user", "is-active", "--quiet", "g1-safety-stops.service"],
                capture_output=True, timeout=5,
            )
        self.assertNotIn("hardware_gate", roles)
        self.assertNotIn("motor_gate", roles)
        for role in ("feedback", "checker", "controller_stop"):
            self.assertIn(role, roles)

    def test_inactive_service_rejected_before_process_creation(self):
        with patch.object(LAUNCH.subprocess, "run") as service:
            service.return_value.returncode = 3
            with self.assertRaisesRegex(RuntimeError, "Safety service is not active"):
                LAUNCH.start_processes(self.context("true"))

    def test_invalid_enable_value_rejected(self):
        with self.assertRaises(ValueError):
            LAUNCH.start_processes(self.context("yes"))
