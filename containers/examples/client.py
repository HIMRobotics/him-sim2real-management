"""Run the same policy container against simulated or measured robot state."""

import json
import os
import socket
import sys
import time
from pathlib import Path

import rclpy
from command import load_schedule
from controllers import ExamplePolicy
from him_safety_interfaces.msg import Abort, JointCommand
from him_safety_interfaces.srv import ArmGate
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trial import MovementTrial


class PolicyNode(Node):
    def __init__(self, policy, mode="sim"):
        super().__init__("example_policy")
        self.policy = policy
        self.mode = mode
        self.task_command = json.loads(os.environ.get("G1_TASK_COMMAND") or
                                       ("null" if mode == "sim" else "[0,0,0]"))
        command_file = os.environ.get("G1_COMMAND_FILE")
        self.schedule = load_schedule(command_file) if command_file else None
        self.command_started = None
        trial_path = os.environ.get("G1_TRIAL_CONFIG")
        self.trial = MovementTrial(json.loads(Path(trial_path).read_text())) if trial_path else None
        if self.trial and (policy.name == "balance" or os.environ.get("G1_THRASH_MODE") != "enforce"):
            raise ValueError("movement trials require velocity commands and thrash enforcement")
        self.state = None
        self.done = False
        self.last_observation = 0
        self.session_id = None
        self.last_received = time.monotonic()
        self.pub = self.create_publisher(JointCommand, "/safety/precheck", 1)
        self.create_subscription(
            String,
            "/safety/policy_state",
            self.on_state,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        abort_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Abort, "/safety/abort", self.on_abort, abort_qos)
        self.abort_pub = self.create_publisher(Abort, "/safety/abort", abort_qos)
        self.create_timer(0.02, self.publish_command)

    def arm(self):
        """Use the existing readiness and session services."""
        def call(service, kind, request):
            client = self.create_client(kind, service)
            try:
                if not client.wait_for_service(timeout_sec=5):
                    raise RuntimeError("missing service: " + service)
                future = client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5)
                result = future.result() if future.done() else None
                if result is None or not result.success:
                    detail = "" if result is None else (result.message or "")
                    raise RuntimeError(
                        "arming rejected: " + service + ((": " + detail) if detail else "")
                    )
                return result.message
            finally:
                self.destroy_client(client)

        if self.mode != "sim":
            call("/safety/controller_stop/ready", Trigger, Trigger.Request())
            call("/safety/motor_gate/ready", Trigger, Trigger.Request())
        session = call("/safety/checker/arm", Trigger, Trigger.Request())
        if self.mode != "sim":
            request = ArmGate.Request()
            request.session_id = session
            call("/safety/motor_gate/arm", ArmGate, request)
        self.session_id = session

    def on_state(self, message):
        if self.done:
            return
        state = json.loads(message.data)
        if self.mode != "sim" and state.get("host_boot_id") != Path("/proc/sys/kernel/random/boot_id").read_text().strip():
            raise RuntimeError("policy state came from another host clock")
        if self.policy.name != "balance" and "base_velocity" not in state:
            raise RuntimeError("policy requires measured base velocity")
        if (self.trial or (self.policy.name == "stairs" and self.task_command is None)) and "position" not in state:
            raise RuntimeError("task requires measured base position")
        if self.session_id is None and self.mode == "sim":
            self.session_id = state["session_id"]
        if state["session_id"] and self.session_id and state["session_id"] != self.session_id:
            raise RuntimeError("policy session changed")
        self.state = state
        self.last_received = time.monotonic()

    def publish_command(self):
        if self.done or not self.session_id or self.state is None:
            return
        state = dict(self.state, session_id=self.session_id)
        if not 0 <= time.monotonic() - state["received_at_secs"] < 0.2:
            raise RuntimeError("stale policy state")
        if state["observation_id"] <= self.last_observation:
            return
        self.last_observation = state["observation_id"]
        if self.schedule is not None:
            if self.command_started is None and state.get("control_ready", False):
                self.command_started = time.monotonic()
            elapsed = 0.0 if self.command_started is None else time.monotonic() - self.command_started
            state["task_command"] = self.schedule.at(elapsed).tolist()
        elif self.task_command is not None:
            state["task_command"] = self.task_command
        if self.trial and state.get("control_ready", False):
            state["task_command"] = self.trial.command(state, time.monotonic())
        started = time.monotonic()
        frame = self.policy.command(state)
        inference_secs = time.monotonic() - started
        if self.trial and inference_secs > 0.02:
            raise RuntimeError("policy inference deadline exceeded")
        command = JointCommand()
        for key, value in frame.items():
            setattr(command, key, value)
        self.pub.publish(command)
        if self.mode != "sim" or self.trial:
            print(json.dumps({"state": state, "command": frame,
                              "inference_secs": inference_secs,
                              "trial": self.trial.result if self.trial else None}, allow_nan=False), flush=True)

    def on_abort(self, message):
        if self.session_id is None:
            return
        print("Policy abort: " + message.reason, flush=True)
        self.done = True


def main():
    mode = os.environ.get("G1_EXECUTION_MODE", "sim")
    if mode not in ("sim", "dry-run", "hardware"):
        raise RuntimeError("invalid execution mode")
    if mode == "sim" and (os.environ.get("G1_SIM_NETNS") != "1" or {
        name for _, name in socket.if_nameindex()
    } != {"lo"}):
        raise RuntimeError(
            "example container requires the loopback-only simulator network"
        )
    name = os.environ.get("G1_SIM_EXAMPLE") or (
        "balance" if os.path.isfile("/example/project/balance.onnx") else "walker"
    )
    policy = ExamplePolicy(name, "/example/project")
    hashes = Path("/example/project/hashes.json")
    if hashes.exists():
        print("Bundle hashes: " + hashes.read_text(), flush=True)
    rclpy.init()
    node = PolicyNode(policy, mode)
    try:
        deadline = time.monotonic() + 15
        while node.state is None and not node.done and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.state is None or node.done:
            raise RuntimeError("no valid policy state before arming")
        if not node.session_id:
            node.arm()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - node.last_received > (0.2 if node.session_id else 15):
                raise RuntimeError("policy state timeout")
    finally:
        if node.session_id and rclpy.ok():
            abort = Abort()
            abort.source, abort.reason = node.get_name(), "policy_container_exit"
            node.abort_pub.publish(abort)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
