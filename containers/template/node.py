"""ROS side of the submission contract. Edit policy.py, not this file."""

import json
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from him_safety_interfaces.msg import Abort, JointCommand
from him_safety_interfaces.srv import ArmGate
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from policy import KD, KP, Policy

JOINTS = 29
STATE_TIMEOUT = 0.2


class SubmissionNode(Node):
    def __init__(self, policy, mode):
        super().__init__("submission_policy")
        self.policy = policy
        self.mode = mode
        self.state = None
        self.last_received = time.monotonic()
        self.last_observation = 0
        self.sequence = 0
        self.session_id = None
        self.done = False
        self.pub = self.create_publisher(JointCommand, "/safety/precheck", 1)
        self.create_subscription(
            String, "/safety/policy_state", self.on_state,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        abort_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Abort, "/safety/abort", self.on_abort, abort_qos)
        self.abort_pub = self.create_publisher(Abort, "/safety/abort", abort_qos)
        self.create_timer(0.02, self.publish_command)

    def call(self, service, kind, request, timeout=5.0):
        client = self.create_client(kind, service)
        try:
            if not client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError("missing service: " + service)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            result = future.result() if future.done() else None
            if result is None or not result.success:
                detail = "" if result is None else result.message
                raise RuntimeError("rejected by " + service + (": " + detail if detail else ""))
            return result.message
        finally:
            self.destroy_client(client)

    def arm(self):
        # The checker's action watchdog starts here: the next command must
        # arrive within 0.2 s, which the 20 ms timer provides.
        if self.mode != "sim":
            self.call("/safety/controller_stop/ready", Trigger, Trigger.Request())
            self.call("/safety/motor_gate/ready", Trigger, Trigger.Request(), timeout=10.0)
        session = self.call("/safety/checker/arm", Trigger, Trigger.Request())
        if self.mode != "sim":
            request = ArmGate.Request()
            request.session_id = session
            self.call("/safety/motor_gate/arm", ArmGate, request)
        self.session_id = session
        print("Armed session " + session, flush=True)

    def on_state(self, message):
        if not self.done:
            self.state = json.loads(message.data)
            self.last_received = time.monotonic()

    def publish_command(self):
        if self.done or not self.session_id or self.state is None:
            return
        state = self.state
        if state["observation_id"] <= self.last_observation:
            return
        self.last_observation = state["observation_id"]
        q = np.asarray(self.policy.act(state), dtype=float)
        if q.shape != (JOINTS,) or not np.all(np.isfinite(q)):
            raise RuntimeError("Policy.act must return 29 finite joint targets")
        command = JointCommand()
        command.session_id = self.session_id
        command.sequence = self.sequence
        command.observation_id = state["observation_id"]
        command.q = q.tolist()
        command.dq = [0.0] * JOINTS
        command.kp = [float(v) for v in KP]
        command.kd = [float(v) for v in KD]
        command.tau = [0.0] * JOINTS
        self.pub.publish(command)
        self.sequence += 1

    def on_abort(self, message):
        if self.session_id is not None:
            print("Abort from " + message.source + ": " + message.reason, flush=True)
            self.done = True


def main():
    mode = os.environ.get("G1_EXECUTION_MODE", "sim")
    if mode not in ("sim", "dry-run", "hardware"):
        raise RuntimeError("invalid G1_EXECUTION_MODE: " + mode)
    policy = Policy(Path(__file__).resolve().parent / "assets")
    rclpy.init()
    node = SubmissionNode(policy, mode)
    try:
        deadline = time.monotonic() + 15
        while node.state is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.state is None:
            raise RuntimeError("no /safety/policy_state within 15 s")
        node.arm()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.02)
            if time.monotonic() - node.last_received > STATE_TIMEOUT:
                print("Policy state stopped; exiting", flush=True)
                break
    finally:
        if node.session_id and not node.done and rclpy.ok():
            abort = Abort()
            abort.source, abort.reason = node.get_name(), "policy_container_exit"
            node.abort_pub.publish(abort)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
