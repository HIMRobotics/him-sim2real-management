"""Run an example policy on the simulation-only ROS state topic."""

import json
import os
import socket
import sys
import time

import rclpy
from controllers import ExamplePolicy
from him_safety_interfaces.msg import Abort, JointCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class PolicyNode(Node):
    def __init__(self, policy):
        super().__init__("example_sim_policy")
        self.policy = policy
        self.done = False
        self.last_observation = 0
        self.session_id = None
        self.last_received = time.monotonic()
        self.pub = self.create_publisher(JointCommand, "/safety/precheck", 1)
        self.create_subscription(
            String,
            "/sim/state",
            self.on_state,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        abort_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Abort, "/safety/abort", self.on_abort, abort_qos)

    def on_state(self, message):
        if self.done:
            return
        state = json.loads(message.data)
        if self.session_id is None:
            self.session_id = state["session_id"]
        if state["session_id"] != self.session_id:
            raise RuntimeError("simulation session changed")
        if state["observation_id"] <= self.last_observation:
            return
        self.last_received = time.monotonic()
        self.last_observation = state["observation_id"]
        frame = self.policy.command(state)
        command = JointCommand()
        for key, value in frame.items():
            setattr(command, key, value)
        self.pub.publish(command)

    def on_abort(self, message):
        print("Simulation abort: " + message.reason, flush=True)
        self.done = True


def main():
    if os.environ.get("G1_SIM_NETNS") != "1" or {
        name for _, name in socket.if_nameindex()
    } != {"lo"}:
        raise RuntimeError(
            "example container requires the loopback-only simulator network"
        )
    policy = ExamplePolicy(os.environ["G1_SIM_EXAMPLE"], "/example/project")
    rclpy.init()
    node = PolicyNode(policy)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - node.last_received > 15:
                raise RuntimeError("no simulation state received for 15 seconds")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
