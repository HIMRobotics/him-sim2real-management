"""Publish a damped hold of the measured pose. Used by ./sim.sh --self-test."""

import sys

import rclpy
from him_safety_interfaces.msg import Abort, JointCommand, RobotFeedback
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

FEEDBACK = "/safety/robot_feedback"
PRECHECK = "/safety/precheck"
ABORT = "/safety/abort"
ARM = "/safety/checker/arm"


class Hold(Node):
    def __init__(self):
        super().__init__("mujoco_hold")
        self.session_id = None
        self.hold_q = None
        self.sequence = 0
        self.done = False
        feedback_qos = QoSProfile(depth=64, reliability=ReliabilityPolicy.BEST_EFFORT)
        command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        abort_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(JointCommand, PRECHECK, command_qos)
        self.create_subscription(RobotFeedback, FEEDBACK, self.on_feedback, feedback_qos)
        self.create_subscription(Abort, ABORT, self.on_abort, abort_qos)
        self.arm = self.create_client(Trigger, ARM)

    def wait_for_session(self):
        if not self.arm.wait_for_service(timeout_sec=10.0):
            print("arm service did not appear", flush=True)
            return False
        future = self.arm.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result is None or not result.success or not result.message:
            print("arm failed", flush=True)
            return False
        self.session_id = result.message
        return True

    def on_abort(self, message):
        print(f"abort {message.reason}", flush=True)
        self.done = True

    def on_feedback(self, message):
        if self.session_id is None or self.done:
            return
        if self.hold_q is None:
            self.hold_q = list(message.q)
        command = JointCommand()
        command.session_id = self.session_id
        command.sequence = self.sequence
        command.observation_id = message.observation_id
        command.q = list(self.hold_q)
        command.dq = [0.0] * len(message.q)
        command.kp = [40.0] * len(message.q)
        command.kd = [2.0] * len(message.q)
        command.tau = [0.0] * len(message.q)
        self.pub.publish(command)
        self.sequence += 1


def main():
    rclpy.init()
    node = Hold()
    try:
        if not node.wait_for_session():
            return 2
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
