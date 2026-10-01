"""Publish the walker on /safety/precheck.

The command file is WALKER_COMMAND (vx, vy, yaw_rate). Base velocity comes
from the sim-only /sim/state topic. Does not publish rt/lowcmd or rt/arm_sdk.
"""

import json
import os
import sys
import threading
import time

import rclpy
from him_safety_interfaces.msg import Abort, JointCommand, RobotFeedback
from policy import NUM_JOINTS, WalkerPolicy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

FEEDBACK = "/safety/robot_feedback"
PRECHECK = "/safety/precheck"
ABORT = "/safety/abort"
ARM = "/safety/checker/arm"
SIM_STATE = "/sim/state"
LOW_STATE = "rt/lowstate"
ROOT = "/opt/balance"


def as_floats(values):
    """ROS 2 rejects numpy scalars. The message fields have to be Python floats."""
    return [float(value) for value in values]


class WalkerNode(Node):
    def __init__(self):
        super().__init__("walker_submission")
        command = os.environ.get("WALKER_COMMAND", f"{ROOT}/stand.yaml")
        self.policy = WalkerPolicy(
            f"{ROOT}/walker.onnx", f"{ROOT}/walker.yaml", command
        )
        print(
            "walker command "
            + command
            + " "
            + " ".join(f"{value:.3f}" for value in self.policy.command),
            flush=True,
        )
        self.session_id = None
        self.sequence = 0
        self.done = False
        self.imu = None
        self.base_velocity = [0.0] * 6
        self.imu_lock = threading.Lock()
        self.sdk = None
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
        self.create_subscription(String, SIM_STATE, self.on_sim_state, feedback_qos)
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
        print("armed " + self.session_id, flush=True)
        return True

    def start_lowstate(self):
        interface = os.environ.get("ROBOT_INTERFACE", "")
        domain = int(os.environ.get("ROBOT_DOMAIN", "0"))
        if not interface:
            raise RuntimeError("ROBOT_INTERFACE is not set")
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

        ChannelFactoryInitialize(domain, interface)
        self.sdk = ChannelSubscriber(LOW_STATE, LowState_)
        self.sdk.Init(self.on_lowstate, 10)
        print(f"listening {LOW_STATE} on {interface} domain {domain}", flush=True)

    def on_lowstate(self, message):
        quat = [float(message.imu_state.quaternion[i]) for i in range(4)]
        gyro = [float(message.imu_state.gyroscope[i]) for i in range(3)]
        with self.imu_lock:
            self.imu = (quat, gyro)

    def on_sim_state(self, message):
        velocity = json.loads(message.data).get("base_velocity")
        if velocity is None or len(velocity) != 6:
            return
        with self.imu_lock:
            self.base_velocity = [float(value) for value in velocity]

    def on_abort(self, message):
        print("abort " + message.reason, flush=True)
        self.done = True

    def on_feedback(self, message):
        if self.done or self.session_id is None:
            return
        q = list(message.q)
        dq = list(message.dq)
        if len(q) != NUM_JOINTS or len(dq) != NUM_JOINTS:
            print("feedback did not contain 29 joints", flush=True)
            self.done = True
            return
        with self.imu_lock:
            imu = self.imu
            velocity = list(self.base_velocity)
        if imu is None:
            return
        if self.sequence == 0:
            target = q
        else:
            target = self.policy.targets(q, dq, imu[0], velocity)
        command = JointCommand()
        command.session_id = self.session_id
        command.sequence = self.sequence
        command.observation_id = int(message.observation_id)
        command.q = as_floats(target)
        command.dq = as_floats([0.0] * NUM_JOINTS)
        command.kp = as_floats(self.policy.kp)
        command.kd = as_floats(self.policy.kd)
        command.tau = as_floats([0.0] * NUM_JOINTS)
        self.pub.publish(command)
        self.sequence += 1


def main():
    started = time.monotonic()
    rclpy.init()
    node = WalkerNode()
    try:
        if not node.wait_for_session():
            return 2
        node.start_lowstate()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.sequence == 0 and time.monotonic() - started > 15:
                print("no feedback and IMU before the startup wait ended", flush=True)
                return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
