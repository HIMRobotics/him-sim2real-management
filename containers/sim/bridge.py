"""ROS 2 side of the MuJoCo container test.

The plant runs in the policy-safety Python, which has MuJoCo. This process is
the system ROS 2 Foxy interpreter. They talk over stdin.
"""

import json
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from him_safety_interfaces.msg import Abort, JointCommand, RobotFeedback
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

SIM_DIR = Path(__file__).resolve().parent
REPO = SIM_DIR.parents[1]
sys.path.insert(0, str(REPO / "robot_safety"))
FEEDBACK = "/safety/robot_feedback"
POLICY_OBSERVATION = "/safety/policy_observation"
PRECHECK = "/safety/precheck"
ABORT = "/safety/abort"
ARM = "/safety/checker/arm"


class PlantClient:
    def __init__(self, python):
        env = os.environ.copy()
        env["PYTHONPATH"] = os.pathsep.join(
            [
                str(REPO / "packages/policy_safety/src"),
                str(REPO / "robot_safety"),
                str(SIM_DIR),
                env.get("PYTHONPATH", ""),
            ]
        )
        self.proc = subprocess.Popen(
            [python, str(SIM_DIR / "plant_server.py")],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=sys.stderr,
            text=True,
            cwd=str(REPO),
            env=env,
        )

    def call(self, request):
        if self.proc.poll() is not None:
            raise RuntimeError("mujoco plant exited")
        self.proc.stdin.write(json.dumps(request) + "\n")
        self.proc.stdin.flush()
        line = self.proc.stdout.readline()
        if not line:
            raise RuntimeError("mujoco plant closed its output")
        return json.loads(line)

    def close(self):
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.proc.kill()


class SimBridge(Node):
    def __init__(self, plant, seconds, report_path, lowstate):
        super().__init__("mujoco_sim")
        self.plant = plant
        self.seconds = seconds
        self.report_path = report_path
        self.lowstate = lowstate
        self.created = time.monotonic()
        self.run_started = None
        self.startup = float(os.environ.get("G1_SIM_STARTUP", "10"))
        self.exit_code = 2
        self.finished = False
        feedback_qos = QoSProfile(depth=64, reliability=ReliabilityPolicy.BEST_EFFORT)
        command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        abort_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.feedback_pub = self.create_publisher(RobotFeedback, FEEDBACK, feedback_qos)
        self.observation_pub = self.create_publisher(
            RobotFeedback, POLICY_OBSERVATION, feedback_qos
        )
        self.sim_state_pub = self.create_publisher(String, "/sim/state", feedback_qos)
        self.abort_pub = self.create_publisher(Abort, ABORT, abort_qos)
        self.create_subscription(JointCommand, PRECHECK, self.on_precheck, command_qos)
        self.create_service(Trigger, ARM, self.on_arm)
        self.create_timer(0.02, self.on_control)
        self.create_timer(0.1, self.repeat_abort)
        self.abort_reason = None
        self.accepted = False
        self.circle = os.environ.get("G1_SIM_CIRCLE") == "1"
        if self.circle:
            self.accepted = True
            self.run_started = time.monotonic()

    def clock(self):
        return time.monotonic() - self.created

    def on_arm(self, _request, response):
        result = self.plant.call({"op": "arm"})
        response.success = bool(result.get("ok"))
        response.message = result.get("session_id") or result.get("reason") or ""
        return response

    def on_precheck(self, message):
        if self.finished:
            return
        frame = {
            "session_id": message.session_id,
            "sequence": int(message.sequence),
            "observation_id": int(message.observation_id),
            "q": list(message.q),
            "dq": list(message.dq),
            "kp": list(message.kp),
            "kd": list(message.kd),
            "tau": list(message.tau),
        }
        result = self.plant.call(
            {"op": "command", "time": self.clock(), "frame": frame}
        )
        if result.get("ok"):
            self.accepted = True
            if self.run_started is None:
                self.run_started = time.monotonic()
        else:
            self.latch(result.get("reason") or "invalid_command")

    def on_control(self):
        if self.finished:
            return
        if self.run_started is None:
            if time.monotonic() - self.created >= self.startup:
                self.finish()
                return
        elif time.monotonic() - self.run_started >= self.seconds:
            self.finish()
            return
        if self.circle:
            stepped = self.plant.call({"op": "circle", "time": self.clock()})
            if stepped.get("reason"):
                self.latch(stepped["reason"])
            state = self.plant.call({"op": "feedback", "time": self.clock()})
            if not state.get("ok"):
                self.latch(state.get("reason") or "plant_error")
                self.finish()
                return
            self.publish_state(state)
            return
        if self.accepted:
            stepped = self.plant.call({"op": "step", "time": self.clock()})
            if stepped.get("reason"):
                self.latch(stepped["reason"])
        state = self.plant.call({"op": "feedback", "time": self.clock()})
        if not state.get("ok"):
            self.latch(state.get("reason") or "plant_error")
            self.finish()
            return
        self.publish_state(state)
        self.lowstate.publish(state)

    def publish_state(self, state):
        message = RobotFeedback()
        message.host_boot_id = "mujoco-sim"
        message.received_at_secs = self.clock()
        message.observation_id = int(state["observation_id"])
        message.q = list(state["q"])
        message.dq = list(state["dq"])
        message.start_button_pressed = False
        self.feedback_pub.publish(message)
        self.observation_pub.publish(message)
        sim_message = String()
        sim_message.data = json.dumps(state)
        self.sim_state_pub.publish(sim_message)

    def latch(self, reason):
        if self.abort_reason is None:
            self.abort_reason = reason
            self.repeat_abort()

    def repeat_abort(self):
        if self.abort_reason is None:
            return
        message = Abort()
        message.source = "mujoco_sim"
        message.reason = self.abort_reason
        self.abort_pub.publish(message)

    def finish(self):
        if self.finished:
            return
        self.finished = True
        report = self.plant.call(
            {"op": "report", "lowstate": bool(getattr(self.lowstate, "ok", False))}
        )
        report.update(
            execution="docker_ros", policy=os.environ.get("G1_SIM_EXAMPLE") or None
        )
        self.report_path.parent.mkdir(parents=True, exist_ok=True)
        self.report_path.write_text(json.dumps(report, indent=2) + "\n")
        status = report.get("status")
        self.exit_code = {"within_limits": 0, "completed": 0, "violation": 1}.get(
            status, 2
        )
        print(
            f"sim {status} reason={report.get('reason')} commands={report.get('commands')}",
            flush=True,
        )


def _routable_interfaces():
    return sorted(
        name
        for _, name in socket.if_nameindex()
        if name != "lo" and name.startswith(("eth", "en", "wl"))
    )


def require_isolated_netns():
    if os.environ.get("G1_SIM_NETNS") != "1":
        print(
            "Refusing to start the sim bridge outside the isolated netns.",
            file=sys.stderr,
        )
        return False
    extra = _routable_interfaces()
    if extra:
        print(
            "Refusing to start: routable interfaces are visible: " + ", ".join(extra),
            file=sys.stderr,
        )
        return False
    return True


def main():
    if not require_isolated_netns():
        return 2
    python = os.environ.get(
        "G1_SIM_PYTHON",
        str(REPO / ".venv-policy-safety/bin/python"),
    )
    seconds = float(os.environ.get("G1_SIM_SECONDS", "10"))
    report = Path(os.environ.get("G1_SIM_REPORT", str(SIM_DIR / "output/report.json")))
    ready = Path(os.environ.get("G1_SIM_READY", str(SIM_DIR / "output/ready")))
    from lowstate import SimLowState

    lowstate = SimLowState(
        os.environ.get("ROBOT_DOMAIN", "0"), os.environ.get("ROBOT_INTERFACE", "lo")
    )
    plant = PlantClient(python)
    armed = plant.call({"op": "arm"})
    if not armed.get("ok"):
        plant.close()
        print(armed, file=sys.stderr)
        return 2
    rclpy.init()
    node = SimBridge(plant, seconds, report, lowstate)
    ready.parent.mkdir(parents=True, exist_ok=True)
    ready.write_text("ready\n")
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.05)
        code = node.exit_code
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plant.close()
    return code


if __name__ == "__main__":
    sys.exit(main())
