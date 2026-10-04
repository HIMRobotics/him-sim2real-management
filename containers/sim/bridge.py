"""ROS 2 side of the MuJoCo container test.

The plant runs in the policy-safety Python, which has MuJoCo. This process is
the system ROS 2 Foxy interpreter. They talk over stdin.
"""

import json
import os
import select
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
    def __init__(self, python, timeout=10.0):
        self.timeout = timeout
        self.failed = False
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
            bufsize=0,
            cwd=str(REPO),
            env=env,
        )

        os.set_blocking(self.proc.stdin.fileno(), False)
        os.set_blocking(self.proc.stdout.fileno(), False)

    def call(self, request):
        if self.failed or self.proc.poll() is not None:
            raise RuntimeError("mujoco plant unavailable")
        deadline = time.monotonic() + self.timeout
        pending = (json.dumps(request) + "\n").encode()
        response = bytearray()
        try:
            while pending or b"\n" not in response:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("mujoco plant response timed out")
                readable, writable, _ = select.select(
                    [] if pending else [self.proc.stdout],
                    [self.proc.stdin] if pending else [],
                    [],
                    remaining,
                )
                if writable:
                    pending = pending[os.write(self.proc.stdin.fileno(), pending) :]
                if readable:
                    chunk = os.read(self.proc.stdout.fileno(), 65536)
                    if not chunk:
                        raise RuntimeError("mujoco plant closed its output")
                    response.extend(chunk)
            return json.loads(response)
        except (OSError, ValueError, RuntimeError):
            # Never reuse a stream after a partial or late response.
            self.failed = True
            raise

    def close(self):
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        self.proc.stdin.close()
        self.proc.stdout.close()


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
        request = {"op": "feedback", "time": self.clock()}
        if self.circle or self.accepted:
            request.update(
                op="circle" if self.circle else "step",
                feedback=True,
                clock_origin=self.created,
            )
        state = self.plant.call(request)
        if state.get("reason"):
            self.latch(state["reason"])
        if not state.get("ok"):
            self.latch(state.get("reason") or "plant_error")
            self.finish()
            return
        self.publish_state(state)
        if not self.circle:
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

    def finish(self, error=None):
        if self.finished:
            return
        if error is None:
            report = self.plant.call(
                {"op": "report", "lowstate": bool(getattr(self.lowstate, "ok", False))}
            )
        else:
            self.latch("plant_error")
            report = {
                "version": 1,
                "status": "error",
                "reason": "plant_error",
                "detail": str(error),
                "hardware_authorization": False,
            }
        self.finished = True
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


def require_isolated_netns():
    if os.environ.get("G1_SIM_NETNS") != "1":
        print(
            "Refusing to start the sim bridge outside the isolated netns.",
            file=sys.stderr,
        )
        return False
    if {name for _, name in socket.if_nameindex()} != {"lo"}:
        print(
            "Refusing to start: the sim network must contain only lo.", file=sys.stderr
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
    node = None
    try:
        armed = plant.call({"op": "arm"})
        if not armed.get("ok"):
            print(armed, file=sys.stderr)
            return 2
        rclpy.init()
        node = SimBridge(plant, seconds, report, lowstate)
        ready.parent.mkdir(parents=True, exist_ok=True)
        ready.write_text("ready\n")
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.05)
        code = node.exit_code
    except (OSError, ValueError, RuntimeError) as exc:
        if node is not None:
            node.finish(error=exc)
        else:
            print(str(exc), file=sys.stderr)
        code = 2
    finally:
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()
        plant.close()
    return code


if __name__ == "__main__":
    sys.exit(main())
