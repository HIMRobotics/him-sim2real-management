"""ROS 2 side of the MuJoCo container test.

The plant runs in the policy-safety Python, which has MuJoCo. This process is
the system ROS 2 Foxy interpreter. They talk over stdin.
"""

import json
import os
import select
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from him_safety_interfaces.msg import Abort, CheckedCommand, JointCommand, RobotFeedback
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

SIM_DIR = Path(__file__).resolve().parent
REPO = SIM_DIR.parents[1]
sys.path.insert(0, str(REPO / "robot_safety"))
sys.path.insert(0, str(REPO))
from containers.sim.netns import loopback_only
from robot_safety.motor_gate import MotorCommandGate

FEEDBACK = "/safety/robot_feedback"
POLICY_OBSERVATION = "/safety/policy_observation"
PRECHECK = "/safety/precheck"
POSTCHECK = "/safety/postcheck"
ABORT = "/safety/abort"
ARM = "/safety/checker/arm"
# The first call includes plant startup, which reads meshes and weights
# through Docker Desktop's bind mount and is slow on a cold cache.
PLANT_STARTUP_SECONDS = 30.0
# The safety deadlines are 0.2 s. A gap this long between the bridge's own
# 20 ms control ticks means Docker or the host stopped running the simulator,
# for example while macOS swaps the Docker VM out.
HOST_STALL_SECONDS = 0.1
# Messages delayed by a stall can still trip a deadline after the loop resumes.
HOST_STALL_REACH_SECONDS = 0.5


def error_report(error):
    return {
        "version": 1,
        "status": "error",
        "reason": "plant_error",
        "detail": str(error),
        "hardware_authorization": False,
        "execution": "docker_ros",
        "policy": os.environ.get("G1_SIM_EXAMPLE") or None,
    }


def write_report(path, report):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2) + "\n")


class PlantClient:
    def __init__(self, python, timeout=10.0, *, external_checker=False):
        self.timeout = timeout
        self.failed = False
        env = os.environ.copy()
        if external_checker:
            env["G1_SIM_THRASH_MODE"] = "off"
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

    def call(self, request, timeout=None):
        if self.failed or self.proc.poll() is not None:
            raise RuntimeError("mujoco plant unavailable")
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
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
    def __init__(self, plant, seconds, report_path, lowstate, checker=None):
        super().__init__("mujoco_sim")
        self.plant = plant
        self.seconds = seconds
        self.report_path = report_path
        self.lowstate = lowstate
        self.checker = checker
        self.motor_gate = MotorCommandGate(checker.host_boot_id, checker.config) if checker else None
        self.pending_states = {}
        self.observation_map = {}
        self.auto_arm = bool(os.environ.get("G1_SIM_EXAMPLE"))
        self.created = time.monotonic()
        self.run_started = None
        self.startup = float(os.environ.get("G1_SIM_STARTUP", "10"))
        self.exit_code = 2
        self.finished = False
        self.last_tick = None
        self.stalls = []
        self.abort_at = None
        feedback_qos = QoSProfile(depth=64, reliability=ReliabilityPolicy.BEST_EFFORT)
        command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        abort_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.feedback_pub = self.create_publisher(RobotFeedback, FEEDBACK, feedback_qos)
        if checker is None:
            self.observation_pub = self.create_publisher(
                RobotFeedback, POLICY_OBSERVATION, feedback_qos
            )
        else:
            self.create_subscription(
                RobotFeedback, POLICY_OBSERVATION, self.on_observation, feedback_qos
            )
        self.sim_state_pub = self.create_publisher(String, "/sim/state", feedback_qos)
        self.policy_state_pub = self.create_publisher(String, "/safety/policy_state", feedback_qos)
        self.abort_pub = self.create_publisher(Abort, ABORT, abort_qos)
        if checker is None:
            self.create_subscription(JointCommand, PRECHECK, self.on_precheck, command_qos)
            self.create_service(Trigger, ARM, self.on_arm)
        else:
            self.create_subscription(CheckedCommand, POSTCHECK, self.on_postcheck, command_qos)
        self.create_subscription(Abort, ABORT, self.on_abort, abort_qos)
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

    def on_observation(self, message):
        """Attach checker-issued command provenance to the extended sim state."""
        state = self.pending_states.pop(message.received_at_secs, None)
        if state is None or self.finished or self.abort_reason:
            return
        supervisor = self.checker.supervisor
        if supervisor.stop_reason:
            self.latch(supervisor.stop_reason)
            return
        if self.auto_arm and supervisor.policy_session_id is None:
            if (self.policy_state_pub.get_subscription_count() == 0
                    and self.sim_state_pub.get_subscription_count() == 0):
                return
            response = self.checker.on_arm(Trigger.Request(), Trigger.Response())
            if not response.success:
                self.latch(response.message)
                return
        self.observation_map[message.observation_id] = (
            state["session_id"], state["observation_id"]
        )
        while len(self.observation_map) > 512:
            del self.observation_map[next(iter(self.observation_map))]
        state = dict(state, observation_id=message.observation_id,
                     session_id=supervisor.policy_session_id or "")
        sim_message = String()
        sim_message.data = json.dumps(state)
        self.sim_state_pub.publish(sim_message)
        self.policy_state_pub.publish(sim_message)

    def on_postcheck(self, message):
        """Apply only a fresh checked command accepted by the shared motor gate."""
        from robot_safety.ros_nodes import command_from_ros

        if self.finished or self.abort_reason:
            return
        now = time.monotonic()
        gate = self.motor_gate
        if gate.policy_session_id is None:
            session = self.checker.supervisor.policy_session_id
            if not session or message.command.session_id != session:
                self.latch("invalid_postcheck_session")
                return
            gate.arm(session, now)
        if not gate.receive(command_from_ros(message.command), message.host_boot_id,
                            message.checked_at_secs, message.expires_at_secs, now):
            self.latch(gate.abort_reason or "invalid_postcheck")
            return
        source = self.observation_map.get(message.command.observation_id)
        if source is None:
            received = self.checker.supervisor.observation_received_times_secs.get(
                message.command.observation_id
            )
            state = self.pending_states.get(received)
            if state is not None:
                source = (state["session_id"], state["observation_id"])
        if source is None:
            self.latch("unknown_sim_observation")
            return
        command = message.command
        frame = {key: list(getattr(command, key)) for key in ("q", "dq", "kp", "kd", "tau")}
        frame.update(session_id=source[0], sequence=int(command.sequence), observation_id=source[1])
        result = self.plant.call({"op": "command", "time": self.clock(), "frame": frame})
        if result.get("ok"):
            self.accepted = True
            if self.run_started is None:
                self.run_started = now
        else:
            self.latch(result.get("reason") or "invalid_command")

    def on_abort(self, message):
        if message.source != "mujoco_sim" and not self.finished:
            self.latch(message.reason or "external_abort")

    def on_control(self):
        if self.finished:
            return
        now = time.monotonic()
        if self.last_tick is not None and now - self.last_tick >= HOST_STALL_SECONDS:
            self.stalls.append((self.last_tick, now))
        self.last_tick = now
        if self.motor_gate is not None:
            self.motor_gate.cycle(time.monotonic())
            if self.motor_gate.abort_reason:
                self.latch(self.motor_gate.abort_reason)
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
            if self.checker:
                request["thrash"] = self.checker.supervisor.thrash_detector.snapshot()
        state = self.plant.call(request)
        if state.get("reason"):
            self.latch(state["reason"], state.get("detail"))
        if not state.get("ok"):
            self.latch(state.get("reason") or "plant_error")
            self.finish()
            return
        self.publish_state(state)
        if not self.circle:
            self.lowstate.publish(state)

    def publish_state(self, state):
        state = dict(state, received_at_secs=time.monotonic(),
                     control_ready=bool(state.get("control_ready", False)))
        message = RobotFeedback()
        message.host_boot_id = self.checker.host_boot_id if self.checker else "mujoco-sim"
        message.received_at_secs = time.monotonic() if self.checker else self.clock()
        message.observation_id = 0 if self.checker else int(state["observation_id"])
        message.q = list(state["q"])
        message.dq = list(state["dq"])
        message.start_button_pressed = False
        if self.checker:
            self.pending_states[message.received_at_secs] = state
            while len(self.pending_states) > 64:
                del self.pending_states[next(iter(self.pending_states))]
            self.feedback_pub.publish(message)
            return
        self.feedback_pub.publish(message)
        self.observation_pub.publish(message)
        sim_message = String()
        sim_message.data = json.dumps(state)
        self.sim_state_pub.publish(sim_message)
        self.policy_state_pub.publish(sim_message)

    def latch(self, reason, detail=None):
        if self.abort_reason is None:
            self.abort_reason = reason
            self.abort_at = time.monotonic()
            if self.motor_gate is not None:
                self.motor_gate.abort(reason)
            if not self.plant.failed:
                if detail is None and self.checker and self.checker.supervisor.stop_reason == reason:
                    detail = self.checker.supervisor.stop_detail
                self.plant.call({"op": "abort", "reason": reason, "detail": detail})
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
            thrash = None
            if self.checker:
                thrash = self.checker.supervisor.thrash_detector.snapshot()
            report = self.plant.call(
                {"op": "report", "lowstate": bool(getattr(self.lowstate, "ok", False)),
                 "thrash": thrash}
            )
        else:
            self.latch("plant_error")
            report = error_report(error)
        self.finished = True
        report.update(
            execution="docker_ros", policy=os.environ.get("G1_SIM_EXAMPLE") or None
        )
        stall = self.host_stall()
        if stall is not None:
            report["host_stall"] = {"longest_gap_secs": round(stall, 3)}
        write_report(self.report_path, report)
        status = report.get("status")
        self.exit_code = {"within_limits": 0, "completed": 0, "violation": 1}.get(
            status, 2
        )
        print(
            f"sim {status} reason={report.get('reason')} commands={report.get('commands')}",
            flush=True,
        )
        if stall is not None:
            print(f"The simulator was stalled for {stall:.2f} s at the stop, so this "
                  "result is inconclusive. Free memory on the host and rerun.", flush=True)

    def host_stall(self):
        """Longest stall that could have caused the stop, in seconds."""
        if self.abort_at is None:
            return None
        gaps = [end - start for start, end in self.stalls
                if start <= self.abort_at and end >= self.abort_at - HOST_STALL_REACH_SECONDS]
        return max(gaps, default=None)


def require_isolated_netns():
    if os.environ.get("G1_SIM_NETNS") != "1":
        print(
            "Refusing to start the sim bridge outside the isolated netns.",
            file=sys.stderr,
        )
        return False
    if not loopback_only():
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
    shared_checker = os.environ.get("G1_SIM_SAFETY_CHECKS", "1") == "1" and (
        os.environ.get("G1_SIM_CIRCLE") != "1"
    )
    plant = PlantClient(python, external_checker=shared_checker)
    node = None
    checker = None
    executor = None
    try:
        armed = plant.call({"op": "arm"}, timeout=PLANT_STARTUP_SECONDS)
        if not armed.get("ok"):
            print(armed, file=sys.stderr)
            write_report(report, error_report(armed.get("detail") or armed.get("reason")))
            return 2
        rclpy.init(args=["--ros-args", "-p", "thrash.mode:=" + (os.environ.get("G1_SIM_THRASH_MODE") or "monitor")])
        from rclpy.executors import SingleThreadedExecutor

        from robot_safety.ros_nodes import SafetyCheckerNode

        checker = SafetyCheckerNode() if shared_checker else None
        node = SimBridge(plant, float(armed.get("launch_seconds", 0.0)) + seconds,
                         report, lowstate, checker)
        executor = SingleThreadedExecutor()
        if checker:
            executor.add_node(checker)
        executor.add_node(node)
        while checker and checker.supervisor.latest_robot_observation is None:
            executor.spin_once(timeout_sec=0.005)
            if time.monotonic() - node.created >= node.startup or checker.abort_reason:
                raise RuntimeError("simulation checker did not receive fresh feedback")
        ready.parent.mkdir(parents=True, exist_ok=True)
        ready.write_text("ready\n")
        while rclpy.ok() and not node.finished:
            executor.spin_once(timeout_sec=0.005)
        code = node.exit_code
    except (OSError, ValueError, RuntimeError) as exc:
        if node is not None:
            node.finish(error=exc)
        else:
            print(str(exc), file=sys.stderr)
            write_report(report, error_report(exc))
        code = 2
    finally:
        if executor is not None:
            executor.shutdown()
        if checker is not None:
            checker.destroy_node()
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()
        plant.close()
    return code


if __name__ == "__main__":
    sys.exit(main())
