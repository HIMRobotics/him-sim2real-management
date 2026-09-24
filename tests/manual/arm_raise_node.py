"""Send a selected arm test through precheck and stop permanently on abort."""

import math
import signal
import time
from enum import Enum

import rclpy
from him_safety_interfaces.msg import Abort, JointCommand, RobotFeedback
from him_safety_interfaces.srv import ArmGate
from std_srvs.srv import Trigger

from robot_safety import ros_topics as topics
from robot_safety.feedback import G1_29_JOINT_NAMES
from robot_safety.hardware_profile import READINESS_CHECK_TIMEOUT_SECS
from robot_safety.ros_nodes import SafetyNode, command_to_ros

from .arm_raise import ArmRaiseSettings, SlowArmRaise
from .small_elbow_step import LEFT_ELBOW_INDEX, SmallElbowSettings, SmallElbowStep
from .unitree_arm_sequence import UnitreeArmSequence, UnitreeArmSettings

ARM_RAISE_START = "/safety/arm_raise/start"


class ArmRaisePhase(Enum):
    """One-shot test lifecycle, with no automatic rearm or reset."""

    WAITING = "waiting"
    CHECKING_GATE = "checking_gate"
    CHECKING_CONTROLLER = "checking_controller"
    ARMING_CHECKER = "arming_checker"
    ARMING_GATE = "arming_gate"
    HOLDING = "holding"
    RUNNING = "running"
    STOPPED = "stopped"


class ArmRaiseNode(SafetyNode):
    """Publish arm-test proposals only after explicit controller and gate arming."""

    def __init__(self):
        super().__init__("dry_run_arm_raise")
        if self.config.joints != len(G1_29_JOINT_NAMES):
            raise ValueError("arm raise requires 29 joints")
        self.test_policy = self.declare_parameter("test_policy", "slow_raise").value
        if self.test_policy == "small_elbow":
            self.settings = SmallElbowSettings()
            self.command_period_secs = self.settings.command_period_secs
        elif self.test_policy == "unitree_arm7":
            self.settings = UnitreeArmSettings()
            self.command_period_secs = self.settings.command_period_secs
        elif self.test_policy == "slow_raise":
            defaults = ArmRaiseSettings()
            self.settings = ArmRaiseSettings(
                angle_rad=self.declare_parameter(
                    "raise_angle_rad", defaults.angle_rad
                ).value,
                duration_secs=self.declare_parameter(
                    "raise_duration_secs", defaults.duration_secs
                ).value,
            )
            self.command_period_secs = self.config.period
        else:
            raise ValueError(
                "test_policy must be slow_raise, unitree_arm7 or small_elbow"
            )
        self.handover_timeout_secs = self.declare_parameter(
            "handover_timeout_secs", 5.0
        ).value
        if (
            not math.isfinite(self.handover_timeout_secs)
            or self.handover_timeout_secs <= 0
        ):
            raise ValueError("handover timeout must be finite and positive")
        self.pending_handover_request = None
        self.phase = ArmRaisePhase.WAITING
        self.latest_observation = None
        self.policy_session_id = None
        self.command_sequence = 0
        self.trajectory = None
        self.handover_command = None
        self.time_started_secs = None
        self.time_last_command_secs = None
        self.pending_arm_request = None
        self.time_arm_requested_secs = None
        self.command_publisher = self.create_publisher(
            JointCommand, topics.PRECHECK_COMMAND, self.command_qos
        )
        self.checker_arm_client = self.create_client(Trigger, topics.CHECKER_ARM)
        self.gate_ready_client = self.create_client(Trigger, topics.GATE_READY)
        self.handover_client = self.create_client(ArmGate, topics.HANDOVER_READY)
        self.gate_arm_client = self.create_client(ArmGate, topics.GATE_ARM)
        self.controller_ready_client = self.create_client(
            Trigger, topics.CONTROLLER_READY
        )
        self.readiness_reported = False
        self.create_subscription(
            RobotFeedback,
            topics.POLICY_OBSERVATION,
            self.on_observation,
            self.feedback_qos,
        )
        self.create_subscription(Abort, topics.ABORT, self.on_abort, self.abort_qos)
        self.create_service(Trigger, ARM_RAISE_START, self.on_start)
        self.create_timer(self.command_period_secs, self.publish_command)

    def on_observation(self, message):
        """Retain the latest checker-assigned observation for command freshness."""
        self.latest_observation = message

    def on_abort(self, message):
        self.stop_test(message.reason or "external_abort")

    def stop_test(self, reason):
        """Latch abort, cancel pending arming and prohibit subsequent proposals."""
        self.phase = ArmRaisePhase.STOPPED
        if self.pending_arm_request is not None:
            self.pending_arm_request.cancel()
            self.pending_arm_request = None
        if self.pending_handover_request is not None:
            self.pending_handover_request.cancel()
            self.pending_handover_request = None
        self.latch_abort(reason)

    def observation_is_ready(self):
        message = self.latest_observation
        return (
            message is not None
            and self.feedback_is_fresh(message, time.monotonic())
            and message.observation_id > 0
            and not message.start_button_pressed
        )

    def on_start(self, _request, response):
        """Begin asynchronous arming once peers and released-Start feedback exist."""
        if (
            self.phase != ArmRaisePhase.WAITING
            or not self.observation_is_ready()
            or not self.checker_arm_client.service_is_ready()
            or not self.gate_ready_client.service_is_ready()
            or (
                self.test_policy in ("small_elbow", "unitree_arm7")
                and not self.handover_client.service_is_ready()
            )
            or not self.gate_arm_client.service_is_ready()
            or not self.controller_ready_client.service_is_ready()
            or self.command_publisher.get_subscription_count() == 0
        ):
            response.success = False
            response.message = (
                "Need fresh released-Start feedback, peers and an unused test node"
            )
            return response
        self.phase = ArmRaisePhase.CHECKING_GATE
        self.time_arm_requested_secs = time.monotonic()
        self.pending_arm_request = self.gate_ready_client.call_async(Trigger.Request())
        response.success = True
        response.message = (
            "Arming requested for the selected motor gate. Any failure latches abort."
        )
        return response

    def advance_arming(self, cur_time_secs):
        """Arm checker then gate without blocking feedback or abort callbacks."""
        timeout = (
            READINESS_CHECK_TIMEOUT_SECS
            if self.phase == ArmRaisePhase.CHECKING_GATE
            else self.config.action_timeout
        )
        if cur_time_secs - self.time_arm_requested_secs >= timeout:
            self.stop_test("arm_raise_arming_timeout")
            return
        if not self.pending_arm_request.done():
            return
        try:
            response = self.pending_arm_request.result()
            if not response.success:
                self.stop_test("arm_raise_arming_rejected")
                return
            if self.phase == ArmRaisePhase.CHECKING_GATE:
                self.time_arm_requested_secs = cur_time_secs
                self.pending_arm_request = self.controller_ready_client.call_async(
                    Trigger.Request()
                )
                self.phase = ArmRaisePhase.CHECKING_CONTROLLER
            elif self.phase == ArmRaisePhase.CHECKING_CONTROLLER:
                self.pending_arm_request = self.checker_arm_client.call_async(
                    Trigger.Request()
                )
                self.phase = ArmRaisePhase.ARMING_CHECKER
            elif self.phase == ArmRaisePhase.ARMING_CHECKER:
                self.policy_session_id = response.message
                request = ArmGate.Request()
                request.session_id = self.policy_session_id
                self.pending_arm_request = self.gate_arm_client.call_async(request)
                self.phase = ArmRaisePhase.ARMING_GATE
            else:
                if self.test_policy == "unitree_arm7":
                    self.trajectory = UnitreeArmSequence(self.settings)
                elif self.test_policy == "small_elbow":
                    self.trajectory = SmallElbowStep(
                        self.latest_observation.q, self.settings
                    )
                else:
                    self.trajectory = SlowArmRaise(
                        self.latest_observation.q, self.settings
                    )
                # Freeze the starting targets throughout ownership transfer.
                self.handover_command = (
                    self.trajectory.command_at(0.0, self.latest_observation.q)
                    if self.test_policy == "unitree_arm7"
                    else self.trajectory.command_at(0.0)
                )
                self.time_started_secs = cur_time_secs
                self.time_last_command_secs = cur_time_secs
                self.phase = (
                    ArmRaisePhase.HOLDING
                    if self.test_policy in ("small_elbow", "unitree_arm7")
                    else ArmRaisePhase.RUNNING
                )
                self.pending_arm_request = None
                self.get_logger().info(
                    f"Arm test: {self.test_policy}, phase={self.phase.value}. Double Start requests abort."
                )
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"Arm raise initialization failed: {error}")
            # RPC or malformed feedback failures must never leave a partial test armed.
            self.stop_test("arm_raise_start_failed")

    def publish_command(self):
        """Send fresh proposals until abort, completion, stale state or a loop stall."""
        if self.phase == ArmRaisePhase.WAITING:
            ready = (
                self.observation_is_ready()
                and self.checker_arm_client.service_is_ready()
                and self.gate_ready_client.service_is_ready()
                and (
                    self.test_policy not in ("small_elbow", "unitree_arm7")
                    or self.handover_client.service_is_ready()
                )
                and self.gate_arm_client.service_is_ready()
                and self.controller_ready_client.service_is_ready()
                and self.command_publisher.get_subscription_count() > 0
            )
            if ready and not self.readiness_reported:
                self.get_logger().info(
                    "Feedback and services discovered. Ready for explicit start request. "
                    "Controller readiness will be checked before arming."
                )
            self.readiness_reported = ready
            return
        if self.phase == ArmRaisePhase.STOPPED:
            return
        message = self.latest_observation
        # Start may be held during a double press, so only require release at arming.
        if message is None or not self.feedback_is_fresh(message, time.monotonic()):
            self.stop_test("arm_raise_feedback_timeout")
            return
        cur_time_secs = time.monotonic()
        if self.phase in (
            ArmRaisePhase.CHECKING_GATE,
            ArmRaisePhase.CHECKING_CONTROLLER,
            ArmRaisePhase.ARMING_CHECKER,
            ArmRaisePhase.ARMING_GATE,
        ):
            self.advance_arming(cur_time_secs)
            if self.phase not in (ArmRaisePhase.HOLDING, ArmRaisePhase.RUNNING):
                return
        if (
            cur_time_secs - self.time_last_command_secs
            >= self.settings.max_publish_gap_secs
        ):
            # Stop instead of jumping forward along the trajectory after a delayed timer.
            self.stop_test("arm_raise_publish_gap")
            return
        if self.phase == ArmRaisePhase.HOLDING:
            if cur_time_secs - self.time_started_secs >= self.handover_timeout_secs:
                self.stop_test("arm_test_handover_timeout")
                return
            if (
                self.pending_handover_request is not None
                and self.pending_handover_request.done()
            ):
                try:
                    complete = self.pending_handover_request.result().success
                except Exception:  # noqa: BLE001
                    self.stop_test("arm_test_handover_query_failed")
                    return
                self.pending_handover_request = None
                if complete:
                    self.phase = ArmRaisePhase.RUNNING
                    self.time_started_secs = cur_time_secs
                    self.get_logger().info(
                        f"Full ownership weight sent. Starting {self.test_policy} trajectory at zero seconds."
                    )
            if (
                self.phase == ArmRaisePhase.HOLDING
                and self.pending_handover_request is None
            ):
                request = ArmGate.Request()
                request.session_id = self.policy_session_id
                self.pending_handover_request = self.handover_client.call_async(request)
        elapsed_secs = (
            0.0
            if self.phase == ArmRaisePhase.HOLDING
            else cur_time_secs - self.time_started_secs
        )
        if elapsed_secs >= self.settings.duration_secs:
            if self.test_policy == "small_elbow":
                target = self.trajectory.command_at(elapsed_secs)["q"][LEFT_ELBOW_INDEX]
                self.get_logger().info(
                    f"Elbow result: target={target:.6f} rad, "
                    f"measured={message.q[LEFT_ELBOW_INDEX]:.6f} rad"
                )
                reason = (
                    "small_elbow_complete"
                    if self.trajectory.followed_target(message.q)
                    else "small_elbow_did_not_follow"
                )
                self.stop_test(reason)
            else:
                self.stop_test("arm_raise_complete")
            return
        self.command_sequence += 1
        if self.phase == ArmRaisePhase.HOLDING:
            command = self.handover_command.copy()
        elif self.test_policy == "unitree_arm7":
            command = self.trajectory.command_at(elapsed_secs, message.q)
        else:
            command = self.trajectory.command_at(elapsed_secs)
        command.update(
            session=self.policy_session_id,
            seq=self.command_sequence,
            state_id=message.observation_id,
        )
        self.command_publisher.publish(command_to_ros(command))
        self.time_last_command_secs = cur_time_secs


def main():
    """Run the manual test producer without adding a production safety-node role."""
    rclpy.init()
    node = ArmRaiseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
