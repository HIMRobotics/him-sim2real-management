"""Read live G1 joint measurements and Start state from SDK rt/lowstate."""

import json
import time

from him_safety_interfaces.msg import RobotFeedback
from std_msgs.msg import String

from . import ros_topics as topics
from .feedback import (
    SDK_DISCOVERY_TIMEOUT_SECS,
    FeedbackInbox,
    base_state_from_sdk,
    policy_state,
)
from .hardware_constants import LOW_STATE_TOPIC, NUM_JOINTS
from .hardware_profile import HardwareProfile
from .ros_nodes import SafetyNode


class RobotFeedbackNode(SafetyNode):
    """Subscribe to G1 state, publish ROS feedback and check input freshness."""

    def __init__(self):
        super().__init__("robot_feedback_bridge")
        if self.config.joints != NUM_JOINTS:
            raise ValueError("feedback bridge requires the 29-joint profile")
        self.sdk_interface = self.declare_parameter("sdk_interface", "").value
        self.sdk_domain = self.declare_parameter("sdk_domain", 0).value
        if not self.sdk_interface:
            raise ValueError("set sdk_interface to the robot-facing interface")
        self.inbox = FeedbackInbox(self.config.runtime_queue_capacity)
        self.time_started_secs = time.monotonic()
        self.time_last_feedback_secs = None
        self.feedback_publisher = self.create_publisher(
            RobotFeedback, topics.ROBOT_FEEDBACK, self.feedback_qos
        )
        self.sdk_subscriber = None
        self.base_subscriber = None
        self.base = None
        self.latest_sample = None
        self.pending = {}
        self.hardware_status = {}
        self.publish_policy = self.declare_parameter("publish_policy_state", False).value
        self.base_topic = self.declare_parameter("base_state_topic", "").value
        self.base_frame = self.declare_parameter("base_velocity_frame", "").value
        self.require_base = self.declare_parameter("require_base_state", False).value
        profile_path = self.declare_parameter("hardware_profile_path", "").value
        self.profile = HardwareProfile.load(profile_path) if profile_path else None
        if self.require_base and (not self.base_topic or self.base_frame not in ("world", "body")):
            raise ValueError("required base state needs an explicit SDK topic and velocity frame")
        self.policy_publisher = self.create_publisher(String, "/safety/policy_state", self.feedback_qos)
        self.create_subscription(RobotFeedback, topics.POLICY_OBSERVATION,
                                 self.on_observation, self.feedback_qos)
        self.create_subscription(String, "/safety/hardware_status", self.on_status, self.command_qos)
        self.create_timer(self.config.period, self.publish_feedback)

    def start_sdk(self):
        """Subscribe to low state and optional measured base state."""
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize,
            ChannelSubscriber,
        )
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

        from .sdk_latest import LatestSdkReader

        ChannelFactoryInitialize(self.sdk_domain, self.sdk_interface)
        # Polled by the publish timer. A callback decodes all 1 kHz samples.
        self.sdk_subscriber = LatestSdkReader(LOW_STATE_TOPIC, LowState_)
        if self.base_topic:
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

            self.base_subscriber = ChannelSubscriber(self.base_topic, SportModeState_)
            self.base_subscriber.Init(self.on_base_state)

    def on_status(self, message):
        self.hardware_status = json.loads(message.data)

    def on_base_state(self, message):
        try:
            latest = self.latest_sample
            now = time.monotonic()
            if self.base_frame == "body" and (
                latest is None or not 0 <= now - latest.received_at_secs < self.config.robot_timeout
            ):
                return
            sample = base_state_from_sdk(message, now, self.base_frame,
                                         latest.quat if latest else ())
            if self.base and sample["base_source_time_secs"] <= self.base["base_source_time_secs"]:
                raise ValueError("nonadvancing_base_state")
            self.base = sample
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            self.latch_abort(str(error))

    def on_observation(self, message):
        if not self.publish_policy or self.abort_reason:
            return
        sample = self.pending.get(message.received_at_secs)
        if sample is None:
            return
        try:
            state = policy_state(sample, message.observation_id, self.host_boot_id)
            if self.profile:
                joints = self.profile.legs + self.profile.joints
                state["ctrl_low"] = [joint.position_min_rad for joint in joints]
                state["ctrl_high"] = [joint.position_max_rad for joint in joints]
            base = self.base
            if base and 0 <= time.monotonic() - base["base_received_at_secs"] < self.config.robot_timeout:
                state.update(base)
                state["base_velocity"] = (*base["linear_velocity"], *sample.gyro)
            elif not self.require_base:
                # lowstate has no translation. Angular velocity is the IMU gyro.
                state["base_velocity"] = (0.0, 0.0, 0.0, *sample.gyro)
            else:
                if time.monotonic() - self.time_started_secs >= SDK_DISCOVERY_TIMEOUT_SECS:
                    self.latch_abort("base_feedback_timeout")
                return
            state["control_ready"] = self.hardware_status.get("handover_complete", False)
            state["left_b_count"] = self.hardware_status.get("left_b_count", 0)
            state["left_b_at_secs"] = self.hardware_status.get("left_b_at_secs")
            if self.hardware_status.get("whole_body_output"):
                state.update({key: self.hardware_status.get(key) for key in (
                    "startup_phase", "startup_stable", "handoff_ready", "policy_handoff_at_secs")})
                state["control_ready"] &= bool(self.hardware_status.get("control_ready"))
            state["control_ready"] &= not bool(self.hardware_status.get("abort_reason"))
            released = self.hardware_status.get("motion_released_at_secs")
            if self.require_base and released is not None:
                state["control_ready"] &= bool(base and base["base_received_at_secs"] > released)
            if self.hardware_status.get("launch_gate"):
                state["control_ready"] &= bool(self.hardware_status.get("policy_launched"))
            output = String()
            output.data = json.dumps(state, allow_nan=False)
            self.policy_publisher.publish(output)
        except (TypeError, ValueError) as error:
            self.latch_abort(str(error))

    def on_sdk_state(self, message):
        # feedback.py converts and queues the sample on the SDK callback thread.
        self.inbox.receive(message, time.monotonic())

    def publish_feedback(self):
        if self.abort_reason is not None:
            return
        if self.sdk_subscriber is not None:
            message = self.sdk_subscriber.take_latest()
            if message is not None:
                self.on_sdk_state(message)
        for _ in range(self.config.runtime_queue_capacity):
            if self.inbox.input_fault.is_set():
                self.latch_abort("invalid_or_overflowing_sdk_feedback")
                return
            # The ROS timer consumes converted samples in receipt order.
            sample = self.inbox.take()
            if sample is None:
                break
            age_secs = time.monotonic() - sample.received_at_secs
            if not 0 <= age_secs < self.config.robot_timeout or (
                self.time_last_feedback_secs is not None
                and sample.received_at_secs < self.time_last_feedback_secs
            ):
                self.latch_abort("stale_or_reordered_sdk_feedback")
                return
            self.time_last_feedback_secs = sample.received_at_secs
            self.latest_sample = sample
            self.pending[sample.received_at_secs] = sample
            while len(self.pending) > self.config.observation_history_size:
                del self.pending[next(iter(self.pending))]
            message = RobotFeedback()
            message.host_boot_id = self.host_boot_id
            message.received_at_secs = sample.received_at_secs
            message.q = list(sample.q)
            message.dq = list(sample.dq)
            message.start_button_pressed = sample.start_button_pressed
            self.feedback_publisher.publish(message)
        if self.time_last_feedback_secs is None:
            if time.monotonic() - self.time_started_secs >= SDK_DISCOVERY_TIMEOUT_SECS:
                self.latch_abort("sdk_discovery_timeout")
        elif (
            time.monotonic() - self.time_last_feedback_secs >= self.config.robot_timeout
        ):
            self.latch_abort("sdk_feedback_timeout")

    def destroy_node(self):
        if self.sdk_subscriber is not None:
            self.sdk_subscriber.close()
        if self.base_subscriber is not None:
            self.base_subscriber.Close()
        return super().destroy_node()
