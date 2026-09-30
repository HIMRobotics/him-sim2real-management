"""Read live G1 joint measurements and Start state from SDK rt/lowstate."""

import time

from him_safety_interfaces.msg import RobotFeedback

from . import ros_topics as topics
from .feedback import SDK_DISCOVERY_TIMEOUT_SECS, FeedbackInbox
from .hardware_constants import LOW_STATE_TOPIC, NUM_JOINTS
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
        self.create_timer(self.config.period, self.publish_feedback)

    def start_sdk(self):
        """Create only a LowState subscriber, never a command publisher."""
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize,
            ChannelSubscriber,
        )
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

        ChannelFactoryInitialize(self.sdk_domain, self.sdk_interface)
        self.sdk_subscriber = ChannelSubscriber(LOW_STATE_TOPIC, LowState_)
        self.sdk_subscriber.Init(self.on_sdk_state, self.config.runtime_queue_capacity)

    def on_sdk_state(self, message):
        # feedback.py converts and queues the sample on the SDK callback thread.
        self.inbox.receive(message, time.monotonic())

    def publish_feedback(self):
        if self.abort_reason is not None:
            return
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
            self.sdk_subscriber.Close()
        return super().destroy_node()
