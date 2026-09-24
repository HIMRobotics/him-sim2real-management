"""Encode arm SDK frames and request stock-controller whole-body damping."""

import time

from .hardware_profile import (
    ARM_WEIGHT_INDEX,
    DAMPING_FSM_ID,
    DAMPING_RPC_TIMEOUT_SECS,
    READINESS_QUERY_ATTEMPTS,
    READINESS_RPC_TIMEOUT_SECS,
    SDK_RPC_TIMEOUT_CODE,
)


class UnitreeArmTransport:
    """Own arm_sdk publication and a separate locomotion RPC client."""

    def __init__(self, interface, domain, on_state):
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize,
            ChannelPublisher,
            ChannelSubscriber,
        )
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        ChannelFactoryInitialize(domain, interface)
        self.make_command = unitree_hg_msg_dds__LowCmd_
        self.crc = CRC()
        self.publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.publisher.Init()
        self.locomotion = LocoClient()
        self.locomotion.SetTimeout(DAMPING_RPC_TIMEOUT_SECS)
        self.locomotion.Init()
        # A slow read-only startup query must not change stop-worker RPC timing.
        self.readiness_client = LocoClient()
        self.readiness_client.SetTimeout(READINESS_RPC_TIMEOUT_SECS)
        self.readiness_client.Init()
        self.readiness_query_results = []
        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        # No SDK queue that could conceal old samples behind new receipt timestamps.
        self.subscriber.Init(on_state)

    def write_arm(self, targets, weight):
        """Send only explicitly mapped upper-body targets, plus SDK ownership weight."""
        message = self.make_command()
        for index, values in targets.items():
            motor = message.motor_cmd[index]
            for field, value in values.items():
                setattr(motor, field, value)
        message.motor_cmd[ARM_WEIGHT_INDEX].q = weight
        message.crc = self.crc.Crc(message)
        return self.publisher.Write(message)

    def request_damping(self):
        """Use Damp's underlying RPC because Damp() discards its return code."""
        return self.locomotion.SetFsmId(DAMPING_FSM_ID)

    def read_ready_fsm(self):
        """Retry one timed-out startup query, requiring a real successful reply."""
        self.readiness_query_results = []
        for _ in range(READINESS_QUERY_ATTEMPTS):
            started = time.monotonic()
            code, fsm_id = self.readiness_client.GetFsmId()
            self.readiness_query_results.append(
                {
                    "rpc_code": code,
                    "fsm_id": fsm_id,
                    "elapsed_secs": round(time.monotonic() - started, 3),
                }
            )
            if code != SDK_RPC_TIMEOUT_CODE:
                return code, fsm_id
        return code, fsm_id

    def read_fsm(self):
        return self.locomotion.GetFsmId()

    def close(self):
        self.subscriber.Close()
        self.publisher.Close()
