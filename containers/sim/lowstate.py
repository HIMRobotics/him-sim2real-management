"""Publish simulated rt/lowstate on the loopback interface only."""

from hardware_constants import LOW_STATE_TOPIC, NUM_JOINTS


class SimLowState:
    """No-op when the Unitree SDK is missing or a robot NIC is visible."""

    def __init__(self, domain, interface):
        self.ok = False
        self.tick = 0
        self.publisher = None
        self.make = None
        self.crc = None
        if interface != "lo":
            return
        try:
            import socket

            if {name for _, name in socket.if_nameindex()} != {"lo"}:
                return
            from unitree_sdk2py.core.channel import (
                ChannelFactoryInitialize,
                ChannelPublisher,
            )
            from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
            from unitree_sdk2py.utils.crc import CRC

            ChannelFactoryInitialize(int(domain), interface)
            publisher = ChannelPublisher(LOW_STATE_TOPIC, LowState_)
            publisher.Init()
        except Exception as exc:  # noqa: BLE001
            print(f"Sim IMU publish unavailable: {exc}", flush=True)
            return
        self.publisher = publisher
        self.make = unitree_hg_msg_dds__LowState_
        self.crc = CRC()
        self.ok = True

    def publish(self, state):
        if not self.ok:
            return
        try:
            message = self.make()
            message.mode_machine = 5
            self.tick = (self.tick + 1) & 0xFFFFFFFF
            message.tick = self.tick
            for index, value in enumerate(state["quat"]):
                message.imu_state.quaternion[index] = value
            for index, value in enumerate(state["gyro"]):
                message.imu_state.gyroscope[index] = value
            for index, value in enumerate(state["accel"]):
                message.imu_state.accelerometer[index] = value
            for index, value in enumerate(state["rpy"]):
                message.imu_state.rpy[index] = value
            for index in range(NUM_JOINTS):
                motor = message.motor_state[index]
                motor.q = state["q"][index]
                motor.dq = state["dq"][index]
            message.crc = self.crc.Crc(message)
            self.publisher.Write(message)
        except Exception as exc:  # noqa: BLE001
            self.ok = False
            print(f"Sim IMU publish stopped: {exc}", flush=True)
