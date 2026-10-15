"""Drive the hardware safety adapter with MuJoCo feedback and output."""

from types import SimpleNamespace

import numpy as np

from robot_safety.config import COMMAND_FIELDS
from robot_safety.hardware_adapter import G1HardwareAdapter
from robot_safety.hardware_profile import HardwareProfile
from robot_safety.standing import standing_height


class SimTransport:
    """Implement motor ownership and damping without loading the Unitree SDK."""

    def __init__(self, plant):
        self.plant = plant
        self.mode = 4
        self.active = None
        self.damping_at = None
        self.body_writes = 0

    def read_ready_fsm(self):
        return 0, self.mode

    read_fsm = read_ready_fsm

    def release_motion(self, **kwargs):
        return 0

    def restore_motion(self):
        return 0

    def write_body(self, targets, machine):
        self.active = {key: np.array([targets[i][key] for i in range(29)])
                       for key in COMMAND_FIELDS}
        self.plant.apply(self.active)
        self.body_writes += 1
        return True

    def clear_body(self):
        self.active = None
        self.plant._apply_arrays(*self.plant.damping_command(self.plant.q(), self.plant.dq()))

    def request_damping(self):
        self.mode = 1
        if self.damping_at is None:
            self.damping_at = float(self.plant.data.time)
        self.clear_body()
        return 0

    def write_arm(self, targets, weight):
        if targets or weight != 0:
            raise ValueError("simulation supports whole-body control only")
        return True


class SimHardware:
    """Run adapter callbacks synchronously on the simulation clock."""

    def __init__(self, plant, profile):
        self.plant = plant
        self.transport = SimTransport(plant)
        self.adapter = G1HardwareAdapter(HardwareProfile.load(profile),
                                         clock=lambda: float(plant.data.time), whole_body=True)
        self.adapter.transport = self.transport
        self.recovery_at = None
        self.recovery_writes = 0
        self.receive()
        self.ready = self.adapter.check_ready_mode() and self.adapter.arm()

    def receive(self, launch=False):
        state = self.plant.state()
        remote = [0] * 40
        if launch:
            remote[3] = (1 << 7) | (1 << 1)
        self.adapter.receive_battery(SimpleNamespace(soc=100))
        self.adapter.receive_state(SimpleNamespace(
            motor_state=[SimpleNamespace(q=q, dq=dq, temperature=(25, 25), tau_est=tau)
                         for q, dq, tau in zip(state["q"], state["dq"],
                                              self.plant.data.actuator_force[self.plant.actuators])],
            imu_state=SimpleNamespace(quaternion=state["quat"], gyroscope=state["gyro"]),
            mode_machine=5, wireless_remote=remote))
        self.damp_if_requested()

    def damp_if_requested(self):
        if self.adapter.stop_requested.is_set() and self.adapter.hold_failed:
            self.adapter.stop_once()

    def abort(self, reason):
        self.adapter.abort(reason)
        self.damp_if_requested()

    def step(self, launch=False):
        self.receive(launch)
        if not self.ready:
            self.transport.request_damping()
            return
        writes = self.transport.body_writes
        self.adapter.step()
        if self.adapter.automatic_soft_reset:
            if self.recovery_at is None:
                self.recovery_at = self.adapter.clock()
            self.recovery_writes += self.transport.body_writes - writes
        self.damp_if_requested()

    def snapshot(self):
        adapter = self.adapter
        state = self.plant.state()
        try:
            height = standing_height(state["q"], state["quat"])
        except ValueError:
            height = None
        return {
            "phase": ("damping" if self.transport.mode == 1 else
                      "recovery" if adapter.walk_stop else
                      "policy" if adapter.policy_launched else "startup"),
            "reason": adapter.abort_reason,
            "arm_rejection": adapter.arm_rejection_reason,
            "recovery_at_secs": self.recovery_at,
            "recovery_writes": self.recovery_writes,
            "damping_at_secs": self.transport.damping_at,
            "standing_height_m": height,
            "upright": state["upright"],
            "synthetic_telemetry": {"battery_soc": 100, "motor_temperature_c": 25},
        }
