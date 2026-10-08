"""Measured startup balancing shared by hardware and simulation."""

import math

import numpy as np

from .balance_policy import BalancePolicy, LEG_KD, LEG_KP, MODEL_PATH


class StartupBalance:
    """Inhibit submissions until stable, then blend targets and gains."""

    min_seconds = 2.0
    stable_seconds = 1.0
    timeout_seconds = 30.0
    max_sample_age = 0.2
    max_tilt = 0.15
    max_gyro = 0.25
    max_joint_speed = 0.6
    entry_seconds = 0.5
    blend_seconds = 1.0

    def __init__(self, policy=None):
        self.policy = policy
        self.started = None
        self.stabilizing_since = None
        self.last_sample = None
        self.stable_since = None
        self.ready = False
        self.handoff = None
        self.arm_pose = None
        self.entry_pose = None
        self.entry_started = None
        self.raw_balance = None
        self.last_balance = None
        self.origin = None
        self.phase = "waiting"

    def prepare(self):
        """Load and validate inference before output ownership changes."""
        if self.policy is None:
            self.policy = BalancePolicy(str(MODEL_PATH))
        self.policy.warmup()

    def observe(self, now, stamp, q, dq, quat, gyro):
        """Only consecutive fresh samples can establish readiness."""
        if not 0 <= now - stamp < self.max_sample_age:
            self.ready = False
            raise ValueError("startup_balance_stale_telemetry")
        arrays = [np.asarray(value, dtype=float) for value in (q, dq, quat, gyro)]
        if any(a.shape != shape or not np.isfinite(a).all()
               for a, shape in zip(arrays, ((29,), (29,), (4,), (3,)))):
            raise ValueError("startup_balance_invalid_telemetry")
        if self.handoff is not None:
            return
        if self.started is None:
            self.started = now
            self.stabilizing_since = now
            self.phase = "balancing"
        if (self.stabilizing_since is not None
                and now - self.stabilizing_since >= self.timeout_seconds):
            self.ready = False
            raise ValueError("startup_balance_timeout")
        if self.last_sample is not None and stamp <= self.last_sample:
            return
        if self.last_sample is not None and stamp - self.last_sample >= self.max_sample_age:
            self.stable_since = None
        self.last_sample = stamp
        norm = float(np.dot(arrays[2], arrays[2]))
        if not 0.9 <= norm <= 1.1:
            raise ValueError("startup_balance_invalid_quaternion")
        tilt = math.acos(np.clip(1 - 2 * (quat[1] ** 2 + quat[2] ** 2) / norm, -1, 1))
        entering = self.entry_started is None or now - self.entry_started < self.entry_seconds
        stable = (not entering and tilt <= self.max_tilt and np.linalg.norm(gyro) <= self.max_gyro
                  and np.max(np.abs(dq)) <= self.max_joint_speed)
        if not stable:
            self.stable_since = None
        elif self.stable_since is None:
            self.stable_since = stamp
        self.ready = bool(stable and stamp - self.stable_since >= self.stable_seconds
                          and now - self.started >= self.min_seconds)
        if self.ready:
            self.stabilizing_since = None
        elif self.stabilizing_since is None:
            self.stabilizing_since = now
        self.phase = "entering_balance" if entering else "ready" if self.ready else "balancing"

    def authorize(self, now):
        """Accept a new authorization only while fresh readiness holds."""
        if (self.ready and self.entry_started is not None
                and now - self.entry_started >= self.entry_seconds
                and self.handoff is None and self.last_balance is not None
                and self.last_sample is not None
                and 0 <= now - self.last_sample < self.max_sample_age):
            self.handoff = now
            self.origin = {k: v.copy() for k, v in self.last_balance.items()}
            self.phase = "handoff"
            return True
        return False

    def command(self, now, stamp, state, submitted, *, automatic=False):
        """Return the sole command eligible for output this cycle."""
        self.observe(now, stamp, state["q"], state["dq"], state["quat"], state["gyro"])
        if self.handoff is None:
            if self.policy is None:
                raise ValueError("startup_balance_model_unavailable")
            if self.arm_pose is None:
                self.arm_pose = np.asarray(state["q"][15:], dtype=float).copy()
                self.entry_pose = np.asarray(state["q"], dtype=float).copy()
                self.entry_started = now
            # The output loop may run faster than the 50 Hz policy.
            if self.raw_balance is None or stamp - self.inferred_at >= 0.019:
                lower = self.policy.update(state["q"], state["dq"], state["quat"], state["gyro"])
                self.raw_balance = {
                    "q": np.concatenate((lower, self.arm_pose)), "dq": np.zeros(29),
                    "kp": np.concatenate((LEG_KP, np.full(14, 40.0))),
                    "kd": np.concatenate((LEG_KD, np.full(14, 1.0))), "tau": np.zeros(29),
                }
                if any(not np.isfinite(v).all() for v in self.raw_balance.values()):
                    raise ValueError("startup_balance_nonfinite_output")
                self.inferred_at = stamp
            alpha = min(1.0, max(0.0, (now - self.entry_started) / self.entry_seconds))
            alpha = alpha * alpha * (3 - 2 * alpha)
            self.last_balance = dict(
                self.raw_balance,
                q=(1 - alpha) * self.entry_pose + alpha * self.raw_balance["q"],
            )
            if automatic:
                self.authorize(now)
            if self.handoff is None:
                return self.last_balance
        alpha = min(1.0, max(0.0, (now - self.handoff) / self.blend_seconds))
        alpha = alpha * alpha * (3 - 2 * alpha)
        self.phase = "running" if alpha >= 1 else "handoff"
        return {k: (1 - alpha) * self.origin[k] + alpha * np.asarray(submitted[k])
                for k in self.origin}

    def fail(self):
        """Expose a latched owner fault without advertising readiness."""
        self.phase = "fault"
        self.ready = False

    def status(self):
        return {"startup_balance_required": True, "startup_phase": self.phase,
                "startup_stable": self.ready, "handoff_ready": self.ready and self.handoff is None,
                "policy_handoff_at_secs": self.handoff, "control_ready": self.handoff is not None and self.phase != "fault"}
