"""Weighted whole-body oscillatory motion detection for the shared checker."""

import math
from copy import deepcopy
from dataclasses import dataclass


@dataclass(frozen=True)
class ThrashConfig:
    """Provisional oscillation settings, calibrated independently of policy commands."""

    mode: str = "monitor"
    slow_cutoff_hz: float = 4.0
    fast_cutoff_hz: float = 25.0
    averaging_secs: float = 0.15
    persistence_secs: float = 0.5
    weighted_rms_limit: float = 1.5
    velocity_scales: tuple = ()
    joint_weights: tuple = ()

    def __post_init__(self):
        if self.mode not in ("off", "monitor", "enforce"):
            raise ValueError("thrash mode must be off, monitor or enforce")
        for name, value in self.__dict__.items():
            if name in ("mode", "velocity_scales", "joint_weights"):
                continue
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"invalid thrash setting: {name}")
        if self.fast_cutoff_hz <= self.slow_cutoff_hz:
            raise ValueError("thrash filter cutoffs must be ordered")
        for name in ("velocity_scales", "joint_weights"):
            values = getattr(self, name)
            if not isinstance(values, tuple) or any(
                type(v) not in (int, float) or not math.isfinite(v) or v <= 0
                for v in values
            ):
                raise ValueError(f"thrash {name} must be finite and positive")


def default_weights(joints):
    """Relative motion-risk weights in G1 SDK order, not measured inertias."""
    if joints == 29:
        leg = (4.0, 4.0, 4.0, 4.0, 2.0, 2.0)
        arm = (3.0, 3.0, 3.0, 1.0, 0.25, 0.25, 0.25)
        return leg * 2 + (4.0,) * 3 + arm * 2
    return (1.0,) * joints


class ThrashDetector:
    """Stop on sustained weighted RMS of filtered joint velocities."""

    def __init__(self, config, joints, max_gap_secs):
        self.config = config
        self.joints = joints
        self.max_gap_secs = max_gap_secs
        self.scales = config.velocity_scales or (1.0,) * joints
        self.weights = config.joint_weights or default_weights(joints)
        if len(self.scales) != joints or len(self.weights) != joints:
            raise ValueError("thrash scales and weights must match the joint count")
        self.weight_sum = sum(self.weights)
        self.reset()

    def reset(self):
        """Discard history at session boundaries or telemetry gaps."""
        self.last_time = None
        self.fast = [0.0] * self.joints
        self.slow = [0.0] * self.joints
        self.energy = [0.0] * self.joints
        self.activity = [0.0] * self.joints
        self.evidence = 0.0
        self.metrics = {}
        self.event = None

    def observe(self, velocities, now):
        """Record the first event and return a stop reason only in enforce mode."""
        reason = self.update(velocities, now)
        if reason and self.event is None:
            self.event = dict(deepcopy(self.metrics), detected_at_secs=now)
        return reason if self.config.mode == "enforce" else None

    def snapshot(self):
        """Copy telemetry and thresholds for reports and viewers."""
        return {
            "mode": self.config.mode,
            "event": deepcopy(self.event),
            "metrics": deepcopy(self.metrics),
            "sample_time_secs": self.last_time,
            "thresholds": {name: getattr(self.config, name) for name in (
                "weighted_rms_limit", "persistence_secs",
            )},
        }

    def save_state(self):
        """Copy filter history for deterministic simulation branches."""
        return deepcopy({name: getattr(self, name) for name in (
            "last_time", "fast", "slow", "energy", "activity", "evidence",
            "metrics", "event",
        )})

    def restore_state(self, state):
        """Restore a checkpoint without sharing mutable filter history."""
        for name, value in deepcopy(state).items():
            setattr(self, name, value)

    def update(self, velocities, now):
        """Consume validated measured velocities once per receipt timestamp."""
        config = self.config
        if config.mode == "off":
            return None
        if self.last_time is not None and now <= self.last_time:
            return None
        if self.last_time is None or now - self.last_time >= self.max_gap_secs:
            event = self.event
            self.reset()
            self.event = event
            self.fast = list(velocities)
            self.slow = list(velocities)
            self.last_time = now
            return None
        dt = now - self.last_time
        self.last_time = now
        fast_alpha = -math.expm1(-2 * math.pi * config.fast_cutoff_hz * dt)
        slow_alpha = -math.expm1(-2 * math.pi * config.slow_cutoff_hz * dt)
        energy_alpha = -math.expm1(-dt / config.averaging_secs)
        for j, velocity in enumerate(velocities):
            self.fast[j] += fast_alpha * (velocity - self.fast[j])
            self.slow[j] += slow_alpha * (self.fast[j] - self.slow[j])
            oscillation = self.fast[j] - self.slow[j]
            self.energy[j] += energy_alpha * (
                (oscillation / self.scales[j]) ** 2 - self.energy[j]
            )
            self.activity[j] += energy_alpha * (
                (self.fast[j] / self.scales[j]) ** 2 - self.activity[j]
            )
        contributions = [w * e / self.weight_sum for w, e in zip(self.weights, self.energy)]
        weighted_energy = sum(contributions)
        score = math.sqrt(weighted_energy)
        self.evidence = (
            min(config.persistence_secs, self.evidence + dt)
            if score >= config.weighted_rms_limit else 0.0
        )
        reason = "thrashing_energy" if self.evidence >= config.persistence_secs else None
        # The listed joints explain the score. They do not have separate stop thresholds.
        leaders = sorted(range(self.joints), key=contributions.__getitem__, reverse=True)[:3]
        self.metrics = {
            "reason": reason,
            "joint_indices": leaders if reason else [],
            "weighted_rms": score,
            "weighted_energy": weighted_energy,
            "weighted_rms_limit": config.weighted_rms_limit,
            "persistence_elapsed_secs": self.evidence,
            "oscillatory_rms": [math.sqrt(value) for value in self.energy],
            "activity_rms": [math.sqrt(value) for value in self.activity],
            "joint_weights": list(self.weights),
            "energy_share": [value / weighted_energy if weighted_energy else 0.0 for value in contributions],
        }
        return reason
