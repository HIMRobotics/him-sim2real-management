"""Bounded, time-domain motion detection for the existing safety checker."""

import math
from collections import Counter, deque
from copy import deepcopy


def default_regions(joints):
    """Group G1 SDK slots into legs, waist and arms."""
    if joints == 29:
        return (0,) * 6 + (1,) * 6 + (2,) * 3 + (3,) * 7 + (4,) * 7
    return tuple(range(joints))


class _Cycles:
    """Track substantial lobes and compare complete, same-direction cycles."""

    def __init__(self, config):
        self.config = config
        self.sign = 0
        self.started = None
        self.excursion = 0.0
        self.previous_lobe = None
        self.complete_lobe = False
        self.cycles = deque(maxlen=6)

    def update(self, velocity, dt, now):
        config = self.config
        while self.cycles and now - self.cycles[0][0] > config.history_secs:
            self.cycles.popleft()
        if self.started is not None and now - self.started > config.history_secs / 2:
            self.sign = 0
            self.previous_lobe = None
            self.complete_lobe = False
            self.cycles.clear()
        sign = 1 if velocity > config.deadband_rad_secs else (
            -1 if velocity < -config.deadband_rad_secs else 0
        )
        if sign and sign != self.sign:
            if self.sign and self.complete_lobe:
                duration = now - self.started
                if self.excursion >= config.min_excursion_rad:
                    if sign == 1 and self.previous_lobe is not None:
                        period, amplitude = self.previous_lobe
                        self.cycles.append((now, period + duration, amplitude + self.excursion))
                    self.previous_lobe = (duration, self.excursion)
                else:
                    self.previous_lobe = None
                    self.cycles.clear()
            self.complete_lobe = bool(self.sign)
            self.sign, self.started, self.excursion = sign, now, 0.0
        self.excursion += abs(velocity) * dt

    def irregularity(self):
        if len(self.cycles) < self.config.min_cycles:
            return 0.0
        variability = []
        for column in (1, 2):
            values = [cycle[column] for cycle in self.cycles]
            mean = sum(values) / len(values)
            variance = sum((value - mean) ** 2 for value in values) / len(values)
            variability.append(math.sqrt(variance) / max(mean, 1e-12))
        return max(variability)


class ThrashDetector:
    """Combine filtered activity, reversals, body regions and decaying evidence."""

    def __init__(self, config, joints, max_gap_secs):
        self.config = config
        self.joints = joints
        self.max_gap_secs = max_gap_secs
        self.scales = config.velocity_scales or (1.0,) * joints
        self.regions = config.regions or default_regions(joints)
        self.reset()

    def reset(self):
        """Discard history at session boundaries or telemetry gaps."""
        self.last_time = None
        self.fast = [0.0] * self.joints
        self.slow = [0.0] * self.joints
        self.energy = [0.0] * self.joints
        self.activity = [0.0] * self.joints
        self.evidence = [0.0] * self.joints
        self.severe_evidence = [0.0] * self.joints
        self.filtered_cycles = [_Cycles(self.config) for _ in range(self.joints)]
        self.motion_cycles = [_Cycles(self.config) for _ in range(self.joints)]
        self.metrics = {}
        self.event = None

    def observe(self, velocities, now):
        """Record the first event and return a stop reason only in enforce mode."""
        reason = self.update(velocities, now)
        if reason and self.event is None:
            self.event = dict(deepcopy(self.metrics), detected_at_secs=now)
        return reason if self.config.mode == "enforce" else None

    def snapshot(self):
        """Copy telemetry and its configured thresholds for reports and viewers."""
        return {
            "mode": self.config.mode,
            "event": deepcopy(self.event),
            "metrics": deepcopy(self.metrics),
            "sample_time_secs": self.last_time,
            "thresholds": {name: getattr(self.config, name) for name in (
                "activity_rms", "severe_rms", "irregularity_cv", "persistence_secs",
            )},
        }

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
        irregularities = []
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
            filtered, motion = self.filtered_cycles[j], self.motion_cycles[j]
            filtered.update(oscillation, dt, now)
            motion.update(self.fast[j], dt, now)
            irregularity = motion.irregularity()
            irregularities.append(irregularity)
            repeated = len(filtered.cycles) >= config.min_cycles
            irregular = (
                irregularity >= config.irregularity_cv
                and self.activity[j] >= config.activity_rms ** 2
            )
            moderate = (repeated and self.energy[j] >= config.activity_rms ** 2) or irregular
            severe = (
                repeated and self.energy[j] >= config.severe_rms ** 2
            ) or (irregular and self.activity[j] >= config.severe_rms ** 2)
            for evidence, suspicious in (
                (self.evidence, moderate), (self.severe_evidence, severe)
            ):
                evidence[j] = min(config.persistence_secs, max(
                    0.0, evidence[j] + (dt if suspicious else -2 * dt)
                ))
        active = [j for j, value in enumerate(self.evidence) if value >= config.persistence_secs]
        severe = [j for j, value in enumerate(self.severe_evidence) if value >= config.persistence_secs]
        regions = Counter(self.regions[j] for j in active)
        reason = None
        if severe:
            reason = "thrashing_severe"
        elif any(count >= config.local_joints for count in regions.values()):
            reason = "thrashing_local"
        elif len(active) >= config.distributed_joints and len(regions) >= 2:
            reason = "thrashing_distributed"
        self.metrics = {
            "reason": reason,
            "joint_indices": sorted(set(active + severe)),
            "oscillatory_rms": [math.sqrt(value) for value in self.energy],
            "activity_rms": [math.sqrt(value) for value in self.activity],
            "irregularity_cv": irregularities,
            "evidence_secs": list(self.evidence),
            "severe_evidence_secs": list(self.severe_evidence),
        }
        return reason
