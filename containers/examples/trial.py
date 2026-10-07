"""Measured movement trials, independent of the policy and motor backend."""

import math


class MovementTrial:
    def __init__(self, settings):
        self.settings = settings
        self.phase = settings["phase"]
        if self.phase not in ("supported_straight", "unsupported_straight", "circle"):
            raise ValueError("unknown trial phase")
        if settings.get("qualified") is not True:
            raise ValueError("trial requires reviewed ramp and stopping qualification")
        for key in ("ramp_secs", "braking_distance_m", "settle_secs", "stationary_speed_m_s"):
            value = settings[key]
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError("qualification must supply positive " + key)
        self.start = None
        self.braking_at = self.settled_at = None
        self.angle = 0.0
        self.previous_angle = -math.pi / 2
        self.max_error = 0.0
        self.result = None

    def command(self, state, now):
        """Ramp task velocity, brake on measured progress, then keep standing."""
        position, velocity, quat = state["position"], state["base_velocity"], state["quat"]
        if (len(position) != 3 or len(velocity) != 6 or len(quat) != 4
                or not all(math.isfinite(v) for v in (*position, *velocity, *quat, now))):
            raise ValueError("invalid trial measurements")
        if self.start is None:
            w, qx, qy, qz = quat
            self.heading = math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            self.start, self.started_at = tuple(position), now
        dx, dy = position[0] - self.start[0], position[1] - self.start[1]
        c, s = math.cos(self.heading), math.sin(self.heading)
        x, y = c * dx + s * dy, -s * dx + c * dy
        circle = self.phase == "circle"
        error = abs(math.hypot(x, y - 1) - 1) if circle else abs(y)
        self.max_error = max(self.max_error, error)
        if circle:
            angle = math.atan2(y - 1, x)
            self.angle += math.atan2(math.sin(angle - self.previous_angle), math.cos(angle - self.previous_angle))
            self.previous_angle = angle
        progress, target = (self.angle, 2 * math.pi) if circle else (x, 2.)
        elapsed = now - self.started_at
        if self.result is None and elapsed >= (100 if circle else 40):
            self.result = {"success": False, "reason": "trial_timeout"}
        if self.result and not self.result["success"]:
            raise RuntimeError(self.result["reason"])
        if self.braking_at is None and progress >= target - self.settings["braking_distance_m"]:
            self.braking_at = now
        scale = min(1., elapsed / self.settings["ramp_secs"])
        if self.braking_at is not None:
            scale *= max(0., 1 - (now - self.braking_at) / self.settings["ramp_secs"])
        stopped = math.hypot(*velocity[:2]) <= self.settings["stationary_speed_m_s"] and abs(velocity[5]) <= 0.05
        if scale == 0 and self.braking_at is not None and stopped:
            if self.settled_at is None:
                self.settled_at = now
            if self.result is None and now - self.settled_at >= self.settings["settle_secs"]:
                endpoint = math.hypot(x, y) if circle else math.hypot(x - 2, y)
                lap = not circle or self.angle >= 2 * math.pi
                success = self.max_error < 0.2 and endpoint <= (0.2 if circle else 0.1) and lap
                self.result = {"success": success, "reason": "completed" if success else "trajectory_error",
                               "endpoint_error_m": endpoint, "max_path_error_m": self.max_error,
                               "angular_progress_rad": self.angle, "duration_secs": elapsed,
                               "requires_operator_review": True}
        else:
            self.settled_at = None
        return [0.1 * scale, 0., 0.1 * scale if circle else 0.]
