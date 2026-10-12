"""Measured movement trials, independent of the policy and motor backend."""

import math


class MovementTrial:
    def __init__(self, settings):
        self.settings = settings = dict(settings)
        self.phase = settings["phase"]
        if self.phase not in ("supported_straight", "unsupported_straight", "circle"):
            raise ValueError("unknown trial phase")
        if settings.get("qualified") is not True:
            raise ValueError("trial requires reviewed ramp and stopping qualification")
        for key, default in (("speed_m_s", 0.1), ("heading_gain", 0.),
                             ("lateral_gain", 0.), ("max_yaw_rate_rad_s", 0.3)):
            settings.setdefault(key, default)
        for key in ("ramp_secs", "braking_distance_m", "settle_secs", "stationary_speed_m_s",
                    "speed_m_s", "heading_gain", "lateral_gain", "max_yaw_rate_rad_s"):
            value = settings[key]
            allow_zero = key in ("heading_gain", "lateral_gain")
            if (type(value) not in (int, float) or not math.isfinite(value)
                    or value < 0 or (value == 0 and not allow_zero)):
                raise ValueError("invalid qualification setting: " + key)
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
        w, qx, qy, qz = quat
        heading = math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        if self.start is None:
            self.heading = heading
            handoff = state.get("policy_handoff_at_secs")
            self.started_at = now if handoff is None else float(handoff)
            if not math.isfinite(self.started_at) or self.started_at > now:
                raise ValueError("invalid trial handoff time")
            self.start = tuple(position)
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
        yaw = self.settings["speed_m_s"]
        if not circle:
            heading_error = math.atan2(math.sin(heading - self.heading), math.cos(heading - self.heading))
            yaw = -self.settings["heading_gain"] * heading_error - self.settings["lateral_gain"] * y
            limit = self.settings["max_yaw_rate_rad_s"]
            yaw = max(-limit, min(limit, yaw))
        return [self.settings["speed_m_s"] * scale, 0., yaw * scale]
