"""Velocity command files shared by the walker container and the direct plant path."""

import math
from pathlib import Path

import numpy as np
import yaml


class CommandSchedule:
    """Each command holds until its end time. The last one stays."""

    def __init__(self, segments):
        self.segments = segments

    def at(self, elapsed):
        if not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError("command time must be finite and non-negative")
        for end, command in self.segments:
            if end is None or elapsed < end:
                return command.copy()
        return self.segments[-1][1].copy()


def _vector(value, path):
    command = np.asarray(value, dtype=np.float32)
    if command.shape != (3,) or not np.isfinite(command).all():
        raise ValueError(f"{path} command must be vx, vy, yaw_rate")
    return command


def load_schedule(path):
    """Read a constant command or a sequence of timed commands."""
    path = Path(path)
    document = yaml.safe_load(path.read_text())
    if not isinstance(document, dict):
        raise ValueError(f"{path} must be a mapping")
    if "segments" in document:
        if "command" in document:
            raise ValueError(f"{path} uses either command or segments")
        segments = []
        cursor = 0.0
        items = document["segments"]
        if not isinstance(items, list) or not items:
            raise ValueError(f"{path} needs at least one segment")
        for index, item in enumerate(items):
            if not isinstance(item, dict):
                raise ValueError(f"{path} segment must be a mapping")
            command = _vector(item.get("command"), path)
            duration = item.get("seconds")
            if duration is None:
                if index != len(items) - 1:
                    raise ValueError(f"{path} only the last segment may omit seconds")
                segments.append((None, command))
                continue
            if (
                isinstance(duration, bool)
                or not isinstance(duration, (int, float))
                or not math.isfinite(duration)
                or duration <= 0
            ):
                raise ValueError(f"{path} segment seconds must be positive")
            cursor += float(duration)
            segments.append((cursor, command))
        return CommandSchedule(segments)
    return CommandSchedule([(None, _vector(document.get("command"), path))])
