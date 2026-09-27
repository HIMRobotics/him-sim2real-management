"""Decode physical Start input and recognize double presses without SDK I/O."""

import math

from .hardware_constants import (
    F3_BUTTON_MASK,
    START_BUTTON_BYTE,
    START_BUTTON_MASK,
    WIRELESS_REMOTE_SIZE,
)


def decode_remote_button(wireless_remote, mask):
    """Return one button bit from the SDK's 40-byte remote payload."""
    if len(wireless_remote) != WIRELESS_REMOTE_SIZE:
        raise ValueError("expected a 40-byte wireless_remote payload")
    if not all(type(value) is int and 0 <= value <= 255 for value in wireless_remote):
        raise ValueError("remote payload must contain bytes")
    return bool(wireless_remote[START_BUTTON_BYTE] & mask)


def decode_start_pressed(wireless_remote):
    """Return Start's physical state from the SDK's 40-byte remote payload."""
    return decode_remote_button(wireless_remote, START_BUTTON_MASK)


def decode_f3_pressed(wireless_remote):
    """Return F3's physical state. F3 sits beside Start on the G1 remote."""
    return decode_remote_button(wireless_remote, F3_BUTTON_MASK)


class StartDoublePressDetector:
    """Emit one event per press-release-press pair using local receipt times."""

    def __init__(self, window_secs, stale_timeout_secs):
        for value in (window_secs, stale_timeout_secs):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("detector intervals must be finite and positive")
        self.window_secs = window_secs
        self.stale_timeout_secs = stale_timeout_secs
        self.reset()

    def reset(self):
        """Discard pending presses and require a fresh release before counting."""
        self.previous_start_pressed = None  # Unknown until a released sample arrives.
        self.time_first_press_secs = None  # Pending pair's first rising edge.
        self.time_last_sample_secs = None  # Detect gaps without fabricating edges.

    def update(self, start_button_pressed, cur_time_secs):
        """Return a stop event for this sample, not a persistent pressed flag."""
        if type(start_button_pressed) is not bool or not math.isfinite(cur_time_secs):
            self.reset()
            raise ValueError("expected a boolean button and finite local time")
        if self.time_last_sample_secs is not None:
            elapsed = cur_time_secs - self.time_last_sample_secs
            if elapsed < 0:
                self.reset()
                raise ValueError("controller sample time went backward")
            if elapsed >= self.stale_timeout_secs:
                self.reset()
        self.time_last_sample_secs = cur_time_secs

        # If the monitor starts with Start held down, wait for release first.
        if self.previous_start_pressed is None:
            if not start_button_pressed:
                self.previous_start_pressed = False
            return False

        rising_edge = start_button_pressed and not self.previous_start_pressed
        self.previous_start_pressed = start_button_pressed
        if not rising_edge:
            return False

        if (
            self.time_first_press_secs is not None
            and cur_time_secs - self.time_first_press_secs <= self.window_secs
        ):
            self.time_first_press_secs = None
            return True

        # A late second press starts a new candidate pair.
        self.time_first_press_secs = cur_time_secs
        return False
