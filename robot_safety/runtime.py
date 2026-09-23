"""Scheduled dry-run arbitration with bounded, in-process input queues."""

import time
from collections import deque
from copy import deepcopy
from threading import Event, Lock

from .config import Config
from .core import Supervisor, vector


class ControlReply:
    """Waitable control result without callbacks on the arbitration thread."""

    def __init__(self):
        self._ready = Event()
        self._result = None

    def set_result(self, result):
        """Publish a result from the runtime owner."""
        self._result = deepcopy(result)
        self._ready.set()

    def result(self, timeout=None):
        """Wait for processing, raising TimeoutError if the runtime is stalled."""
        if not self._ready.wait(timeout):
            raise TimeoutError("runtime has not processed this request")
        return deepcopy(self._result)


class DryRunRuntime:
    """One loop owns the core. Producers submit inputs and read snapshots.

    No SDK, policy IPC or motor publishing is provided here. The output sink
    retains only the latest fake command in the status snapshot.
    """

    def __init__(self, config=None, *, clock=time.monotonic):
        self.config = Config() if config is None else config
        self._clock = clock  # Local monotonic clock, injectable for tests.
        self._supervisor = Supervisor(self.config)
        self._input_lock = Lock()  # Protects producer queues and status snapshots.
        self._step_lock = Lock()  # Rejects overlapping arbitration calls.
        self._observations = deque()  # Receipt-time ordered robot samples.
        self._controls = deque()  # Bounded operator requests and reply objects.
        self._pending_stop = False  # Stop cannot be blocked by a full queue.
        self._pending_input_fault = None  # First input fault awaiting arbitration.
        self._completed_cycles = 0  # Advances only after output is selected.
        self._snapshot = self._supervisor.status(self._clock())
        self._snapshot.update(completed_cycles=0, time_last_completed_cycle_secs=None)

    def submit_observation(self, observation, start_button_pressed):
        """Copy joint feedback and timestamp receipt, returning enqueue success."""
        valid = isinstance(observation, dict) and type(start_button_pressed) is bool
        if valid:
            valid = all(
                vector(observation.get(name), self.config.joints, *bounds)
                for name, bounds in (
                    ("q", self.config.robot_position_bounds_rad),
                    ("dq", self.config.robot_velocity_bounds_rad_per_sec),
                )
            )
        with self._input_lock:
            if not valid:
                self._pending_input_fault = (
                    self._pending_input_fault or "invalid_robot_input"
                )
                return False
            if len(self._observations) >= self.config.runtime_queue_capacity:
                self._pending_input_fault = (
                    self._pending_input_fault or "robot_input_overflow"
                )
                return False
            copied = {name: list(observation[name]) for name in ("q", "dq")}
            self._observations.append((self._clock(), copied, start_button_pressed))
        return True

    def request_stop(self):
        """Assert a stop independently of bounded observation/control queues."""
        with self._input_lock:
            self._pending_stop = True

    def request_control(self, command):
        """Queue arm/reset/status and return a waitable result."""
        reply = ControlReply()
        with self._input_lock:
            if command not in ("arm", "reset", "status"):
                result = {"error": "use arm/reset/status or request_stop()"}
            elif len(self._controls) >= self.config.runtime_queue_capacity:
                result = {"error": "control queue full"}
            else:
                self._controls.append((command, reply))
                return reply
        reply.set_result(result)
        return reply

    def status(self):
        """Return the last completed cycle's snapshot, without accessing the core."""
        with self._input_lock:
            return deepcopy(self._snapshot)

    def step(self):
        """Process a bounded input batch and select one fake output."""
        if not self._step_lock.acquire(blocking=False):
            raise RuntimeError("only one runtime loop may call step")
        replies = []
        try:
            with self._input_lock:
                observations, self._observations = self._observations, deque()
                controls, self._controls = self._controls, deque()
                stop_requested, self._pending_stop = self._pending_stop, False
                input_fault, self._pending_input_fault = self._pending_input_fault, None
            cur_time_secs = self._clock()
            core = self._supervisor
            stop_time_before_batch = core.time_stop_latched_secs
            core.watchdogs(cur_time_secs)
            if input_fault:
                core.input_fault(input_fault, cur_time_secs)
                observations.clear()
            if stop_requested:
                core.stop("operator_stop", cur_time_secs)
            controller_stop_in_batch = False
            for received_at, observation, pressed in observations:
                stop_event = core.state(
                    observation, pressed, cur_time_secs, received_time_secs=received_at
                )
                # Invalid feedback returns None and also blocks reset in this batch.
                controller_stop_in_batch |= stop_event is not False

            stop_in_batch = (
                stop_requested
                or input_fault
                or controller_stop_in_batch
                or core.time_stop_latched_secs != stop_time_before_batch
            )
            for command, reply in controls:
                if command in ("reset", "arm") and stop_in_batch:
                    result = {"error": "stop takes precedence in this cycle"}
                else:
                    result = core.control(command, cur_time_secs)
                replies.append((reply, result))

            # Stops arriving during batch processing win before output selection.
            with self._input_lock:
                if self._pending_stop or self._pending_input_fault:
                    if self._pending_input_fault:
                        core.input_fault(self._pending_input_fault, self._clock())
                    else:
                        core.stop("operator_stop", self._clock())
                    self._pending_stop = False
                    self._pending_input_fault = None
                    self._observations.clear()
                    replies = [
                        (reply, {"error": "stop superseded control request"})
                        for reply, _ in replies
                    ]
                core.cycle(self._clock())
                self._completed_cycles += 1
                self._snapshot = core.status(self._clock())
                self._snapshot.update(
                    completed_cycles=self._completed_cycles,
                    time_last_completed_cycle_secs=self._clock(),
                )
        finally:
            self._step_lock.release()
        for reply, result in replies:
            reply.set_result(result)
        return self.status()

    def run(self, shutdown_event):
        """Run at the configured target period until shutdown, then latch stop."""
        deadline = self._clock()
        try:
            while not shutdown_event.is_set():
                self.step()
                deadline += self.config.period
                cur_time_secs = self._clock()
                if deadline <= cur_time_secs:
                    deadline = cur_time_secs + self.config.period
                shutdown_event.wait(deadline - cur_time_secs)
        finally:
            self.request_stop()
            self.step()


def main():
    """Run an inhibited dry-run loop without robot input or motor output."""
    runtime = DryRunRuntime()
    print("Dry-run runtime. No robot input or motor output. Ctrl+C stops.", flush=True)
    try:
        runtime.run(Event())
    except KeyboardInterrupt:
        print("Stopped with fake damping selected.", flush=True)


if __name__ == "__main__":
    main()
