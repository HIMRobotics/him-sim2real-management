"""Stdio front end so the ROS bridge can run in a different Python.

Requests are one JSON line each and get one JSON line back. After
{"op": "realtime"} the server keeps its own clock instead: it steps the plant
against host monotonic time, takes "command" lines whenever they arrive (only
rejections are answered), applies the newest one at the hardware adapter's
output tick `gate_phase_ms` into each 20 ms step, and writes each step's state as
{"event": "state", ...}. Other requests are still answered, tagged with their "id".
"""

import json
import os
import select
import sys
import time
from contextlib import redirect_stdout

from plant import CONTROL_DT, PHYSICS_DT
from runner import Runner

# Behind real time by more than this, the server stops catching up and drops the lag.
MAX_LAG_SECS = 0.1


def write(message):
    sys.stdout.write(json.dumps(message) + "\n")
    sys.stdout.flush()


def handle(runner, request):
    try:
        with redirect_stdout(sys.stderr):
            return runner.handle(request)
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "reason": "plant_error", "detail": str(exc)}


class Realtime:
    """Steps the runner on the host clock and serves requests between physics steps."""

    def __init__(self, runner, request):
        self.runner = runner
        self.origin = float(request["clock_origin"])
        self.gate = max(0, min(runner.plant.substeps - 1,
                               round(float(request.get("gate_phase_ms", 10.0)) / 1000.0 / PHYSICS_DT)))
        self.thrash = request.get("thrash")
        self.anchor = time.monotonic() - runner.sim_time
        self.buffer = b""
        self.stopped = False
        self.sent_at = {}
        self.applied = []
        self.overruns = 0

    def clock(self):
        return time.monotonic() - self.origin

    def due(self, sim_time):
        return self.anchor + sim_time

    def serve_until(self, deadline):
        """Answer requests and take commands until monotonic time `deadline`."""
        while not self.stopped:
            remaining = deadline - time.monotonic()
            readable, _, _ = select.select([sys.stdin], [], [], max(0.0, remaining))
            if readable:
                chunk = os.read(sys.stdin.fileno(), 65536)
                if not chunk:
                    self.stopped = True
                    return
                self.buffer += chunk
                while b"\n" in self.buffer:
                    line, self.buffer = self.buffer.split(b"\n", 1)
                    if line.strip():
                        self.request(json.loads(line))
            if remaining <= 0:
                return

    def request(self, request):
        op = request.get("op")
        if op == "command":
            response = handle(self.runner, {**request, "time": self.clock()})
            if not response.get("ok"):
                write({"event": "command_rejected", **response})
            return
        if op == "realtime_stop":
            self.stopped = True
        elif op == "report":
            response = handle(self.runner, request)
            response["command_delay"] = self.delay_summary()
        else:
            response = handle(self.runner, request)
        write({"id": request.get("id"), **response})

    def before_gate(self):
        """The adapter's output tick: take every command that has arrived by now."""
        self.serve_until(self.due(self.runner.sim_time))
        observation = self.runner.command_observation_id
        if observation in self.sent_at:
            self.applied.append(self.runner.sim_time - self.sent_at[observation])

    def run(self):
        runner = self.runner
        while not self.stopped:
            start = runner.sim_time
            if time.monotonic() - self.due(start) > MAX_LAG_SECS:
                self.overruns += 1
                self.anchor = time.monotonic() - start
            self.serve_until(self.due(start))
            if self.stopped:
                break
            runner.external_thrash = self.thrash
            with redirect_stdout(sys.stderr):
                try:
                    result = runner.step(self.clock(), self.gate, self.before_gate)
                except Exception as exc:  # noqa: BLE001
                    result = {"reason": "plant_error", "detail": str(exc)}
            result["ok"] = result.get("reason") not in ("plant_error", "nonfinite_simulation")
            result["detail"] = runner.detail
            self.serve_until(self.due(start + CONTROL_DT))
            if result["ok"]:
                with redirect_stdout(sys.stderr):
                    result.update(runner.feedback(self.clock()))
                self.sent_at[result["observation_id"]] = runner.sim_time
                while len(self.sent_at) > 64:
                    del self.sent_at[next(iter(self.sent_at))]
            write({"event": "state", **result})
            if not result["ok"]:
                self.serve_until(float("inf"))

    def delay_summary(self):
        """Sim time from each state to the output tick that applied the command answering it."""
        values = sorted(round(d * 1000.0, 1) for d in self.applied)
        if not values:
            return None
        counts = {}
        for value in values:
            counts[str(value)] = counts.get(str(value), 0) + 1
        return {"gate_phase_ms": self.gate * PHYSICS_DT * 1000.0, "applied": len(values),
                "ms": counts, "overruns": self.overruns}


def main():
    safety_checks = os.environ.get("G1_SIM_SAFETY_CHECKS", "1")
    if safety_checks not in ("0", "1"):
        raise ValueError("G1_SIM_SAFETY_CHECKS must be 0 or 1")
    cpus = os.environ.get("G1_SIM_PLANT_CPUS")
    if cpus:
        os.sched_setaffinity(0, {int(c) for c in cpus.split(",")})
    with redirect_stdout(sys.stderr):
        runner = Runner(
            os.environ.get("G1_SIM_POSE", "standing"),
            scene=os.environ.get("G1_SIM_SCENE") or None,
            safety_checks=safety_checks == "1",
            control_path="direct" if os.environ.get("G1_SIM_CIRCLE") == "1" else "submission",
        )
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            request = json.loads(line)
        except ValueError as exc:
            write({"ok": False, "reason": "plant_error", "detail": str(exc)})
            continue
        if request.get("op") == "realtime":
            write({"id": request.get("id"), "ok": True})
            Realtime(runner, request).run()
            return
        write(handle(runner, request))


if __name__ == "__main__":
    main()
