# G1 robot-side supervisor first, frontend integration later

## Summary

Updated September 22, 2026. **Current stage: Stage 1, partially complete.**
The dry-run core, configuration, Start double-press detector, read-only SDK
controller monitor and 23 deterministic tests are implemented. The full
supervisor runtime, policy transports, IPC, CLI and test producer are pending. Stages 2 and 3 have not started.

Revise `ESTOP_PLAN.md` around staged delivery inside `him-sim2real-management`. Implement the robot-side supervisor, watchdogs, local controls, and fault tests first. Stage one defaults to dry-run and never publishes motor commands.

The first test runner is a **stand-in for an actual policy**: it generates predictable proposed joint commands and can deliberately freeze, disconnect, or send invalid data. It exercises the same interface that real runners will use.

Keep that interface independent of a training framework. Unitree RL Gym becomes the first real adapter; multiple compatible policies can then share it. RL Lab and other runners can adopt the same interface later.

## Stage 1 — Robot supervisor and dry-run validation (in progress)

**Implemented so far**

- [x] `robot_safety/core.py`: operating-state enum, stop latch, explicit arm/reset, session revocation, action validation and fake command selection.
- [x] Four configurable 200 ms watchdog checks. These run when the core is called, not in a background process yet.
- [x] `robot_safety/config.py`: immutable settings, documented units, configurable placeholder bounds and observation history size.
- [x] Descriptive internal state names and concise documentation. Core formatting and lint checks pass with Ruff.
- [x] 23 deterministic core/config/controller tests pass. The operator also verified Start double-press detection with the actual G1 controller. Physical damping has not been tested and the monitor sends no motor commands.

**Start double-press detection (implemented, controller input verified)**

- [x] Add a controller-input module that decodes Start from the installed SDK mapping. Remove the unused Select-based configuration placeholder.
- [x] Detect press → release → press within a configurable 500 ms initial dry-run window, measured between rising edges and inclusive at the boundary. This interval is our proposed detector setting, not a verified firmware specification.
- [x] Emit a stop event on the second press. Holding Start or repeating pressed samples cannot count as additional presses. Consume a completed pair. An expired second press becomes the first press of a new pair.
- [x] Require an observed release after startup or input reset before counting presses, so attaching while Start is held cannot fabricate a gesture.
- [x] Separate `stop_requested`, the detected event, from `start_button_pressed`, the physical input used to block reset. The old `stop_pressed` argument has been replaced.
- [x] Keep damping latched after release. Require fresh state, Start released and explicit reset followed by separate arm. Clear pending gesture history on reset or stale input.
- [ ] Serialize stop events ahead of reset/arm requests in each runtime cycle. Timestamp equality alone is not sufficient arbitration.
- [x] Test single/double/triple presses, exact window boundaries, expired windows, long holds, repeated samples, startup while held, stale input and reset/rearm behavior.

The Start double-press binding follows the operator's description of this G1.
The operator supplied a monitor trace showing Start decoding (`0400`) and two
detected double presses about 160 ms apart. A 1.06-second interval was rejected
by the 500 ms window. This verifies received input detection only. Firmware
priority over custom policies and physical damping still require hardware verification. Our detector will run independently of policy inference.

The remaining Stage 1 items below are pending except for the core behavior
and configuration identified above.

**Next task:** build the continuously scheduled supervisor and local IPC/test producer. The manual diagnostic `tools/controller_monitor.py` is available now, but does not provide that runtime or motor control. See `robot_safety/README.md` for actual-controller test instructions.

**Implementation structure**

- Add a Python `robot_safety/` package using the installed G1-compatible Python SDK. The local C++ SDK checkout lacks G1 message definitions, so C++ migration is deferred.
- Run the supervisor in its own process, separate from inference, recording, and the web application.
- Provide two transports: synthetic robot state for automated tests, and read-only G1 state for observing controller input.
- Both transports route selected commands to a fake output sink. Stage one has no option to enable real motor publication.
- Use a configurable 5 ms arbitration period as an initial software target, not a guaranteed real-time deadline.

**Control interface**

Provide a local Unix-domain socket interface and CLI with `status`, `stop`, `reset`, `arm`, and test-runner launch commands. Restrict socket access using filesystem permissions and peer credentials.

Use separate operator-control and policy-action sockets so flooded policy input cannot queue ahead of a stop. Action messages contain a session ID, increasing sequence, supervisor-issued state ID, and per-joint proposed targets/gains. Keep only the latest valid action; bound message sizes and processing work.

Supply robot state through this interface so future isolated policy workers do not need direct DDS access.

**Stop behavior**

- Start `INHIBITED`; never automatically arm after launch, reconnect, or restart.
- Use `ARMED`, `RUNNING`, and `STOP_LATCHED` states with a separate fault reason.
- Double-press Start is the controller stop binding. Latch the stop on the second detected press in every control phase.
- A stop revokes the policy session, discards pending actions, and selects damping output on the next arbitration cycle.
- Damping output uses zero stiffness, zero desired velocity, zero feedforward torque, and a configured damping profile. Stage-one gains are dry-run placeholders, not validated G1 damping profiles.
- Stop takes precedence over arm/reset and policy commands. Start being physically pressed prevents reset, independently of the stop event.
- Reset returns to inhibited and requires a separate arm action. No stale session becomes valid again.
- A normal policy exit, disconnected action client, or invalid action immediately latches damping.
- No 30-second grace period or stationary controller is included, following the selected immediate-damping behavior.

**Watchdogs**

Use monotonic time and independently track:

- **Action freshness:** latch damping at 200 ms without a valid new action.
- **Observation freshness:** reject actions based on supervisor-issued state aged 200 ms or more.
- **Robot-state freshness:** latch damping at 200 ms without fresh state.
- **Loop progress:** a gap of 200 ms latches a fault when execution resumes.

These are initial configurable dry-run thresholds. Replayed or malformed messages do not refresh deadlines. The action watchdog starts on arm. Robot-state age is measured from the last valid observation, and loop progress from the last cycle. No policy command is selected until a valid action is available.

Do not label these measurements as motor-application latency. Report that latency as unavailable unless a verified robot acknowledgment mechanism is added. Likewise, fresh telemetry does not establish a live controller radio link.

## Stage 2 — Service setup and first real runner (not started)

**Watchdog service setup**

- Provide installation/check scripts and a systemd service template; Bash handles setup, not timing-critical monitoring.
- Use systemd notification/watchdog support with a 1-second process-health timeout. Notifications depend on fresh arbitration-loop progress, not merely a living background thread.
- Restart failed services into inhibited dry-run mode; never relaunch or resume a policy automatically.
- Separate loop execution from logging and control-request handling. Logging failures cannot block arbitration.
- Detect systems without systemd and provide foreground execution. This environment does not currently show systemd as PID 1, so service behavior requires validation on the actual robot host.
- Treat service restart as recovery. It does not guarantee damping during a supervisor or computer failure.

**Real policy adapter**

Integrate Unitree RL Gym by replacing all motor writes—including startup poses and shutdown—with supervisor calls. Preserve the author’s observation construction, joint mapping, gains, and timing.

Run the worker without robot-network access and supply state through the supervisor. Until that isolation is enforced, describe the integration as cooperative, not bypass-proof. Restrict unmanaged publishers on the robot network as part of hardware commissioning.

Validate the real runner against the fake output sink before enabling hardware transport.

## Stage 3 — Physical G1 acceptance, then frontend (not started)

**Physical control**

Add real SDK motor publication only after recording the exact robot variant, firmware, joint mapping, damping profile, and controller ownership procedure.

Require an explicit hardware configuration and activation step. Do not automatically terminate the stock controller or compete with another publisher. Before ownership, the supervisor must report that it cannot enforce motor damping.

With the robot appropriately supported:

- Verify controller stop during startup, pose transitions, and stalled inference.
- Verify command-loss behavior when the supervisor freezes or loses communication.
- Measure controller-input latency, supervisor response, and observed robot response separately.
- Establish whether radio loss can be detected and whether firmware provides a bounded protective response.

Unverified whole-computer or communication-failure protection remains a stated limitation.

**Frontend integration**

After robot-side acceptance, add authenticated stop/status endpoints and a persistent Emergency damping button. Stop requests bypass recording/upload/export work and require no confirmation. Preserve the separate Stop recording action.

Show stop requested, supervisor latched, damping publication, and available robot feedback as distinct evidence. An acknowledgment must not display “robot safe.” Frontend reconnect never clears the latch.

## Tests and completion criteria

- Deterministic clock tests at, below, and above each watchdog deadline.
- Controller stop during every lifecycle state, including concurrent arm/reset.
- Frozen/crashed policy, stale observations, duplicate sequences, invalid values, and flooded action input.
- No policy command selected after a latch; old sessions remain rejected after reset.
- Dry-run mode creates no motor-command publisher and never releases stock-controller ownership.
- Supervisor stalls, restart behavior, unavailable systemd, and logging failure.
- Existing recorder tests continue passing.

**Robot-side deliverable (Stage 1 plus Stage 2 service setup):** revised plan, working dry-run supervisor, test producer, local CLI, service/setup files, and automated fault tests. No frontend changes or physical motor activation in this milestone.
