# ROS 2 safety pipeline — dry-run checkpoint

The intended flow is:

```text
policy -> runner joint commands -> /safety/precheck
    -> safety_checker -> /safety/postcheck
    -> motor_command_gate -> Unitree motor controller -> actuators

controller_stop -> /safety/abort -> checker and motor gate
```

The current motor gate publishes **only `/safety/dry_run_output`**, not a Unitree motor topic. The position/velocity/gain/torque frame belongs before the Unitree controller. No code is inserted between its internal controller and actuators.

## Node responsibilities

- `SafetyCheckerNode`: callback-driven validation of whole precheck frames using the existing core. One invalid joint latches the fuse, prevents all later relaying, and publishes abort immediately then every 100 ms. Each valid frame is relayed once, never replayed as a heartbeat.
- `MotorGateNode`: separate process with an independent 5 ms target timer. It watches postcheck freshness and immutable expiry, rejects replayed sequences and wrong sessions, and selects fake damping on abort or timeout. A frozen checker cannot refresh its deadlines.
- `ControllerStopNode`: observes physical Start, detects a double press and publishes abort directly to both nodes. It also aborts on stale feedback. Fresh DDS data does not prove a live radio link.

The core remains the ROS-independent checker logic. `motor_gate.py` contains the independently testable final-gate logic. The earlier `runtime.py` is retained as an in-process test/reference implementation, not the scheduler used by these ROS nodes. ROS executors run callbacks and timers. Run roles as separate OS processes in deployment to avoid a checker freeze freezing the gate too.

## Topics and services

| Topic | Typed message | Role |
| --- | --- | --- |
| `/safety/robot_feedback` | `RobotFeedback` | Future SDK bridge supplies measurements and physical Start |
| `/safety/policy_observation` | `RobotFeedback` | Checker assigns observation IDs for policy requests |
| `/safety/precheck` | `JointCommand` | Runner proposes a complete frame with session, sequence and observation ID |
| `/safety/postcheck` | `CheckedCommand` | Checker forwards approved frame and fixed expiry |
| `/safety/abort` | `Abort` | Every message asserts a latch, no clear flag |
| `/safety/dry_run_output` | `DryRunOutput` | Diagnostic policy/damping selection only |

`/safety/checker/arm` uses `std_srvs/Trigger`, returning the session ID in `message`. `/safety/motor_gate/arm` uses `ArmGate` and takes that session. These are integration primitives, not a finished operator workflow. A future coordinator must arm both nodes and start a producer within the deadlines. There is deliberately no distributed reset service in this checkpoint. Any aborted node remains latched for its lifetime. Restarted nodes start inhibited and may receive retained abort from peers. Do not restart one node as a way to clear a stop.

Command topics use reliable, volatile, keep-last depth 1. Abort uses reliable, transient-local depth 1 plus periodic assertion, so late joiners can receive a retained stop. Feedback uses best effort with bounded history. Topic transport does not guarantee cross-topic ordering, delivery latency or exclusion of unauthorized publishers. Once received, abort wins locally over subsequent motion. DDS security/network isolation and coordinated reset remain required before physical use.

For this checkpoint all participants must run on the **same Linux host/boot and clock namespace**. Feedback receipt and checked-command expiry use `time.monotonic()` plus `/proc/sys/kernel/random/boot_id`. This deliberately rejects another host's clock rather than pretending clocks are synchronized. A future multi-host design needs an explicit freshness protocol.

## Build and inspect on this machine

Use the system Python matching ROS 2 Foxy, not the Python 3.12 uv environment:

```bash
source /opt/ros/foxy/setup.bash
colcon build --base-paths ros2 --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
export ROS_DOMAIN_ID=97
export ROS_LOCALHOST_ONLY=1
ros2 run him_safety_nodes safety_node checker
```

In other terminals with the same setup/domain, use roles `motor_gate` and `controller_stop`. No ROS-to-Unitree bridge or policy producer is supplied yet, so these commands do not constitute a full live robot test. With no feedback the controller-stop node deliberately aborts after its freshness timeout. Keep this isolated test graph separate from active robot control.

Run ROS graph tests after building and sourcing the overlay:

```bash
ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 -m unittest discover -s tests/ros2 -v
```

The tests verify typed postcheck delivery, invalid-command abort delivery and independent gate damping when the checker stops executing. Ordinary safety tests do not require ROS:

```bash
python3 -m unittest discover -s tests -p 'test_robot_safety*.py'
```

## Next review checkpoint

Add the read-only SDK-to-ROS feedback bridge and synthetic policy producer, then design coordinated arm/reset. Per-joint hardware profiles, ROS publisher access isolation, real motor publication, service watchdogs and frontend integration remain pending. This checkpoint makes no claim of physical stop capability or hard-real-time timing.

---

# Controller stop detection (dry-run)

`controller.py` separates SDK byte decoding from gesture recognition:

- `decode_start_pressed(payload)` reads bit 2 of byte 2 in the 40-byte G1 `wireless_remote` payload, following the installed SDK example.
- `StartDoublePressDetector.update(start_button_pressed, cur_time_secs)` emits a one-sample stop event for press → release → press within `Config.double_press_window_secs` (default 0.5 seconds).
- The detector requires a released sample after startup/reset. Holds and repeated pressed samples do not count again. A stale sample gap clears pending history. This does not establish radio-link health.

`Supervisor.state(observation, start_button_pressed, cur_time_secs, *, stop_requested=False)` now receives the physical Start state. It runs the detector and latches its event. The optional `stop_requested` argument accepts an independent event even with Start released. Physical Start blocks reset/arm, while the latch persists after release. Explicit reset clears gesture history and returns to inhibited, with separate arming required.

The old `stop_pressed` argument/status field is replaced by `start_button_pressed`. Callers that previously passed a stop event positionally must migrate to `stop_requested=`. There is still no motor publisher or full supervisor runtime.

## Test with the actual controller

Use the Python environment that already imports the Unitree G1 SDK and its native DDS libraries. From the repository root:

```bash
python3 -m tools.controller_monitor --interface YOUR_ROBOT_INTERFACE
```

Substitute the robot-facing interface used by your existing deployment. If `python3` is not your SDK interpreter, use its absolute path. The manager's uv environment does not automatically contain the SDK. The monitor defaults to DDS domain 0 and `rt/lowstate`. To follow the installed SDK's lower-frequency controller example instead:

```bash
python3 -m tools.controller_monitor --interface YOUR_ROBOT_INTERFACE --topic rt/lf/lowstate
```

Release Start, press once, release, then press again within 500 ms. Expect Start pressed/released messages and `STOP REQUESTED: Start double press detected. No command sent.` Test a single press, long hold and two widely spaced presses too. Ctrl+C exits. The detector window can be set using `--window-secs`.

This tool only subscribes to state. It neither publishes motor commands nor releases control modes. Nevertheless, the stock robot firmware may respond to controller buttons independently. Test with the G1 appropriately supported and under operator control. This confirms the software detects received gestures, not that our code physically damps the robot or overrides a running policy.

No fresh state produces a warning and resets the gesture. Backlogged or invalid input cannot be used to complete a pair. Short presses missed between telemetry samples cannot be reconstructed. Robot packet freshness does not prove remote radio freshness. The monitor does not filter on `LowState.tick`, whose behavior must be verified on the installed firmware.

## Offline validation

```bash
python3 -m unittest discover -s tests -p 'test_robot_safety*.py'
```

## Troubleshooting missing double presses

Run with `--debug` to print the two raw button bytes, robot tick, sample gaps, and intervals between detected presses. If no Start transitions appear on `rt/lowstate`, try `--topic rt/lf/lowstate`, as used by the installed SDK controller example. Both topics still require verification on your firmware. A Start press should set bit 2 in the first printed button byte.

If transitions appear but no stop event, check that two rising edges occur within the configured window and that sample gaps stay below the 200 ms freshness timeout. Repeated robot ticks no longer erase gesture history in this diagnostic tool. A diagnostic observation is not a verified radio-health or motor-stop guarantee.

## Recorded operator validation

On September 22, 2026, the operator confirmed successful detection using the actual G1 controller. The supplied debug trace showed Start as `0400`, two double-press events with approximately 160 ms between presses, and rejection of an interval of approximately 1.06 seconds. Displayed telemetry gaps were around 1 ms. Firmware version, interface and selected topic were not recorded in the supplied trace. This validates controller-input decoding and gesture detection only, not motor damping or stop priority over a policy.

## Offline runtime reference

`runtime.py` is the earlier in-process implementation, retained for deterministic
queue and scheduling tests. ROS nodes do not run it. Use the ROS build/run
instructions above for the current architecture.

## Identifying the physical setup

The operator reports G1 EDU. Confirm the exact joint configuration from the order
specification or Unitree support using the robot serial number. Do not infer the
variant from the size of a fixed SDK message array or move joints to identify it.
The installed SDK has separate 5-joint and 7-joint arm examples, and marks wrist
pitch/yaw unavailable on the 23-DoF variant. Waist availability must be confirmed
separately. Hands/grippers and attached payloads also need to be recorded.

Support arrangement means the actual physical setup, not software: identify the
gantry/hoist and harness or other manufacturer-approved support, its documented
load rating and attachment method, whether the feet bear weight, and clear space
for the arm and any possible fall/swing. A chair or person holding the robot is
not automatically an adequate fall-arrest arrangement. Confirm suitability with
the lab's responsible operator and manufacturer guidance before motor tests.
