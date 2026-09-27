# ROS 2 safety pipeline — dry-run checkpoint

The intended flow is:

```text
policy -> runner joint commands -> /safety/precheck
    -> safety_checker -> /safety/postcheck
    -> motor_command_gate -> Unitree motor controller -> actuators

controller_stop -> /safety/abort -> checker and motor gate
```

The default motor gate publishes **only `/safety/dry_run_output`**. An explicitly selected `hardware_gate` now provides SDK output, pending physical validation. See [hardware adapter setup and boundaries](HARDWARE_ADAPTER.md). The position/velocity/gain/torque frame belongs before the Unitree controller. No code is inserted between its internal controller and actuators.

## Background safety watchdog

Run `./install_safety_stops.sh install` from the repository root on the robot
host to install, enable and immediately start the user systemd service
`g1-safety-stops.service`. With no argument, the script offers installation or
removal. It writes the unit under `${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user`
with paths to this clone, reloads systemd and enables user lingering so the
service can start at boot and survive logout. If enabling linger fails, the
script prints the required administrator command. Re-run installation after
moving the clone to update its paths.

The service runs [run_safety_stops.sh](../tests/manual/run_safety_stops.sh), which
builds the ROS overlay under `~/.him-safety` and starts a hardware motor gate.
It does not start an arm trajectory or arm a policy. The gate reads the robot's
controller telemetry directly and watches for press–release–press gestures
within 0.5 seconds: **double F3 requests balance with the arms held**, and
**double Start requests stock whole-body damping**. F3 uses the local GR00T
balance network, not the stock walking controller. The service can issue these
commands even without a test policy running. While armed, it also checks command
and feedback freshness. Systemd restarts the process on failure, subject to the
unit's retry limits. An active service alone does not establish physical stop
performance.

Startup initializes the motion-switcher and optional speech clients, loads the
balance model, and runs one dummy inference without changing policy history or
writing motor commands. These objects are retained for subsequent button presses.
The unit uses `Type=notify`: it stays `activating` until the model is warmed,
both workers are alive, and fresh telemetry shows Start and F3 released.
Look for `Safety monitor ready` in the log. Missing model/telemetry prevents
readiness and eventually reaches the 180-second startup timeout. This is a
startup check, not continuous health monitoring or a motor-response guarantee.
Controller release/restore and damping still require SDK RPC replies.

To deploy this startup change, run `./install_safety_stops.sh install` with the
robot supported and foreground hardware tests closed. This updates the unit and
restarts the service, temporarily interrupting monitoring. Merely editing the
source does not update an already running process.

The launcher uses ROS localhost domain 97, SDK domain 0, interface `eth0` and
[g1_type5.lenient.json](profiles/g1_type5.lenient.json) by default. It requires
ROS 2 Foxy, colcon, the system Python's Unitree SDK/native libraries and balance
runtime, and `.cache/groot_balance/GR00T-WholeBodyControl-Balance.onnx` in this
clone. Installation does not provision those dependencies. Service environment
variables `HIM_SAFETY_PROFILE` and `HIM_SAFETY_ROS_INSTALL` override the profile and
overlay path. Set them through a systemd user-service override if needed.

```bash
systemctl --user status g1-safety-stops.service
journalctl --user -u g1-safety-stops.service -f
./install_safety_stops.sh uninstall
```

Uninstall stops and disables the service and removes its unit. It leaves the
build files and user lingering in place. Stopping an active gate can request
damping, so finish hardware tests with the robot supported before removal.
See the [manual arm-raise guide](../tests/manual/README.md) to verify this
background gate through a policy run without starting a second motor gate.

## Node responsibilities

- `SafetyCheckerNode`: callback-driven validation of whole precheck frames using the existing core. One invalid joint latches the fuse, prevents all later relaying, and publishes abort immediately then every 100 ms. Each valid frame is relayed once, never replayed as a heartbeat.
- `MotorGateNode`: separate process with an independent 5 ms target timer. It watches postcheck freshness and immutable expiry, rejects replayed sequences and wrong sessions, and selects fake damping on abort or timeout. A frozen checker cannot refresh its deadlines.
- `ControllerStopNode`: observes physical Start, detects a double press and publishes abort directly to both nodes. It also aborts on stale feedback. Fresh DDS data does not prove a live radio link.

The core remains the ROS-independent checker logic. `motor_gate.py` contains the independently testable final-gate logic. The earlier `runtime.py` is retained as an in-process test/reference implementation, not the scheduler used by these ROS nodes. ROS executors run callbacks and timers. Run roles as separate OS processes in deployment to avoid a checker freeze freezing the gate too.

## Topics and services

| Topic | Typed message | Role |
| --- | --- | --- |
| `/safety/robot_feedback` | `RobotFeedback` | Read-only SDK bridge supplies measurements and physical Start |
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

In other terminals with the same setup/domain, use roles `motor_gate` and `controller_stop`. The read-only bridge is available below. A policy producer is still pending, so these commands do not constitute a full control test. With no feedback the controller-stop node deliberately aborts after its freshness timeout. Keep this isolated test graph separate from active robot control.

Run ROS graph tests after building and sourcing the overlay:

```bash
ROS_DOMAIN_ID=191 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 -m unittest discover -s tests/ros2 -v
```

Automated tests require localhost domain 191 and reject other domains. Domain 97
is reserved for the manual test launcher, including physical output.

The tests verify typed postcheck delivery, invalid-command abort delivery and independent gate damping when the checker stops executing. Ordinary safety tests do not require ROS:

```bash
python3 -m unittest discover -s tests -p 'test_robot_safety*.py'
```

## Next review checkpoint

The feedback adapter, manual policies, per-joint profile and opt-in hardware gate are implemented. A systemd-managed hardware watchdog is available below. Coordinated reset, publisher access isolation and frontend integration remain pending. Physical stop acceptance is unresolved after the reported joint shutdown. See [ESTOP_PLAN.md](../ESTOP_PLAN.md) for next-session work.

---

# Controller stop detection (dry-run)

`controller.py` separates SDK byte decoding from gesture recognition:

- `decode_start_pressed(payload)` reads bit 2 of byte 2 in the 40-byte G1 `wireless_remote` payload, following the installed SDK example.
- `StartDoublePressDetector.update(start_button_pressed, cur_time_secs)` emits a one-sample stop event for press → release → press within `Config.double_press_window_secs` (default 0.5 seconds).
- The detector requires a released sample after startup/reset. Holds and repeated pressed samples do not count again. A stale sample gap clears pending history. This does not establish radio-link health.

`Supervisor.state(observation, start_button_pressed, cur_time_secs, *, stop_requested=False)` now receives the physical Start state. It runs the detector and latches its event. The optional `stop_requested` argument accepts an independent event even with Start released. Physical Start blocks reset/arm, while the latch persists after release. Explicit reset clears gesture history and returns to inhibited, with separate arming required.

The old `stop_pressed` argument/status field is replaced by `start_button_pressed`. Callers that previously passed a stop event positionally must migrate to `stop_requested=`. Motor publication requires the explicitly enabled hardware gate.

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

The operator confirms G1 EDU, 29 DOF, seven joints per arm and no dexterous
hand attachments. On September 23, 2026, a read-only SDK subscription to
`rt/lowstate` on `eth0`, DDS domain 0, reported `mode_machine=5`. Unitree maps
this to `g1_29dof_rev_1_0`, with type 6 identifying the locked-waist counterpart.
The operator confirms the two waist fixing pieces are absent. This is consistent
with an unlocked three-joint waist. Waist yaw, roll and pitch retain SDK indices
12, 13 and 14. The existing 29-joint mapping already includes all three, so no
joint-count or mapping change is needed.

See the [official machine-type table](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_description/README.md).
This records the robot's reported configuration and the operator's inspection.
It does not verify calibration, motor health or control authority. No app setting
was changed, and no motion command was used for identification. Do not infer
movable joints from the fixed SDK message array.
The operator reports a 1,200-pound rated gantry/hoist with suitable harness,
feet bearing weight and arm clearance. Hardware command limits, gains and
mode-specific stop behavior remain unvalidated.

## Read-only hardware feedback checkpoint

Run on the same host as the other safety nodes, using the system Python SDK:

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=97
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_HOME=/home/unitree/cyclonedds_ws/install/cyclonedds
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib:$LD_LIBRARY_PATH"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run him_safety_nodes safety_node feedback --ros-args -p sdk_interface:=eth0 -p sdk_domain:=0
```

On this machine, the explicit SDK library path avoids Foxy loading an incompatible
Cyclone DDS library (`undefined symbol: ddsi_sertype_v0`). Fast DDS handles ROS
while the SDK uses its own Cyclone DDS installation.

`sdk_domain` selects the robot DDS domain independently of the ROS safety domain.
Use the robot-facing interface verified for your machine. This creates only an
SDK subscriber to `rt/lowstate`. ROS publishers are feedback and abort only.
Start the feedback bridge and wait for telemetry before the controller-stop
process. The bridge allows five seconds for initial DDS discovery, then applies
the 200 ms feedback deadline. The controller-stop process still requires feedback
within 200 ms of startup. No commands are permitted by this discovery grace. Coordinated
startup and reset are pending. Do not run beside an armed safety deployment.

`feedback.py` maps SDK indices 0 through 28 in their original order. Waist slots
remain present even if locked. Slots 29 through 34 are excluded, not interpreted
as additional joints. Positions are radians and velocities are radians/second.
`observation_id` remains zero until the checker assigns it.

The SDK callback copies state and stamps local monotonic receipt time into a
bounded FIFO. The ROS timer drains it without replacing button transitions with
only the latest sample. Invalid data, queue overflow, stale/reordered samples or
missing feedback latch abort and stop feedback publication until process restart.
The bridge never refreshes a cached sample timestamp. A bridge crash relies on
the downstream freshness watchdogs. DDS loss before the callback, frozen robot
telemetry and remote radio health are not independently detected by this bridge.
No robot tick or firmware version semantics are assumed.

Installed software observed September 23, 2026: Python SDK 1.0.1 at commit
`65691c8`, Cyclone DDS 0.10.2. PC4 manifest lists master service 1.0.0.2,
patch 1.0.0.6 and video hub 1.0.2.3. Live LowState version `[0, 0]` did not
identify motion-controller or motor firmware.

Live bridge smoke test on September 23, 2026: received 1,000 ROS feedback
messages from real `rt/lowstate`, each with 29 position/velocity slots, without a
bridge abort. The final sample age was approximately 5.7 ms, not a latency bound.
No motor output was created. The SDK printed `[Reader] take sample error` during
subscriber shutdown. Fault/gesture propagation through the complete live graph
is still pending.

## Manual arm-raise test

The one-off trajectory, producer, launch file and launcher live in
[`tests/manual`](../tests/manual/README.md). They are not installed as reusable
safety nodes. See that guide for the single-command dry-run test.

## Opt-in hardware output

See [HARDWARE_ADAPTER.md](HARDWARE_ADAPTER.md) for the implemented arm SDK adapter,
required profile and whole-body damping request path. Dry-run launch behavior is
unchanged. The operator has exercised the small elbow test and reported a joint
shutdown during a T-pose stop. Stop behavior is not accepted for deployment.
See [the next-session plan](../ESTOP_PLAN.md#next-session--september-25-custom-e-stop-damping).
