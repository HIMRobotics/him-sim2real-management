# G1 checked hardware output adapter

The adapter sends approved commands to the robot using the Unitree SDK. It acts
as the middleman between the safety checker and the actual robot motors.

`hardware_gate` replaces the diagnostic `motor_gate` process. The other safety
nodes keep using the same topics and arm service. The default dry-run launcher
still starts only the diagnostic gate. Never run both gates together.

## Output and stop paths

```text
precheck -> checker -> postcheck -> hardware gate -> rt/arm_sdk
controller abort ----------------------|                  |
                                      |             upper-body targets
                                      |             stop: hold measured pose, weight 1
                                      -> SetFsmId(1) only if that hold cannot continue
```

`hardware_node.py` connects the existing session/sequence/expiry gate to
`hardware_adapter.py`. An SDK output thread checks local feedback and the original
command expiry independently of ROS scheduling. It does not renew expiry when
republishing a target. A separate thread requests damping after a latched stop,
including when the output worker stops making progress. These threads are not
independent computers or a hardware emergency-stop circuit.

`unitree_transport.py` publishes CRC-protected `LowCmd` frames to `rt/arm_sdk`,
following the official upper-body example. Indices 12–28 contain waist and arm
targets. Index 29 carries ownership weight. Explicit `whole_body_output:=true`
selects the existing `rt/lowcmd` encoder for all 29 checked joint commands.
The soft-reset balance helper calls this same encoder.
Upper-body ownership ramps from zero to one with a smooth cubic curve. Commands
use the profile gains and validated feedforward torque. Proposals are clipped to
joint-position bounds with warnings in `/safety/joint_diagnostics` and ROS logs.
The command-to-measurement gap is diagnostic only, including at startup.
Total PD plus feedforward torque and desired speed must satisfy per-joint bounds.
Stationary-start velocity checks remain enabled. No pose-gap threshold stops motion.

An abort stops subsequent motion writes. Runtime motion and health faults request
stock damping directly. For other aborts, if this process has taken arm ownership
and feedback is fresh, the stop worker keeps publishing the measured upper-body
pose from the moment of the stop: target speed 0, profile `kp` and `kd`, zero
feedforward torque, and ownership weight 1. The legs stay in the current standing
mode. This hold falls back to `SetFsmId(1)` when feedback goes
stale, an arm write fails, or the process is exiting. That fallback then writes a
zero-weight arm release and queries the stock FSM. Repeats continue while the
process is alive. A zero-weight release alone is not whole-body damping.
In-flight DDS writes cannot be recalled. Firmware priority between damping
requests and arm traffic is still unverified.

The SDK callback checks `mode_machine=5`, copies fresh measured state and detects
Start double presses directly, in addition to the existing ROS controller-stop
node. Arm requires fresh feedback with all 29 joints within their position and
initial-speed bounds, Start and F3 released, and a
stock FSM included in the explicit profile. It never requests standing, releases
the stock motion service, changes calibration, or automatically rearms.

While armed, all 29 proposals are validated, but only waist/arm commands are
written. Fresh feedback checks model velocity ceilings, total estimated torque,
available measured torque, 6 rad/s body spin per axis and 1 rad upright tilt.
Temperature ceilings are 85 C casing and 120 C winding, with a 20% minimum battery
charge. BMS arrives separately on `rt/lf/bmsstate` with a local 5 s freshness
limit. Missing health data prevents arming. Runtime motion and health faults
request damping. The arm SDK path still cannot control the legs.

Diagnostics distinguish pending `command_gap_rad` from `response_error_rad`
measured after a preceding successful full-weight waist/arm write. Rejected or
failed writes never become `q_applied`. No response-error threshold stops motion.
See the [profile guide](profiles/README.md) for state names and exact formulas.

## Required hardware profile

`profiles/g1_type5.template.json` contains the settings used by the explicit
hardware test launcher. Official joint limits and local tolerances are documented
in [profiles/README.md](profiles/README.md). The separate proposed JSON was removed.
These settings still require physical validation.

Required fields:

- `expected_machine_type`: 5, matching the observed regular 29-DOF revision-1.0 model.
- `allowed_start_fsm_ids`: `[4]`, the locked-standing/ready state confirmed by the
  operator during a successful read-only SDK query on September 24, 2026. This
  operating-state ID is distinct from `mode_machine=5`, the hardware model.
- `handover_secs`: ownership ramp duration.
- `max_publish_gap_secs`: maximum output-worker gap, no greater than 0.2 seconds.
- `initial_velocity_tolerance_rad_secs`: required near-stationary start tolerance.
- `joints`: all 29 named joints, including legs. Each needs a positive `max_torque_nm`, position bounds, positive
  `kp` and `kd`, and maximum velocity.

The adapter's feedback deadline is 200 ms. Its output cadence targets 20 ms, matching the operator-tested SDK example.
The separate stop watchdog polls every 5 ms.
Stop-worker RPC calls have a 100 ms SDK timeout, with retries no sooner than 100 ms after the
previous attempt completes. These are software settings, not physical stop-time
guarantees. Stock whole-body damping gains are not controlled by `Config.damping`
or by this profile's arm-control `kd`.

After ROS build/environment setup, the hardware gate is explicitly selected with:

```bash
ros2 run him_safety_nodes safety_node hardware_gate --ros-args \
  -p enable_hardware_output:=true \
  -p hardware_profile_path:=/absolute/path/to/reviewed-profile.json \
  -p sdk_interface:=eth0 -p sdk_domain:=0
```

This is a hardware-capable command. No motor writes occur merely from starting
it. Successful arming enables automatic fault damping, and subsequent accepted
commands can move the robot. Explicit double Start can request damping before arming. The test launcher selects it only with explicit hardware enablement.
The existing Start service can arm this gate, so do not attach it to an unrelated
running test graph. Only one authorized arm publisher may be present. Publisher
exclusivity is an operational prerequisite, not enforced by ROS topic names.

## Evidence and limitations

`/safety/hardware_status` reports output enablement, arm authorization, abort reason,
damping RPC result, observed FSM and arm-release write result. A reported damping
FSM or successful write is not physical confirmation. `armed` records the granted
session, and does not clear the abort latch. After abort, restart is required and
fresh commands cannot clear it.

Orderly shutdown attempts damping for up to one second before worker shutdown.
It is best effort. SIGKILL, power loss, a frozen interpreter, hung SDK calls or a
network failure can prevent stopping. A service restart is not a motor watchdog.
Robot-side command-loss behavior and damping priority require supported hardware
validation before relying on this path. DDS delivery does not prove fresh radio
input or actual motor execution. No end-to-end 200 ms actuator guarantee exists.

The operator reported a successful small elbow test and a jerky T-pose stop
that shut down a joint. Physical stop acceptance remains unresolved. Stronger
custom damping and its transition are next-session design work in
[ESTOP_PLAN.md](../ESTOP_PLAN.md). Current stops still use stock FSM 1.

References:

- [Unitree arm SDK example](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/g1/high_level/g1_arm7_sdk_dds_example.py)
- [Unitree locomotion client and damping RPC](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/g1/loco/g1_loco_client.py)

## Joint-limit sources

The profile uses the official revision-1.0 model's position/velocity limits
for all 17 waist/arm joints.
The source values and checksum are recorded in
`robot_safety/profiles/g1_type5.official_limits.json`. Gains still match the tested
SDK example. Handover settings remain local choices. Model
effort ceilings are checked against total estimated torque and available torque feedback.

## Launch the complete physical test

From the repository root, run `./tests/manual/run_unitree_arm_test.sh --hardware`.
Once ready, run `./tests/manual/run_unitree_arm_test.sh --start` in another terminal.
The first command starts the five processes, replacing the diagnostic gate with
the hardware gate. The second requests arming and starts the SDK arm sequence.
Double Start latches stop and repeatedly requests whole-body damping. See
[the manual test instructions](../tests/manual/README.md#physical-sdk-arm-test).

## Diagnosing an arming rejection

A readiness or arming rejection before control is granted latches a software
abort without sending damping or arm-release commands. The robot remains under
its stock controller. Restart the test graph after correcting the issue.
After successful arming, faults still request whole-body damping.
Explicit double Start requests damping even during the readiness check.

The producer first calls `/safety/motor_gate/ready`. This read-only SDK mode query
uses a separate client with a 2-second timeout and a 5-second producer deadline.
Only after success does it arm the controller/checker/gate sequence. This prevents
the checker's 200 ms command watchdog from expiring during the startup query.
The mode confirmation must still be less than 200 ms old when the gate arms.
Fresh feedback, released Start and stationary-joint checks are repeated then.
Active command and feedback deadlines and the stop-worker timeout are unchanged.

The gate logs `Hardware readiness rejected:` or `Hardware arming rejected:`
with the failed check and measured value. The same detail appears in the gate
service response and `/safety/hardware_status.arm_rejection_reason`.
The startup speed threshold remains 0.05 rad/s on every waist/arm joint.
SDK code 3104 means client API timeout. This does not establish a speed violation.

Startup retries a code-3104 timeout once, using a new SDK request. Each attempt
has a 2-second SDK timeout, within a 5-second producer deadline. Other errors
and successfully reported incorrect modes are not retried. Both failed attempts
still reject startup without damping. Logs include the code, mode and elapsed
time for each attempt. This tolerates a transient lost reply but does not
establish the cause of the observed timeouts. Stop RPC timing is unchanged.

## Same container in simulation and hardware

`setup.sh` now defaults to preflight. It validates the selected joint profile and
reports code, profile and image hashes without starting robot processes.
The example container consumes `/safety/policy_state` in either environment,
then sends the same `JointCommand` through the checker and motor gate. Inference
stays inside the container. `/sim/state` remains a compatibility/reporting topic.

Prepare one example image and select the generated profile in both backends:

```bash
python3 containers/examples/prepare.py walker
docker build -t g1-policy:local .cache/sim_examples/walker/container
export G1_JOINT_PROFILE="$PWD/.cache/sim_examples/walker/container/project/joint_profile.json"
G1_TASK_COMMAND='[0,0,0]' ./sim.sh --image g1-policy:local --seconds 10
./setup.sh --image g1-policy:local --profile "$G1_JOINT_PROFILE"
```

For Compose, `G1_JOINT_PROFILE` must name the profile inside the `/workspace`
mount. The generated profile retains the current physical limits and selects
the model's gains. Review it before deployment. The gate freezes that profile
at startup, overrides submitted gains with its gains, and checks torque with
those exact values. Requalify changes instead of loosening failing limits.

Hardware observations reuse low-state joints/IMU. Policies needing translation
also require an explicitly configured `SportModeState_` SDK source:
`G1_BASE_STATE_TOPIC` and `G1_BASE_VELOCITY_FRAME=world|body`. Position must already
be in the same world frame as the low-state quaternion. Verify this contract,
measurement accuracy, advancing source timestamps and availability after stock
control release under support. There is no assumed topic or zero-velocity
fallback. Set `G1_REQUIRE_BASE_STATE=false` only for a policy such as stationary
balance that needs neither base velocity nor position. Movement trials require
both. Unsupported sensor contracts need a bridge extension, not guessed data.

```bash
# Same live feedback, checks and inference, with no motor publishers/control RPCs.
./setup.sh --image g1-policy:local --profile "$G1_JOINT_PROFILE" --mode dry-run
# Only after supported qualification and deliberate watchdog-service transition.
./setup.sh --image g1-policy:local --profile "$G1_JOINT_PROFILE"   --mode hardware --enable-hardware-output
```

The launcher uses a separate localhost ROS domain (98 by default), starts the
existing feedback, checker, controller-stop and hardware-gate roles, and records
their logs under `.cache/hardware_runs`. Dry-run leaves the background service
alone. Hardware mode refuses an active `g1-safety-stops.service`; transition it
manually while the robot is supported. The gate and direct balance launcher
share `/tmp/g1-motor-writer.lock`. Already-running older software must be stopped
before relying on this exclusion. No service is automatically restarted.

Without a trial file the example requests zero task velocity and maintains
policy control until stopped. `G1_TASK_COMMAND` selects the same explicit task
command in either backend. Whole-body faults restore stock motion and request
damping, retaining the fault latch. Restart the entire session deliberately
after recovery. RPC success is not proof of physical stopping.

## Measured trial campaign

Qualify the exact checked path in simulation, then hardware inference without
output. Under the gantry, verify takeover, zero-command standing, normal stop,
controller/fault stop and stock restoration. Record tested ramps and braking
distance. Keep monitoring during calibration; scored trials require
`G1_THRASH_MODE=enforce` (simulation: `G1_SIM_THRASH_MODE=enforce`).

Set `G1_TRIAL_CONFIG` to a reviewed JSON file for either backend. Required fields
are `phase` (`supported_straight`, `unsupported_straight`, or `circle`),
`qualified: true`, `ramp_secs`, `braking_distance_m`, `settle_secs`, and
`stationary_speed_m_s`. Numeric settings must be positive measured/qualified
values. `qualified` records operator signoff, not an automated safety claim.
The provisional yaw settling threshold is 0.05 rad/s. Never invent braking data
to enable a trial. Initial speeds are fixed at 0.10 m/s and circle yaw at
0.10 rad/s, ramped together. Changing these settings requires requalification.

Straight trials target 2 m with endpoint error at most 0.10 m and maximum
lateral deviation below 0.20 m. Circles require a measured counterclockwise full
lap of radius 1 m, radial error below 0.20 m and return error at most 0.20 m.
The 40/100 s deadlines include braking and settling. Timeout fails the trial.
Successful completion continues zero-command standing control until the operator
ends the session. A trial's geometric result is provisional until logs and
physical observations are reviewed, including clipping warnings and any timing
violations, safety trips, joint faults, falls or balance assistance.

Require three reviewed clean supported straight runs, then three unsupported
straight runs, then three circles. Advancement is manual. Supported trials keep
feet weight-bearing, with the gantry following without towing and a second
operator handling stopping. Hashes, commands, measured state, inference timing,
trial results, clipping diagnostics, thrash telemetry and stop reasons are saved
in the run logs. Configuration/policy changes require renewed qualification.
Hardware joint diagnostics include the most recent applied targets, velocities,
feedforward torques, effective gains and write timestamp, sampled once a second
and on faults. Proposed container commands are logged at inference cadence.

## Implementation validation

The implementation passed 116 safety tests, 8 balance/trial/preflight tests,
72 simulation tests, and 26 distinct ROS integration tests. These include fake
hardware and the shared container client through the existing checker/gate.
Use focused tests for subsequent changes rather than repeating all suites.

Docker image build/run remains unverified because this account cannot access
the Docker daemon. No hardware output was enabled or robot services restarted.
Measured base feedback after control release, physical stopping/restoration,
and the supported qualification campaign remain to be validated on the robot.
