# G1 checked hardware output adapter

The adapter sends approved commands to the robot using the Unitree SDK. It acts
as the middleman between the safety checker and the actual robot motors.

`hardware_gate` replaces the diagnostic `motor_gate` process. The other safety
nodes keep using the same topics and arm service. The default dry-run launcher
still starts only the diagnostic gate. Never run both gates together.

## Consolidated policy deployment

The supported policy route is `submit.sh --mode real` → `setup.sh`
→ Docker policy container → checker → `g1-safety-stops.service` → the existing
single motor writer. The container publishes `/safety/precheck`. The host runs
startup Balance and applies joint commands. Uploaded images use this same gate.
The arm-only manual examples below retain stock leg control and are not this
whole-body policy deployment route.

Whole-body deployments default to the bundled official GR00T Balance checkpoint
before submitted actions reach the motors, including the Holosoma example.
An explicit `--startup holosoma` overrides the startup controller.
Double F3 uses a separate Holosoma standing policy at zero velocity on all
29 joints with model gains. It resets inference history from measured state
and skips the startup crouch ramp. Recovery shares the 20 ms policy clock and
2 ms command refresh. Double Start still requests stock damping.

The startup gate requires at least 2 seconds of Balance and a continuous 1-second window with tilt at most 0.15 rad, gyro norm
at most 0.25 rad/s, and every joint speed at most 0.6 rad/s. Samples must be finite
and younger than 200 ms. Repeated timestamps or SDK ticks do not advance the
window. These provisional thresholds passed local simulation, not physical
qualification. Stillness does not establish weight-bearing under a gantry.

Hardware continues Balance while waiting for a new left + B press after
`handoff_ready: true`. Early/held presses cannot queue a launch. Instability
revokes readiness. Failure to establish stability within 30 seconds faults.
GR00T startup balance controls the lower body and eases the arms from their
measured pose to the sim stand keyframe (all arm joints at zero: elbows bent
90 degrees, forearms forward), peaking at 0.5 rad/s. Readiness waits for the
arms to arrive. Arm targets transition to the submitted policy only after
the operator authorizes handoff.
Once stable, Balance continues while waiting for authorization without an
operator-wait timeout. Losing stability starts a new 30-second recovery deadline.
Health, command and telemetry deadlines remain active throughout. Targets,
desired velocities, feedforward torques and gains blend over 1 second. Torque checks use the gains actually written. Double Start, health
checks, command expiry, output watchdogs and fault latching remain active.

`/safety/hardware_status` and `/safety/policy_state` expose `startup_phase`,
`startup_stable`, `handoff_ready` and `policy_handoff_at_secs`. `control_ready`
marks authorized policy handoff. Local, container and direct simulation use the
same stability gate and authorize automatically after stability. The shared
example client warms inference/history using live state with zero velocity
before handoff. Command schedules and trial clocks start at handoff. Uploaded
clients must honor `control_ready` for their own clocks and maintain observation
history while waiting. The host inhibits their output regardless of compliance.

### Separate startup and soft reset

Startup Balance begins automatically after arming. It does not synthesize a
controller button or announce "Soft Reset". D-pad left + B authorizes policy
handoff after stability. Double F3 separately cancels the submitted policy and
runs the existing recovery Balance path. Recovery continues until a hard stop,
a real fault, or service shutdown, with no scheduled transition to damping.
Double Start retains priority and requests damping.

After policy handoff, an automatic fault selects zero-velocity Holosoma
soft reset only when the measured body-up axis dotted with world-up is at least
`cos(42 degrees)` and estimated pelvis height is at least **0.4884 m**.
`hardware_constants.py` defines the **0.74 m** nominal GR00T base height and
the **0.66** minimum fraction. Both orientation and joint telemetry must be
finite and less than 200 ms old. Missing, invalid or stale telemetry selects damping.
The 42-degree standing threshold is separate from the tighter startup readiness threshold.
The original fault stays latched and submitted commands remain inhibited.
`/safety/hardware_status` reports `automatic_soft_reset` while this path is active.

`standing.py` computes vertical pelvis-to-sole distances using both measured
six-joint leg chains and the normalized pelvis IMU quaternion. Geometry matches
the bundled G1 model, including the sole contact shapes. The lowest sole defines
the level-ground height estimate so a raised swing foot does not lower it.
On the gantry, planting the feet is the operator's responsibility. Software
does not add a contact or weight-bearing gate. The 66% cutoff is provisional,
not a physically qualified recovery limit.

Before handoff, startup balance faults damp after ownership rather than entering
automatic recovery. During startup balance and submitted policy operation,
fresh feedback failing either standing check immediately clears body commands
and requests damping, without a persistence timer. Both checks also run before
and after startup inference. Explicit double F3 can still request recovery before
handoff, but must pass the same standing checks before releasing stock control.

Double Start and service shutdown still damp. Failed writes, uncertain motion
release, output-worker stalls and invalid hardware state also retain damping
because recovery requires a working output path. Recovery needs a loaded policy
and never takes ownership solely because a fault occurred before arming/release.
Automatic and double-F3 recovery recheck height and orientation on feedback,
before inference and before motor output. Recovery damps if height drops below
0.4884 m, tilt exceeds 42 degrees, telemetry expires, or inference/output fails.

The recovery writer updates its output heartbeat so the stopped submission's
old timestamp cannot trigger a false stall. Repeated notifications of the
latched fault and the policy client's normal exit cannot convert recovery into
damping. A new fault during recovery still overrides it. The recovery helper
retains its trained gains, but its targets are clipped to the physical joint and
torque envelope. The shared health check includes standing height, tilt, telemetry
freshness, battery charge and motor temperatures. It runs on feedback, before
inference and before motor output. A failure of those checks enters damping.

A finite measured torque overload stops the submitted policy and selects recovery
when the health and standing checks permit it. Recovery records measured overloads
as `measured recovery torque` diagnostics instead of rejecting its first command
or damping solely because the overload persists. There is no torque-only grace
timer. Nonfinite torque feedback still fails the health check. Recovery command
targets retain the 90% predicted torque budget and the hard command checks.
Gantry startup overloads continue to damp without automatic recovery.

### Initial takeover and readiness

Whole-body arming checks the accepted stock mode, fresh telemetry, health,
released stop buttons, absolute joint positions and physical joint-speed ceilings.
A failed stock-mode query rejects arming for both startup policies; an unknown
mode is never treated as a confirmed damping or developer mode.
It does not reject a start because one measured velocity exceeds the old
0.05 rad/s near-stationary tolerance. There is no separate stationary wait,
position-drift gate or per-joint RMS gate. The initial desired command-speed
bound and the legacy arm-only measured-speed check retain the profile tolerance.

Real deployments default the existing weighted oscillatory RMS detector to
`enforce`. It runs in the checker once armed and faults sustained thrashing
through the existing abort path. `G1_THRASH_MODE` remains an explicit override,
and the launcher prints the selected mode. Its weights, filtering, score limit
and persistence are unchanged. This metric detects oscillation, not all motion
or actual kinetic energy. Physical joint-speed and torque ceilings, tilt checks,
telemetry and command deadlines, and controller stops remain independent.
The measured Balance stability window still gates the later policy handoff.

The initial stock-controller to Balance transition is distinct from the later
left + B policy handoff. After a successful release, the output worker rechecks
feedback, command expiry and stops immediately, without another 20 ms sleep.
It resets Balance history, captures the measured pose and begins at that position
target. During `startup_phase: entering_balance`, position targets blend from
that captured pose to live Balance outputs over 0.5 seconds using cubic smoothstep
(`3t² - 2t³`). Balance gains remain active, desired speeds and feedforward torques
remain zero, and the existing torque projection and hard checks still apply.
The stability window starts only after entry completes. Left + B cannot bypass it.

This reduces the initial position-target jump. It does not preserve the stock
controller's supporting torque, remove delays inside its release RPC, establish
foot loading, or guarantee smooth physical motion. A fault after release invokes
the stop path even if the first body command has not yet been written. Failed
release plus failed restoration is treated as uncertain ownership.

Offline comparison used the first recorded poses from hardware logs
`20261008-080746` and `20261008-081225`, placed on simulated flat ground.
The 0.5-second entry reduced peak joint speeds from 3.41/3.18 to 2.11/2.17 rad/s
over 8-second runs. Both remained upright. A 3-second experimental entry caused
falls in both cases and is not deployed. The final implementation also completed
a 10-second first-steps simulation. Reports and experiment code are in
`.cache/takeover-validation/`. Simulation records some torque exceedances as
warnings, including proposals that the host inhibits, and is not a hardware
qualification. No gantry or stock-controller release is simulated. Physical
jerking remains an unresolved commissioning issue until operator testing confirms
the cause and response. No physical joint commands were issued to validate this change.

### Torque-limited targets

Whole-body output projects excessive position targets into a feasible PD torque
interval using the current measured position/velocity and the gains being sent.
The command budget is 90% of each existing torque ceiling, leaving headroom
between output cycles. A 35 Nm ankle limit therefore uses a 31.5 Nm predicted
command budget. The hard profile limits are unchanged.

The limiter applies to startup Balance, handoff and submitted whole-body output.
Checked simulation applies the same projection. `torque_target_clipping` in
joint diagnostics records requested effort, the target correction and its count.
Normal proposals inside the budget are unchanged. During startup and submitted
policy control, measured torque and effort recomputed from the last written
command still fault at the original ceiling. That fault stops the submission,
with recovery eligibility evaluated separately through the health pipeline.
Nonfinite inputs, excessive feedforward torque and infeasible combinations of
position and torque constraints still fault on hardware.

This avoids rejecting a new proposal solely for a small predicted overshoot.
It does not guarantee actual torque or stability between samples. Persistent
large corrections require review of the stance, support and policy behavior.
The change passed targeted math/fake-hardware checks and nominal standing and
first-steps simulations. It has not been actuated on the physical robot.

### Operator commands

Prepare and build without starting robot processes:

```bash
cd /home/unitree/him-sim2real-management
.venv-policy-safety/bin/python containers/examples/prepare.py walker
sudo docker build -t g1-example-walker:local .cache/sim_examples/walker/container
sudo ./setup.sh --mode preflight --image g1-example-walker:local \
  --profile .cache/sim_examples/walker/container/project/joint_profile.json
```

Start the zero-velocity commissioning session yourself, with the robot in its
accepted ready mode, feet weight-bearing and fall-arrest support available:

```bash
sudo ./submit.sh --mode real --image g1-example-walker:local \
  --profile .cache/sim_examples/walker/container/project/joint_profile.json \
  --command containers/examples/commands/walker/stand.yaml
```

This command enables joint output for startup Balance immediately after arming.
Left + B gates policy handoff, not the initial Balance actuation. Watch for
`startup_phase: ready`, then release and press left + B. The supervisor may
reload the older safety service to install this gate and select the requested
profile. This reload belongs to the operator-run procedure.

After reviewing zero-command takeover and controller stopping, use the same
launch command with `--command containers/examples/commands/walker/first_steps.yaml`.
It requests 2 seconds standing, 1 second at 0.05 m/s, 2 seconds at 0.10 m/s,
1 second at 0.05 m/s, then zero velocity indefinitely. Zero velocity retains
Balance control. Double Start requests damping. End the container with Ctrl+C
or `sudo ./submit.sh --mode real stop` after the operator has secured the robot.
The long-lived safety service stays running.

Do not launch the image with a standalone `docker run`: the supervisor supplies
host checks, service configuration, command mounts and logs. Logs are under
`.cache/hardware_runs/`. Dry-run rejects `--enable-hardware-output` and disables
transport writes/control RPCs. Preflight starts no robot processes.

The hardware terminal hides routine telemetry, bundle hashes and startup banners.
It prints `D-pad Left + B detected` once per new press and preserves errors,
tracebacks and the safety-service reload warning. Detection alone does not mean
handoff was accepted. The existing stability gate and button mapping are unchanged.
Full container output remains in the run's `policy.jsonl`, with code/image/profile
identifiers in `configuration.txt`. This filtering also works with existing
example images. The host service must load the updated button-event telemetry.

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
Temperature ceilings are 85 C casing and 120 C winding, with a 10% minimum battery
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
- `initial_velocity_tolerance_rad_secs`: initial desired command-speed bound and
  legacy arm-only measured-speed tolerance. Whole-body measured speed uses the
  physical per-joint ceilings, not this near-stationary threshold.
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
session, and does not clear the abort latch. Fresh commands cannot clear it.
Follow the [manual recovery procedure](../tests/manual/README.md#recovery-between-trials)
before starting another arm test.

Orderly shutdown attempts damping for up to one second before worker shutdown.
It is best effort. SIGKILL, power loss, a frozen interpreter, hung SDK calls or a
network failure can prevent stopping. A service restart is not a motor watchdog.
Robot-side command-loss behavior and damping priority require supported hardware
validation before relying on this path. DDS delivery does not prove fresh radio
input or actual motor execution. No end-to-end 200 ms actuator guarantee exists.

The operator reported a successful small elbow test and a jerky T-pose stop
that shut down a joint. Physical stop acceptance remains unresolved. Damping
requests use stock FSM 1.

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
[the manual test instructions](../tests/manual/README.md#run-an-arm-test-through-the-systemd-gate).

## Diagnosing an arming rejection

A readiness or arming rejection before control is granted latches a software
abort without sending damping or arm-release commands. The robot remains under
its stock controller. Restart the test graph after correcting the issue.
After whole-body ownership, startup faults damp. After policy handoff, automatic
faults select standing recovery or damping as described above. Arm-only faults
retain the existing hold/damping behavior.
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

Hardware uses localhost ROS domain 97 and the persistent safety service. Dry-run
uses domain 98 by default and leaves that service alone. The supervisor records
logs under `.cache/hardware_runs` and reloads an outdated or latched service
during an operator-run hardware launch. It verifies the startup gate and exact
profile before opening a policy session. The gate and direct balance launcher
share `/tmp/g1-motor-writer.lock`.

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
to enable a trial. `speed_m_s` defaults to 0.10. For straight trials, optional
`heading_gain` and `lateral_gain` steer relative to the measured starting pose,
bounded by `max_yaw_rate_rad_s` (default 0.30). Both gains default to zero.
Circles use yaw rate equal to forward speed for the 1 m radius. Forward and
yaw commands ramp together. Changing these settings requires requalification.

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

Startup-gate unit tests cover persistence, early authorization, instability,
repeated/stale telemetry, timeout, invalid telemetry, missing models, inference
failure, blended gains and latched faults. Fake hardware tests cover actual-gain
torque checks, repeated SDK ticks, aborts during Balance/handoff and blocked
writes. Launcher tests use inert runtimes. Four ROS hardware-node tests passed
with fake SDK transport on reserved localhost domain 191.

Local walker, scheduled walker, direct walker and Balance simulations completed
10 seconds without a fall. Handoff occurred after 2 seconds of measured
stability. The 12-second first-steps command simulation also completed without
faults or a fall. Reports/traces are in `.cache/startup-validation/`.
These results do not qualify physical takeover or stopping.

The host Python 3.8 Balance inference warmup passed with the bundled model,
whose SHA256 matches `containers/examples/policies/walker/provenance.json`. The running
hardware service was inspected but not restarted. Docker build/run remains
unverified because the account lacks Docker socket access. No physical joint
commands were sent. The operator runs the build and hardware commands above.
