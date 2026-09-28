# Offline policy safety screening

This package runs explicitly adapted ONNX neural policies in MuJoCo. It produces
preflight evidence for operator review. A result is specific to the model,
adapter, robot model, controller settings, limits, and scenarios tested. It does
not certify hardware safety or authorize a robot launch.

## Install and run

Use a dedicated Python 3.12 environment. The robot host's default `python` is
Python 3.8 for ROS Foxy and cannot install this package's dependencies. Keep that
system interpreter unchanged. With `uv` installed, run from the repository root:

```bash
uv venv --python 3.12 .venv-policy-safety
uv pip install --python .venv-policy-safety/bin/python ./packages/policy_safety
source .venv-policy-safety/bin/activate
python --version
policy-safety prepare-g1 \
  --submission examples/sample_submissions/unitree_g1_stairs \
  --output /tmp/g1-screening-bundle
policy-safety evaluate \
  --model /tmp/g1-screening-bundle/walker.onnx \
  --adapter g1-walker-99-v1 \
  --robot-config /tmp/g1-screening-bundle/robot.json \
  --output /tmp/g1-screening-result
```

The version check should show Python 3.12. If you already have Python 3.12 and
prefer pip, use `python3.12 -m venv .venv-policy-safety`, activate that environment,
then run `python -m pip install ./packages/policy_safety`. The package requires
Python 3.10 or newer, and validation used Python 3.12.

If pip reports that `numpy>=1.26` is unavailable and lists versions only up to
1.24.4, first check `python --version` and `python -m pip --version`. On this host,
that error came from using Python 3.8 outside the safety environment.

The preparation command copies the example's ONNX and external weights, model
configuration, robot XML, meshes, and third-party notices into a portable bundle.
It adds a flat scene and explicit controller configuration. Output directories
must not already exist. Preparation reads files without importing the submitted
Python or running its scripts. The sample files must be present to prepare this
example, but are not dependencies of the installed package.

You can copy the bundle and install this package on another device. No repository,
ROS installation, robot network, or Unitree SDK is required to evaluate it.
MuJoCo and ONNX Runtime must have compatible wheels for that device.

The repository entry point accepts exactly the same arguments. Run it with the
safety environment activated as above:

```bash
python tools/check_policy_safety.py evaluate \
  --model /tmp/g1-screening-bundle/walker.onnx \
  --adapter g1-walker-99-v1 \
  --robot-config /tmp/g1-screening-bundle/robot.json \
  --output /tmp/g1-screening-result-2
```

## What the first adapter tests

`g1-walker-99-v1` implements the sample's neural observation contract: local base
linear velocity, local angular velocity, projected gravity, joint offsets, joint
velocities, previous network action, and commanded forward/lateral/yaw velocity.
The sample ONNX graph already contains normalization. Outputs become joint
position targets using explicit per-joint defaults and scales, with the sample's
position actuator gains and 20 ms control / 2 ms physics timing.

This tests all 29 network outputs. It excludes the submission's scripted phase
machine, hop modifications, and arm overrides. Consequently, its results do not
describe the complete stairs submission. Decoded targets outside actuator ranges
are violations even though the simulator clips applied targets.

Other ONNX exports need a trusted adapter matching their own contract. Matching
tensor dimensions is insufficient. Implement `PolicyAdapter`, register the adapter
in `ADAPTERS`, and extend the CLI choices and robot backend validation when adding
support. Adapters must save/restore all history and recurrent state and declare
observation frames, normalization, action semantics, gains, and timing. Arbitrary
submitted Python adapters are not auto-loaded. The initial simulator backend is
specific to the G1 walker contract and position actuators.

## Scenarios and limits

The default medium suite uses one seed (0). Six independent commands cover standing,
forward walking, lateral walking in both directions, and turning both ways.
Ten-second nominal runs yield checkpoints at 25%, 50%, and 75%. Each checkpoint
has a three-second unperturbed continuation and eight pushed continuations: pelvis
and torso, two horizontal axes, both signs. Pushes are 10 N for 100 ms, starting
one second into a branch. Six five-second counterfactuals cover standing, floor
sitting, supine, prone, and both sides. This is **174 scenarios**, totaling at most
**9.6 simulated minutes**: six nominal runs, 18 unperturbed continuations, 144
perturbations, and six counterfactuals. Wall-clock time depends on the device.

Pushes are deterministic, not random. The seed controls the small initial base
position offset. The medium suite omits arm pushes and additional seeds to reduce
runtime. To restore the original 954-scenario suite, pass
`--suite packages/policy_safety/profiles/extended-suite.json` from the repository
root. That suite represents up to 84 simulated minutes and may require a larger
`--timeout`. The JSON file can also be copied outside the repository.

Counterfactuals use zero commands and reset policy history. Only standing starts
use upright fall criteria. Floor-contact poses must initialize within the 5 mm
penetration tolerance. Poses do not require recovery. Each continuation restores
the full simulator integration state, including the base pose, joint positions,
velocities and solver state, plus the previous policy action, command and
random-generator state. A checkpoint is not just the 29 joint angles. Limits and
forces are checked/applied at every physics step.

`--suite FILE.json` overrides fields in `config.DEFAULT_SUITE`. For a short smoke
test retaining every scenario family:

```json
{
  "seeds": [0],
  "commands": {"stand": [0, 0, 0]},
  "nominal_seconds": 0.4,
  "branch_seconds": 0.2,
  "counterfactual_seconds": 0.2,
  "checkpoints": [0.25, 0.5, 0.75],
  "push_bodies": ["pelvis", "torso_link", "left_shoulder_roll_link", "right_shoulder_roll_link"],
  "push_start_seconds": 0.02,
  "push_seconds": 0.02
}
```

Two profiles apply by default. `--profile` selects global simulator limits
(`profiles/provisional.json`), including model joint/control/force ranges, a
0.05 rad measured-position tolerance, and provisional upright fall criteria.
`--joint-profile` selects per-joint movement rules. Its default is a packaged,
byte-for-byte copy of `robot_safety/profiles/g1_type5.template.json`, not the
relaxed `lenient` profile. No repository or ROS dependency is needed at runtime.

The joint rules actively reject target positions outside their individual bounds,
excessive commanded velocity, excessive changes in position and velocity targets,
nonzero feedforward torque, and target-to-measured tracking error. On the first
command they also enforce 0.03 rad position mismatch and 0.05 rad/s desired speed.

| Joint group | Target speed (rad/s) | Target velocity change per second (rad/s²) | Tracking error (rad) |
| --- | ---: | ---: | ---: |
| Waist yaw | 32 | 0.5 | 0.05 |
| Waist roll/pitch | 30 | 0.5 | 0.05 |
| Shoulders/elbows | 37 | 1 | 0.30 |
| Wrist roll | 37 | 1 | 0.05 |
| Wrist pitch/yaw | 22 | 1 | 0.05 |

Position-target deltas must satisfy `abs(q_target - previous_q_target) <= speed_limit * control_dt`.
Velocity-target deltas must satisfy `abs(dq_target - previous_dq_target) <= acceleration_limit * control_dt`.
Tracking error must satisfy `abs(q_target - measured_q) <= tracking_limit`.
Commands are checked before applying them. Tracking is checked again after every
physics step. Checkpoints preserve the previous accepted position and velocity
targets, so branches do not reset or bypass the delta checks.

At simulator startup, the configured standing or counterfactual pose is installed
as both measured position and held target. The shared startup check validates this
hold before inference. The first policy target then uses the ordinary command-delta,
tracking and torque checks against that baseline. The 0.03 rad alignment tolerance
is not applied to the first requested movement. Checkpoint branches retain their
previous command history and do not repeat startup. Hardware arm handover keeps its
existing initial-command alignment check.

The current position-target adapter has desired velocity and feedforward torque
equal to zero. Its commanded-velocity-change check is therefore trivially zero.
It does **not** bound measured acceleration or acceleration derived from target
positions. The position-target delta and tracking checks still apply. Simulator
timesteps supply `control_dt`, not wall-clock inference duration. Hardware
ownership handover and replacement of the first hardware output with measured
positions are not simulated. Policy-specific gains are retained rather than
replaced with the hardware profile's gains.

The standard profile covers **all 29 joints**, including the legs. Hardware and
simulation call the same dependency-free `robot_safety/joint_checks.py` functions.
The standalone wheel includes that source as `joint_checks` without ROS or SDK imports.
These limits screen submitted policies. Hardware F3 soft-reset GR00T balancing
does not apply them. The hardware gate remains restricted to arm/waist submissions.
Missing joint entries are errors, including older 17-joint custom profiles.

Leg position, target velocity and torque bounds use the Unitree model values.
Local commanded-velocity-change limits are 1 rad/s² for all legs. Tracking
allowances are 0.146667 rad for hip pitch/yaw, 0.231667 rad for hip roll,
0.173750 rad for knees and 0.218750 rad for ankles. They derive from
`0.25 * model_effort / reference_kp`, adapting NVIDIA's G1 action-scaling rule
as a provisional tracking-error budget. This adaptation is our screening choice,
not a published tracking safety threshold. The simulator retains its actual gains
for the independent full PD-torque check. See the
[profile documentation](../../robot_safety/profiles/README.md) for sources and derivation.

The evaluator checks PD torque demand before each physics step and applied joint
torque afterward. Both must satisfy the shared profile's Nm bounds. The existing
MuJoCo model bounds are also retained. Torque demand uses the simulator's actual
actuator gains, so clamping cannot conceal an excessive demand.

To select the current repository template explicitly, add:

```bash
--joint-profile robot_safety/profiles/g1_type5.template.json
```

Global measured velocity/acceleration, base-speed and contact-force maxima still
require separate thresholds. Scalar metrics use rad/s, rad/s², rad, m/s, rad/s,
N, and Nm as appropriate. Contact force is the largest contact force-vector
magnitude at that physics step, including self-contact.

Supply `--profile FILE.json` to set `maxima` and reviewed fall/position limits.
Limits are scalar maxima across joints, in addition to the model's per-joint
bounds. A profile marked `calibrated: true` requires every metric threshold and an
empty `uncalibrated_metrics` list. This flag records operator review, it is not
automatic calibration. Never tune thresholds solely to make a submitted policy pass.

## Reports, replay, and workflow

Each run writes `report.json`, `request.json`, `worker.log`, and numbered NPZ
traces. Reports include effective configuration, dependency versions, SHA-256
fingerprints, scenario seeds, coverage, first violations, maxima, and displacement.
All required model weights and the robot asset manifest are fingerprinted.
`joint_movement_checks` records the selected joint-profile values, rules and joint
coverage. `hashes.joint_profile` identifies its exact contents. Joint-rule
violations include the joint, observed value, threshold, units and simulation time.

NPZ files use plain numerical arrays and strings, loadable with
`numpy.load(path, allow_pickle=False)`. They contain initial and per-step MuJoCo
integration states, initial policy and RNG states, observations, raw actions,
decoded/applied targets, joint measurements, torque, base motion, metrics, and
external wrenches. `metric_names` defines the order of `metrics` columns.
`last_raw_output` preserves the last network output, including a rejected
nonfinite output when no physics step could be recorded.
Rejected commands are stored in `last_attempted_target`,
`last_attempted_clipped_target`, `last_attempted_target_velocity`,
`last_attempted_feedforward_torque`, and `last_measured_position_at_command`.
These are attempted commands, not necessarily applied commands. A pre-application
rejection can have zero physics samples. The two `initial_previous_target*`
arrays preserve the movement-check history for checkpoint replay.
For numerical replay, load the same robot configuration and dependency versions,
restore `initial_integration_state` with `mj_setState` and the saved `state_spec`,
restore policy/RNG state, then replay observations or saved controls and wrenches.
Exact replay across different dependency versions or devices is not promised.

| Exit | Meaning |
| --- | --- |
| 0 | All scenarios completed within a calibrated global profile with full joint-rule coverage |
| 1 | Violations found, without missing scenarios or execution errors |
| 2 | Error, timeout, invalid initialization, or incomplete coverage |
| 3 | Completed without violations, using an uncalibrated profile |

Incomplete coverage takes precedence over violations in the exit status. Both
remain visible in the report. A nominal failure marks unavailable checkpoint
branches incomplete. An episode that reaches a violation counts as tested and
terminates early. A worker failure preserves already written scenario results.
`--timeout` limits worker wall time, defaulting to 1800 seconds.
The terminal displays an updating progress bar every second, including elapsed
wall time, reported/total scenarios, violations (`v`), errors (`err`), skipped
branches (`skip`) and the active test where terminal width permits. Redirected
output uses plain lines every five seconds. Unreviewed profiles display
`UNCALIBRATED` throughout the run.
This is a heartbeat, not proof that inference is advancing. During initialization
it reports that the model and simulator are loading. `report.json` is updated
before and after each scenario. Set `--timeout 7200` for a two-hour wall-time limit
if needed. The full suite may exceed the default 30-minute limit on slower devices.

A **violation** means the policy produced an invalid output or a tested trajectory
crossed an enforced limit, including decoded target range, measured joint range,
or upright fall criteria. A configured metric maximum is also enforced. An
**error** means evaluation could not run reliably, such as an inference exception,
invalid starting pose, or simulator numerical warning/nonfinite state. Worker
crashes and timeouts mark the overall report incomplete. These outcomes do not
establish whether the policy is physically safe.

Zero violations only means no *enforced* checks failed in the scenarios reported
so far. The provisional global profile has no scalar thresholds for measured joint
velocity/acceleration, target rate, tracking error, base speeds, or contact force.
The separate joint profile nevertheless enforces command deltas and tracking for
its covered joints. `acceptance_checks` lists global gaps and
`joint_movement_checks` lists per-joint coverage. The evaluator checks unsaturated PD torque demand as well as the resulting
applied torque. The independent decoded-target check prevents target clipping from
hiding out-of-range requests. Counterfactuals run after the nominal and branch
tests for each seed, so they may not yet be represented in intermediate counts.

The repository wrapper makes the report and exit code available to deployment
tooling and operator review. Existing watchdogs continue checking live feedback
and commands. This utility does not publish ROS/SDK messages, arm or stop a robot,
or change watchdog services. Preflight results do not expand the hardware adapter's
arm/waist command scope.

The worker boundary and timeout contain ordinary evaluation failures. They are
not an operating-system security sandbox or memory limit for hostile models.
Use an appropriately isolated environment for untrusted submissions.

## Tests

```bash
python -m pip install pytest
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest packages/policy_safety/tests -q
```

Integration tests use the sample submission when available. They exercise the
neural contract, checkpoint replay, impulse duration, initialization, failures,
metrics, and CLI reporting. The real sample is not assumed to pass screening.
