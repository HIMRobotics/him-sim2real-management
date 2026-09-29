# Policy safety validation

Current setup, limits, and scenario definitions are in [README.md](README.md).
The dated entries below preserve results from the versions tested at the time.

## Regression checks — September 29, 2026

- Policy-safety suite: 74 tests passed in `.venv-policy-safety`.
- Repository unittest suite: 125 tests passed. Local HTTP tests required socket
  access outside the execution sandbox.
- Changed documentation's local links resolve. `git diff --check` and the
  safety-stop launcher's shell syntax check passed.

These checks did not run the full screening battery or operate robot hardware.

## Recorded sample observations — September 29, 2026

These observations were moved from the former `SCREENING.md`. They describe
selected sample runs and do not establish completion of the 402-scenario suite.

Standing and forward walking missed their targets without sustained flailing.
The clearest tracking miss was at the ankles: a planted foot stayed near
−0.4 rad while the network requested about +0.1 rad, with about 15 Nm applied.
The recorded interpretation was insufficient policy torque to overcome the
floor constraint. The seated start requested targets outside joint ranges and
produced sustained high joint speed.

The medium pelvis push produced a 9.5 rad/s peak lasting 12 ms. The heavy push
produced a 13 rad/s peak lasting 18 ms, with the commanded target stationary.
The seated run stayed above 9 rad/s for about 300 ms and peaked at 27 rad/s.
These observations motivated the 50 ms overspeed hold: brief foot impacts do
not count as flailing, while the seated motion does. A 100 ms hold would add
50 ms before detection, equivalent to about 80 degrees at a constant 27 rad/s.
This is the recorded rationale, not a measured stop-distance guarantee.

## September 28, 2026

### Startup alignment separated from policy motion

The simulator validates the initialized hold with the shared startup checks before
inference. It then seeds command history with that hold, so the first policy
output uses the same delta/tracking/torque rules as later outputs. Hardware arm
handover behavior and the soft-reset exemption are unchanged.

- 152 tests and 70 subtests passed, including a 0.04 rad first movement for both
  a leg and an arm, rejection of an initial hold mismatch, and checkpoint history.
- Rebuilt and installed the wheel, then evaluated the actual sample with the
  default 174-scenario suite. It reported 12 violations, zero errors and 162
  unavailable branches, returning exit 2.
- The sample no longer fails the 0.03 rad startup check. Standing instead fails
  the ongoing left-knee tracking bound: 0.196988 rad requested error versus
  0.173750 rad. This still occurs before the first physics step, so this run
  provides no walking or perturbation validation. Limits were not retuned to pass it.

The following results predate this startup correction.

### Shared checks and leg coverage

The hardware adapter and simulator now call the same stdlib-only command checks.
The standard and lenient profiles include all 29 joints and torque bounds. Leg
tracking tolerances now use a provisional quarter-effort budget derived from
NVIDIA G1 action scaling and this profile's model effort/reference gains. This
is a local adaptation, not an upstream tracking safety threshold. Commanded
acceleration limits remain local settings.

- Regression suite after the soft-reset scope correction: 150 tests and 70 subtests passed. Tests cover all-joint tracking, commanded velocity
  changes, both signs of torque, leg target deltas, and rejection before actuation.
- Hardware tests use fake transport. Soft-reset GR00T balancing bypasses submission
  command and torque checks. Inference failures still request damping.
- PD demand is checked before each physics step, independently of actuator clamping.
  Applied torque is also checked. Hardware checks estimated demand, not measured torque.
- A standalone wheel installed and ran from `/tmp` without ROS or repository imports.
  Its report exactly matched the repository wrapper, including the shared-code hash.
- The real sample's default suite reported 12 violations, zero errors and 162
  unavailable checkpoint branches, returning exit 2. All 29 joints were covered.
  Standing failed its first leg target mismatch (left hip yaw, 0.072946 rad versus
  the 0.03 rad startup allowance), before any physics step.

This supersedes the 17-joint coverage described below. No hardware services were
restarted and no motor commands were sent. These are not calibrated walking limits.

### Earlier arm/waist enforcement update

The evaluator now loads the packaged standard hardware template by default and
enforces its 17 arm/waist command and tracking rules. This supersedes the earlier
screening behavior described below, which did not enforce those rules.

- Package tests: 57 passed, including comparisons against the real hardware
  adapter's command checker, per-joint boundaries, pre-actuation rejection,
  tracking checks between policy updates, checkpoint command-history restoration,
  custom profile selection, exact profile fingerprints, and uncovered-joint status.
- The packaged template is tested for byte-for-byte agreement with
  `robot_safety/profiles/g1_type5.template.json`. No lenient profile is selected
  implicitly. No hardware commands are issued by these tests.
- The installed evaluator ran the sample with the 174-scenario medium suite.
  It recorded 12 violations, no evaluator errors, and 162 unavailable checkpoint
  branches. Exit code 2 reflects incomplete coverage following nominal rejection.
- Example: the standing policy's first waist-roll target differed by
  0.0367124 rad from the initial measured position, exceeding the standard
  0.03 rad startup tolerance. Rejection occurred before applying the command.

The 12 leg joints remain uncovered by these per-joint movement rules. Measured
acceleration thresholds are not inferred from commanded-velocity-change limits.
No new physical calibration or hardware validation is claimed.

### Earlier validation before joint-rule integration

Validated on Linux aarch64 with Python 3.12.14, MuJoCo 3.3.2, ONNX Runtime
1.22.0, and NumPy 2.5.3 in an isolated Python environment.

- Package tests: 24 passed. Includes neural observation/action semantics,
  checkpoint replay and history, pulse duration/clearing, six initial poses,
  nonfinite and excessive actions, oscillation limits, inference failure,
  missing checkpoint coverage, external weights, calibration requirements,
  worker errors, and timeout handling.
- Existing non-ROS repository tests: 119 passed, plus 70 subtests. Local HTTP
  tests required loopback socket access outside the execution sandbox.
- Ruff checks, formatting, and `git diff --check` passed.
- Installed a regular wheel into a temporary environment and ran its console
  command from `/tmp` without a repository path or ROS on `PYTHONPATH`.

The supplied walker was evaluated with the README's shortened suite: 58 scenarios,
53 completed within provisional limits and five violations. Seated, supine,
prone, and both side-lying starts requested decoded joint targets outside the
actuator ranges. These were recorded before clipping could hide the requests.
The repository wrapper and installed command produced identical JSON reports
and numerical trace arrays.

An optional default-suite run was stopped after 66 completed scenarios because
of its runtime on this device. Its report remained incomplete, with 888 of 954
scenarios unreported and exit code 2. The full default suite has not been validated
to completion. The shortened suite covers all scenario families.

ROS-specific tests require the ROS test environment and were not run successfully
in this Python 3.12 environment. No physical robot tests were performed. The
acceptance profile remains provisional and uncalibrated.
