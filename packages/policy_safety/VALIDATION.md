# Validation — September 28, 2026

## Per-joint enforcement update

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

## Earlier validation before joint-rule integration

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
