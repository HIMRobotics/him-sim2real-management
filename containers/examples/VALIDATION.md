# Example validation

Validated on the development ARM64 host with real ONNX inference and MuJoCo, using the local backend and default prepared environments. No robot motor commands were sent.

## Current command-gap and response checks

Position proposals are clipped to the joint bounds with per-joint warnings.
The runtime command-gap rejection has been removed. Neither that gap nor changes
between consecutive proposals reject commands. Measured speed uses the per-joint
model ceilings immediately. Torque, body motion, freshness and startup checks
remain enabled. Response error is diagnostic only. The experiments below used
older checks.

Ten-second local simulation requests with real policy inference produced:

| Policy | Result with checks enabled |
| --- | --- |
| GR00T balance | Completed 500 commands over 10 seconds within limits. |
| Walker | Completed 500 commands over 10 seconds within limits. Stayed upright with 5.2346 m forward displacement and 1.6509 m lateral drift. Position clipping warned 71 times at the right ankle pitch and 8 at the left. |
| Stair walker | Completed 500 commands over 10 seconds within limits, climbed all three treads and finished upright. |

The balance run's largest recorded response error was 0.317639 rad, diagnostic
only. Temporary rollout outputs were removed. Hardware was not activated.
The walker previously stopped after five accepted commands on an out-of-range
position proposal, then after 17 commands on the 0.5 rad gap guard once position
clipping was enabled. Neither condition now ends the run. Torque and measured
motion checks remain active. A validated detector for thrashing/flailing is the
next priority, as described in the [profile guide](../../robot_safety/profiles/README.md#next-priority-thrashing-and-flailing-detection).

## Earlier walker target-change comparison, October 6, 2026

With tracking checked after actuation, the original 0.180 rad target-change guard
rejects the first walking knee command (0.238263 rad). The new 0.5 rad per 20 ms
guard accepts it. After that interval, the independent knee tracking check still
rejects 0.194863 rad against its unchanged 0.173750 rad allowance.

An uninterrupted ten-second diagnostic rollout at 0.6 m/s stayed upright. Of 499
target transitions, 180 exceeded 0.180 rad and none exceeded 0.5 rad. The maximum
was 0.350117 rad at the left ankle pitch. This supports the wider target-change
setting for this simulated rollout, not a hardware safety claim or a full safety
pass. Torque and measured-speed thresholds were unchanged. The inline diagnostic
left no test script or output files in the repository.

Sampling measured speed every 2 ms in the same uninterrupted rollout found a
10.209674 rad/s peak at the left knee. Only two samples exceeded 10 rad/s, a
4 ms excursion. The longest per-joint excursion above 9 rad/s lasted 22 ms,
below the existing 50 ms persistence requirement. Measured-speed protection
therefore would not stop this nominal run. This does not explain Unitree's
choice of a 10 rad/s default or validate a hardware threshold.

## Earlier walker startup experiment, October 6, 2026

Twenty ten-second trials compared ten startup variants at 0.6 m/s, once each
with safety enforced and disabled. These used the earlier pre-actuation tracking
check and target-change limits. Model, gains, and limits were unchanged per trial.
Target blends over 0.1–1 s delayed the first tracking rejection from 0.02 s to
0.08–0.14 s. Command ramps, active zero-command startup, and longer static holds
also failed the checks. All enforced runs subsequently fell. All diagnostic
runs stayed upright, but the baseline violated command rules on 499/500 frames.
Startup smoothing alone did not resolve compatibility. A 1 s static hold let
pelvis height drop to 0.641 m before recovery. These simulation-only comparisons
do not validate hardware handover or establish safe replacement thresholds.
The temporary experiment script was removed after testing. Saved reports and
replays remain in the ignored `containers/sim/output/startup-transition-01/`.

## Earlier validation

| Policy | Safety checks | Duration | Result |
| --- | --- | --- | --- |
| Stair walker | Off | 20 simulated seconds | 1,000 accepted commands. Foot contact on all three treads, stood on top, finished upright, no fall. Minimum pelvis height approximately 0.770 m. |
| Stair walker | On | 20 simulated seconds | Second policy command rejected for `joint_position_target_change_limit` at the left ankle pitch. Requested change approximately 0.197 rad, limit 0.18 rad. The rejected target was not applied. The runner retains the last accepted hold unless a runtime fault triggers damping. |
| GR00T balance | Off | 10 simulated seconds | 500 accepted commands. Remained upright, no fall. Minimum pelvis height approximately 0.752 m. |
| GR00T balance | On | 10 simulated seconds | Rejected for `joint_tracking_error_limit` at the left ankle pitch after three accepted commands. Error approximately 0.243 rad, limit 0.21875 rad. The rejected target was not applied. The runner retains the last accepted hold unless a runtime fault triggers damping. |

The unchecked stair run demonstrates stair climbing and an upright finish. It did **not** satisfy the stricter no-platform-contact airborne-gap condition, so `course_success` is false. Safety thresholds were not relaxed to obtain a pass.

The walker example commands 0.6 m/s forward and zero yaw. Local and Docker
ten-second runs produced approximately 5.23 m of forward displacement and 1.65 m
of lateral drift, with no fall. Zero commanded yaw does not guarantee a perfectly
straight trajectory from this policy. The movement regression requires more than
3 m forward and an upright finish with safety checks disabled. Its earlier
0.3 m/s command produced only 0.084 m forward. Circular motion is requested
explicitly with the separate `--circle` mode or a turning `--command` file.

Regression tests run both real models, verify the stair and balance outcomes, confirm enabled checks reject their commands, export the recorded environment for browser replay, and load the prepared image context independently of repository imports. Both model bundles are committed and required by the example tests.

Validation after the example-folder cleanup on October 4, 2026 completed with
120 tests passing and six subtests across the simulator and policy-screening
suites, with no skips. Ruff checks on changed Python files, shell syntax checks, and workflow
YAML parsing passed. Both walker and balance weights were available on this host.

The subsequent simulation cleanup passed 126 tests and eight subtests, with no
skips (210.22 seconds). After the final observation-clock adjustment, all seven
bridge tests and the combined-step timestamp test passed again. Ruff on changed
Python files, shell syntax, and `git diff --check` also passed.

After rebasing onto the desktop-simulation changes, 128 tests and eight subtests
passed with no skips (237.87 seconds). Desktop hold, walker, and circle CLI checks
ran on the Linux development host and verified multi-frame replay traces. Shell
syntax, changed-Python lint, and YAML parsing passed. Docker Compose execution
was not tested because this host does not have the Compose plugin.

After launcher consolidation and replacing vendored viewer sources with a pinned
download cache, 135 tests and eight subtests passed with no skips (255.16 seconds).
Live stairs, desktop hold, battery viewing, and multi-frame replay passed HTTP
smoke checks, including Start gating, matching scene geometry, port fallback,
and process shutdown. A fresh asset download and offline cache reuse both passed.
Compose lifecycle tests use a fake runtime and cover failure cleanup and all six
counterfactual poses. Actual Docker/ROS execution still needs a final rerun.

Additional local checks confirmed:

- Twenty stateful inference calls through the refactored submission produced
  exactly the same joint targets and gains as the previous implementation.
  Its staged adapter files import independently of repository paths.
- A real MuJoCo subprocess accepted arm, combined step/feedback, combined
  circle/feedback, and report requests, then shut down cleanly.
- The trail code ran in JavaScriptCore against the bundled Three.js classes.
  Seeking reused the buffer, live frames appended correctly, capacity growth
  preserved earlier points, and a reset replaced the visible path. This checks
  JavaScript behavior, not browser rendering.
- Local preparation omitted Docker client sources and ROS interfaces while
  retaining a loadable model and scene. Report-only runs retained metrics
  without pose history, and live/replay regressions passed.

The GitHub Actions simulation job installs the screening package and runs both
suites on Python 3.12. A clean checkout includes the walker bundle, so missing
walker files now fail instead of silently skipping screening coverage. The GR00T balance rollout also uses its committed model bundle. Current hosted CI status is tracked on the PR.

Direct circle runs report `completed` and `control_path: direct`, with explicit
per-check flags. Replay labels submission checks as bypassed even when opened
from a trace without its report. Tests also confirm that enabled runtime fall
monitoring remains active and that summaries preserve partial validation.

The user attempted the walker example Docker build. It failed while installing
ONNX Runtime 1.18.1 because pip found no matching Python 3.8 ARM64 distribution
and listed 1.16.3 as the newest available version. The example Dockerfile now
pins 1.16.3, matching the separate walker submission image.

The subsequent walker container run built successfully but crashed before sending
commands: the original walker graph declared IR 10, while Runtime 1.16.3 supports
IR 9. Its report recorded `no_command`, zero steps, and a single initial pose.
The walker graph is now IR 9 with its one exporter-only value-info annotation
removed. Computation and weights are unchanged. Both Dockerfiles now initialize
the packaged model during the build, including a test inference.

Using an isolated Runtime 1.16.3 installation under host Python 3.8, walker,
stairs, and balance initialization and inference passed. Twenty fixed observations
produced identical walker actions before and after the graph-format change on the
host runtime. Comparing Runtime 1.16.3 against those original-model outputs gave
a maximum absolute action difference of 1.43e-6. The full ONNX checker also passed.
The user subsequently ran the corrected Linux Docker/ROS path. Saved reports
were inspected: the smoke test returned `within_limits` with 51 commands over
one simulated second, and the unchecked walker and balance runs each returned
`completed` with 501 commands over ten simulated seconds. Walker command settings
changed during these manual checks, so these reports establish transport and
inference execution rather than validation of every final policy setting.

The final rebased Docker/ROS path has not been rerun. Docker Compose execution
also remains unverified on this host, which lacks the Compose plugin. The commands
in [README.md](README.md#full-dockerros-execution) reproduce the integration checks.
No hardware deployment was tested. Local rollout success does not establish
matching Docker wall-clock timing or hardware performance.
