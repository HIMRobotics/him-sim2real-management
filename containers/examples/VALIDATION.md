# Example validation

Validated on the development ARM64 host with real ONNX inference and MuJoCo, using the local backend and default prepared environments. No robot motor commands were sent.

| Policy | Safety checks | Duration | Result |
| --- | --- | --- | --- |
| Stair walker | Off | 20 simulated seconds | 1,000 accepted commands. Foot contact on all three treads, stood on top, finished upright, no fall. Minimum pelvis height approximately 0.770 m. |
| Stair walker | On | 20 simulated seconds | Second policy command rejected for `joint_position_target_change_limit` at the left ankle pitch. Requested change approximately 0.197 rad, limit 0.18 rad. The rejected target was not applied. The runner retains the last accepted hold unless a runtime fault triggers damping. |
| GR00T balance | Off | 10 simulated seconds | 500 accepted commands. Remained upright, no fall. Minimum pelvis height approximately 0.752 m. |
| GR00T balance | On | 10 simulated seconds | Rejected for `joint_tracking_error_limit` at the left ankle pitch after three accepted commands. Error approximately 0.243 rad, limit 0.21875 rad. The rejected target was not applied. The runner retains the last accepted hold unless a runtime fault triggers damping. |

The unchecked stair run demonstrates stair climbing and an upright finish. It did **not** satisfy the stricter no-platform-contact airborne-gap condition, so `course_success` is false. Safety thresholds were not relaxed to obtain a pass.

Regression tests run both real models, verify the stair and balance outcomes, confirm enabled checks reject their commands, export the recorded environment for browser replay, and load the prepared image context independently of repository imports. Both model bundles are committed and required by the example tests.

Validation after the example-folder cleanup on October 4, 2026 completed with
120 tests passing and six subtests across the simulator and policy-screening
suites, with no skips. Ruff checks on changed Python files, shell syntax checks, and workflow
YAML parsing passed. Both walker and balance weights were available on this host.

The GitHub Actions simulation job installs the screening package and runs both
suites on Python 3.12. A clean checkout includes the walker bundle, so missing
walker files now fail instead of silently skipping screening coverage. The GR00T balance rollout also uses its committed model bundle. The hosted CI job has not yet been run for this change.

Direct circle runs report `completed` and `control_path: direct`, with explicit
per-check flags. Replay labels submission checks as bypassed even when opened
from a trace without its report. Tests also confirm that enabled runtime fall
monitoring remains active and that summaries preserve partial validation.

Full Docker image building, ROS delivery, and network namespace execution remain **unverified on this host by the agent** because Docker access requires interactive sudo. The commands in [README.md](README.md#full-dockerros-execution) are the remaining manual integration check. Local rollout success does not establish matching Docker wall-clock timing or hardware performance.
