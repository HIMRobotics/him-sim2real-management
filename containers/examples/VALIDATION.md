# Example validation

Run the automated simulator and policy-screening tests from the repository root:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv-policy-safety/bin/python -m pytest -q \
  containers/sim packages/policy_safety/tests
```

These tests exercise real ONNX inference and MuJoCo, plus mocked transport and
launcher boundaries. They do not operate robot hardware. See
[the example guide](README.md) for local and Docker/ROS execution and
[simulation checks](../SIM_RUN.md#shared-thrash-detection) for detector settings.

Policy bundles and provenance are documented under
[`examples/policies`](../../examples/README.md). Validation applies to the exact
model, settings and runtime tested. Local simulation does not establish Docker
wall-clock timing or hardware performance.

## Weighted thrash detector calibration

The shared detector uses whole-body weighted oscillatory RMS, with a provisional
limit of 1.5 continuously exceeded for 500 ms. Simulation defaults to monitoring.
Hardware submission defaults to enforcement.

Recorded upright circle walking (30 s), straight walking (20 s), and a 10°
pitch/10° yaw start (20 s) produced no detections. Replaying measured velocities
at 2 ms and 20 ms intervals also produced no detections. Supine circle and
straight-walker recordings triggered at approximately 0.73–0.74 s at both rates.
These are simulation observations, not hardware validation.

A separate forward-pitch sweep used the existing straight walker, fixed 10° yaw,
20 s per case, monitoring only, and other safety stops disabled. The root rotated
about a fixed toe anchor without changing joint angles. All four toe collision
points started in contact, with 0.01 mm numerical overlap and zero velocity.

| Initial pitch | First detection | Outcome |
| --- | --- | --- |
| 20° | None | Recovered upright |
| 25° | None | Recovered upright |
| 30° | 0.970 s | Fell |
| 35° | 0.922 s | Fell |
| 40° | 0.770 s | Fell |

This is one deterministic trajectory per angle, not a universal safe tilt limit.
The custom scenes and recordings were temporary experiments, not new examples.
