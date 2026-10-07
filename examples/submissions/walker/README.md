# NVIDIA walker submission

The submission uses the official NVIDIA Decoupled WBC Walk and Balance models
from `examples/policies/walker/`. Its shared adapter builds 516-input observation
history and produces 15 lower-body targets, with neutral upper-body targets.
The supplied command files express forward and sideways velocity in m/s and
yaw rate in rad/s. Near-zero velocity selects NVIDIA's Balance checkpoint.

The preferred shared submission entry point is:

```bash
./submit.sh --mode sim --example walker --local --seconds 10 \
  --command examples/commands/walker/forward.yaml --view
```

Drop `--local` to build and run the shared ROS container in isolated simulation.
For hardware, use `./submit.sh --mode real --example walker --command FILE`.
`real.sh` is a compatibility wrapper. The host runs GR00T Balance until measured stability, then requires a fresh
left + B press and blends into the policy. Double Start requests damping.
See [operator commands](../../../robot_safety/HARDWARE_ADAPTER.md#operator-commands). Simulation validation
is not hardware qualification.

`build.sh` preserves the separate legacy submission packaging workflow:

```bash
./examples/submissions/walker/build.sh
```

It writes `.cache/walker_submission/submission.tar` and does not launch it.
The image contains both official models, provenance and NVIDIA's model license.
It uses ROS Foxy and ONNX Runtime 1.16.3 on ARM64. It copies the host Unitree
SDK and CycloneDDS bindings, configurable through `UNITREE_SDK2PY`,
`CYCLONEDDS_PY`, and `CYCLONEDDS_LIB`.

The legacy node publishes `/safety/precheck` and subscribes to simulated base
state and Unitree low state. `WALKER_COMMAND` selects its command YAML, defaulting
to `stand.yaml`. The shared container uses `G1_COMMAND_FILE` and is the path
used by `--example walker` for both simulation and hardware.
