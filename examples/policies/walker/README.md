# NVIDIA Decoupled WBC walker

This bundle contains NVIDIA's unmodified `GR00T-WholeBodyControl-Walk.onnx`
(as `walker.onnx`) and `GR00T-WholeBodyControl-Balance.onnx` (as `balance.onnx`).
Both come from [NVlabs/GR00T-WholeBodyControl](https://github.com/NVlabs/GR00T-WholeBodyControl)
at revision `b042411fae38ee4d1af9aac82a37a1f8d14d6dd0`.
`provenance.json` records the original names and SHA-256 hashes. `LICENSE`
is NVIDIA's model license.

`NvidiaWalker` in `policy_safety/adapter.py` follows the upstream
`decoupled_wbc/control/policy/g1_gear_wbc_policy.py` contract: six chronological
86-value frames, command scaling `[2, 2, 0.5]`, body gyro scaling `0.5`, joint
velocity scaling `0.05`, height command `0.74`, and 15 lower-body actions scaled
by `0.25`. Commands with norm below `0.05` select Balance, sharing observation
history and previous actions across transitions. Control runs at 50 Hz.
The upper 14 joints stay at neutral targets, rather than running upper-body IK.
Gains, joint order and initial pose are recorded in `walker.yaml`.

The former arena stair model is preserved separately in `../stairs/` and is
used only by the stairs example and its legacy adapter tests.

From the repository root:

```bash
./submit.sh --mode sim --example walker --local --seconds 10 \
  --command examples/commands/walker/forward.yaml --output .cache/walker-forward
```

Add `--view` for interactive simulation, or replay the saved trace:

```bash
./sim.sh replay .cache/walker-forward/trace.npz
```

`examples/commands/walker/forward.yaml` accepts `forward_velocity`,
`sideways_velocity`, and `yaw_rate`. Existing `command: [vx, vy, yaw_rate]`
files and timed segments are also supported. `forward_stop.yaml` ramps down
to Balance without the old stair model's yaw correction.

For editable Python variables, run:

```bash
python3 examples/commands/walker/run.py --seconds 10 --view
```

To prepare an offline screening bundle:

```bash
.venv-policy-safety/bin/policy-safety prepare-g1 \
  --submission examples/policies/walker --output /tmp/nvidia-walker-screen
```

Use adapter `nvidia-decoupled-wbc-516-v1` for that bundle. Both models are
included in screening fingerprints. Simulation results do not qualify hardware.
