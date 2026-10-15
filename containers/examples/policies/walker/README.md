# NVIDIA Decoupled WBC walker

This bundle holds NVIDIA's unmodified `GR00T-WholeBodyControl-Walk.onnx` (saved
as `walker.onnx`) and `GR00T-WholeBodyControl-Balance.onnx` (saved as
`balance.onnx`), taken from
[NVlabs/GR00T-WholeBodyControl](https://github.com/NVlabs/GR00T-WholeBodyControl)
at revision `b042411fae38ee4d1af9aac82a37a1f8d14d6dd0`. `provenance.json` records
their original names and SHA-256 hashes, and `LICENSE` is NVIDIA's model license.
`walker.yaml` records the gains, joint order, and default joint positions the
actions are added to. The simulator starts the robot in the crouch GR00T
balance settles into, not in these defaults.

`NvidiaWalker` in `policy_safety/adapter.py` follows the upstream
`decoupled_wbc/control/policy/g1_gear_wbc_policy.py` contract, and Balance takes
over whenever the commanded velocity is near zero. The upper 14 joints hold
neutral targets instead of running upper-body IK. The full input layout,
scaling, and screening commands are in [ONNX_BUNDLE.md](../../../../ONNX_BUNDLE.md).

The velocity schedules in [`../../commands/walker/`](../../commands/walker/)
accept `forward_velocity`, `sideways_velocity`, and `yaw_rate`, as well as the
older `command: [vx, vy, yaw_rate]` form and timed segments. For editable Python
variables instead, run `python3 containers/examples/commands/walker/run.py
--seconds 10 --view`.

The former arena stair model is kept separately in [`../stairs/`](../stairs/)
and is only used by the stairs example and its legacy adapter tests.
