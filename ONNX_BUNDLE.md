# ONNX policy bundle

This repository does not load a YAML file to run a policy on the robot. Unitree RL Gym and Unitree RL Lab each have their own deploy YAML, and those files belong to those runners. They are not accepted here.

The walker uses the [shared policy container](containers/examples/README.md), and its model bundle is in [containers/examples/policies/walker/](containers/examples/policies/walker/). `policy-safety prepare-g1` reads the policy YAML plus the ONNX files it names. Command-only YAML files may share the directory, but exactly one policy definition is required. The contract below is `nvidia-decoupled-wbc-516-v1`. `./start.sh` and `OPEN_APP.sh` will not launch either one.

- Adapter: `packages/policy_safety/src/policy_safety/adapter.py` (`NvidiaWalker`)

## Walker files the checker expects

Keep the Walk and Balance checkpoints together. The adapter loads both. These files are committed in `containers/examples/policies/walker/`.

| File | Role |
| --- | --- |
| `walker.yaml` | Observation layout, joint order, defaults, scales, gains, timing |
| `walker.onnx` | 516-input, 15-output NVIDIA walking network |
| `balance.onnx` | Official companion standing network. Required. |

Screen a directory that contains them with:

```bash
policy-safety prepare-g1 \
  --submission containers/examples/policies/walker \
  --output /tmp/g1-screening-bundle
policy-safety evaluate \
  --model /tmp/g1-screening-bundle/walker.onnx \
  --adapter nvidia-decoupled-wbc-516-v1 \
  --robot-config /tmp/g1-screening-bundle/robot.json \
  --output /tmp/g1-screening-result
```

## What the network expects

One float32 input of shape `[1, 516]` and one float32 output of shape `[1, 15]`.
Normalization is explicit in the adapter. The input contains six chronological
86-value frames, initially zero padded. Quaternion layout is `w, x, y, z`.

| Frame slice | Contents |
| --- | --- |
| 0:3 | Command `[vx, vy, yaw_rate]` scaled by `[2, 2, 0.5]` |
| 3 | Height command `0.74` |
| 4:7 | Zero torso roll, pitch, yaw command |
| 7:10 | Body angular velocity scaled by `0.5` |
| 10:13 | Gravity projected into the body frame |
| 13:42 | All 29 joint positions minus padded defaults |
| 42:71 | All 29 joint velocities scaled by `0.05` |
| 71:86 | Previous 15 raw lower-body actions |

Commands with norm below `0.05` select Balance. Otherwise Walk is selected.
Both networks share history. The upper 14 joints hold neutral targets.

The decoded motor target is a position:

```text
q_target = default_position + raw_output * action_scale
```

Desired velocity and feedforward torque are zero. The position PD law used in screening is `kp * (q_target - q) - kd * dq`. Control period is 20 ms. Physics period in the checker is 2 ms.

`prepare-g1` copies the defaults, scales, and gains from that directory's policy YAML into `robot.json`.

A different network replaces those files and needs its own contract. Matching tensor sizes is not enough: observation order, frames, normalization, scales, gains, and timing have to match the exported network.
