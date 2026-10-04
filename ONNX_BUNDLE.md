# ONNX policy bundle

This repository does not load a YAML file to run a policy on the robot. Unitree RL Gym and Unitree RL Lab each have their own deploy YAML, and those files belong to those runners. They are not accepted here.

The walker test container is in [examples/submissions/walker/](examples/submissions/walker/), and its model bundle is in [examples/policies/walker/](examples/policies/walker/). `policy-safety prepare-g1` reads the policy YAML plus the ONNX files it names. Command-only YAML files may share the directory, but exactly one policy definition is required. The contract below is `g1-walker-99-v1`. `./start.sh` and `OPEN_APP.sh` will not launch either one.

- Adapter: `packages/policy_safety/src/policy_safety/adapter.py` (`G1Walker`)

## Walker files the checker expects

Keep the weights file next to the model. ONNX Runtime loads both. These files are committed in `examples/policies/walker/`.

| File | Role |
| --- | --- |
| `walker.yaml` | Observation layout, joint order, defaults, scales, gains, timing |
| `walker.onnx` | 99-input, 29-output float network |
| `walker.onnx.data` | External weights. Required. |

Screen a directory that contains them with:

```bash
policy-safety prepare-g1 \
  --submission examples/policies/walker \
  --output /tmp/g1-screening-bundle
policy-safety evaluate \
  --model /tmp/g1-screening-bundle/walker.onnx \
  --adapter g1-walker-99-v1 \
  --robot-config /tmp/g1-screening-bundle/robot.json \
  --output /tmp/g1-screening-result
```

## What the network expects

One input named by the graph, shape `[1, 99]`, dtype float32. One output, shape `[1, 29]`, dtype float32. Normalization is inside the ONNX graph (`normalization: embedded`). The policy directory does not carry a mean or standard-deviation file.

Quaternion layout is `w, x, y, z`. The 99 values, in order:

| Slice | Count | Contents |
| --- | ---: | --- |
| 0:3 | 3 | Base linear velocity in the body frame, m/s |
| 3:6 | 3 | Base angular velocity in the body frame, rad/s |
| 6:9 | 3 | Gravity direction in the body frame. World vector is `[0, 0, -1]`. |
| 9:38 | 29 | Joint position minus `default_position`, rad, `joints` order below |
| 38:67 | 29 | Joint velocity, rad/s, same order |
| 67:96 | 29 | Previous raw network output, not the decoded joint target |
| 96:99 | 3 | Command `[vx, vy, yaw_rate]` |

The decoded motor target is a position:

```text
q_target = default_position + raw_output * action_scale
```

Desired velocity and feedforward torque are zero. The position PD law used in screening is `kp * (q_target - q) - kd * dq`. Control period is 20 ms. Physics period in the checker is 2 ms.

`prepare-g1` copies the defaults, scales, and gains from that directory's policy YAML into `robot.json`.

A different network replaces those files and needs its own contract. Matching 99 and 29 is not enough: observation order, frames, normalization, scales, gains, and timing have to match the exported network.
