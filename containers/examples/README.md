# Example submissions

These are finished reference policies with their weights baked in. One shared
image holds all of them, and `G1_SIM_EXAMPLE` (set by `--example NAME`) picks
which one runs. They show the simulator and the hardware path working with real
policies. To start your own submission, use [the template](../template/README.md)
instead; nothing in the simulator depends on this directory.

The commands for running them are in the [containers README](../README.md).

| Example | Controller | Scene |
| --- | --- | --- |
| `walker` | NVIDIA Decoupled WBC Walk/Balance, walking forward at 0.3 m/s by default | Flat floor |
| `balance` | The GR00T lower-body balance network, with the arms held at their starting pose | Flat floor |
| `holosoma` | Holosoma FastSAC locomotion. Its weights are not in this repository; set `HIM_HOLOSOMA_ONNX` | Flat floor |

Every controller sends a hold at the measured pose as its first command. After
that its targets go straight to the normal checks, without smoothing or any
change to the safety thresholds.

| Path | Contents |
| --- | --- |
| [`policies/walker/`](policies/walker/) | NVIDIA Walk and Balance checkpoints, their YAML contract, license, and provenance |
| [`policies/balance/`](policies/balance/) | The GR00T balance checkpoint |
| [`commands/walker/`](commands/walker/) | Velocity schedules for the walker: stand, forward, forward then stop, circle, first steps |
| `client.py`, `controllers.py`, `command.py`, `trial.py` | The shared ROS client and the per-example controllers |
| `prepare.py`, `Dockerfile`, `entrypoint.sh` | Build-context generation and the image definition |

## How the image is built

`prepare.py NAME` writes a build context to `.cache/sim_examples/NAME/container`
and the matching MuJoCo scene to `.cache/sim_examples/NAME/environment`. The
context holds only the controller code, that example's model, a generated joint
profile, and the ROS message package. `--example NAME` runs this step, builds the
image as `g1-sim-example-NAME:local`, and uses the generated scene unless you
pass `--scene`.

The Dockerfile pins ONNX Runtime 1.16.3, the newest version for ROS 2 Foxy's
Python 3.8 on ARM64, so the walker graph is stored as ONNX IR 9. Each build
loads the model and runs one test inference before it finishes.

`G1_TASK_COMMAND='[vx, vy, yaw_rate]'` overrides an example's velocity command,
and `--command FILE` passes a whole schedule through `G1_COMMAND_FILE`. Neither
changes which example was built.

## Build and export the shared image

To produce a normal `submission.tar` from an example without running anything:

```bash
.venv-policy-safety/bin/python containers/examples/prepare.py walker
docker build --platform linux/arm64 -t g1-example-walker:local .cache/sim_examples/walker/container
docker save g1-example-walker:local -o .cache/sim_examples/walker/submission.tar
```

The archive contains the shared client, both walker checkpoints, the model
license, and provenance. It does not include the host's Unitree SDK or
CycloneDDS. When running that image on hardware with `--image`, pass the
matching profile with
`--profile .cache/sim_examples/walker/container/project/joint_profile.json`.

## Shared policy observations

The shared client reads `/safety/policy_state`. In simulation it carries these
fields; on hardware the base pose and velocity appear only when a qualified
measurement source is configured.

| Field | Meaning |
| --- | --- |
| `session_id`, `observation_id` | IDs to copy into the joint command |
| `q`, `dq` | 29 joints in Unitree motor order |
| `quat`, `gyro`, `accel` | Pelvis IMU, with the quaternion in w, x, y, z order |
| `position` | Pelvis world position in metres |
| `base_velocity` | Free-joint velocity: world linear velocity, then local angular velocity |
| `ctrl_low`, `ctrl_high` | Actuator target bounds, in motor order |
| `sim_time`, `height`, `upright` | Simulation time and pelvis state |

## Provenance

The walker runs
through `NvidiaWalker` in `policy_safety/adapter.py`, and the balance example
through `robot_safety/balance_policy.py`.
