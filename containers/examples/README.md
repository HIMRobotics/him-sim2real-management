# Runnable simulation policies

`sim.sh` can run real ONNX policies with safety checks on or off, record their motion, and replay their environment. `--example` supplies a policy producer and a matching scene. Safety checks remain enabled by default.

| Example | Controller | Environment |
| --- | --- | --- |
| `stairs` | The retained walker ONNX plus the original submission's walk, pause, cross, hop, and arm overrides | Restored three low stair treads, top landing, gap, and finish pad |
| `walker` | The walker ONNX with a 0.6 m/s forward command and zero commanded yaw | Flat floor |
| `balance` | The existing GR00T/Gear WBC lower-body balance ONNX, with arms held at their initial pose | Flat floor |

All controllers send a measured-pose hold as their first command. After that, their targets go directly to the normal sim checks. They do not smooth targets to evade those checks or change safety thresholds. MuJoCo integrates the floating base. The controller never writes root positions.

## Run now without sudo

Local mode uses the real ONNX models, MuJoCo, and the same `Runner` as the ROS plant. It does not use Docker or ROS. Durations and command timestamps use simulation time, so this mode does not test network latency or wall-clock command deadlines.

From the repository root. On a Mac or Windows laptop use `./desktop_sim.sh` with the same flags.

```bash
# Stairs, safety enabled. A violation exits with code 1.
./desktop_sim.sh --example stairs --local --seconds 20 --output containers/sim/output/stairs-on
# Same command works as ./sim.sh --example stairs --local ... on Linux.

# Same controller and environment, safety disabled.
./desktop_sim.sh --example stairs --local --seconds 20 --no-safety-checks --output containers/sim/output/stairs-off

# Balance, both settings.
./desktop_sim.sh --example balance --local --seconds 10 --output containers/sim/output/balance-on
./desktop_sim.sh --example balance --local --seconds 10 --no-safety-checks --output containers/sim/output/balance-off
```

Each output directory contains `report.json` and `trace.npz`. Named output directories preserve both recordings. Exit code 0 means a checked run passed or an unchecked run completed, code 1 is a safety violation, and code 2 is an execution error or missing commands. A completed unchecked run is not necessarily a successful task.

The balance weights are committed at `examples/policies/balance/GR00T-WholeBodyControl-Balance.onnx`. Set `HIM_BALANCE_ONNX` to select another copy. `--example stairs` and `--example walker` use the committed `examples/policies/walker/walker.onnx`, `walker.onnx.data`, and `walker.yaml`. The generated container context names its policy specification `policy.yaml`.

## View a result

```bash
./sim.sh replay containers/sim/output/stairs-off/trace.npz
```

Open the printed URL (normally `http://127.0.0.1:8765`). The replay shows the robot, stairs, safety mode, result, and any rejection reason. Left drag orbits, right drag pans, scroll zooms, and Space pauses. A blue line on the floor is the pelvis path. Ctrl+C stops the server. Use the `stairs-on`, `balance-on`, or `balance-off` trace to view the other runs. With either launcher, `--view` waits for Start and streams the run live, including `--local`. `./sim.sh --view --circle` is the same page for one flat circle from the cached walker, with no submission. The [viewer setup](../SIM_RUN.md#how-to-visualize) describes the initial dependency download and offline use.

If running on the robot over SSH, forward port 8765 with `ssh -L 8765:127.0.0.1:8765 unitree@<robot>` and open the same URL on your laptop.

## Full Docker/ROS execution

Drop `--local`. The script prompts for sudo, prepares the example files, builds its Docker image using the local Docker daemon, and starts it in the existing loopback-only simulation network. Enter the password in your own terminal.

```bash
# First check the host ROS bridge and MuJoCo plumbing.
./sim.sh --self-test

# Build and run the actual balance container.
./sim.sh --example balance --seconds 10 --no-safety-checks --output containers/sim/output/docker-balance-off
./sim.sh --example balance --seconds 10 --safety-checks --output containers/sim/output/docker-balance-on

# Or run the stair policy container in its course.
./sim.sh --example stairs --seconds 20 --no-safety-checks --output containers/sim/output/docker-stairs-off
./sim.sh --example stairs --seconds 20 --safety-checks --output containers/sim/output/docker-stairs-on

./sim.sh replay containers/sim/output/docker-stairs-off/trace.npz
```

The first build needs network access for the ROS base image, apt packages, and Python wheels. Subsequent builds reuse Docker layers. Both example Dockerfiles pin ONNX Runtime 1.16.3 for ROS 2 Foxy's Python 3.8 on ARM64. The previous 1.18.1 pin failed because pip could not find a matching distribution for that environment. Rerunning `sim.sh --example` refreshes the build context and rebuilds with the corrected pin. The walker graph uses IR 9 for that runtime. Builds initialize the packaged model and run a test inference before producing the image. Manual Linux Docker results and remaining checks are recorded in [VALIDATION.md](VALIDATION.md). The host needs its existing ROS 2 Foxy installation, `colcon`, Docker, and `.venv-policy-safety`.

On a Mac or Windows laptop, `./desktop_sim.sh --example walker --view` uses
Docker Compose and waits for Start before streaming the container run. The same
path accepts `--image NAME` or an archive in `containers/upload`. The laptop needs
Docker Desktop and the viewer Python environment, but no host ROS or `colcon`.

The generated directories are `.cache/sim_examples/<name>/container` and `.cache/sim_examples/<name>/environment`. The container context includes only controller code, its model, and the ROS message package. The environment is loaded by the host MuJoCo process. The image is tagged `g1-sim-example-<name>:local`.

`bridge.log` and `policy.log` accompany Docker run reports. `execution: docker_ros` distinguishes this path from `execution: local`. Docker runs use wall-clock deadlines, so their numerical trajectory can differ from local runs. The bridge publishes feedback after advancing physics to avoid giving the controller a state that is already one step old.

The example containers require a network containing only `lo`. They are simulation clients and cannot be launched with `setup.sh` on the robot network. They publish only `/safety/precheck`.

## Simulation-only policy observations

The existing robot topics remain available. The bridge additionally publishes `/sim/state` as `std_msgs/String` containing JSON. This provides the base pose and velocity needed by the stair controller without pretending that hardware has an odometry feed.

| Field | Meaning |
| --- | --- |
| `session_id`, `observation_id` | IDs to echo in the joint command |
| `q`, `dq` | 29 joints in Unitree motor order |
| `quat`, `gyro`, `accel` | Pelvis IMU values, quaternion in wxyz order |
| `position` | Pelvis world position in metres |
| `base_velocity` | Free-joint velocity: world linear velocity followed by local angular velocity |
| `ctrl_low`, `ctrl_high` | Model actuator target bounds, in motor order |
| `sim_time`, `height`, `upright` | Simulated time and pelvis state |

The session already exists when the sim starts. These example clients use the ID in `/sim/state`. Other submissions can continue obtaining it from `/safety/checker/arm`.

`--circle` and `--command` use a separate direct-control path. Successful runs
report `completed`, with `control_path: direct` and explicit per-check flags.
They do not establish that the walker passes submission validation.

## Use another scene and interpret results

Add `--scene path/to/scene.xml` to any example command to override its prepared scene. It must follow the [G1 scene contract](../SIM_RUN.md#submit-a-mujoco-environment). The stair controller's phases use the original course coordinates, so changing that course may also require adapting the controller. `--counterfactuals` works with either backend and writes per-pose reports and a summary.

`report.json` records the safety outcome and independent `motion` measurements. A fall after a safety rejection reflects the subsequent hold or damping response. It is not evidence that the rejected policy continued controlling the robot. For the original stair course, `climbed_stairs` requires foot contact on at least two treads, standing on the top landing, and no fall. `course_success` additionally requires an airborne gap crossing and an upright finish on the finish pad. The stricter gap condition requires no foot contact with any course platform while over the gap.

The restored stair geometry and scripted controller come from this repository's `examples/sample_submissions/unitree_g1_stairs` before commit `d541eb4`. The retained model history is in [the walker bundle README](../../examples/policies/walker/README.md). The GR00T adapter is shared with `robot_safety/balance_policy.py`. Current measured results and remaining integration checks are in [VALIDATION.md](VALIDATION.md).
