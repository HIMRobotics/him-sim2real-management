# How to submit a policy

A submission is a single file, `submission.tar`, which is a Docker image saved
with `docker save`. The robot loads that image and runs its entrypoint. While
it runs, your program reads the robot's state from ROS topics and publishes
joint targets. It never talks to the motors directly: host safety software
checks every target first and is the only thing allowed to drive the robot.

## Quick start

The easiest way to build a submission is to start from
[`template/`](template/README.md). It already follows every rule on this page,
so the only thing you need to write is the policy itself.

Begin by copying the template with `cp -R containers/template my_policy`. Put
your weights and config files in `my_policy/assets/`, write your controller in
the `Policy.act(state)` method of `my_policy/policy.py`, and list any Python
packages it needs in `my_policy/requirements.txt`.

When you are ready to try it, build the image. This produces
`my_policy/submission.tar`:

```bash
HIM_REPO=$PWD my_policy/build.sh my-policy:local
```

Then run that archive in the simulator. A page opens in your browser; press
Start to watch the run live.

```bash
cp my_policy/submission.tar containers/upload/
./desktop_sim.sh --view     # on Mac or Windows; use ./sim.sh --view on the robot's Linux host
```

Your submission passes when `containers/sim/output/report.json` reports
`"status": "within_limits"`. At that point it is ready to
[run on the robot](#run-it-on-the-robot).

The [`examples/`](examples/README.md) directory contains finished reference
policies (the NVIDIA walker, GR00T balance, and Holosoma) that are useful
for comparison. They are not a good starting point, because their weights are
baked into a shared image that bundles several policies together.

## Files you need to write

`submission.tar` is not a zip of your files. It is a built Docker image: you
write the files below, `docker build` bakes them into an image, and `docker
save` writes that image out as the tar. On the robot, `setup.sh` loads the tar
and starts the container, and the container runs your start script.

- [ ] **`Dockerfile`**: the recipe for the image. It installs ROS and your
  dependencies, copies in everything else on this list, and names your start
  script as the `ENTRYPOINT`. It is used to build the tar but does not need to
  be inside it.
- [ ] **A start script**, such as `entrypoint.sh`: the script that runs when the
  container starts. It sources ROS 2 and the built message package, then
  launches your program. This is the only script that runs on the robot, so do
  not add a `setup.sh` of your own.
- [ ] **Your program**: the ROS node that reads state and publishes commands as
  described in [the rules](#the-rules).
- [ ] **Weights and config files** your program loads, such as ONNX models or
  YAML.
- [ ] **A dependency list**, such as `requirements.txt`, that the `Dockerfile`
  installs.
- [ ] **A copy of `ros2/him_safety_interfaces`** from this repository, which the
  `Dockerfile` builds so your program can use the message types.

In the template these are `Dockerfile`, `entrypoint.sh`, `node.py` with
`policy.py`, `assets/`, and `requirements.txt`. Its `build.sh` copies in the
message package, then builds and saves the tar for you.

## The rules

If you write your own image instead of using the template, it has to do
everything described in this section. In the template, this logic lives in
`node.py`.

### 1. Package the image

Build one Docker image for `linux/arm64` on top of `ros:foxy-ros-base`, which
gives you Ubuntu 20.04, ROS 2 Foxy, and Python 3.8. The image also needs
`rmw_fastrtps_cpp`, `std_srvs`, and this repository's
`ros2/him_safety_interfaces` message package built inside it. Install every
dependency and every weight file at build time, because the simulator gives
the container no network access.

Set a single `ENTRYPOINT` that starts your program. Do not add a `setup.sh` to
the image: the robot has its own `setup.sh`, which loads your archive and runs
the entrypoint for you. The robot starts the container as the host's normal
user with `HOME=/tmp`, so make sure your program works without root.

When the image is built, save it with `docker save IMAGE -o submission.tar`.

The launcher passes in the variables below. Leave them as they are rather than
setting them in the image.

| Variable | Value |
| --- | --- |
| `G1_EXECUTION_MODE` | `sim`, `dry-run`, or `hardware` |
| `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY`, `RMW_IMPLEMENTATION` | ROS settings for the safety graph (`rmw_fastrtps_cpp`) |
| `ROBOT_INTERFACE` | `eth0` on the robot and `lo` in simulation |

### 2. Read the robot's state

Subscribe to `/safety/policy_state` with best-effort QoS. Each message is a
`std_msgs/String` containing JSON, and a new one arrives roughly every 20 ms.

| Key | Meaning |
| --- | --- |
| `q`, `dq` | Positions (rad) and velocities of the 29 joints, in the [joint order](#joint-order) below |
| `quat`, `gyro` | Pelvis IMU orientation (w, x, y, z) and angular velocity |
| `observation_id` | An ID to copy into the command you compute from this state |
| `position`, `base_velocity` | Pelvis position and velocity. Always present in simulation, but on the robot only when a qualified base-state source is configured |

If you prefer, you can also read `/safety/policy_observation`, a
`RobotFeedback` message carrying `q`, `dq`, and `observation_id`, or the
Unitree `rt/lowstate` topic. Both are read-only. The robot does not provide
camera images, lidar, or odometry.

### 3. Arm once

Once the first state message has arrived, ask the safety checker for a
session. In simulation, call the `/safety/checker/arm` service
(`std_srvs/Trigger`); the session ID comes back in `response.message`.

On the robot (`dry-run` and `hardware` modes) there are a few more steps.
First call `/safety/controller_stop/ready` and `/safety/motor_gate/ready`, which
are both `Trigger` services. Then call `/safety/checker/arm` as above, and
finally pass the session ID you received to `/safety/motor_gate/arm`, which
takes an `ArmGate` request.

Arming starts a watchdog, so your first command needs to follow within 0.2
seconds.

### 4. Publish joint commands

For every new observation, publish one `him_safety_interfaces/JointCommand` on
`/safety/precheck` with reliable QoS and a queue depth of 1. There must never
be more than 0.2 seconds between commands.

| Field | Value |
| --- | --- |
| `session_id` | The ID you received when arming |
| `sequence` | Starts at 0 and goes up by 1 with every message |
| `observation_id` | Taken from the state you used, which must be less than 0.2 s old |
| `q` | 29 position targets (rad), in the [joint order](#joint-order) |
| `dq`, `tau` | 29 desired velocities and 29 feedforward torques, normally all zero |
| `kp`, `kd` | 29 gains each, between 0–500 and 0–50. The host replaces them with the gains from its approved joint profile |

Every command must include all 29 joints, so give any joint your policy does
not control a fixed target to hold. The host rejects a command if a target is
outside the joint profile or would need more torque than that joint can
safely produce. It also stops the robot if it falls or moves too fast.

### 5. Stop when you are told to

Subscribe to `/safety/abort` (`Abort`, reliable, transient-local). When a
message arrives, stop publishing and exit. An abort stays in effect, and
restarting your container will not clear it.

### What a submission must never do

A submission must never publish to `rt/lowcmd`, `rt/arm_sdk`, or any other
motor topic. It must not stop, restart, or replace `g1-safety-stops` or any of
the other safety nodes, and it must not leave a joint out of its commands.

### Joint order

Every 29-value array uses this order. In MuJoCo and URDF files the full names
add the group prefix and `_joint`, as in `left_hip_pitch_joint` or
`waist_yaw_joint`.

| Index | Joints |
| --- | --- |
| 0–5 | Left leg: `hip_pitch`, `hip_roll`, `hip_yaw`, `knee`, `ankle_pitch`, `ankle_roll` |
| 6–11 | Right leg, in the same order |
| 12–14 | Waist: `yaw`, `roll`, `pitch` |
| 15–21 | Left arm: `shoulder_pitch`, `shoulder_roll`, `shoulder_yaw`, `elbow`, `wrist_roll`, `wrist_pitch`, `wrist_yaw` |
| 22–28 | Right arm, in the same order |

## Test in simulation

Use `./desktop_sim.sh` on a Mac or Windows machine and `./sim.sh` on the
robot's Linux host. Both accept the same flags. They run your image against a
simulated G1 in MuJoCo on an isolated network, with the same safety checks the
robot uses.

```bash
./desktop_sim.sh --view                    # run the archive in containers/upload/ and watch it live
./desktop_sim.sh --image my-policy:local   # run a locally built image without exporting it
./desktop_sim.sh --no-safety-checks --view # see what the policy does with the checks turned off
./desktop_sim.sh --counterfactuals         # repeat the run from six different starting poses
./sim.sh replay                            # watch the last run again
```

Every run starts the way it does on the robot: GR00T balance holds the robot
for five seconds, standing in for the operator who waits before pressing
left + B. Your commands are checked during that time but not applied, and the
state's `control_ready` field turns true when they take over. The run then
lasts `--seconds` (10 by default). Afterwards, check the `status` field in
`containers/sim/output/report.json`. Only `within_limits` counts as a pass. A
run with checks turned off reports `completed` instead, and if a check fails,
`reason` and `detail` say which one. A stop that also records `host_stall`
happened while your computer had paused the simulator, so rerun it. The other
statuses are explained in [Reading the report](SIM_RUN.md#reading-the-report).

You can also test in your own MuJoCo environment by placing it beside the
archive and passing `--scene`. [Submit a MuJoCo
environment](SIM_RUN.md#submit-a-mujoco-environment) explains what the scene
must contain. The [containers README](README.md) has more commands to try.

## Run it on the robot

Copy the archive to the robot, then work through three stages: a preflight
that only loads and inspects the image, a dry run that exercises the full
safety path with motor output disabled, and finally the real run.

```bash
scp submission.tar unitree@<g1>:~/him-sim2real-management/containers/upload/submission.tar
ssh unitree@<g1>
cd ~/him-sim2real-management
./setup.sh
./setup.sh --mode dry-run
./submit.sh --mode real
```

Keep exactly one archive in `containers/upload/`. During the real run, the
robot balances under GR00T until you press left + B, which hands control over
to your policy. Pressing Start twice requests damping at any point, and Ctrl+C
ends the session. To use a joint profile other than the default, pass
`--profile FILE`. The
[operator guide](../robot_safety/HARDWARE_ADAPTER.md#operator-commands)
explains startup, handoff, and recovery in full.
