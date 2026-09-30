# Containers

A submission is one Docker image. `./setup.sh` on the G1 loads it and starts it. The host safety service stays in charge of motor writes and stops.

The image may contain several models, and the code inside may be laid out however the author wants. The platform runs it only if it follows the rules below. It does not install dependencies for a submission, and it does not adapt a different sensor set or control API onto the robot.

## Rules

| | What the submission must do | What the platform supports |
| --- | --- | --- |
| Packaging | One `linux/arm64` image, dependencies already installed, one `ENTRYPOINT`. Delivered as `containers/upload/submission.tar`. | Loads that archive and starts the entrypoint. Does not look for ONNX files, a Python layout, or a second container. |
| Inputs | Read robot state from the topics below. Do not assume a camera, lidar, or odometry feed. | `/safety/robot_feedback` (29 joint `q` and `dq`) and `rt/lowstate` (IMU and the same 29 motors). No camera images. |
| Outputs | Publish a 29-slot `JointCommand` on `/safety/precheck` every control step. Stop when `/safety/abort` arrives. | The host checks that message and is the only writer of motor commands. |
| Limits | Stay inside the joint profile. A command uses a fresh `observation_id` (0.2 s) and the next one arrives before the action timeout (0.2 s). | Rejects a bad frame and latches the abort. In sim, the first command must arrive within 10 s, then the run lasts `--seconds` (default 10). On the robot the process runs until the operator stops it or the abort latches. No memory or CPU quota is applied. |
| Access | Talk to sensors and motors only through those interfaces. | Reads state and takes `/safety/precheck`. Does not grant `rt/lowcmd`, `rt/arm_sdk`, or control of `g1-safety-stops`. |

Details, joint order, and the save command are in [INSTRUCTIONS.md](INSTRUCTIONS.md).

## Base image

Unitree does not publish a G1 image. Start from `ros:foxy-ros-base`, platform `linux/arm64`. This robot is Ubuntu 20.04 / ROS 2 Foxy.

## What the image contains

- ROS 2 Foxy and `rmw_fastrtps_cpp`, so it can see the host safety graph.
- The submission program and any models it needs.
- One start script, set as the image `ENTRYPOINT`. That script runs when the container starts. It is not `setup.sh`. `setup.sh` stays on the robot and only launches the image.

## File to upload

One archive:

`containers/upload/submission.tar`

Build and save it on an arm64 machine, or with `docker buildx` for `linux/arm64`. Topic names, safety rules, and the save command are in [INSTRUCTIONS.md](INSTRUCTIONS.md).

## Try it in MuJoCo

`./sim.sh` runs that same archive against a simulated G1. The container is attached to a loopback-only network, so it cannot see `eth0` or write robot motors. `g1-safety-stops` stays up. The sim publishes the same feedback topics and applies `/safety/precheck`. Safety checks are enabled by default. Use `--no-safety-checks` to disable simulation checks, or `--safety-checks` to enable them explicitly. The result is `containers/sim/output/report.json`, which records the selected mode.

```bash
./sim.sh --self-test
./sim.sh
./sim.sh --counterfactuals
./sim.sh --scene containers/upload/environment/scene.xml
./sim.sh --scene containers/upload/environment/scene.xml --no-safety-checks
./sim.sh replay
./sim.sh stop
```

`--counterfactuals` repeats the run from standing, floor sitting, supine, prone, and both sides. Only the standing start treats a fall as a failure.

The default scene is a flat floor and the G1. Submitters can upload their own MuJoCo XML scene and assets beside `submission.tar`, then select it with `--scene`. The scene retains the G1 command interface and can add terrain, obstacles, and passive objects. Scene requirements and replay controls are in [SIM_RUN.md](SIM_RUN.md). Unchecked runs report `completed`, not a safety pass.
