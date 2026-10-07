# Walker test submission

This folder contains the code and Dockerfile for one simulation test submission. Models live in `examples/policies/walker/`, and command files live in `examples/commands/walker/`. The network is the G1 walker in `walker.onnx`: 99 inputs, 29 joint targets. `stand.yaml` and `circle.yaml` set a constant `vx`, `vy`, and `yaw_rate`. `forward_stop.yaml` walks at 0.5 m/s for 2 s, then steps the command down by 0.1 m/s each second, with a yaw command that cancels this walker's drift. It does not publish `rt/lowcmd` or `rt/arm_sdk`. The GR00T balance file is not this network and is not loaded.

This is not a general deployment Dockerfile or a validated robot policy launcher. It relies on simulator base velocity. Use it with `sim.sh` to test the submission archive interface.

## Image

Base image `ros:foxy-ros-base`, platform `linux/arm64`. The image installs `rmw_fastrtps_cpp`, builds `him_safety_interfaces`, and contains the ONNX file and `unitree_sdk2py`. `examples/submissions/walker/entrypoint.sh` is the image `ENTRYPOINT`. `setup.sh` stays on the host and only launches the image.

| Source | Use |
| --- | --- |
| `/safety/checker/arm` | `session_id` |
| `/safety/robot_feedback` | 29 joint `q` and `dq`, `observation_id` |
| `rt/lowstate` | IMU quaternion (w, x, y, z) and gyro |
| `/sim/state` | Base velocity for the walker observation. Sim only. |
| `/safety/precheck` | 29-slot `JointCommand` |
| `/safety/abort` | Stop publishing |

`policy.py` uses the shared `policy_safety.adapter.G1Walker` inference adapter,
also used by the simulator circle and example clients. The build stages the
adapter and its configuration module into the image.

The first command holds the measured joints. Later commands are the walker targets, with gains from `walker.yaml`. Feedforward torque is zero. `WALKER_COMMAND` selects the yaml. The default is `stand.yaml`. A gait step does not fit the submission tracking window, so `./sim.sh --command` applies that yaml on the plant instead of through the image.

`--circle` and `--command` use direct control and report `completed` on success.
They do not start this image. The report `policy` field is the command file
name (`circle`, `stand`, or `forward_stop`), not this container. With safety
enabled, their reports list the remaining runtime checks and the viewer labels
safety as `PARTIAL`. Use this image through `containers/upload` or `--image`,
or use `--example walker`, to exercise submission checks. `WALKER_COMMAND`
applies only inside this image.

## Build

From the repository root, on this arm64 machine:

```bash
./examples/submissions/walker/build.sh
```

That writes `.cache/walker_submission/submission.tar` and does not start a container. The image contains `walker.onnx` and `walker.onnx.data`. It uses ONNX Runtime 1.16.3 because that is the newest build published for Python 3.8 on linux/arm64. CycloneDDS is copied from this robot's Python 3.8 install, since PyPI has no matching wheel. The SDK default is `/home/unitree/Workspace/unitree_sdk2_python/unitree_sdk2py`. Override with `UNITREE_SDK2PY`.

## Run in sim

```bash
cp .cache/walker_submission/submission.tar containers/upload/submission.tar
./sim.sh --seconds 10 --view
```

`./sim.sh` attaches the container to a loopback-only network and sets `ROBOT_INTERFACE=lo`. It does not use `eth0`. Safety checks stay on. A target the joint profile rejects ends the run as `violation`. With `--view`, open `http://127.0.0.1:8765` and press Start. Playback starts at the first frame. The camera stays put, and a blue line is the pelvis path. That path is only in the browser. `./sim.sh --command examples/commands/walker/stand.yaml --view` holds the walker stance. `./sim.sh --command examples/commands/walker/circle.yaml --view` is the loop. `./sim.sh --command examples/commands/walker/forward_stop.yaml --seconds 7 --view` walks at 0.5 m/s for 2 s, then steps down to a stop. None of these load the GR00T balance file. The image still packages `stand.yaml` and `circle.yaml`; the walk-then-stop file is played by `--command` on the plant.
