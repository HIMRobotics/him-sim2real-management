# Container instructions

A submission is one `linux/arm64` image with its dependencies installed. Models and code inside it can be organized in any way. The platform will not install packages for it, and will not connect some other sensor or motor API.

| | Rule |
| --- | --- |
| Packaging | `containers/upload/submission.tar`. One `ENTRYPOINT`. |
| Inputs | `/safety/robot_feedback` and `rt/lowstate` only. No camera, lidar, or odometry topic. |
| Outputs | 29-slot `JointCommand` on `/safety/precheck`. Stop on `/safety/abort`. |
| Limits | Fresh observation within 0.2 s, next command within 0.2 s, targets inside the joint profile. Sim: first command within 30 s, then `--seconds` (default 10). The robot process runs until stop or abort. No memory or CPU quota. |
| Access | State in and joint commands out through the topics below. Not `rt/lowcmd`, `rt/arm_sdk`, or `g1-safety-stops`. |

`./setup.sh` loads that archive and starts the entrypoint. `g1-safety-stops` on the host checks commands and writes motors.

`setup.sh` sets:

| Variable | Value |
| --- | --- |
| `ROBOT_INTERFACE` | `eth0` |
| `ROBOT_DOMAIN` | `0` |
| `ROS_DOMAIN_ID` | `97` |
| `ROS_LOCALHOST_ONLY` | `1` |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` |

## Inputs

| Source | What it is |
| --- | --- |
| `/safety/robot_feedback` | 29 joint `q` and `dq`, `start_button_pressed`, `observation_id` |
| `rt/lowstate` | IMU quaternion (w, x, y, z), gyro, accelerometer, rpy, and the same 29 motors |

Joint order: `left_hip_pitch`, `left_hip_roll`, `left_hip_yaw`, `left_knee`, `left_ankle_pitch`, `left_ankle_roll`, `right_hip_pitch`, `right_hip_roll`, `right_hip_yaw`, `right_knee`, `right_ankle_pitch`, `right_ankle_roll`, `waist_yaw`, `waist_roll`, `waist_pitch`, `left_shoulder_pitch`, `left_shoulder_roll`, `left_shoulder_yaw`, `left_elbow`, `left_wrist_roll`, `left_wrist_pitch`, `left_wrist_yaw`, `right_shoulder_pitch`, `right_shoulder_roll`, `right_shoulder_yaw`, `right_elbow`, `right_wrist_roll`, `right_wrist_pitch`, `right_wrist_yaw`.

The platform does not provide odometry, camera images, or lidar. `rt/lowstate` accelerometer is not odometry. A driver shipped inside the image does not add a second way to command motors.

## Output

Publish `JointCommand` on `/safety/precheck` every control step. All 29 slots, same order.

| Field | Value |
| --- | --- |
| `q`, `dq`, `kp`, `kd`, `tau` | Target, desired velocity, gains, feedforward torque |
| `session_id` | From `/safety/checker/arm` |
| `sequence` | Increases by 1 each frame |
| `observation_id` | From the `RobotFeedback` used for that frame |

A joint the policy does not command stays in the message and is damped: `kp` 0, `kd` 2, `dq` 0, `tau` 0, `q` held at the measured angle.

Subscribe to `/safety/abort` and stop publishing when it arrives.

The host gate writes approved waist and arm targets (indices 12–28) to `rt/arm_sdk`. It checks all 29 joints. A precheck frame does not become a 29-motor `rt/lowcmd` stream.

## Safety

Allowed:

- Read `/safety/robot_feedback` and `rt/lowstate`.
- Publish `/safety/precheck`.
- Subscribe to `/safety/abort` and stop.
- Run a driver that was installed in the image. That driver still cannot command motors.

Not allowed:

- Publish `rt/lowcmd` or `rt/arm_sdk`.
- Stop, restart, or replace `g1-safety-stops`.
- Clear an abort by restarting one safety node.
- Drop a joint from the 29-slot command.

Double Start is handled by the host service and requests stock damping.

## Save and upload

From the image build directory, on arm64 or with buildx:

```bash
docker build --platform linux/arm64 -t submission:local .
docker save submission:local -o submission.tar
```

Copy that file to the robot:

```bash
scp submission.tar unitree@<g1>:~/him-sim2real-management/containers/upload/submission.tar
```

On the robot:

```bash
./setup.sh
```

Ctrl+C stops it. `./setup.sh stop` removes it. The safety service keeps running.

## MuJoCo

`./sim.sh` loads the same archive and runs it against a simulated G1 on the robot's Linux host. Mac and Windows use `desktop_sim.sh` at the repo root, with the same flags. Host modes (`--local`, `--self-test`, `--circle`, `--battery`, `replay`) run MuJoCo without Docker. The ROS submission path uses `containers/sim/Dockerfile` and `compose.yaml`. Use this before `./setup.sh`, and for the safety check. It does not use `eth0` and does not stop `g1-safety-stops`. The runner, the report, and the replay controls are in [SIM_RUN.md](SIM_RUN.md).

```bash
./sim.sh --self-test
./sim.sh --seconds 10
./sim.sh --scene containers/upload/environment/scene.xml
./sim.sh --scene containers/upload/environment/scene.xml --no-safety-checks
./sim.sh replay
```

Safety checks are enabled by default. `--no-safety-checks` disables the sim's limits, freshness/timeouts, overspeed, and fall checks. Session/sequence validation and 29 finite values per command array remain required. `--safety-checks` explicitly enables checking. These flags only affect simulation.

Submitters may include an `environment/` directory beside `submission.tar`, containing a complete G1 MuJoCo scene and its assets. Select its host XML path with `--scene`. The XML must retain the named G1 joints, actuators, floating base, pelvis IMU sensors, and `stand` keyframe. See [SIM_RUN.md](SIM_RUN.md#submit-a-mujoco-environment) for an example and counterfactual requirements.

`./sim.sh --view` opens `http://127.0.0.1:8765` before the run. Press Start and the page plays from the first frame. The camera stays put, and a blue line is the pelvis path. `./sim.sh replay` serves the last recording on that same URL. Left drag orbits, right drag pans, scroll zooms, space pauses, arrow keys step one frame. Replay uses the recorded G1 and environment, including obstacles. The recording is `containers/sim/output/trace.npz`. `./sim.sh --circle` walks one flat circle with the cached walker and does not load a submission.

`./sim.sh stop` removes the sim containers. The report is `containers/sim/output/report.json`. `within_limits` means commands were accepted with checking enabled and no violation. `completed` means the run had commands with checking disabled or used direct circle control without submission validation. `violation` names a failed check. `error` indicates an unchecked run failed protocol or simulation validation. `no_command` means nothing was accepted. Reports include `safety_checks`, `control_path`, per-check `checks` flags, and `scene`. For direct circle control, `safety_checks: true` enables runtime monitoring only and does not establish submission acceptance.

The sim serves `/safety/checker/arm`, publishes `/safety/robot_feedback` and `/safety/policy_observation`, and publishes `rt/lowstate` on `lo` inside that isolated network. `ROBOT_INTERFACE` is `lo`. With checks enabled, a command that fails a joint check is not applied. `/safety/abort` follows, and later commands stay rejected.

The timed window starts when the first command is accepted. If none arrives within 30 seconds, the report is `no_command`. A rejected joint command keeps the last accepted hold. A fall, overspeed, or action timeout drops that hold and damps. `./sim.sh --self-test` needs no archive: it holds the stand for one second. The checks themselves are `containers/sim/test_runner.py`.
