# Sim run

`./sim.sh` runs a submission container against a simulated Unitree G1. Use it to check the container before `./setup.sh`, and to see whether its joint commands pass the safety checks. It does not write robot motors, and it does not stop `g1-safety-stops`.

The default scene is a flat floor and the 29-joint G1. Submitters can supply a MuJoCo XML scene and assets beside their image and select it with `--scene`. Safety checks are enabled by default and can be disabled with `--no-safety-checks`. The container sees the same ROS topics it sees on the robot. The recording plays in a browser.

The submission rules — packaging, inputs, outputs, limits, and access — are in [README.md](README.md). Topic fields are in [INSTRUCTIONS.md](INSTRUCTIONS.md). Disabling simulation checks does not change the robot launcher or the host safety service.

## What it is for

- Run `containers/upload/submission.tar` without the real robot moving.
- Reject commands that break the same joint rules the host checker uses.
- Record the motion so it can be orbited after the run.
- Check the sim itself with `./sim.sh --self-test`, which needs no archive.

## How it works

`./sim.sh` starts a Docker container whose network is loopback only. The submission joins that network. `eth0` is not there, so the submission cannot reach the robot.

Two processes share that network:

| Piece | Role |
| --- | --- |
| `containers/sim/bridge.py` | ROS 2 Foxy side. Publishes state, reads `/safety/precheck`, publishes `/safety/abort`. |
| `containers/sim/plant_server.py` | MuJoCo side, in `.venv-policy-safety`. Steps the G1 and applies the checks. |

The bridge and the plant talk over stdin. The plant is the only thing that advances physics.

The bridge publishes `RobotFeedback` on a 0.02 s timer and receives asynchronous `JointCommand` messages for all 29 joints. After the first accepted command, each timer callback advances MuJoCo by ten 0.002 s steps using the latest accepted command. The run duration is wall-clock time from that first acceptance. If none arrives within 30 seconds, the report is `no_command`, unless a rejected command has already latched a failure.

What the container gets:

| Source | What it is |
| --- | --- |
| `/safety/checker/arm` | Returns `session_id`. The command must use that id. |
| `/safety/robot_feedback` | 29 joint `q` and `dq`, plus `observation_id`. |
| `/safety/policy_observation` | The same feedback, for a node written against the host checker. |
| `rt/lowstate` on `lo` | IMU and the same 29 motors. `ROBOT_INTERFACE` is `lo`. |

What the container sends:

| Topic | What it is |
| --- | --- |
| `/safety/precheck` | One 29-slot `JointCommand` per step. |
| `/safety/abort` | Subscribe and stop. The sim publishes this when a check fails. |

A failed command is not applied. Later commands stay rejected. A rejected joint command keeps the last accepted hold. A fall, overspeed, or action timeout drops that hold and damps. The host safety service is left running.

With safety checks enabled, `containers/sim/runner.py` checks:

- `session_id` matches the armed session, `sequence` increases, and `observation_id` is still fresh.
- Targets, gains, and feedforward torque are finite and inside the sanity bounds.
- The command matches the joint profile: position, tracking, target rate, zero feedforward torque, and torque demand.
- After the step, a held overspeed latches. A fall latches only from the standing start.

`hardware_authorization` in the report is always false. A passing sim run is not permission to run on the robot.

With `--no-safety-checks`, the sim bypasses observation age, action timeout, numeric sanity bounds, joint position/tracking/rate/torque limits, feedforward-torque restrictions, overspeed, and falls. It keeps applying the last command until another arrives. MuJoCo's own physical joint and actuator limits remain part of the model. The ROS protocol still requires a matching session, increasing sequence, and an observation ID issued by the sim. Every command array must contain 29 finite numbers. Malformed commands and non-finite simulation state still terminate acceptance and report `error`.

Unchecked runs and direct circle runs report `completed`, never `within_limits`. Reports and counterfactual summaries include `safety_checks` and the selected `scene` path (`null` means the bundled scene). `--safety-checks` explicitly re-enables checking. If both flags are supplied, the last one wins.

`control_path` is `submission` for commands through the normal checks and `direct`
for `--circle` or `--command`. Reports include a `checks` mapping for command
protocol, command limits, observation freshness, action timeout, joint speed,
base angular speed, falls, and finite simulation state. These flags describe
enabled checks, not proof that every fault condition was exercised.

Direct control bypasses command protocol, command limits, and observation
freshness checks. With safety enabled, action timeout, speed and standing-fall
monitoring remain enabled. Its viewer displays `PARTIAL` and explicitly labels
submission checks as bypassed. This metadata is also stored in the trace so a
standalone replay retains that distinction.

## How to use

From the repo root:

To run a real policy immediately on a Mac or Windows laptop, use `./desktop_sim.sh --example walker --view`. That builds the example image, opens the site, and streams the live container after Start. A new submission works the same way: put one `submission.tar` in `containers/upload/` and run `./desktop_sim.sh --view`. `--local` skips Docker and is not the submission path. On the robot Linux host, `./sim.sh --example stairs --view` is the namespace equivalent. Complete commands are in [Runnable simulation policies](examples/README.md).

```bash
./sim.sh --self-test
./sim.sh --battery
./sim.sh --seconds 10
./sim.sh --counterfactuals
./sim.sh --scene containers/upload/environment/scene.xml
./sim.sh --scene containers/upload/environment/scene.xml --no-safety-checks
./sim.sh stop
```

`--self-test` holds the standing pose for one second and should finish `within_limits`. `./sim.sh` with no flag loads the one archive in `containers/upload/` and starts standing. `./sim.sh stop` removes the sim containers.

`--counterfactuals` is the same six starts the offline screening used: standing, floor sitting, supine, prone, and both sides. Each pose starts a new container, so the submission's own state is reset. Floor sitting folds the legs. The four lying poses keep the stand joint angles and turn the pelvis. Standing is the only pose that can fail for falling. A floor pose that stays down is not a fall, and the run does not require the robot to get up. Joint checks apply unless disabled, including the first command having to match the measured joints. The selected scene and safety setting apply to all six runs.

Each pose writes `containers/sim/output/poses/<pose>/report.json` and a trace. The battery writes `containers/sim/output/poses/summary.json`. Replay one pose with `./sim.sh replay containers/sim/output/poses/supine/trace.npz`. `--self-test --counterfactuals` holds the measured joints of every pose for one second. A floor pose can still fail a joint check while the body settles. That failure is not a fall.

Other flags:

| Flag | Effect |
| --- | --- |
| `--seconds 10` | Length of the run after the first accepted command. |
| `--image name` | Use this image instead of the archive in `containers/upload`. |
| `--example stairs\|walker\|balance` | Prepare a real ONNX policy, its matching scene, and its Docker image. Mutually exclusive with `--image` and `--self-test`. |
| `--local` | Run an `--example` directly against MuJoCo without Docker, ROS, or sudo. |
| `--output DIR` | Save a run separately so safety-on/off results can both be replayed. |
| `--scene path/to/scene.xml` | Load a complete MuJoCo G1 scene from the host filesystem. Relative paths use the caller's working directory. |
| `--safety-checks` | Enable simulation safety checks, the default. |
| `--no-safety-checks` | Run without simulation safety limits, freshness/timeouts, or fall checks. |
| `--pose name` | Start pose. Default `standing`. Also `seated`, `supine`, `prone`, `left_side`, `right_side`. |
| `--counterfactuals` | Run all six poses, each in a fresh container. Default length is 5 s. |
| `--battery` | Flat floor, all six poses, then a 10 N, 50 N, and 150 N pelvis push on each. Writes `containers/sim/output/battery/report.json`. |
| `--push none\|small\|medium\|heavy` | Run that one battery case. Add `--pose` to pick the start. Add `--view`, press Start, and watch it. |
| `--battery --circle` | Same pushes while the cached walker circles. Standing only. A gait step is applied directly, and the fall check stays on. |
| `--circle` | Walk one flat circle with the cached G1 walker. No submission. The fall check stays on. The submission tracking window is not applied, because a gait step does not fit it. |
| `--view` | Open the page first. Press Start to run, then the recording plays from the first frame. |
| `replay` | Serve the last recording. Does not start a new run. |
| `--help` | Show usage without starting Docker or requesting sudo. |

`./sim.sh` asks for `sudo` so it can enter the isolated network. `replay` does not.

`./sim.sh` needs Linux network namespaces, so it does not run on macOS or Windows. Use `./desktop_sim.sh` at the repo root, with the same flags. `--local`, `--self-test`, `--circle`, `--battery`, and `replay` run on the host and do not need Docker. The ROS submission path uses only Docker Compose: `containers/sim/Dockerfile` and `compose.yaml` define a loopback-only `net` service, a `sim` service on that network, and an optional `submission` service. The host viewer still serves `http://127.0.0.1:8765`. Reports stay in `containers/sim/output/`.

`--local` also runs without sudo. Local examples use simulated timestamps. Docker/ROS runs enforce wall-clock freshness and action deadlines. The default durations are unchanged. The scene and safety settings apply to both backends and all counterfactual poses.

Both entry scripts use the CLI and viewer handling in `sim.sh`. `desktop_sim.sh`
selects the Compose backend. The backend implementations are
`containers/sim/namespace.sh` and `containers/sim/compose.sh`. Local runs with
`--view` also wait for Start and stream live on both platforms.

The run writes:

| File | What it is |
| --- | --- |
| `containers/sim/output/report.json` | Status, reason, command count, and simulated time. |
| `containers/sim/output/trace.npz` | Poses, timestamps, and a compiled model snapshot for replay. |
| `containers/sim/output/bridge.log` | ROS bridge log, if the run fails early. |
| `containers/sim/output/policy.log` | Submitted container stdout/stderr, captured before removal. |

When `--output` is supplied, these files go in that directory. Docker runs can be stopped with `./sim.sh stop --output DIR`. Reports include `execution`, the selected example `policy` when applicable, and independent `motion` measurements. The simulation-only `/sim/state` topic supplies base pose and velocity to example controllers. Its schema is in [the examples guide](examples/README.md#simulation-only-policy-observations).

`status` in the report:

| Status | Meaning |
| --- | --- |
| `within_limits` | Submission checks were enabled, at least one command was accepted, and none failed. |
| `violation` | `reason` is the check that failed. `detail` names the joint when there is one. |
| `completed` | Commands ran with safety checks disabled, or through direct circle control without submission validation. This is not a safety pass. |
| `error` | An unchecked run failed protocol or simulation validation. |
| `no_command` | Nothing was accepted before the startup wait ended. |

The same checks, without Docker, are `containers/sim/test_runner.py`:

```bash
.venv-policy-safety/bin/python -m unittest containers.sim.test_runner containers.sim.test_scene
```

## Submit a MuJoCo environment

Upload a complete MJCF scene and its assets beside the image:

```text
containers/upload/
  submission.tar
  environment/
    scene.xml
    g1_robot.xml
    third_party/unitree_g1/assets/...
```

`--scene` selects a host file, not a path inside the image. Include files and meshes follow MuJoCo's normal path resolution from the scene and its compiler settings. Keep the asset directory layout when copying the environment. The host user who launches the sim must be able to read these files.

To start with this repository's G1, copy its model and assets:

```bash
mkdir -p containers/upload/environment
cp -R packages/policy_safety/src/policy_safety/robots/g1/. containers/upload/environment/
```

Create `containers/upload/environment/scene.xml`:

```xml
<mujoco model="submission_environment">
  <include file="g1_robot.xml"/>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <geom name="safety_floor" type="plane" size="0 0 0.05" friction="1.15"/>
    <geom name="step" type="box" pos="2 0 0.1" size="0.4 0.8 0.1"
          rgba="0.6 0.3 0.1 1"/>
  </worldbody>
</mujoco>
```

Then run `./sim.sh --self-test --scene containers/upload/environment/scene.xml`, or drop `--self-test` to run the submitted image.

The scene can change terrain, friction, gravity, obstacles, and passive objects. It must retain the G1's 29 named hinge joints and matching affine position actuators, `pelvis`, `floating_base_joint`, the two named pelvis IMU sensors, and a `stand` keyframe. Additional passive joints/free bodies are supported. If adding joints changes the model's position layout, update the `stand` keyframe accordingly. Additional actuators and different robots are not supported by the current command interface.

`standing` resets to the submitted `stand` keyframe, including its root position and orientation. Other poses require a ground geom named `safety_floor` and space to place the robot without substantial penetration. With checks enabled, the existing fall criterion still uses pelvis world height below 0.28 m or uprightness below 0.3, so choose the environment's vertical origin accordingly. Simulation timing remains 20 ms control / 2 ms physics even if the XML specifies another timestep.

## How to visualize

The first viewer export downloads Three.js r160 and its two addons from the
upstream release into the ignored `.cache/three-r160/` directory. Each file is
checked against a pinned SHA-256 hash. To prepare for offline viewing, run once
while online:

```bash
python3 containers/sim/viewer_assets.py
```

Subsequent exports use the verified cache. Each exported page includes local
copies of the dependencies and their MIT license, so the browser needs no CDN
connection. Headless simulations do not download viewer assets.

After a run:

```bash
./sim.sh replay
```

Open `http://127.0.0.1:8765`. The page plays the recorded G1 and environment from `containers/sim/output/trace.npz`. The trace embeds the compiled model, so replay does not need the original XML or asset files. Replay uses the installed MuJoCo version to load that snapshot. Older traces without a model snapshot use the bundled scene.

Replay draws mesh geometry, planes, boxes, spheres, ellipsoids, cylinders, capsules, and heightfield surfaces, including passive moving objects. Materials use solid colors and opacity. Textures and the scene's lighting are not reproduced.

| Control | Action |
| --- | --- |
| Left drag | Orbit |
| Right drag | Pan |
| Scroll | Zoom |
| Space | Pause |
| Left / right arrow | One frame back or forward while paused |

Ctrl+C stops the page. `./sim.sh --view` opens the page before the run. Press Start, and playback begins at the first frame while the run is going. The camera stays where you leave it. A blue line on the floor is the pelvis path. `./sim.sh --seconds 10 --view --circle` is that page with the cached walker and no submission archive.

`./sim.sh replay path/to/trace.npz` plays a different recording. The page is rebuilt into `view/` beside that trace. Both live and replay servers use the first free port starting at `G1_SIM_VIEW_PORT` (default 8765). Open the URL printed by the launcher.

The ROS bridge keeps its Foxy interpreter separate from the MuJoCo process.
Each active control tick steps physics and returns feedback in one request.
Pipe requests have a ten-second deadline, including partial responses. A failed
transport ends the run as `error` and the plant process is cleaned up.

Headless battery cases retain metrics without storing pose history. Live battery
viewing still streams poses. Local example and Docker runs continue recording
replay traces. When constructing `Runner` directly, pass `record_trace=True` or
set `G1_SIM_TRACE` before construction to retain poses for a later report.
Local examples prepare model and scene assets without staging Docker client
sources or ROS interfaces.
