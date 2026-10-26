# Sim run

`./sim.sh` runs a submission container against a simulated Unitree G1. Use it to check the container before `./setup.sh`, and to see whether its joint commands pass the safety checks. It does not write robot motors, and it does not stop `g1-safety-stops`.

The default scene is a flat floor and the 29-joint G1. Submitters can supply a MuJoCo XML scene and assets beside their image and select it with `--scene`. Safety checks are enabled by default and can be disabled with `--no-safety-checks`. The container sees the same ROS topics it sees on the robot. The recording plays in a browser.

The submission rules (packaging, inputs, outputs, limits, and access) and the topic fields are in [INSTRUCTIONS.md](INSTRUCTIONS.md). Disabling simulation checks does not change the robot launcher or the host safety service.

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
| `containers/sim/bridge.py` | ROS 2 side. Runs the shared safety checker, feeds measured state, and passes accepted postcheck frames through the shared motor gate to the plant. |
| `containers/sim/plant_server.py` | MuJoCo side, in `.venv-policy-safety`. Steps the G1 and applies the checks. |

The bridge and the plant talk over stdin. The plant is the only thing that advances physics.

With safety enabled, the ROS route is `/safety/precheck` →
`robot_safety.ros_nodes.SafetyCheckerNode` → `/safety/postcheck` →
`robot_safety.motor_gate.MotorCommandGate` → the MuJoCo plant. The checker and
bridge share an executor for simulation. They use the same command expiry,
session validation, thrash detector and abort latch as the robot gateway.
The plant retains its additional joint-profile and physics checks. Simulation
does not reproduce the hardware deployment's process isolation or stop dynamics.
Unchecked and direct-circle runs retain their bypass of the ROS checker.

The bridge publishes `RobotFeedback` on a 0.02 s timer and receives asynchronous `JointCommand` messages for all 29 joints. After the first accepted command, each timer callback advances MuJoCo by ten 0.002 s steps using the latest accepted command. For the first five seconds startup balance holds the robot, as it does on hardware before the operator presses left + B, and accepted commands are not applied. The run lasts those five seconds plus `--seconds` of wall-clock time from the first acceptance. If none arrives within 30 seconds, the report is `no_command`, unless a rejected command has already latched a failure.

The container sees the same topics and services it would see on the robot,
which are listed in [the rules](INSTRUCTIONS.md#the-rules). Two things differ
in simulation: `rt/lowstate` is published on `lo` (so `ROBOT_INTERFACE` is
`lo`), and the sim itself publishes `/safety/abort` when a check fails.
`/safety/robot_feedback` carries raw joint state with no authorized
observation ID, so commands must take their IDs from `/safety/policy_state` or
`/safety/policy_observation`.

A failed command revokes policy output and latches the first fault. Later
commands stay rejected. Both local simulation and ROS aborts clear the plant's
active command and select simulated damping. Hardware executes its own stop
procedure. The host safety service is left running.

## Shared safety decisions

Local simulation and the ROS supervisor use the same command protocol validator
and watchdog decision function in `robot_safety/core.py`, with defaults from
`Config`. Action, observation, feedback and loop deadlines default to 0.2 s.
Expired commands cannot be rescued by a late replacement or fresh feedback.
ROS retains its `invalid_action` fault label for protocol errors, while local
reports include the specific protocol rejection.

Checked simulation and the hardware adapter use `joint_checks.checked_targets`
for clipping, target velocity, feedforward and PD torque
checks. Both select gains from the same joint profile. Submitted gains are
validated for sanity but do not override the profile. The processed targets
and gains are the ones applied to the plant. Profile parsing and torque-demand
arithmetic are shared with the portable policy evaluator as well.

MuJoCo physics, Unitree I/O, hardware health and stop execution stay in their
respective adapters. Hardware currently publishes upper-body targets and
monitors all 29 joints, while simulation actuates all 29. The offline policy
experiment evaluator still uses its explicit bundle gains and torque-scale
scenarios. Those configurations and unchecked/direct experiments do not claim
hardware-equivalent actuation.

In simulation, a joint velocity or torque past the model rating is a warning.
The command stays applied, MuJoCo still saturates actuator force at the model
force range, and the report names the worst exceedance. Protocol errors,
timeouts, non-finite state, the startup speed gate, base angular speed, falls,
and enforced thrash still stop the run. Hardware checks are separate: measured
speed and torque still reject there. Reports record a real stop as
`stop_time_seconds`.

## Shared thrash detection

Local and ROS simulations both use `robot_safety/thrash.py` and `ThrashConfig`,
the same implementation and defaults as the real checker. Local mode samples
measured velocities at each 2 ms physics step using simulation time. Safety-enabled
ROS mode uses checker feedback receipt times at the bridge's 20 ms cadence.
With other checks disabled, ROS runs sample in the plant as local runs do.
There is only one active detector per path, with different sampling latency.
The walker scenarios reuse the existing walker model and submission command
path, not the direct `--circle` shortcut:

```bash
# Belly-up, only thrash enforcement enabled.
./sim.sh --example walker/thrash --local --seconds 20 --view

# Same experiment, readings without thrash stops.
./sim.sh --example walker/thrash-monitor --local --seconds 20 --view

# Upright comparison to check false positives.
./sim.sh --example walker/thrash-monitor --pose standing --local --seconds 20 --view
```

Use `./desktop_sim.sh` with the same arguments on desktop. Omit `--local` to
exercise the ROS submission path. Both scenarios default to `supine` with
other safety checks disabled. Explicit `--pose` and `--safety-checks` override
those defaults. Protocol validation and non-finite simulation failures remain
active. Settings live in `containers/examples/prepare.py`, without a separate
policy or detector implementation.

Ordinary submissions monitor thrashing when safety checks are enabled and
turn detection off with `--no-safety-checks`. For custom simulation setups,
`G1_SIM_THRASH_MODE=monitor|enforce|off` configures the shared detector. Walker
scenarios select their own mode. Hardware uses the `thrash.mode` checker ROS
parameter. There is no detector-specific launcher flag.

`G1_SIM_LIMIT_MODE=clip` runs checked submissions through the hardware adapter
with `limit_mode=clip`, the same mode the robot gets from `G1_LIMIT_MODE=clip`
(see `robot_safety/README.md`). Targets are still projected into the 90% torque
budget. Measured torque or PD demand above the profile ceiling and base gyro
speed above 6 rad/s become warnings (`base_gyro_x/y/z` in the report) instead of
stops. Everything else the adapter checks still stops and hands over to
recovery: command, timing and joint-speed limits, tilt and standing posture,
arming. The default `stop` enforces both limits.

On Docker Compose, a single run without `--view` whose stop the bridge flags as
inconclusive (`host_stall` in the report) restarts Docker Desktop and reruns.
`G1_SIM_STALL_RETRIES` sets the number of reruns (default 1, 0 disables).

`--lockstep` (`G1_SIM_LOCKSTEP=1`) runs the ROS path like training: after each
step the bridge waits for the policy's command to that state before the next
20 ms step, and the bridge, checker, motor gate and plant all read the
simulated clock. Playback stays at or below real time, a host stall pauses the
run instead of tripping a deadline or dropping commands, and the report says
`"clock": "lockstep"`. Policy state is best effort, so a reply missing for
0.5 s of wall time counts as lost: the plant steps without it and the report
counts it in `lockstep_missed_replies`, and the usual deadlines decide on the
simulated clock. Lockstep hides a policy that is too slow for 50 Hz, so use
the default wall clock to check timing. A policy that exits when its state
pauses must allow longer than a host stall.

A checked ROS run that reaches `--seconds` without a stop ends the way a robot
run does when the submission container exits: the bridge stops forwarding the
policy, the hardware adapter receives `policy_container_exit` and selects the
automatic Holosoma zero-velocity balance if the robot is standing, and the sim
keeps running that for `G1_SIM_HANDOFF_SECONDS` (default 5, 0 ends immediately).
The status is unchanged; the report's `handoff` field records the time, the
controller in charge at the end (`holosoma` or `damping`), and the final
standing height and uprightness. Runs without checks have no adapter and end
at `--seconds`.

Reports include `thrash.mode`, the first `thrash.event`, current metrics, and
separate `checks.thrash_monitoring` / `checks.thrash_enforcement` flags. The
finished-run viewer shows the mode and any detection. Other checks passing
does not mean a monitored thrash warning was absent.

Traces and live views show whole-body weighted oscillation RMS, its limit,
and the continuous time above that limit. Per-joint rows show oscillatory RMS,
motion RMS, weight, and percentage of the total weighted energy. Hips, knees
and waist weigh 4, shoulders 3, ankles 2, elbows 1, and wrists 0.25. The shared
score is `sqrt(sum(weight * oscillatory_RMS²) / sum(weight))`, with a provisional
limit of 1.5 for 500 ms. See [detector settings](../robot_safety/README.md#thrash-detection-in-the-existing-checker)
for details. These are motion-risk weights, not measured joint inertias.
Detection defaults remain provisional, without automatic threshold adaptation
or task exemptions.

### Calibration observations

These are simulation observations, not hardware validation. Recorded upright
circle walking (30 s), straight walking (20 s), and a start tilted 10° in pitch
and yaw (20 s) produced no detections, including when the measured velocities
were replayed at 2 ms and 20 ms intervals. Supine circle and straight-walker
recordings triggered at about 0.73–0.74 s at both rates.

A forward-pitch sweep used the straight walker with a fixed 10° yaw, 20 s per
case, monitoring only and other stops disabled. The robot was rotated about its
toes with the joint angles unchanged, all four toe contacts touching.

| Initial pitch | First detection | Outcome |
| --- | --- | --- |
| 20° | None | Recovered upright |
| 25° | None | Recovered upright |
| 30° | 0.970 s | Fell |
| 35° | 0.922 s | Fell |
| 40° | 0.770 s | Fell |

That is one deterministic trajectory per angle, not a universal safe tilt limit.

## Stop reasons in the viewer

The viewer records the first policy-stop reason, time, affected joint or base
axis, measured value and limit where available. Thrash enforcement includes the
whole-body score, limit, duration and largest joint contributors. The card distinguishes
PD torque demand from applied torque. Live frames and new traces retain this
metadata, and older replays can recover it from the adjacent report. Later
faults do not replace the original cause. A stopped policy may be followed by
simulated damping, so total replay duration is not the time of the fault.

No current-to-target position-gap or response-error threshold stops motion,
including the first command. Position clipping is a warning. A very large or
malformed command can still fail protocol sanity bounds, and position error
still contributes to the separate PD torque calculation.

| Stop source | When active |
| --- | --- |
| Joint target/measured velocity, feedforward torque, PD/applied torque | Checked submissions, with measured velocity also checked in direct circle mode |
| Base angular speed and standing fall | Safety checks on, with upright fall disabled for floor poses |
| Command/observation/feedback/loop deadlines | Safety checks on, according to the control path |
| Thrashing | Enforcement mode, including `walker/thrash`, independently of other safety checks |
| Malformed commands, wrong session/sequence, unknown observation IDs | Submission protocol, including unchecked runs |
| Non-finite simulation, policy/plant exceptions, process shutdown | All modes |
| External abort, expired postcheck envelope or silent checker | ROS gateway paths |

A requested duration ending is normal completion. Waiting for a policy until
the ROS startup deadline expires produces `no_command`, rather than a safety
pass. With `walker/thrash-monitor`, motion limits and thrash
stops are disabled, while diagnostics continue.

## Other simulation checks

With safety checks enabled, `containers/sim/runner.py` checks:

- `session_id` matches the armed session, `sequence` increases, and `observation_id` is still fresh.
- Targets, gains, and feedforward torque are finite and inside the sanity bounds.
- The shared processor clips positions and desired joint speed. Feedforward torque, torque demand, applied torque, and measured joint speed are warnings; the startup speed gate still rejects.
- During each physics step, a fall latches only from the standing start. Joint overspeed is recorded and the step continues.

`hardware_authorization` in the report is always false. A passing sim run is not permission to run on the robot.

With `--no-safety-checks`, the sim bypasses freshness deadlines, numeric range checks, desired-speed clipping, torque-budget projection, measured joint-limit warnings, body-spin checks, and falls. Output positions are still clipped to joint bounds, and MuJoCo's physical joint and actuator limits remain part of the model. Thrash detection is configured separately. The last command remains active until another arrives. The ROS protocol still requires a matching session, increasing sequence, and an observation ID issued by the sim. Every command array must contain 29 finite numbers. Malformed commands and non-finite simulation state still terminate acceptance and report `error`.

Unchecked runs and direct circle runs report `completed`, never `within_limits`. Reports and counterfactual summaries include `safety_checks` and the selected `scene` path (`null` means the bundled scene). `--safety-checks` explicitly re-enables checking. If both flags are supplied, the last one wins.

`control_path` is `submission` for commands through the normal checks and `direct`
for `--circle` or `--command`. Reports include a `checks` mapping for command
protocol, command limits, observation freshness, action timeout, joint speed,
base angular speed, falls, and finite simulation state. These flags describe
enabled checks, not proof that every fault condition was exercised.

Direct control bypasses command protocol, command limits, and observation
freshness checks. With safety enabled, action timeout and standing-fall
monitoring remain enabled. Measured joint speed and torque past the model
rating are shown on the viewer and do not stop the run. Its viewer displays `PARTIAL` and explicitly labels
submission checks as bypassed. This metadata is also stored in the trace so a
standalone replay retains that distinction.

## Flags and run modes

This section is the full reference. For a guided tour of the common commands,
see the [containers README](README.md).

`--self-test` holds the standing pose for one second and should finish `within_limits`. `./sim.sh` with no flag loads the one archive in `containers/upload/` and starts standing. `./sim.sh stop` removes the sim containers.

`--counterfactuals` is the same six starts the offline screening used: standing, floor sitting, supine, prone, and both sides. Each pose starts a new container, so the submission's own state is reset. Floor sitting folds the legs. The four lying poses keep the stand joint angles and turn the pelvis. Standing is the only pose that can fail for falling. A floor pose that stays down is not a fall, and the run does not require the robot to get up. Joint checks apply unless disabled, including the first command having to match the measured joints. The selected scene and safety setting apply to all six runs.

Each pose writes `containers/sim/output/poses/<pose>/report.json` and a trace. The battery writes `containers/sim/output/poses/summary.json`. Replay one pose with `./sim.sh replay containers/sim/output/poses/supine/trace.npz`. `--self-test --counterfactuals` holds the measured joints of every pose for one second. A floor pose can still fail a joint check while the body settles. That failure is not a fall.

The policy package, the container, and the velocity file are three different choices. `report.json` field `policy` is the name recorded for the run.

| Choice | How to specify it | What runs | `policy` in the report |
| --- | --- | --- | --- |
| Policy package | `--example walker`, `balance`, or `holosoma` | `containers/examples/policies/<name>` and the client in `containers/examples`. The image is `g1-sim-example-<name>:local`. | `walker`, `balance`, or `holosoma` |
| Container | `--image NAME`, or the one archive in `containers/upload` | That image's entrypoint. [The template](template/README.md) builds one. See [build and export](examples/README.md#build-and-export-the-shared-image) for the shared walker image. `G1_TASK_COMMAND` sets the shared client's task velocity. | `null` unless an example was selected |
| Velocity file on the plant | `--command containers/examples/commands/walker/FILE.yaml` | No policy container. The network stays the cached walker in `containers/examples/policies/walker`. `G1_SIM_WALKER` can point at another copy of those weights. | The file name without `.yaml`, such as `forward_stop` |
| Same file in the walker container | `--example walker --command containers/examples/commands/walker/FILE.yaml` | The `g1-sim-example-walker:local` container reads that file and publishes joint commands. `--local` uses the same file without Docker. | `walker` |
| Circle shortcut | `--circle` | Same cached walker. The velocity file is `containers/examples/commands/walker/circle.yaml`. | `circle` |

`--command` does not select a policy package or start a container. `--example` does not read `containers/examples/commands/walker/`.

Other flags:

| Flag | Effect |
| --- | --- |
| `--seconds 10` | Length of the run after the five-second startup period. |
| `--image name` | Use this image instead of the archive in `containers/upload`. |
| `--example walker\|balance\|holosoma` | Prepare that policy package, its scene, and its Docker image. Mutually exclusive with `--image` and `--self-test`. |
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
| `--circle` | Cached walker, velocity file `circle.yaml`. No container. Report `policy` is `circle`. Uses direct policy commands, with the fall check on and active checks listed in the report. |
| `--command FILE` | Same runner, with that velocity file. No container. Report `policy` is the file name without `.yaml`. `forward_stop.yaml` walks at 0.5 m/s for 2 s, then steps down 0.1 m/s each second to zero. Yaw is zero, and the final zero velocity selects NVIDIA Balance. Use `--seconds 7` to see the stop. |
| `--view` | Open the page first. Press Start to run, then the recording plays from the first frame. |
| `replay` | Serve the last recording. Does not start a new run. |
| `--help` | Show usage without starting Docker or requesting sudo. |

`./sim.sh` asks for `sudo` so it can enter the isolated network. `replay` does not. On the robot's Linux host it also needs the existing ROS 2 Foxy installation, `colcon`, Docker, and `.venv-policy-safety`.

`./sim.sh` needs Linux network namespaces, so it does not run on macOS or Windows. Use `./desktop_sim.sh` at the repo root, with the same flags. `--local`, `--self-test`, `--circle`, `--battery`, and `replay` run on the host and do not need Docker. The ROS submission path uses only Docker Compose: `containers/sim/Dockerfile` and `compose.yaml` define a loopback-only `net` service, a `sim` service on that network, and an optional `submission` service. The host viewer still serves `http://127.0.0.1:8765`. Reports stay in `containers/sim/output/`.

`--local` also runs without sudo. Local examples use simulated timestamps. Docker/ROS runs enforce wall-clock freshness and action deadlines. The default durations are unchanged. The scene and safety settings apply to both backends and all counterfactual poses.

Both entry scripts use the CLI and viewer handling in `sim.sh`. `desktop_sim.sh`
selects the Compose backend. The backend implementations are
`containers/sim/namespace.sh` and `containers/sim/compose.sh`. Local runs with
`--view` also wait for Start and stream live on both platforms.

## Reading the report

The run writes:

| File | What it is |
| --- | --- |
| `containers/sim/output/report.json` | Status, reason, command count, and simulated time. |
| `containers/sim/output/trace.npz` | Poses, timestamps, and a compiled model snapshot for replay. |
| `containers/sim/output/bridge.log` | ROS bridge log, if the run fails early. |
| `containers/sim/output/policy.log` | Submitted container stdout/stderr, captured before removal. |

When `--output` is supplied, these files go in that directory. Docker runs can be stopped with `./sim.sh stop --output DIR`. Reports include `execution`, the selected example `policy` when applicable, and independent `motion` measurements. The shared client receives base pose and velocity through `/safety/policy_state`. Its schema is in [the examples guide](examples/README.md#shared-policy-observations).

`status` in the report:

| Status | Meaning |
| --- | --- |
| `within_limits` | Submission checks were enabled, at least one command was accepted, and none failed. |
| `violation` | `reason` is the check that failed. `detail` names the joint when there is one. |
| `completed` | Commands ran with safety checks disabled, or through direct circle control without submission validation. This is not a safety pass. |
| `error` | An unchecked run failed protocol or simulation validation. |
| `no_command` | Nothing was accepted before the startup wait ended. |

The launcher's exit code follows the status: 0 for `within_limits` or
`completed`, 1 for a safety violation, and 2 for an execution error or no
command.

Docker runs keep the robot's 0.2 s deadlines, so a host that pauses the
Docker VM can stop a run. On a Mac short of memory, macOS swaps the VM out and
every container freezes for a few hundred milliseconds. When the simulator's
own 20 ms loop was stalled at least 0.1 s at the stop, the report adds
`host_stall` with the longest gap, and the result is inconclusive rather than a
policy fault. Free memory, or lower Docker Desktop's memory limit (the stack
needs about 1 GB), and rerun.

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

The G1's collision geoms have `priority="1"`, so contacts use the robot's
friction (1.0) and a floor `friction` alone has no effect. Give the floor
`priority="2"` to set ground friction. `robots/g1/scene_carpet.xml` (next to
`g1_robot.xml`) is the bundled floor with carpet-like sliding friction 1.2.

The scene can change terrain, friction, gravity, obstacles, and passive objects. It must retain the G1's 29 named hinge joints and matching affine position actuators, `pelvis`, `floating_base_joint`, the two named pelvis IMU sensors, and a `stand` keyframe. Additional passive joints/free bodies are supported. If adding joints changes the model's position layout, update the `stand` keyframe accordingly. Additional actuators and different robots are not supported by the current command interface.

`standing` resets to the submitted `stand` keyframe, including its root position and orientation, then raises or lowers the robot until its feet just meet `safety_floor` or a platform. Start the keyframe slightly above the support. Other poses require a ground geom named `safety_floor` and space to place the robot without substantial penetration. With checks enabled, the existing fall criterion still uses pelvis world height below 0.28 m or uprightness below 0.3, so choose the environment's vertical origin accordingly. Simulation timing remains 20 ms control / 2 ms physics even if the XML specifies another timestep.

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

Space pauses and arrow keys step both motion and readings. **Go to start / Home**
jumps to the first frame and pauses. **Restart replay / R** plays from the
beginning, preserving the camera and without rerunning the simulation.

When the sim runs on the robot over SSH, forward the port with
`ssh -L 8765:127.0.0.1:8765 unitree@<robot>` and open the same URL on your laptop.

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
