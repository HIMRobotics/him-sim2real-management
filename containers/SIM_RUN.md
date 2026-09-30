# Sim run

`./sim.sh` runs a submission container against a simulated Unitree G1. Use it to check the container before `./setup.sh`, and to see whether its joint commands pass the safety checks. It does not write robot motors, and it does not stop `g1-safety-stops`.

The scene is a flat floor and the 29-joint G1. The container sees the same ROS topics it sees on the robot. A desktop MuJoCo window is not used. The recording plays in a browser.

The submission rules — packaging, inputs, outputs, limits, and access — are in [README.md](README.md). Topic fields are in [INSTRUCTIONS.md](INSTRUCTIONS.md). The sim accepts that same container and does not relax the rules.

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

Each control step is 0.02 s. The plant publishes a `RobotFeedback`, the container answers with one `JointCommand` for all 29 joints, and the plant either steps MuJoCo or latches a violation. The clock for the run starts when the first command is accepted. If none arrives within 10 seconds, the report is `no_command`.

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

A failed check is not applied. Later commands stay rejected. The host safety service is left running.

Checks, in `containers/sim/runner.py`:

- `session_id` matches the armed session, `sequence` increases, and `observation_id` is still fresh.
- Targets, gains, and feedforward torque are finite and inside the sanity bounds.
- The command matches the joint profile: position, tracking, target rate, zero feedforward torque, and torque demand.
- After the step, a held overspeed latches. A fall latches only from the standing start.

`hardware_authorization` in the report is always false. A passing sim run is not permission to run on the robot.

## How to use

From the repo root:

```bash
./sim.sh --self-test
./sim.sh --seconds 10
./sim.sh --counterfactuals
./sim.sh stop
```

`--self-test` holds the standing pose for one second and should finish `within_limits`. `./sim.sh` with no flag loads the one archive in `containers/upload/` and starts standing. `./sim.sh stop` removes the sim containers.

`--counterfactuals` is the same six starts the offline screening used: standing, floor sitting, supine, prone, and both sides. Each pose starts a new container, so the submission's own state is reset. Floor sitting folds the legs. The four lying poses keep the stand joint angles and turn the pelvis; `rt/lowstate` carries that orientation on the IMU. Standing is the only pose that can fail for falling. A floor pose that stays down is not a fall, and the run does not require the robot to get up. Joint checks still apply, including the first command having to match the measured joints.

Each pose writes `containers/sim/output/poses/<pose>/report.json` and a trace. The battery writes `containers/sim/output/poses/summary.json`. Replay one pose with `./sim.sh replay containers/sim/output/poses/supine/trace.npz`. `--self-test --counterfactuals` holds the measured joints of every pose for one second. A floor pose can still fail a joint check while the body settles. That failure is not a fall.

Other flags:

| Flag | Effect |
| --- | --- |
| `--seconds 10` | Length of the run after the first accepted command. |
| `--image name` | Use this image instead of the archive in `containers/upload`. |
| `--pose name` | Start pose. Default `standing`. Also `seated`, `supine`, `prone`, `left_side`, `right_side`. |
| `--counterfactuals` | Run all six poses, each in a fresh container. Default length is 5 s. |
| `--view` | When a single run finishes, serve the replay page. |
| `replay` | Serve the last recording. Does not start a new run. |

`./sim.sh` asks for `sudo` so it can enter the isolated network. `replay` does not.

The run writes:

| File | What it is |
| --- | --- |
| `containers/sim/output/report.json` | Status, reason, command count, and simulated time. |
| `containers/sim/output/trace.npz` | Pelvis and joint poses for the replay. |
| `containers/sim/output/bridge.log` | ROS bridge log, if the run fails early. |

`status` in the report:

| Status | Meaning |
| --- | --- |
| `within_limits` | At least one command was accepted and none failed. |
| `violation` | `reason` is the check that failed. `detail` names the joint when there is one. |
| `no_command` | Nothing was accepted before the startup wait ended. |

The same checks, without Docker, are `containers/sim/test_runner.py`:

```bash
.venv-policy-safety/bin/python -m unittest containers.sim.test_runner
```

## How to visualize

After a run:

```bash
./sim.sh replay
```

Open `http://127.0.0.1:8765`. The page is a flat floor and the G1, playing `containers/sim/output/trace.npz`.

| Control | Action |
| --- | --- |
| Left drag | Orbit |
| Right drag | Pan |
| Scroll | Zoom |
| Space | Pause |
| Left / right arrow | One frame back or forward while paused |

Ctrl+C stops the page. `./sim.sh --view` serves that page when the run finishes, instead of requiring a second command.

`./sim.sh replay path/to/trace.npz` plays a different recording. The page is rebuilt into `containers/sim/output/view/`.
