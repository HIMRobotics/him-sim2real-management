# Containers

Everything that runs a policy container lives here: the simulator, a template
for new submissions, and a set of working example policies. To write your own
submission, follow [INSTRUCTIONS.md](INSTRUCTIONS.md). This page is a tour of
the commands, mostly using the examples.

| Path | What it is |
| --- | --- |
| [`template/`](template/README.md) | The starting point for a new submission. It holds the robot's starting pose until you replace its policy |
| [`examples/`](examples/README.md) | Finished reference policies with their weights baked in: the NVIDIA walker, GR00T balance, and Holosoma |
| [`sim/`](SIM_RUN.md) | The MuJoCo simulator. It runs any image that follows the instructions |
| `upload/` | Where `submission.tar` (and an optional `environment/` scene) goes for the simulator and the robot |

All commands run from the repository root. On a Mac or Windows machine use
`./desktop_sim.sh`; on the robot's Linux host use `./sim.sh`. The two take the
same flags, so every `./desktop_sim.sh` command below also works as `./sim.sh`.
Desktop runs need Docker Desktop to be running.

## Watch an example policy

The quickest way to see the simulator working is to run the NVIDIA walker and
watch it in the browser. The command builds the walker's image, opens a page,
and starts the run when you press Start.

```bash
./desktop_sim.sh --example walker --view
./desktop_sim.sh --example balance --view
./desktop_sim.sh --example holosoma --view
```

Every run starts in the same stance: the crouch GR00T balance settles into,
with the upper arms at the sides and the elbows bent 90 degrees so the
forearms point forward. For the first five seconds balance holds the robot, as
it does on the real robot before the operator presses left + B. It drives the
legs and waist and holds the arms in that pose. The policy takes over after
that and runs for `--seconds`. The viewer shows how far into the startup the
run is and when the policy takes over.

Each run writes `containers/sim/output/report.json` and a recording you can
watch again with `./desktop_sim.sh replay`. To keep several runs side by side,
give each one its own output directory and replay it by path:

```bash
./desktop_sim.sh --example walker --seconds 20 --output containers/sim/output/walker-on
./desktop_sim.sh replay containers/sim/output/walker-on/trace.npz
```

Adding `--local` runs an example directly in MuJoCo, without Docker or ROS.
This is faster to iterate on, but it skips the container path that a real
submission takes.

```bash
./desktop_sim.sh --example walker --local --seconds 10 --view
```

## Hardware safety in simulation

Checked submissions use `G1HardwareAdapter` through a MuJoCo transport in both
local and ROS/container runs. Startup balancing, measured readiness, the policy
handoff, physical limits, recovery and damping follow the hardware state machine.
The simulated operator presses left+B after five seconds once readiness is met.
After a submission fault, simulation keeps stepping the adapter's recovery or
damping output. The original policy remains stopped.

The adapter uses the existing Holosoma checkpoint. On this robot it resolves from
`/home/unitree/holosoma`. Elsewhere, set `HIM_HOLOSOMA_ONNX` to
`fastsac_g1_29dof.onnx`, or set `HIM_HOLOSOMA_DIR` to the Holosoma checkout.
Compose mounts that file read-only. Missing recovery weights prevent checked
output. CI downloads a pinned, checksum-verified checkpoint without adding a
second copy to this repository.

Run reproducible trials through the local sim and its normal trace/viewer:

```bash
./sim.sh --local --example holosoma --scenario recovery_tilt --tilt-degrees 20 --seconds 8 --output containers/sim/output/recovery
./sim.sh --local --example holosoma --scenario policy_tilt --tilt-degrees 35 --seconds 8 --output containers/sim/output/policy-tilt
./sim.sh --local --example holosoma --scenario gantry_tilt --tilt-degrees 35 --seconds 8 --output containers/sim/output/gantry-tilt
./sim.sh --local --example holosoma --scenario policy_crouch --seconds 8 --output containers/sim/output/crouch
./sim.sh replay containers/sim/output/recovery/trace.npz
MUJOCO_GL=egl .venv-policy-safety/bin/python containers/sim/render.py containers/sim/output/recovery/trace.npz containers/sim/output/recovery/video.mp4
```

`recovery_tilt` imposes a zero-velocity pose and injects a policy fault. A 20° tilt
alone does not cross the 42° stop threshold. The other trials test immediate
damping when orientation or pelvis-to-sole height fails. Scenario commands return
success only when the expected response occurs. The run still reports the
injected safety violation. Reports include recovery writes, damping time and
standing height, and replay banners distinguish recovery from damping.

Motor temperatures (25°C), battery charge (100%) and stock-controller RPCs are
synthetic. Joint/IMU feedback and applied motor output come from MuJoCo. Adapter
callbacks run on simulation time while submission deadlines retain their input
timestamps. Gantry mechanics and SDK/network latency are not modeled. Foot contact
is not an extra gate, matching the operator's responsibility on hardware.
Unchecked runs and the direct `--circle` path bypass the hardware adapter and
remain explicitly identified in the report.

## Turn the safety checks off

Safety checks are on by default, so a policy that breaks a limit is stopped
and the report says `violation`. Running the same example with
`--no-safety-checks` shows what the policy would have done without them. Such
a run reports `completed`, which is not a pass.

```bash
./desktop_sim.sh --example walker --seconds 20 --output containers/sim/output/walker-on
./desktop_sim.sh --example walker --seconds 20 --no-safety-checks --output containers/sim/output/walker-off
```

## Steer the walker

The walker follows a velocity schedule. The schedules in
[`examples/commands/walker/`](examples/commands/walker/) cover standing still,
walking forward, walking and then stopping, and walking in a circle. Passing
one with `--example walker` plays it through the walker's container.

```bash
./desktop_sim.sh --example walker --command containers/examples/commands/walker/forward_stop.yaml --seconds 7 --view
./desktop_sim.sh --example walker --command containers/examples/commands/walker/circle.yaml --view
```

`--circle` is a shortcut that drives the walker's network around one flat
circle with no container at all. It is useful for checking the viewer and the
physics, but it bypasses the submission checks.

```bash
./desktop_sim.sh --circle --view
```

## Stress test with the battery

The battery starts the robot from every pose and then pushes the pelvis with
10, 50, and 150 N. It runs on the host without Docker and writes
`containers/sim/output/battery/report.json`. To watch one case, pick a push and
add `--view`.

```bash
./desktop_sim.sh --battery
./desktop_sim.sh --battery --push heavy --view
./desktop_sim.sh --battery --circle           # the walker gaits instead of holding still
```

`--counterfactuals` is the container version of the same idea. It reruns the
policy from six starting poses and writes one report per pose.

```bash
./desktop_sim.sh --example walker --counterfactuals
```

## Try a different scene

Any run can load another MuJoCo scene with `--scene`. On the desktop the scene
must be inside this repository so the simulator container can see it.

```bash
./desktop_sim.sh --example walker --scene containers/upload/environment/scene.xml --view
```

[Submit a MuJoCo environment](SIM_RUN.md#submit-a-mujoco-environment) shows
how to build one from this repository's G1 model.

## Run an example the way a submission runs

The examples can also be exported as a normal `submission.tar`, which is a
good way to check the upload path end to end.

```bash
.venv-policy-safety/bin/python containers/examples/prepare.py walker
docker build --platform linux/arm64 -t g1-example-walker:local .cache/sim_examples/walker/container
docker save g1-example-walker:local -o containers/upload/submission.tar
./desktop_sim.sh --view
```

To run a built image without exporting it, pass it by name:

```bash
./desktop_sim.sh --image g1-example-walker:local --view
```

Your own submission runs the same way. Every command on this page works without
`--example` once your archive is in `containers/upload/`; the
[test section of INSTRUCTIONS.md](INSTRUCTIONS.md#test-in-simulation) lists the
ones that matter most.

## Run an example on the robot

On the robot, `./submit.sh --mode real` runs the same walker container through
the hardware safety gate. Start with the robot supported and the standing
schedule, then move on to its first steps. The
[operator guide](../robot_safety/HARDWARE_ADAPTER.md#operator-commands) covers
the handoff and how to stop.

```bash
./submit.sh --mode real --example walker --command containers/examples/commands/walker/stand.yaml
./submit.sh --mode real --example walker --command containers/examples/commands/walker/first_steps.yaml
```

## Housekeeping

```bash
./desktop_sim.sh --self-test   # check the simulator itself; needs no submission
./desktop_sim.sh stop          # remove any simulator containers left running
```

The `holosoma` example also exists but needs its weights from outside this
repository; set `HIM_HOLOSOMA_ONNX` to the ONNX file before using
`--example holosoma`. [SIM_RUN.md](SIM_RUN.md) is the full reference for every
flag, the report fields, and the viewer controls.
