# G1 dance (BeyondMimic tracking policy)

A BeyondMimic motion-tracking dance policy for the 29-DOF G1, packaged as a
container submission. GR00T Balance holds the robot during startup, then hands
over to the dance, which plays a 46 s clip (2304 frames at 50 Hz) and holds the
last frame.

## Contents

| Path | What it is |
|---|---|
| `st_jzfwe_groot_robust/` | Current policy, trained with task `Tracking-Flat-G1-Wo-State-Estimation-Robust-v0`: checkpoint `model_19999.pt`, `exported/policy.onnx`, and the Isaac Lab `params/` it was trained with. |
| `submission/` | The container: `policy.py` (observation, MLP, and clip clock), `node.py` (ROS 2 node), `Dockerfile`, `build.sh`, `joint_profile.json`, and `assets/` exported from the ONNX. |
| `tools/` | Offline MuJoCo checks without Docker or ROS. |
| `legacy/` | The previous policy (`dance.pt`) and its Isaac rollout export. It falls about 6 s in once joint friction is modelled, which is why the robust policy was trained. |

The built image, `submission/submission.tar` (about 730 MB), is not committed.
`build.sh` recreates it.

## Policy contract

- **Inputs:** 154 observations, in Isaac Lab joint order:
  - reference joint positions and velocities (29 + 29)
  - first two columns of `R_torso^T R_ref` (6)
  - pelvis gyro (3)
  - `q - default` (29)
  - `dq` (29)
  - last action (29)
- **No normalizer:** the ONNX actor has no observation normalizer.
- **Actions:** `target = default + action * action_scale`. The gains are
  `kp = armature * (2π·10)²` and `kd = 4 * armature * 2π·10`. The scale is
  `0.25 * effort / kp`.
- **Heading alignment:** the reference heading is aligned to the robot's torso
  yaw at the first `control_ready` state.
- **Clip clock:** the clip frame follows the state clock (`sim_time` in
  simulation, `received_at_secs` on the robot), so a missed control tick does
  not slow the clip.
- **Command timing:** `node.py` answers each state as it arrives. The command
  then lands on the next 20 ms tick, as in training.
- **Sim robot model:** `packages/policy_safety/src/policy_safety/robots/g1/g1_robot.xml`
  uses the same collision geometry as training (unitree_rl_mjlab's 7 capsules
  per sole) and the same armature and torque limits.

## Re-export assets from a new checkpoint

Export the ONNX in Isaac Lab, then run this from `dance/submission`:

```bash
../../.venv-policy-safety/bin/python export_policy.py ../st_jzfwe_groot_robust/exported/policy.onnx
```

It checks the observation names, joint order, gains, and action scales. It then
writes `assets/actor.npz` and `assets/motion.npz`, both in Unitree motor order.

## Build and run in the MuJoCo sim

```bash
cd dance/submission
sudo -E ./build.sh dance:local
cd ../..
G1_JOINT_PROFILE=dance/submission/joint_profile.json \
  ./sim.sh --image dance:local --seconds 47 --no-safety-checks --view \
  --output containers/sim/output/dance
```

To confirm the timing is right, check `containers/sim/output/dance/report.json`.
`commands` should be close to 50 per second of run time.

## Offline checks (no Docker, ROS, or sudo)

```bash
.venv-policy-safety/bin/python dance/tools/groot_pipeline.py      # lockstep: GR00T startup, handover at 5 s, full clip
.venv-policy-safety/bin/python -c "import sys; sys.path.insert(0, 'dance/tools'); \
  import bridge_emulation as b; print(b.run(node_period=None))"   # bridge timing, event-driven replies
.venv-policy-safety/bin/python dance/tools/trace_drift.py containers/sim/output/dance/trace.npz
```

On the current model, `groot_pipeline.py` finishes the full clip without a fall
at joint friction 0 and 0.3 Nm. The median absolute joint error is about
0.08 rad.

## Known issues

- **Ankle torque vs the safety profile:** ankle pitch peaks at about 45 Nm
  during the dance. The motor limit is 50 Nm, but HIM's safety profile caps
  ankles and waist roll/pitch at 35 Nm. Runs with safety checks on will stop,
  so use `--no-safety-checks` until that limit is decided.
  `dance/tools/torque_peaks.py` prints each joint's peak against its limit.
  With `--safety-checks` and `HIM_HOLOSOMA_ONNX` set to Holosoma's
  `fastsac_g1_29dof.onnx`, it runs the hardware adapter on the sim clock. The
  adapter clips the left ankle target, then stops at 10.3 s (5.3 s into the
  clip) on `left_ankle_pitch: joint_torque_limit`, and Holosoma recovers.
  With ankles hard-capped at 35 or 31.5 Nm and no stop, the clip still finishes
  without a fall.
- **Body spin:** at about 17 s (12 s into the clip) the dance yaws at about
  10 rad/s, over the 6 rad/s body-spin limit. The hardware adapter enforces it.
  Clip mode (`G1_SIM_LIMIT_MODE=clip` in the sim, `G1_LIMIT_MODE=clip` on the
  robot) runs the same adapter but turns the torque and spin stops into
  warnings. In Docker with `--safety-checks --lockstep` the full clip then plays
  without a fall: left ankle up to 48 Nm PD demand and 44.5 Nm measured, right
  ankle 39 Nm, yaw up to 9.8 rad/s. On Docker's wall clock, host stalls delay or
  drop commands and the dance falls at varying times.
- **Physics timestep:** HIM's sim uses a 2 ms timestep. The training brief
  specifies 5 ms, and this has not been changed.
