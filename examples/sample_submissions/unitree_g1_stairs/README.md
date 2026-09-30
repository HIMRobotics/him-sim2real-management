# Parkour v4 — Real Physics Stairs Jump — Unitree G1

**Challenge:** [HIM Arena Open Build](https://arena.himrobotics.com/challenges/open-build) (`open-build-challenge`)  
**Controller:** **pretrained + scripted** — third-party G1 walker ONNX (luckyrobots) + scripted velocity-command phase machine — **not trained by us for this course**, **not hardware-ready**  
**Base mode:** **free-base** (kinematic root assist **REMOVED**; gravity / inertia / MuJoCo contacts only)  
**Simulator:** MuJoCo · **Robot:** Unitree G1 29-DoF (`google-deepmind/mujoco_menagerie`)

## What shipped (brutally honest)

v3 floated the pelvis through the air by writing free-joint `qpos` every substep (`_apply_kinematic_root` / `assist_target`). Derek correctly rejected that as fake physics.

**v4 deletes that path entirely.** After reset, the environment never writes `data.qpos[0:7]`. Root motion comes only from physics. If supporting contact is lost, the robot falls.

**Stunt attempted:** free-base handstand stair ascent was **not** achieved honestly. Shipped instead:

**Standing curb-stair climb (visible knee bend, real foot contacts on `stair_1..3` and `top_landing`) → upright on top → standing walk/hop across a 0.15 m aerial gap → finish-pad foot landing.**

Stairs are intentionally low (~3 cm risers). Higher free-base handstand / tall-step parkour remains future work.

## Quick start

```bash
./run.sh
```

Creates/uses `.venv`, installs pinned deps (including `onnxruntime`), runs policy-contract + causality tests, writes `result.mp4` (+ `artifacts/midframe.png`).

## Policy contract

`policy.py`:

- `ScriptedParkourPolicy.reset()` / `.act(observation) -> action` — ONNX walker residual + scripted `cmd=(vx,vy,yaw)` phase schedule
- `NeutralPolicy` holds home pose (must fail)
- Actions are clipped to actuator `ctrlrange` (joint position targets)
- Label: **pretrained+scripted** · base: **free-base** · **no `assist_target`**

## Course layout

| Geom | Role | Top z |
|------|------|-------|
| `start_pad` | approach | 0.03 |
| `stair_1` … `stair_3` | foot tread contacts (~3 cm risers) | 0.03 … 0.09 |
| `top_landing` | upright latch | 0.10 |
| gap (0.15 m) | aerial clearance | — |
| `finish_pad` | land + hold | 0.10 |

## Success metric

All required (`parkour_env.py`):

1. **Foot contacts** on ≥ **2 distinct** stair treads
2. **Stood upright on top landing** (zone + orientation + foot contact, latched)
3. **Aerial gap**: pelvis x in gap with no foot contact on stair/top/finish
4. **Finish**: pelvis in finish zone, upright, foot contact on `finish_pad`
5. **No collapse**: pelvis `z` never below `0.28`

## Causality

Same seed (`0`): pretrained+scripted succeeds with stair/top/finish foot contacts + aerial gap; neutral fails. Logged: call counts, action bounds, contacts, knee joint range (bending), `root_qpos_writes_after_reset==0`.

## Kinematic assist removal

- Deleted `ParkourEnv._apply_kinematic_root` and all `assist_target` usage
- `step()` only sets `data.ctrl` then `mj_step` — never assigns free-joint `qpos`
- Contract test asserts assist helpers are gone from source

## Third-party

- Unitree G1 from [mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie) — see `THIRD_PARTY_NOTICES.md`
- Walker ONNX from [luckyrobots/g1-manipulation-challenge](https://github.com/luckyrobots/g1-manipulation-challenge) (`policies/walker.onnx`) — locomotion prior, not trained on this parkour course

## Limitations

- Free-base handstand / flip **not** shipped (would require faking or much stronger control)
- Stairs are low curbs; not a tall parkour set
- Gap crossing is a brisk walk/hop, not a dramatic standing long-jump
- Pretrained walker is third-party; phase schedule is open-loop thresholds on pelvis state
- Simulation only — not hardware-validated
- HUD / slow-mo is editorial only
