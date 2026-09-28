# G1 type-5 profile using Unitree model limits

`g1_type5.template.json` uses the official position, velocity and effort limits for all
29 joints in Unitree's `g1_29dof_rev_1_0.urdf`. The remaining local settings are distinguished below. No launcher
selects hardware output automatically.

Source: [Unitree revision-1.0 model](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_description/g1_29dof_rev_1_0.urdf),
retrieved September 24, 2026. `g1_type5.official_limits.json` records the extracted
values, source URL and SHA-256 of the downloaded model. Machine type 5 and ready
FSM 4 were read from this robot, with the operator confirming locked standing.

## Official model limits

Both sides have the same limits unless shown separately. Position is in radians,
velocity in radians/second and effort in Nm.

| Joint | Minimum position | Maximum position | Velocity | Effort |
| --- | ---: | ---: | ---: | ---: |
| Hip pitch | -2.5307 | 2.8798 | 32 | 88 |
| Left hip roll | -0.5236 | 2.9671 | 20 | 139 |
| Right hip roll | -2.9671 | 0.5236 | 20 | 139 |
| Hip yaw | -2.7576 | 2.7576 | 32 | 88 |
| Knee | -0.087267 | 2.8798 | 20 | 139 |
| Ankle pitch | -0.87267 | 0.5236 | 30 | 35 |
| Ankle roll | -0.2618 | 0.2618 | 30 | 35 |
| Waist yaw | -2.618 | 2.618 | 32 | 88 |
| Waist roll/pitch | -0.52 | 0.52 | 30 | 35 |
| Shoulder pitch | -3.0892 | 2.6704 | 37 | 25 |
| Left shoulder roll | -1.5882 | 2.2515 | 37 | 25 |
| Right shoulder roll | -2.2515 | 1.5882 | 37 | 25 |
| Shoulder yaw | -2.618 | 2.618 | 37 | 25 |
| Elbow | -1.0472 | 2.0944 | 37 | 25 |
| Wrist roll | -1.972222054 | 1.972222054 | 37 | 25 |
| Wrist pitch/yaw | -1.614429558 | 1.614429558 | 22 | 5 |

The adapter enforces the position bounds and uses the velocity value both to
check target dq and limit position-target changes between SDK writes. This
position-change check is our software rule, not a guarantee of actual joint speed.
The previous 0.1/0.25/0.5 rad/s test caps are removed. Model velocities are high:
37 rad/s is about 2,120 degrees/s. These are model limits, not requested speeds
or recommendations for personnel-adjacent testing. The policy still sets the
motion sequence. The profile alone no longer enforces a slow raise.

`max_torque_nm` enforces the model effort bound against estimated PD demand
using the gains of the active controller. The simulator also checks applied
torque. Hardware feedback does not currently expose measured torque to this
checker, so this is a command estimate, not a measured-torque monitor or a
firmware guarantee. Feedforward torque remains zero.

Active arm-policy output and the offline evaluator use the same `joint_checks.py`
functions. The evaluator checks all 29 joints. F3 soft-reset GR00T balancing is
exempt from these submission movement and torque limits. Feedback freshness and
output-error handling still apply. The hardware arm gate still rejects whole-body
submissions, so the configured leg limits currently run in offline evaluation,
not a live submitted leg-policy path.
Older custom profiles must add all 12 leg entries and every joint's torque bound.

## Gains and local settings

kp=60 Nm/rad and kd=1.5 Nm·s/rad come from the operator-tested SDK arm example,
not from the URDF. They apply to every waist/arm joint. Stock whole-body damping
is requested separately and does not use these gains.

| Setting | Value | Source/meaning |
| --- | --- | --- |
| Expected machine type | 5 | Observed SDK hardware configuration |
| Allowed ready FSM | 4 | Observed and operator-confirmed locked standing |
| Ownership handover | 3 seconds | Local smooth ownership ramp |
| Initial position mismatch | 0.03 rad / 1.72° | Local first-target check |
| Initial measured speed | 0.05 rad/s / 2.86°/s | Local near-stationary startup check |
| Maximum publishing gap | 0.10 seconds | Local output deadline |
| Command period | 0.02 seconds / 50 Hz | SDK example cadence |
| Feedback age | Less than 0.20 seconds | Local freshness deadline |

The handover begins with the first SDK write. The small elbow and full SDK arm
tests hold captured targets until a full-weight write is confirmed, then start
the trajectory clock. First output uses measured positions and zero ownership weight.

| Joint group | Change in target dq per second | Target-to-measured error |
| --- | ---: | ---: |
| Hip pitch/yaw | 1.0 rad/s² | 0.146667 rad (8.4°) |
| Hip roll | 1.0 rad/s² | 0.231667 rad (13.3°) |
| Knees | 1.0 rad/s² | 0.173750 rad (10.0°) |
| Ankles | 1.0 rad/s² | 0.218750 rad (12.5°) |
| Shoulders/elbows | 1.0 rad/s² | 0.30 rad (17.2°) |
| Wrists | 1.0 rad/s² | 0.05 rad (2.9°) |
| Waist | 0.5 rad/s² | 0.05 rad (2.9°) |

Leg tracking allowances are now derived from `0.25 * max_torque_nm / kp`,
rounded to six decimals, using this profile's Unitree model effort limits and
reference gains (hips 150, knees 200, ankles 40 Nm/rad). Both sides use the same
values. The standard, lenient and standalone profiles share these leg allowances.

The quarter-effort scaling comes from
[NVIDIA's G1 policy parameters](https://github.com/NVlabs/GR00T-WholeBodyControl/blob/b042411fae38ee4d1af9aac82a37a1f8d14d6dd0/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/policy_parameters.hpp).
Upstream uses it to scale actions around a default pose, **not** as a tracking
fault threshold. Applying it to target-minus-measured error is our explicit
engineering choice for a provisional screening default. It replaces the copied
arm tolerances without claiming a manufacturer-approved tracking envelope.
At the reference kp, the proportional correction at the threshold is about 25%
of model effort. This is not a bound on total PD torque or a continuous-duty rating.

The simulator retains policy gains, so the fixed radian thresholds are not
recomputed for each submission. Its separate full PD-demand check uses the actual
simulator gains and includes damping. No arm/waist values, command-change limits,
startup tolerances or soft-reset behavior change with this tracking update.
The leg commanded-velocity-change allowance remains a local 1 rad/s² setting.
With desired dq=0, that check does not bound acceleration implied by the position
trajectory. No hardware jerk limiter is implemented.

### Upstream safety checks reviewed September 28, 2026

- [Unitree's G1 runtime termination checks](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/g1/common/terminations.hpp)
  default to 10 rad/s measured joint speed, 6 rad/s body angular velocity and
  1 rad tilt. These are runtime helper defaults, not per-joint tracking tolerances.
  The 10 rad/s value is not currently implemented by this profile's command-speed
  field. That field checks desired dq and position-target changes instead.
- [NVIDIA's G1 joint safety monitor](https://github.com/NVlabs/GR00T-WholeBodyControl/blob/b042411fae38ee4d1af9aac82a37a1f8d14d6dd0/decoupled_wbc/control/envs/g1/utils/joint_safety.py)
  explicitly excludes legs. Its 6 rad/s arm limit cannot justify a leg setting.
- Neither checker supplies a leg tracking-error or commanded-acceleration bound.
  The Unitree URDF supplies position, velocity and effort bounds, not those two metrics.

The PD-demand check uses model effort bounds and actual controller gains.
The derived tracking defaults still require controller-specific validation.
Neither upstream repository establishes these values as validated fault thresholds.

The SDK example can still trigger tracking, timing or other checks. Replacing
position and velocity caps does not establish that it will complete. Whole-body
stop priority, control handover, tracking and calibration remain physically
unverified. No motor commands were sent when updating this profile.


## Tracking allowance and observed ranges

Shoulder/elbow tracking allowance was increased from 0.15 to 0.30 rad after the
full sequence exceeded 0.15 rad following handover. This is a local tuning value,
not an official Unitree bound or a physically validated safety envelope. Wrist
and waist tracking allowances remain 0.05 rad. Position, velocity, acceleration,
initial-pose and initial-speed checks, gains and stop deadlines are unchanged.
Allowing more tracking error also allows greater corrective PD torque.

The hardware gate publishes per-joint signed extrema on
`/safety/joint_diagnostics` once per second. Use these observations to review
the thresholds explicitly. The code does not automatically increase any limit.
Observed peaks from one run do not establish safe bounds.
