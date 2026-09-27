# G1 type-5 profile using Unitree model limits

`g1_type5.template.json` uses the official position and velocity limits for all
17 waist/arm joints in Unitree's `g1_29dof_rev_1_0.urdf`. The remaining local settings are distinguished below. No launcher
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

Effort is recorded for reference. The adapter does not enforce a total PD torque
limit or assert that firmware implements the model's effort bounds. Feedforward
torque remains zero. Joint bounds alone do not prevent collisions or joint damage.

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
| Shoulders/elbows | 1.0 rad/s² | 0.30 rad (17.2°) |
| Wrists | 1.0 rad/s² | 0.05 rad (2.9°) |
| Waist | 0.5 rad/s² | 0.05 rad (2.9°) |

These acceleration and tracking thresholds remain local proposals. The official
model does not specify replacements for them. The acceleration check applies to
commanded dq. Because the example sends dq=0, it does not bound acceleration
implied by the position trajectory. No hardware jerk limiter is implemented.

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
