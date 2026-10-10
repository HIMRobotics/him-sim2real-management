# G1 type-5 safety profile

The supplied profiles use Unitree revision-1.0 joint position, velocity and
model effort limits. They screen proposals for every submission, not just the
walker. Model ceilings are not continuous-duty ratings or hardware validation.

Source: [Unitree revision-1.0 model](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_description/g1_29dof_rev_1_0.urdf).
`g1_type5.official_limits.json` records the extracted values and source hash.

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

## Command and response semantics

All positions are absolute joint angles in radians. At 50 Hz, an interval lasts
20 ms. The policy's own decoder converts any action offsets into absolute angles.

- `q_proposed`: current proposal, not yet accepted or sent.
- `q_commanded`: proposal clipped to the configured joint-position bounds.
- `q_measured`: latest measured joint angle.
- `q_applied`: accepted command retained after simulation application or a
  successful full-weight hardware write. SDK write success does not prove motion.
- `command_gap = q_commanded - q_measured`: diagnostic only, including at startup.
- `response_error = q_measured - q_applied`: measured after execution, diagnostic only.

`max_velocity_rad_secs` caps the desired velocity field. A proposal past that
cap is clipped and recorded as a warning, on the same path as position clipping.
Measured velocity still rejects immediately, with no 50 ms persistence delay.
`max_torque_nm` bounds feedforward torque, total
estimated PD plus feedforward demand, and available measured torque. Simulator
checks use its actual gains. Checked submissions and the hardware arm adapter use profile gains, which
are the gains it actually sends, and forwards validated feedforward torque.

Position clipping records a warning with the original and clipped angles, bounds,
count and largest correction per joint. Clipping alone does not latch a stop.
Whole-body output additionally projects position targets into a 90% torque
budget using measured state and effective gains. The hard torque ceiling stays
unchanged. `torque_target_clipping` reports these corrections. Infeasible targets
and measured or last-written-command effort above the hard ceiling still fault.
Arm-only helpers retain rejection without this projection. A small response error does not establish safety. Response diagnostics
never latch a stop or silently increase any limit.

## Thrashing and flailing detection

The shared thrash detector evaluates measured joint velocities over time using
filtered, weighted oscillation energy and a persistence window. Hardware
submission defaults to enforcement. Simulation supports monitor, enforce and
off modes. See [thrash detection](../README.md#thrash-detection-in-the-existing-checker)
for configuration and telemetry. Command gaps and response errors remain
diagnostic only, while physical speed and torque limits remain active.

## Runtime and startup checks

The shared body-spin ceiling is **6 rad/s on each axis**. Upright walking uses
**1 rad tilt** from vertical. Non-upright simulation scenarios omit that tilt
check, while speed and torque checks remain. These values are configurable SDK
helper defaults, not published universal operating recommendations.
[Unitree helper source](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/g1/common/terminations.hpp).

Hardware also checks casing temperature **85 C**, winding temperature **120 C**,
and battery charge **at least 10%**. Temperatures come from `LowState` and battery
charge from `rt/lf/bmsstate`. Missing temperature or battery data prevents arming.
The lower-frequency BMS stream has a local **5 s** freshness deadline. Motor
feedback retains its **200 ms** deadline and command publication its **100 ms**
deadline. Invalid numeric data is rejected. Thermal and measured-torque checks
run on fresh hardware feedback. Simulation reports battery/temperature checks
as unavailable rather than inferring them from motion.

Initial desired command speed remains **0.05 rad/s**. The legacy arm-only path
also uses that measured startup-speed tolerance and a **3 s** ownership ramp.
Whole-body arming uses physical per-joint speed ceilings, without the separate
near-stationary gate. Real deployments default to enforcing the existing weighted
RMS thrashing detector. Hardware startup still requires the verified ready FSM,
machine type, fresh telemetry and health checks. Whole-body entry into Balance
uses the separate 0.5 s target blend described in
[HARDWARE_ADAPTER.md](../HARDWARE_ADAPTER.md).

## Hardware scope and diagnostics

The arm adapter writes only waist/arm joints. It validates all 29 proposed
commands but cannot command the legs through `arm_sdk`. Response diagnostics use
only waist/arm commands from successful full-weight writes and newer feedback.
During partial ownership the effective controller command is unknown, so no
response sample is recorded. Hardware feedback receipt time establishes ordering,
not an exact 20 ms actuator response interval.

`/safety/joint_diagnostics` reports signed `command_gap_rad`, `response_error_rad`
and `measured_velocity_rad_secs` extrema. Rejected proposals contribute command
gaps but never become applied commands. The existing F3 balance/stop controller
is a separate recovery path, not a screened submission.
