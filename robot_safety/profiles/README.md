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
- `q_measured`: latest measured joint angle.
- `q_applied`: accepted command retained after simulation application or a
  successful full-weight hardware write. SDK write success does not prove motion.
- `command_gap = q_proposed - q_measured`: checked before issuing a command.
- `response_error = q_measured - q_applied`: measured after execution, diagnostic only.

`max_command_gap_rad` is **0.5 rad** for every joint. It is a local allowance,
not a Unitree specification, and is independent of elapsed time. There is no
consecutive-command delta or desired-velocity-change rejection. Removed profile
fields are `max_position_target_rate_rad_secs`, `max_acceleration_rad_secs2` and
`max_tracking_error_rad`. Custom profiles must remove these and explicitly supply
`max_command_gap_rad`. They are not silently reinterpreted.

`max_velocity_rad_secs` checks both the desired velocity field and measured
velocity using the model values above, immediately on each sample. No 50 ms
persistence delay remains. `max_torque_nm` bounds feedforward torque, total
estimated PD plus feedforward demand, and available measured torque. Simulator
checks use its actual gains. The hardware arm adapter uses profile gains, which
are the gains it actually sends, and forwards validated feedforward torque.

A command within the 0.5 rad allowance can still violate position or torque
bounds. A small response error does not establish safety. Response diagnostics
never latch a stop or silently increase any limit.

## Runtime and startup checks

The shared body-spin ceiling is **6 rad/s on each axis**. Upright walking uses
**1 rad tilt** from vertical. Non-upright simulation scenarios omit that tilt
check, while speed and torque checks remain. These values are configurable SDK
helper defaults, not published universal operating recommendations.
[Unitree helper source](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/g1/common/terminations.hpp).

Hardware also checks casing temperature **85 C**, winding temperature **120 C**,
and battery charge **at least 20%**. Temperatures come from `LowState` and battery
charge from `rt/lf/bmsstate`. Missing temperature or battery data prevents arming.
The lower-frequency BMS stream has a local **5 s** freshness deadline. Motor
feedback retains its **200 ms** deadline and command publication its **100 ms**
deadline. Invalid numeric data is rejected. Thermal and measured-torque checks
run on fresh hardware feedback. Simulation reports battery/temperature checks
as unavailable rather than inferring them from motion.

Initial alignment remains **0.03 rad**, initial speed **0.05 rad/s**, and the
hardware ownership ramp **3 s**. The simulator validates its initial hold before
policy motion. Hardware startup still requires the verified ready FSM and
machine type. No launcher enables hardware output automatically.

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

No robot service was restarted when updating these profiles. Applying them to a
physical robot still requires the existing explicit hardware launch procedure.
