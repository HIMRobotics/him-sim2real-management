# Manual G1 safety tests

The scripts build and launch five processes: telemetry, command checker, motor
gate, controller-stop monitor and test policy. ROS uses localhost domain 97.
The Unitree SDK uses domain 0 and interface `eth0` by default.

**Current stop:** after arm ownership is taken, double Start, a fault, or
completion holds the measured arm pose in locked standing. Stock whole-body
damping (`SetFsmId(1)`) runs only if that hold cannot continue or the process
is exiting. A stop before any ownership does not command the motors.

## Full arm sequence

With the supported gantry setup and robot in locked standing/ready mode:

```bash
./tests/manual/run_unitree_arm_test.sh --hardware
```

Once ready, request one run from another terminal:

```bash
./tests/manual/run_unitree_arm_test.sh --start
```

Omit `--hardware` for diagnostic motor output. Do not run another test graph or
the original SDK motor publisher concurrently.

The policy holds captured arm/waist positions until the adapter confirms a
full-weight SDK write for its session. The profile's ownership ramp lasts
3 seconds from the first SDK write. Then the 18-second trajectory starts at zero:

- 0–3 seconds: targets move toward zero.
- 3–9 seconds: both arms raise using the SDK example's targets.
- 9–18 seconds: targets return toward zero.
- Completion: latch stop and hold the measured arm pose.

The trajectory retains the SDK example's feedback-based position formulas.
This is not the small, extra-slow elbow test. The stock controller owns the legs
during arm control. A full-weight write is not proof of physical takeover.

Double press Start within 0.5 seconds for the hard stop: stock whole-body damping,
which can fold the robot. Double press F3, the button beside Start, for the soft
stop: the legs enter walk mode at zero speed, and the arms freeze at the pose
they had at the press. The balancer may take a step. The arms stay on this
process, at full ownership weight. Tracking, freshness and command-gap
checks remain active during handover and movement. Other faults hold the measured
arm pose. The graph stays stopped until restart.

## Small elbow test

```bash
./tests/manual/run_unitree_arm_test.sh --small-step --hardware
# Once ready, in a second terminal:
./tests/manual/run_unitree_arm_test.sh --start
```

After the same captured-pose handover, only the left elbow target decreases by
0.05 rad (2.9 degrees) over 10 seconds. A quintic curve gives zero endpoint
velocity and acceleration, with peak requested speed 0.009375 rad/s.

The final target is held for 2 seconds. The measured elbow must finish within
0.02 rad of that target. The result is `small_elbow_complete` or
`small_elbow_did_not_follow`. Both results hold the measured arm pose.
The operator reported this test worked.

## Readiness and configuration

```bash
./tests/manual/run_unitree_arm_test.sh --check-ready
```

This queries stock mode without arming or moving. A fresh launcher is required
after a latched failure. Startup requires FSM 4, fresh released-Start telemetry,
machine type 5 and near-stationary joints. Readiness has two SDK attempts of
2 seconds each within a 5-second producer deadline. A startup rejection before
arming sends no damping or arm-release commands. Explicit double Start remains
active. `--start` repeats readiness before granting control.

The robot is a 29-DOF G1 EDU with seven joints per arm, no dexterous hands and
no waist fixing pieces. Locked standing refers to the operating mode, not a
mechanically locked waist.

Settings are in [g1_type5.template.json](../../robot_safety/profiles/g1_type5.template.json).
Use `hardware_profile_path:=/absolute/path/to/settings.json` to select another
reviewed profile. Interface/domain overrides are `sdk_interface:=eth0` and
`sdk_domain:=0`.

Current local settings include kp=60, kd=1.5, a 100 ms output-gap limit, a
0.03 rad initial position tolerance and a 0.05 rad/s startup-speed tolerance.
Tracking allowance is 0.30 rad for shoulders/elbows and 0.05 rad for wrists/waist.
See [profile documentation](../../robot_safety/profiles/README.md) for all limits
and their sources. Profile kd does not configure stock emergency-stop damping.

## Joint extrema

```bash
./tests/manual/run_unitree_arm_test.sh --joint-deltas
```

The topic `/safety/joint_diagnostics` publishes signed min/max, maximum absolute
value and sample count for each controlled joint:

- `tracking_error_rad`: proposed q minus measured q.
- `target_step_rad`: proposed q minus the previous successful SDK-write q.
- `measured_velocity_rad_secs`: measured SDK dq at the output check.

All 17 joints are sampled before validation, including a rejected frame.
These are checked proposals, not proof of execution. Samples include handover
and movement, stop when output stops and reset on process restart. Nonfinite
values are excluded from extrema but still fail validation.

The launcher prints the largest tracking-error range once a second and all
joint ranges on stop. Final lines are retained in the ROS launch log.
No limits are automatically adjusted from observed extrema.

## Original dry-run test

```bash
./tests/manual/run_arm_raise_dry_run.sh
# Once ready, in another terminal:
./tests/manual/run_arm_raise_dry_run.sh --start
```

This uses real telemetry, the checker, controller monitoring and a diagnostic
motor gate. It proposes a 0.15 rad shoulder raise over 10 seconds without SDK
motor writes. It does not validate physical tracking or stopping.

Automated ROS tests use localhost domain 191 and reject manual domain 97.
