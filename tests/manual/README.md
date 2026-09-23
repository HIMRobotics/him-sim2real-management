# Slow arm raise with diagnostic output

## Recorded robot configuration

- G1 EDU, 29 body joints, seven joints per arm, no dexterous hand attachments.
- Live SDK `rt/lowstate.mode_machine` was **5** on September 23, 2026.
- Unitree identifies type 5 as `g1_29dof_rev_1_0`, distinct from locked-waist type 6.
- Operator confirms both waist fixing pieces are absent. The expected waist
  configuration is unlocked yaw/roll/pitch, at SDK indices 12/13/14.
- Operator reports a 1,200-pound rated gantry/hoist with suitable harness, feet
  bearing weight and clearance for the arms.

[Official model mapping](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_description/README.md).
The existing 29-slot mapping already matches this configuration. Identification
was read-only and does not validate calibration, gains or physical damping.
The test continues to request diagnostic whole-body damping on abort. No physical
motor output is enabled by recording these settings.

## Run the diagnostic test


`arm_raise.py` generates one left shoulder-roll excursion of 0.15 rad (about
8.6 degrees) over 10 seconds. It captures all 29 measured positions when arming
finishes, then changes only the left shoulder-roll target. Quintic interpolation
starts and ends with zero velocity and acceleration. Peak target speed is about
0.0281 rad/s. Other target positions remain at their captured values, not zero.
This relative excursion does not prove physical clearance from an arbitrary pose.

`arm_raise_node.py` publishes these proposals to precheck. It subscribes to
checker-assigned observations and abort, and exposes `/safety/arm_raise/start`.
An explicit start call checks controller readiness, then arms the checker and gate asynchronously for the same
session. Service discovery and fresh released-Start feedback are required first.
This is a one-shot dry-run helper, not the future distributed reset coordinator.
There is no retry or automatic restart. A successful start response means arming
was requested, not that both nodes are already armed.

Start the complete graph from one terminal:

```bash
cd /home/unitree/him-sim2real-management
./tests/manual/run_arm_raise_dry_run.sh
```

The script builds the packages, sources ROS Foxy and the overlay, selects the
installed SDK library, and launches all five processes on local ROS domain 97.
All logs appear in this terminal. No automatic arm or restart occurs. Ctrl+C
stops the group. If any process exits, launch shuts down the remaining group.
This shutdown policy is for the diagnostic graph, not a hardware stop mechanism.

Wait for `Ready for explicit start request`. Then, from a second terminal, run:

```bash
cd /home/unitree/him-sim2real-management
./tests/manual/run_arm_raise_dry_run.sh --start
```

The start helper configures its own environment. No five-terminal setup or manual
service sequencing is required. The launch terminal prints when the raise starts
and when an abort latches. The controller readiness service verifies fresh,
released-Start input before the producer requests checker/gate arming.

The controller detector allows five seconds for its first sample in this launch,
then enforces the normal 200 ms deadline. It cannot report ready before receiving
feedback. An expired startup deadline stays latched even if telemetry later arrives.
Service presence alone does not establish controller readiness. Distributed run
identity, reset coordination and publisher isolation are still pending.

For a different robot interface:

```bash
./tests/manual/run_arm_raise_dry_run.sh sdk_interface:=eth0
```

With an existing ROS/SDK environment, the underlying command is
`ros2 launch tests/manual/arm_raise_dry_run.launch.py`.

Double-press Start during the ten-second raise. Expect `damping: true`, with zero
position stiffness, target velocity and feedforward torque across **all 29 slots**,
including legs and waist. Positive `kd` represents velocity damping, not posture
holding. Zero `q` in this output has no position effect because `kp` is zero.
The producer stops proposals and cannot be started again in that process.
Stock firmware can independently react to Start even though this software sends
no motor commands. The operator's supported test arrangement still applies.

Completion also aborts, so missing the gesture does not leave an indefinite hold.
Stale observations, arming failures and publishing gaps of 20 ms or more abort.
The gap limit avoids a trajectory catch-up step and may trip under CPU load.
The independent gate retains its 200 ms watchdog. These are software deadlines,
not guarantees of hardware response time.

For a slower or smaller dry-run, set `raise_duration_secs` and
`raise_angle_rad` as ROS parameters on the producer. `ArmRaiseSettings` contains
the placeholder test gains and publishing gap limit. Whole-body damping currently
uses `Config.damping` (1.0), also a placeholder. These settings are not commissioned
hardware gains or motion limits. Do not connect this producer to a real motor
adapter without hardware validation. No physical damping can be assessed here.

To repeat after abort, stop the entire isolated test graph and start it again
inhibited, then explicitly request a new run. Do not restart only one node to
clear an abort. Retained abort coordination and run identity remain pending.

Automated ROS testing covers service arming, proposed-command delivery, simulated
Start press-release-press, all-joint diagnostic damping and refusal to restart.
A manual controller test with this producer remains to be performed by the operator.

## Checked trajectory rates

The default 0.15 rad excursion over 10 seconds has peak target velocity
0.028125 rad/s (1.61 degrees/s) and peak target acceleration approximately
0.0086603 rad/s² (0.496 degrees/s²). A nominal 5 ms step changes the target
by at most 0.000140625 rad (0.0081 degrees). Position, velocity and acceleration
are continuous at the trajectory endpoints. Quintic jerk is finite but is not
continuous when entering or leaving a stationary hold.

Settings reject excursions above 0.15 rad and durations below 10 seconds, so a
parameter override cannot make this test faster or larger. These restrictions
bound this diagnostic trajectory, not the physical robot's safety envelope.
Automated checks cover acceleration, command increments and parameter rejection.
Actual motion still depends on control handover, measured initial velocity,
tracking, gains and load. Whole-body abort damping is a separate transition and
is not governed by the arm-raise curve.

## Operator controller test — September 23, 2026

The operator supplied logs showing the trajectory starting, followed about
8.94 seconds later by `controller_double_start`. The checker, producer and motor
gate latched abort. The gate's abort log appeared approximately 18.6 ms after
the controller detector's log. This is a software log interval for one run,
not a hardware stopping-time measurement. No physical motion or damping occurred.
