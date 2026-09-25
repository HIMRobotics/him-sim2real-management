# G1 safety pipeline: robot side first, frontend later

Updated September 24, 2026. **Default launch remains dry-run. Arm SDK output and physical test policies are implemented. Stop behavior is not accepted for deployment.**

## Architecture and current checkpoint

```text
policy -> runner -> precheck -> safety checker -> postcheck -> motor gate
                                                              |
                                                    dry_run_output
                                                    Unitree arm SDK
                                                              |
                                                           actuators

controller stop -> abort -> checker and motor gate
```

The checker relays each accepted whole-joint frame once. An invalid frame latches
abort and blocks future relaying. The independent motor gate selects damping on
abort, command expiry or missing postcheck messages. Controller double press
broadcasts directly to both nodes. Run roles in separate processes.

ROS topics and typed messages are defined under `ros2/`. Commands use bounded
latest-value histories. Abort is retained and repeated every 100 ms. Receipt and
expiry timestamps currently require the same Linux host/boot and clock namespace.
ROS topic naming alone does not prevent unauthorized publishers or enforce latency.

## Stage 1 — Software checks and ROS dry-run graph

Completed:

- [x] Core operating states, command validation, session/sequence checks and stop latch.
- [x] Configurable 200 ms freshness checks and a 5 ms target arbitration cadence.
- [x] Start press-release-press detector with a configurable 500 ms window.
- [x] Physical button state is separate from the one-shot stop event.
- [x] Operator verified actual-controller input: `0400`, two approximately 160 ms double presses, and rejection of a 1.06-second interval. Physical damping was not tested.
- [x] Typed precheck/postcheck, feedback, observation, abort and diagnostic output messages.
- [x] ROS checker, motor gate and controller-stop adapters with initial explicit arm services.
- [x] Motor gate independently expires commands when the checker is silent.
- [x] Core, controller, configuration, gate and isolated ROS message-delivery tests.

The earlier `runtime.py` remains an offline reference with synthetic queue tests.
It is not used by the ROS nodes, whose callbacks and timers run in ROS executors.

Next review checkpoints, in order:

1. Implemented for review: a read-only SDK bridge to `/safety/robot_feedback`, preserving local receipt
   time, joint mapping and physical Start state. No motor publisher.
2. Implemented for review: a one-shot, slow left-arm-raise producer with explicit
   start and abort on completion. ROS tests cover simulated double Start through
   whole-body fake damping. Existing graph tests cover invalid commands and checker
   silence. Manual controller testing with the new producer is pending.
3. Add coordinated arm/reset and run identity. The core has local reset, but the
   current ROS nodes deliberately expose no independent reset service. Retained
   aborts and old sessions must not permit partial or automatic recovery.
4. Define policy end/disconnection notification and operator lease. Topic silence
   already causes a timeout, but immediate process-exit notification is not implemented.

Stage 1 completion: invalid input and controller abort produce fake damping,
checker silence triggers the independent gate, startup/reconnect cannot resume
motion, and no node can create Unitree motor output.

## Stage 2 — Managed services and first real runner

- Add systemd units and installation/check scripts. Bash performs setup only.
- Tie health notifications to actual callback/timer progress, with an initial
  1-second process watchdog separate from command deadlines. Restart inhibited.
- Integrate one real runner, initially RL Gym, replacing all direct motor writes
  with precheck publication, including startup and cleanup.
- Preserve the author's joint mapping, observations, gains and timing.
- Isolate policy workers from Unitree motor interfaces and restrict who can
  publish postcheck. Registration or lock files do not establish exclusivity.
- Test stalled inference, flooded inputs, process death, lost connections,
  logging failure and reboot. A service restart is recovery, not a physical stop.

## Stage 3 — Physical G1 acceptance, then frontend

Known hardware: operator reports **G1 EDU, 29 DOF, seven joints per arm**, with
no dexterous hand attachments. Read-only SDK telemetry on September 23, 2026
reported `mode_machine=5`, matching `g1_29dof_rev_1_0`. The operator confirms
both waist fixing pieces are absent. Together these identify the expected
unlocked three-joint waist configuration, not proof of calibration or motor health.
Operator reports a 1,200-pound
rated gantry/hoist with suitable harness, feet bearing weight and arm clearance.

Installed Python SDK: 1.0.1, commit `65691c8`, Cyclone DDS 0.10.2. PC4 manifest:
master service 1.0.0.2, patch 1.0.0.6, video hub 1.0.2.3. Live LowState version
was `[0, 0]`, so motion-controller and motor firmware remain unidentified.

The read-only bridge maps all 29 body slots, including waist yaw/roll/pitch at
indices 12/13/14. No waist joint is excluded for this identified configuration.
This mapping is not a commissioned command profile or proof of movable joints.

Before motor output, establish a validated per-joint profile, controller ownership,
mode-specific stop behavior and an appropriate supported test setup. The 29-joint
software default and uniform limits/gains do not identify or validate the hardware.

Physical transition:

1. Real telemetry with fake output and a small smooth arm trajectory.
2. Fault tests and verification of the independent stop path while supported.
3. Arm control handover holding fresh measured joint positions, without motion.
4. A small jerk-limited arm movement with validated position, speed, acceleration,
   gain and tracking limits. No timing-stall catch-up jump.
5. Expand range and policy complexity only after recorded acceptance.

An arm SDK adapter must preserve the selected stock-controller ownership model.
It is distinct from full-body low-level takeover. Normal controlled deceleration,
arm-control release and whole-body emergency damping are different responses.
Damping can cause a fall. Verify downstream command-loss behavior when the motor
gate or computer fails, and record unsupported failure boundaries.

Only after robot-side acceptance, add frontend stop/status APIs, explicit
coordinated reset/rearm, operator connection handling and asynchronous event
recording. Keep Stop recording separate. Show request, latch, publication and
physical feedback as distinct evidence. Never infer physical safety from an ACK.

## Validation records and references

See `robot_safety/README.md` for build/test commands, topic contracts and the
operator's controller-input test. Automated and physical tests remain distinct.

- [Unitree RL Gym runner](https://github.com/unitreerobotics/unitree_rl_gym/blob/main/deploy/deploy_real/deploy_real.py)
- [Unitree G1 arm example and joint mapping](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/g1/high_level/g1_arm7_sdk_dds_example.py)

## Arm-raise review checkpoint

The operator requests whole-body damping on abort, even during an arm-only test.
The assumed setup is standing on both feet and attached to the gantry. Losing
active posture support may load the harness. The implementation still publishes
only diagnostic output. A real adapter must establish whole-body stop authority
over the active stock controller before any supported physical arm-raise test.

The dry-run launch now manages all five processes from one command, combines
logs and shuts down the group on process exit. Initial controller discovery has
a five-second grace, followed by the normal 200 ms deadline. Starting remains
explicit and checks fresh released-Start input from the controller detector before
arming. This helper does not implement distributed reset or production supervision.

The arm-raise scenario and its launch tooling live under `tests/manual`, separate
from the reusable `robot_safety` package.

Hardware identification reference: [Unitree G1 machine-type mapping](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_description/README.md).
Type 5 is the regular revision-1.0 model, type 6 its locked-waist counterpart.
The SDK reading is an observed configuration, not a setting changed by this code.

## Current hardware implementation

- Type-5 G1 EDU profile with official model position/velocity limits, SDK example
  gains, confirmed ready FSM 4 and local tracking/startup tolerances.
- Arm SDK motor gate with ownership ramp, fresh feedback, command expiry and
  direct controller double-Start detection.
- Read-only readiness query before checker arming, with one timeout retry.
  Intermittent first-query timeouts remain unexplained.
- Captured-pose handover before the full SDK arm sequence or small elbow test.
- Shoulder/elbow tracking allowance of 0.30 rad and waist/wrist allowance of
  0.05 rad. Joint extrema support further measurement-based tuning.
- On abort and completion, hold the measured arm pose at full ownership weight
  while feedback is fresh. Stock `SetFsmId(1)` runs only if that hold fails or
  the process exits after taking ownership. No automatic rearm.
- Explicit manual launch/start commands in [tests/manual](tests/manual/README.md).

## Physical observations

The operator reported that the small elbow test worked. Larger arm tests exposed
tracking errors during and after handover, leading to separate handover timing
and the revised local tracking allowance.

A double-Start stop from a T-pose was reported to be jerky and caused a joint
shutdown requiring a robot restart. The affected joint's fault code and cause
have not been established. Successful stop RPCs are not physical stop acceptance.

## Next session — September 25: custom E-stop damping

Requested: stronger custom damping with a controlled transition that avoids the
reported jerk and joint shutdown.

A healthy stop now holds the measured arm pose with the profile's `kp` and `kd`
and does not call `SetFsmId(1)`. Stock damping remains the fallback when the
hold cannot be published. The profile's `kd` still does not tune stock damp mode.

Next work:

1. Identify the affected joint and fault details from the T-pose stop.
2. Determine the supported SDK control path for adjustable damping and its
   ownership interaction with stock balance and whole-body control.
3. Design the stop transition with explicit torque, velocity and timing bounds.
   Increasing damping alone can increase braking torque.
4. Define gain configuration and supported physical validation, keeping immediate
   fault handling and double-Start latching intact.

This is deferred design work. No stop gains or behavior were changed during
cleanup. Managed process watchdogs, policy isolation and frontend integration
remain outstanding as described above.
