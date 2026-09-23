# G1 safety pipeline: robot side first, frontend later

Updated September 23, 2026. **Stage 1 is partially complete. All output remains dry-run.**

## Architecture and current checkpoint

```text
policy -> runner -> precheck -> safety checker -> postcheck -> motor gate
                                                              |
                                                    dry_run_output (today)
                                                    Unitree controller (future)
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
2. Add a synthetic ROS command producer, including deliberately invalid actions
   and freezes. Verify real feedback plus fake output through the full graph.
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
no dexterous hand attachments. A locked-waist configuration is available, but the
current physical lock state remains unconfirmed. Operator reports a 1,200-pound
rated gantry/hoist with suitable harness, feet bearing weight and arm clearance.

Installed Python SDK: 1.0.1, commit `65691c8`, Cyclone DDS 0.10.2. PC4 manifest:
master service 1.0.0.2, patch 1.0.0.6, video hub 1.0.2.3. Live LowState version
was `[0, 0]`, so motion-controller and motor firmware remain unidentified.

The new read-only bridge maps all 29 body slots, including locked waist slots.
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
