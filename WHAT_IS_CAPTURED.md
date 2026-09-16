# Capture contract and data interpretation

## Live streams

The recorder subscribes to HG G1 messages on DDS domain 0 by default:

| Topic | Stored fields |
|---|---|
| `rt/lowstate` | Raw motor slots: q, dq, ddq, estimated torque, temperature, voltage, mode, sensor/status fields when present; IMU quaternion, gyro, acceleration, RPY, temperature; remote bytes, tick, mode and version fields |
| `rt/lowcmd` | Raw motor command slots: mode, requested q/dq, feedforward torque, kp/kd; available mode fields |
| `rt/arm_sdk` | Same command fields, including the extra control/blend slots exactly as observed |

All motor slots in the received HG message are preserved. Current upstream HG state contains 35 slots; the report summarizes the first 29 G1 joints. It does not treat extra slots as physical joints. Missing optional fields are `null`, never silently converted to zero. Temperature may be a two-sensor array and is preserved as such. Nonfinite q/dq in the first 29 slots causes a counted decoding error rather than a valid sample.

Official schema/API references checked September 16, 2026:

- [Unitree HG MotorState](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/idl/unitree_hg/msg/dds_/_MotorState_.py)
- [Unitree HG LowState](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/idl/unitree_hg/msg/dds_/_LowState_.py)
- [Unitree SDK2 channel API](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/core/channel.py)

The kit never creates a command publisher. It does not claim compatibility with every G1 firmware, non-HG schema, custom DDS topic, ROS-only topic, or arbitrary Arena container. The live doctor is required on the actual deployment environment.

## Sampling and clocks

Each subscribed topic is sampled at up to 50 Hz by default; change with `--hz`. The source may publish faster or slower. The collector intentionally rate-limits and records that count. Each retained callback is copied to a bounded memory queue; a separate main loop writes JSONL. Queue overflow and decoding errors are counted. DDS/network loss before callback delivery is not observable as a precise drop count. Source tick values and saved timestamp gaps help diagnose discontinuities; tick decreases may indicate reboot or wrap and are not interpreted as a definitive reset.

Times are **host receipt monotonic seconds and host wall-clock nanoseconds**. They are not camera exposure, policy start, or motor application timestamps. Approximate simultaneous receipt of command and state is not a validated tracking error or end-to-end latency measurement. This release therefore does not report those as measured metrics.

Reports contain sampled joint velocity/estimated torque/temperature extrema, IMU roll/pitch magnitude, command counts, saved-state rate/gaps, collector errors/drops, and operator outcome summaries. Extrema can miss peaks between retained samples. There are no universal hardware safety pass/fail thresholds in the report.

## Folder layout

```text
runs/<UTC>_<name>_<unique-id>/
  manifest.json           identity, configuration, clock, start/end, errors, counts
  READY.json              valid state arrived; not robot qualification
  CLOSED.json             recorder finalized (even if collection failed)
  status.json             last periodic live status; may be absent for short runs
  telemetry/state.jsonl   retained raw state messages
  telemetry/command.jsonl retained raw commands from either supported topic
  events/*.json           immutable labels, trial starts, notes, attachment records
  policy/events.jsonl     optional policy hook data
  evidence/*              copied policy/config/video/calibration/log files
  summary.json            machine-readable diagnostics and manual outcomes
  report.html             standalone browser report
  checksums.json          added/rebuilt at export
```

`ACTIVE.json` exists only while recording or after an unclean interruption. An empty/error startup may have no telemetry files. Such runs remain exportable with their error and zero measurements plainly reported. Duplicate names produce separate folders; nothing silently replaces an older run. Records added after an export require a fresh export.

## Storage and custody

No camera or video is opened automatically. State plus both command topics can consume several GB per hour depending on source activity/rate. A 10-second pilot lets operators check actual disk use. Default reserve is 2 GiB; crossing it ends collection with an error. No runs are automatically deleted. External writers and large attachments can consume disk independently; monitor free space.

Exports copy the entire run, verify each archived payload against SHA-256, and create an archive checksum. This detects accidental file changes and transfer corruption; it is not independent authenticity or signed chain of custody. Export only after the collector, policy hook, and other run-folder writers have stopped. Keep external evidence intact as well.

## Outcomes and missing measurements

`success / (success + partial + failure + aborted)` is an exploratory **operator-labeled** fraction among resolved valid attempts. Invalid and unknown attempts are displayed separately. A trial-start without a label stays unknown. Unmarked attempts cannot be counted. Labels are task-specific; two policy percentages are not comparable unless the task, success definition, operating conditions and attempt accounting match.

Not automatically available: video/RGB-D, ball tracks, contact or placement truth, task-level reward, calibration accuracy, exact model identity without supplied artifacts, policy inference timing without instrumentation, applied motor commands, or an official HIM benchmark score. Attach existing camera outputs and independent reference measurements when available. Preserve failures and unknowns.
