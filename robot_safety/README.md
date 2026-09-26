# Controller stop detection (dry-run)

`controller.py` separates SDK byte decoding from gesture recognition:

- `decode_start_pressed(payload)` reads bit 2 of byte 2 in the 40-byte G1 `wireless_remote` payload, following the installed SDK example.
- `StartDoublePressDetector.update(start_button_pressed, cur_time_secs)` emits a one-sample stop event for press → release → press within `Config.double_press_window_secs` (default 0.5 seconds).
- The detector requires a released sample after startup/reset. Holds and repeated pressed samples do not count again. A stale sample gap clears pending history. This does not establish radio-link health.

`Supervisor.state(observation, start_button_pressed, cur_time_secs, *, stop_requested=False)` now receives the physical Start state. It runs the detector and latches its event. The optional `stop_requested` argument accepts an independent event even with Start released. Physical Start blocks reset/arm, while the latch persists after release. Explicit reset clears gesture history and returns to inhibited, with separate arming required.

The old `stop_pressed` argument/status field is replaced by `start_button_pressed`. Callers that previously passed a stop event positionally must migrate to `stop_requested=`. There is still no motor publisher or full supervisor runtime.

## Test with the actual controller

Use the Python environment that already imports the Unitree G1 SDK and its native DDS libraries. From the repository root:

```bash
python3 -m tools.controller_monitor --interface YOUR_ROBOT_INTERFACE
```

Substitute the robot-facing interface used by your existing deployment. If `python3` is not your SDK interpreter, use its absolute path. The manager's uv environment does not automatically contain the SDK. The monitor defaults to DDS domain 0 and `rt/lowstate`. To follow the installed SDK's lower-frequency controller example instead:

```bash
python3 -m tools.controller_monitor --interface YOUR_ROBOT_INTERFACE --topic rt/lf/lowstate
```

Release Start, press once, release, then press again within 500 ms. Expect Start pressed/released messages and `STOP REQUESTED: Start double press detected. No command sent.` Test a single press, long hold and two widely spaced presses too. Ctrl+C exits. The detector window can be set using `--window-secs`.

This tool only subscribes to state. It neither publishes motor commands nor releases control modes. Nevertheless, the stock robot firmware may respond to controller buttons independently. Test with the G1 appropriately supported and under operator control. This confirms the software detects received gestures, not that our code physically damps the robot or overrides a running policy.

No fresh state produces a warning and resets the gesture. Backlogged or invalid input cannot be used to complete a pair. Short presses missed between telemetry samples cannot be reconstructed. Robot packet freshness does not prove remote radio freshness. The monitor does not filter on `LowState.tick`, whose behavior must be verified on the installed firmware.

## Offline validation

```bash
python3 -m unittest discover -s tests -p 'test_robot_safety*.py'
```

## Troubleshooting missing double presses

Run with `--debug` to print the two raw button bytes, robot tick, sample gaps, and intervals between detected presses. If no Start transitions appear on `rt/lowstate`, try `--topic rt/lf/lowstate`, as used by the installed SDK controller example. Both topics still require verification on your firmware. A Start press should set bit 2 in the first printed button byte.

If transitions appear but no stop event, check that two rising edges occur within the configured window and that sample gaps stay below the 200 ms freshness timeout. Repeated robot ticks no longer erase gesture history in this diagnostic tool. A diagnostic observation is not a verified radio-health or motor-stop guarantee.

## Recorded operator validation

On September 22, 2026, the operator confirmed successful detection using the actual G1 controller. The supplied debug trace showed Start as `0400`, two double-press events with approximately 160 ms between presses, and rejection of an interval of approximately 1.06 seconds. Displayed telemetry gaps were around 1 ms. Firmware version, interface and selected topic were not recorded in the supplied trace. This validates controller-input decoding and gesture detection only, not motor damping or stop priority over a policy.
