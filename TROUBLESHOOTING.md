# Troubleshooting for tonight

**Web app:** launch with `bash OPEN_APP.sh`. Keep the app terminal open. If the browser does not open, copy the exact link printed there, including its access key. That link is only for the local recorder; do not share it.

If the page says the local app is unreachable, reopen its terminal/link. The last displayed counters may be stale. A browser refresh does not stop the recorder. If the terminal/process crashed, reopen the app; any unfinished run appears as incomplete. Recovery refuses to modify a recorder whose PID is still alive. If it is still running, stop it from its original terminal.

If Check connection fails, use **Connection settings** to select the actual robot-facing interface and the full path to the existing SDK Python. Expand **Connection details** for the precise error. A fresh check is required after changing settings and before a new run when the check is older than five minutes.

An app already running in this same folder blocks a second instance. Use the existing terminal link. When the default web port is occupied by another service, the app chooses another port and prints it. It never kills the other service.

For a missing or malformed web interface, the original terminal recorder below remains available.


## `python3` or Unitree SDK is missing

Run on the existing robot-side Linux environment that already has Unitree SDK2. This kit supports Python 3.9+. It does not need NumPy, OpenCV, or RealSense packages of its own.

```bash
export PYTHON=/absolute/path/to/the/existing/sdk/environment/bin/python
./kit.sh doctor
```

Keep that environment selected for `start.sh`. If no working SDK2 environment exists, have the robot operator provision it using [Unitree's official instructions](https://github.com/unitreerobotics/unitree_sdk2_python). Installing only a Python package may not supply the compatible native Cyclone DDS library. Do not replace the working policy environment or update robot firmware for this collector. The kit includes no bundled SDK/native binaries and cannot guarantee an unprovisioned computer will connect tonight.

## No valid `rt/lowstate` / automatic interface selection fails

- Confirm that the computer is the robot's onboard development computer or connected robot-control laptop, not an unrelated desktop.
- Confirm that the normal Unitree state reader works in that Python environment.
- Use the exact robot-facing interface. The doctor's first lines list interface names:

```bash
./kit.sh doctor --interface enp3s0
```

- Match the configured DDS domain with `--domain NUMBER`. This kit defaults to 0.
- Confirm the robot provides **HG G1 `LowState_`**. A custom ROS topic, simulator-only message, different robot message family, or changed DDS QoS may need an adapter. Do not interpret a timeout as a robot fault.
- If optional command subscriptions have a type conflict, use `./kit.sh record --name YOUR_RUN --interface YOUR_INTERFACE --state-only` and collect policy logs separately. Live state is still required.

A failed collection leaves a named error run with no invented samples. You can export it to Arena with the terminal/error log. A wrapped policy is not launched if live-state startup fails.

## `RECORDING READY`, but command count stays at zero

This is not automatically a recorder failure. A stationary robot may have no publisher on those command topics, or the policy may use high-level motion RPC, ROS, a bridge, custom topics, or another computer/domain. State still provides useful measured motion, temperatures and IMU data. Attach the policy log and note the launch architecture. The report clearly says command measurements are missing. Do not switch controller ownership merely to produce command data.

## `STALE` / state count stops growing

Watch the robot and use its established stop procedure if needed. The collector does not command a stop. Preserve the run, note the time, and check robot power/network/policy behavior. It continues waiting and recording any returned stream; receipt gaps and warnings remain visible. Do not reset or delete the run to hide the interruption.

## Low disk space

The default reserve is 2 GiB. Collection stops with an error when free space falls below it. A separately launched or wrapped policy may still be running: check and stop it normally. Free space by moving completed, verified run archives to your normal storage. The kit never deletes recordings. For longer runs, inspect disk use after a short pilot and reduce saved rate using `--hz 20` only if the lower rate suits the analysis. Never assume a sampling gap metric is control-loop latency.

## I pressed Ctrl+C and the policy is still running

In the recommended separate-terminal workflow, this is expected: Ctrl+C in the collector terminal stops **only collection**. Stop the policy using its own procedure first.

Wrapper mode requests SIGINT but never force-kills a policy or guarantees a safe robot state. Follow the robot's normal stop process and keep the command/evidence writers stopped before exporting. A running wrapped PID blocks export on the same host.

## Terminal/power died; export says run is active

First ensure the recorder, policy, and any log writer are actually stopped. Then:

```bash
./kit.sh recover "/full/RUN/DIRECTORY" --confirm-writers-stopped
./kit.sh export "/full/RUN/DIRECTORY"
```

Recovery refuses a known live recorder PID on the same host. It preserves raw data and flags the result `recovered_incomplete`; it does not invent lost samples, silently discard a truncated final JSONL line, or certify completeness. If recovery reports a still-live PID, stop it normally or ask Arena to inspect the folder rather than deleting the lock blindly.

## Video is large / another process is writing evidence

Wait for its recording to stop before `attach`. Attaching makes a copy and needs disk space for that copy. Export additionally needs room for a compressed archive. If storage is tight, send the video separately with the exact run name and include a note; the video then is **not inside the checked run bundle**. Do not claim it was attached. Paths with spaces must be quoted.

## I added labels after exporting

Run `export` again and send the new pair of files. Each export uses a unique name. Older archives remain unchanged. Hash verification of an older extracted run is expected to fail if its contents were edited afterward.

## Script permission denied after copying/unzipping

```bash
chmod +x kit.sh start.sh label.sh
```

Or invoke the launcher with `bash kit.sh doctor`. Keep the whole kit together; moving only a shell script separates it from the collector.

## Guided SSH setup or policy transfer fails

- **SSH check fails:** open the first-time login command shown in the app in your normal terminal. Confirm the address, host fingerprint, and login with the operators. The app needs an existing unlocked key/agent; it cannot enter a password, accept a host key, or configure access for you. See [REMOTE_SETUP.md](REMOTE_SETUP.md).
- **SDK missing:** choose the absolute path to the remote Python that already imports Unitree SDK2. Python 3.9+ is required. An SSH check alone does not prove live telemetry.
- **Upload fails:** check free disk space on both machines (file size plus 2 GiB), keep the app terminal open, then retry. A successful transfer displays the exact remote path and verified SHA-256. A policy is copied only, never unpacked or launched.
- **Remote recorder disconnects:** stop the policy normally, restore SSH, check access and reopen the recorder. Saved runs remain in the remote `~/.local/share/arena-g1-kit/data/runs/` folder. Recover an interrupted run through the recorder before exporting. No remote runtime data lives inside the replaceable recorder code folder.
- **Windows laptop:** native app hosting is not supported. Run the app on the existing Linux computer and use the manual SSH tunnel/browser flow in CONNECTION_GUIDE.md.
