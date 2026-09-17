# Record G1 policies and return useful data

This folder records the G1 while you run a policy in your normal way. **It never sends motor commands.** You do not need to modify either policy.

Use one run per policy attempt. Keep failures as well as successes. Send Arena the resulting archive and checksum file for each run.

## 1. Put the folder on the robot-side computer

Copy the entire `G1_SESSION_KIT` folder to **the G1 onboard computer or the Linux laptop that already talks to the robot through Unitree SDK2**. Unpack it if you received a zip. Run the following commands in a terminal inside this folder:

```bash
chmod +x kit.sh start.sh label.sh
./kit.sh doctor
```

This checks for live G1 state for 10 seconds, without starting a policy. Look for **PASS: live G1 state received**. No commands during a stationary check is normal.

Use the Python environment that already has the robot SDK. If needed:

```bash
export PYTHON=/absolute/path/to/the/robot/environment/bin/python
./kit.sh doctor --interface YOUR_ROBOT_NETWORK_INTERFACE
```

Replace that interface with the actual name, such as `eth0` or `enp3s0`; the doctor prints available names. `auto` is the default. DDS domain defaults to `0`; use `--domain NUMBER` if the operators configured another domain. Do not guess or change the robot's network configuration to match this kit.

**If doctor fails, stop here and read [TROUBLESHOOTING.md](TROUBLESHOOTING.md).** The kit needs Python 3.9+ and the existing Unitree SDK2 Python / Cyclone DDS environment. It does not install or change robot software. A successful doctor checks connectivity and message decoding, not policy compatibility or safe motion.

## 2. Start recording in terminal A

```bash
./start.sh
```

Answer the prompts. Suggested names:

- `policy-a-standing-01`
- `policy-a-task-01`
- `policy-b-standing-01`
- `policy-b-task-01`

For the policy label, enter the Arena submission name/ID and version if available. In notes, include its Arena URL, what task you are trying, what counts as success, and anything changed from the previous run. Record the robot identity and operator names.

Wait for **RECORDING READY**. Copy the printed **RUN DIRECTORY** somewhere convenient. Every run gets a timestamp and unique suffix; earlier runs are never overwritten.

The terminal prints counts and state freshness every two seconds. State counts should increase. Command counts may stay at zero if the policy uses another interface. A **STALE** warning means the recorder is no longer seeing fresh robot state.

## 3. Run policy A normally in terminal B

Launch the policy exactly as its operators normally would. No launch-command changes are needed. Leave the recorder running through setup, motion, failure, and the return to rest. Capture 10–20 seconds before and after each attempt when practical.

The policy operators remain responsible for commissioning, compatible controller ownership, normal stopping, and the robot emergency stop. The recorder is not a motion supervisor. Stopping it does not stop a separately launched policy.

**Record video too, if available.** Use the policy's existing recording function or a phone view of the robot/task. Say the run name and trial number aloud at the start; include the whole attempt, not just successful motion. The kit deliberately does not open the robot camera, because the policy may already be using it. Phone timestamps are not precise camera/robot synchronization.

## 4. Mark what happened

You can label during collection from terminal C or immediately afterward:

```bash
./label.sh "/full/RUN/DIRECTORY"
```

Enter a trial ID such as `001`, the outcome, and useful notes. Press Enter at an empty trial ID to finish.

| Outcome | Use it for |
|---|---|
| `success` | Met the success definition recorded in the run notes |
| `partial` | Did part of the task but did not fully meet that definition |
| `failure` | Valid attempt that failed, including a policy crash |
| `invalid` | Attempt was invalid for a reason independent of policy success; explain why |
| `aborted` | Operator stopped/intervened before completion; explain why |
| `unknown` | Outcome could not be determined |

Include interventions, falls, missed contacts, unusual movement, error text, and a video timestamp. Do not omit failed attempts. Reusing a trial ID adds a correction; the report uses its latest label and preserves the original events.

For better attempt accounting, mark each trial **before it begins**:

```bash
./kit.sh mark "/full/RUN/DIRECTORY" --kind trial-start --trial 001 \
  --operator "operator name" --notes "first standing attempt"
```

An attempt marked at the start but never labeled remains `unknown`. The collector cannot detect unmarked attempts automatically.

## 5. Stop normally, then finish recording

First stop the policy using its usual robot-safe procedure. Allow a short recording of the robot at rest. Then press **Ctrl+C in terminal A** and wait for **Run saved**.

Do not close the terminal or power down while it is finishing. The run now contains `report.html`, `summary.json`, and raw telemetry. Open `report.html` in a browser. Check the policy name, nonzero state count, quality warnings, and your labels.

## 6. Attach evidence and create the return files

After the external recording/log writer has stopped, attach any relevant files. Each is copied into the run and hashed. These commands are examples; use real existing files:

```bash
./kit.sh attach "/full/RUN/DIRECTORY" "/path/to/video.mp4" --role video
./kit.sh attach "/full/RUN/DIRECTORY" "/path/to/policy.log" --role policy-log
./kit.sh attach "/full/RUN/DIRECTORY" "/path/to/config.yaml" --role policy-config
```

Useful extras: exact policy configuration, checkpoint digest/file if practical, calibration, existing camera recording, an installed-paddle photo for table tennis, and the policy launch/error log. A run label alone does not establish which model weights actually ran. Avoid attaching credentials or unrelated files.

Create the final package:

```bash
./kit.sh export "/full/RUN/DIRECTORY"
```

The command prints **SEND BOTH FILES TO ARENA**:

- `exports/<timestamp>_<run-name>_<id>.tar.gz`
- the matching `.tar.gz.sha256` file

Send both through your usual file transfer. There is no upload, account login, or Arena publication in this kit. Export verifies the contents of the archive it actually wrote. Keep the original `runs` folder until Arena confirms receipt. If you add labels or evidence later, export again and send the new pair.

## 7. Repeat for policy B

Run `./start.sh` again with policy B's identity. Use comparable tasks, setup, and success definitions where the policies support them. Do not combine the policies into one run. A failed launch is still useful: retain the run, attach the error log, and label the attempt.

## What Arena gets

- Live joint positions, velocities, estimated torques, temperature arrays, available motor status, IMU and remote state.
- Observed command messages on the two supported G1 DDS command topics, when present.
- Receipt timestamps, sampling gaps, collector drop/error counts, and any policy exit code from wrapper mode.
- Your attempt labels, notes, and attachments with file integrity checks.
- A portable HTML report and machine-readable JSON plus raw logs for deeper sim-to-real analysis.

This works without knowing either policy's architecture. It does **not** automatically measure task success, perception accuracy, camera-to-motor latency, or sim-to-real score differences. Those require the task's reference measurements and policy instrumentation. See [WHAT_IS_CAPTURED.md](WHAT_IS_CAPTURED.md).

**Local software tests pass; physical G1 collection has not been tested by the development team.** Run the live doctor and a short stationary recording on your exact robot before relying on the kit for the session.
