# G1 Session Kit — the simple version

**Use the Linux computer that already receives G1 data through Unitree SDK2.** This app runs locally, needs no internet or account, and never controls the robot. Keep using the policies' normal launch and stop procedures.

For the laptop/onboard setup options and a real-hardware acceptance check, use [CONNECTION_GUIDE.md](CONNECTION_GUIDE.md). The same short guide is linked inside the app.

## Open it

Unzip the kit. Open a terminal in the `G1_SESSION_KIT` folder and run:

```bash
bash OPEN_APP.sh
```

A browser opens. If it does not, open the exact link printed in the terminal. **Keep that terminal open.**

Use the existing robot Python environment. If it is not already active, launch with:

```bash
PYTHON=/path/to/the/robot/environment/bin/python bash OPEN_APP.sh
```

Python 3.9+ is required. The app itself adds no third-party packages. If Unitree SDK2 is in a different environment, you can also select its Python under **Connection settings** in the app.

## Record each policy

1. **Check connection.** Wait for “Connected. Live G1 state received.” If it fails, open Connection settings and select the robot network interface / existing SDK Python. Do not start the policy based on a failed check.
2. **Name the run.** For example, `policy-a-standing-01`. Enter the operator, policy/Arena ID, and what task counts as success.
3. **Start recording.** Wait for the green **RECORDING** indicator. Then launch the policy normally in its own terminal. The state sample count should grow.
4. **Stop the policy normally, then click Stop recording.** Wait for **SAVED**.
5. **Add an outcome** for each attempt. Include failures and operator interventions. Optionally choose completed video, log, or configuration files to include.
6. **Prepare data for Arena → Download return ZIP.** Send Arena that **one ZIP**. It contains the report, raw data, labels, attachments, and checksum.

Repeat with a different name for policy B. Keep the local `runs` folders until Arena confirms receipt. If you change labels or add files later, prepare and send a fresh ZIP.

**Do a short stationary recording first.** The software has been tested locally, including the browser workflow; the exact G1, SDK and network combination must pass its live check. Seeing telemetry is not a qualification of an unknown policy.

## Three things to remember

- The app records only. **Stop recording does not stop the policy or robot.**
- Refreshing or closing the browser does not stop recording. Reopen the terminal's link to reconnect. Stop recording before closing the app terminal.
- Task outcomes are operator labels. Camera/video is not opened automatically; attach existing footage if available. No data is uploaded to Arena or sent to Arena automatically.

## Try the buttons without a robot

```bash
bash OPEN_APP.sh --demo
```

The whole app shows **DEMO**, and every recording stays marked synthetic. Reopen without `--demo` for real collection. Never send a demo as robot evidence.

## If the G1 computer is accessed by SSH

Run the app on that computer and use an SSH port forward to view it on your laptop:

```bash
ssh -L 8765:127.0.0.1:8765 YOUR_USER@YOUR_G1_COMPUTER
```

In that SSH terminal, run `bash OPEN_APP.sh --no-browser`. Open its printed `http://127.0.0.1:8765/#key=...` link on your laptop. Keep the SSH session open. If the app prints a different port because 8765 is busy, forward that exact port instead. The app listens on loopback only; do not expose it on the public network.

Need the command-line fallback? [ADVANCED_TERMINAL.md](ADVANCED_TERMINAL.md). More detail: [TROUBLESHOOTING.md](TROUBLESHOOTING.md).
