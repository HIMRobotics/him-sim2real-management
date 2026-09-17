# HIM Sim2Real Manager — the simple version

New to G1 policies? Read [which files to get and how to run the author’s deployment](POLICY_GUIDE.md), also linked inside the app.

**Open this on a Mac/Linux laptop to connect over SSH, or directly on the Linux computer that already receives G1 data through Unitree SDK2.** This app runs locally, needs no internet or account, and never controls the robot. Keep using the policies' normal launch and stop procedures.

For the laptop/onboard setup options and a real-hardware acceptance check, use [CONNECTION_GUIDE.md](CONNECTION_GUIDE.md). The same short guide is linked inside the app.

**Current robot support: Unitree G1.** Other robot integrations are planned but are not available in this release.

## Open it

Unzip the kit. Open a terminal in the `G1_SESSION_KIT` folder and run:

```bash
bash OPEN_APP.sh
```

A browser opens. If it does not, open the exact link printed in the terminal. **Keep that terminal open.**

## Connect from your laptop — guided buttons

Choose **Set up the G1 over SSH**:

1. Enter their existing SSH address (`user@host` or saved SSH alias) and the remote Python that has Unitree SDK2. Leave the SSH port blank unless they use a custom port.
2. **Check SSH access.** This needs an already trusted host and an unlocked SSH key/agent. If their normal terminal still asks for a password, follow [REMOTE_SETUP.md](REMOTE_SETUP.md) first; the app cannot handle password prompts.
3. **Copy recorder to G1.** Wait for a verified destination path.
4. Optional: choose the policy file or ZIP, then **Transfer selected policy**. Give the verified destination path to the operator. The app does not unpack or run the policy.
5. **Open robot recorder**, then follow the link to its browser tab. The app runs the fixed setup commands and opens the SSH tunnel for you.
6. In the robot recorder, choose the actual robot-facing interface under **Connection settings** and follow the recording steps below.

Keep the laptop app terminal and SSH connection alive. Finish recording and download the return ZIP before clicking **Close recorder connection**. Remote recordings stay under `~/.local/share/arena-g1-kit/data/runs/`, including after copying a new recorder version.

## Already on the Linux control computer?

Choose **Record on this computer**. Use the Python environment that already contains Unitree SDK2, or select its absolute interpreter path under **Connection settings**. Python 3.9+ is required locally and remotely. The app adds no third-party web packages; it does not install the SDK or change the policy environment.

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

## Manual fallback

If SSH automation is unavailable, [CONNECTION_GUIDE.md](CONNECTION_GUIDE.md) has the manual SSH tunnel instructions. [ADVANCED_TERMINAL.md](ADVANCED_TERMINAL.md) provides recording commands without the web app. More detail: [TROUBLESHOOTING.md](TROUBLESHOOTING.md).
