# Connect to a G1 and manage a recording session

This app can copy recorder code and policy files through existing SSH access, open a remote recorder, and manage **recording sessions**: connection checks, recording, labels, attachments and exports. It does not power on, stand up, select a motion mode, load a policy, release a motion service, or stop the robot.

The operators keep their existing G1 bring-up, policy launch and robot-stop procedures. Unknown Arena policies are not qualified merely because telemetry is visible.

## Choose where the recorder runs

| Your setup | Where to run `OPEN_APP.sh` | Where to open the browser |
|---|---|---|
| Linux control laptop already communicating with the G1 | That laptop, in its existing SDK environment | Same laptop |
| SDK/data access on G1 onboard Linux | Onboard Linux, in its existing SDK environment | Any laptop through SSH forwarding |
| Mac or Windows laptop without a working Unitree SDK environment | Use the G1/onboard or another connected Linux computer | Mac/Windows browser through SSH forwarding |

Native Windows execution is not supported by this release. macOS was used for software/demo testing; physical collection is intended for the operators' already working Linux SDK setup. Do not assume a browser running on your laptop can itself discover the robot.

## Path A: existing Linux control laptop

1. Have the operator connect the laptop to the robot using the **same cable/network configuration already used for G1 development**. Use their actual robot-facing interface; do not change IP addresses or firmware for this recorder.
2. Open a terminal in the extracted kit folder. Activate the Python environment that already contains Unitree SDK2, or provide its interpreter explicitly:

   ```bash
   PYTHON=/absolute/path/to/existing/sdk/python bash OPEN_APP.sh
   ```

   Python 3.9+ is required by the kit. The SDK's own minimum version may be lower; an SDK environment using Python 3.8 is not sufficient for this release. Resolve that environment before the session rather than changing the policy's dependencies during a run.

3. In the app, open **Connection settings**. Choose the actual robot-facing network interface. `auto` works only if DDS chooses the correct interface. Ethernet interface names vary; `eth0` is an example, not a required value.
4. Leave DDS domain at `0` unless the operator deliberately configured a different domain. This value is not a robot IP address. The recorder subscribes to DDS topics; it has no robot-IP text field.
5. Click **Check connection**. Continue only after **Connected. Live G1 state received.** The same check is repeated implicitly when the recorder waits for live data before declaring recording readiness.

If the laptop does not already have a working G1 SDK/network configuration, use the operators' vendor setup procedure first. The recorder does not configure the Ethernet interface, install native DDS libraries, or discover SSH credentials. See Unitree's [SDK setup instructions](https://github.com/unitreerobotics/unitree_sdk2_python#installation) and [network quick start linked by the SDK](https://support.unitree.com/home/en/developer/Quick_start), using the settings appropriate to the actual G1 revision.

## Path B: guided remote setup from a Mac/Linux laptop

Run `bash OPEN_APP.sh` locally and choose **Set up the G1 over SSH**. Enter the known SSH destination and SDK Python, then use **Check SSH access → Copy recorder to G1 → Transfer selected policy (optional) → Open robot recorder**. Follow the opened link and complete the live telemetry check. Full instructions and first-time SSH requirements: [REMOTE_SETUP.md](REMOTE_SETUP.md).

This uses existing trusted-host entries and an unlocked key/agent. It runs fixed Python commands over SSH, verifies copied bytes, and manages the loopback tunnel. It does not execute the uploaded policy. Keep the local app terminal open until recording is saved and downloaded. Disconnecting the managed session asks its recorder to finish; it does not stop the policy.

## Manual alternative: recorder on onboard Linux, browser on a laptop

1. Use the operator's known SSH username and G1/onboard address. This kit does not assume a default IP or password.
2. From the laptop, establish the tunnel:

   ```bash
   ssh -L 8765:127.0.0.1:8765 YOUR_USER@YOUR_G1_COMPUTER
   ```

3. In that SSH session, go to the kit folder on the G1 computer and run:

   ```bash
   PYTHON=/absolute/path/to/existing/sdk/python bash OPEN_APP.sh --no-browser --port 8765
   ```

4. Copy the full printed `http://127.0.0.1:8765/#key=...` link into the laptop browser. Keep the terminal and SSH session open. If the app reports a different port, forward that actual port as well; do not expose the app on `0.0.0.0` or a public server.
5. Choose the onboard computer's actual robot-facing DDS interface, then click **Check connection**.

The browser's “localhost” reaches the recorder through SSH. SDK subscriptions run on the onboard computer. Opening the laptop browser without the tunnel will not connect to the onboard app.

## Two-minute acceptance check before testing policies

1. Put the robot in the operators' normal safe stationary state. Confirm their normal G1 state tool already works.
2. Check connection in this app. Enter the robot's actual identity in Connection settings. This is an operator label; the app does not authenticate the serial number from DDS. Make sure the interface/domain reaches the intended G1, not another robot or a simulator.
3. Name a run `g1-stationary-check`, start recording, and wait for **RECORDING**. Let it capture 15–30 seconds. The state sample count should increase. Command count may remain zero while idle.
4. Click **Stop recording**, wait for **SAVED**, then open the report. Require nonzero state samples, no collector error, and no unexplained long gaps or decode failures.
5. Click **Prepare data for Arena**, then **Download return ZIP**. Keep this check run. This confirms the real SDK → disk → report → export route on that computer; it is stronger evidence than this project's synthetic tests.

If any step fails, preserve the error/log and use [TROUBLESHOOTING.md](TROUBLESHOOTING.md). Do not proceed based on demo data. There is no physical-G1 verification result in the release yet.

## Manage the actual session

Use a separate named run for each policy/attempt. Start recording before launching the policy. Launch the policy through its existing, already checked operator procedure. The app does not manage incompatible controller ownership or automatically switch between policies.

Stop the policy using its own procedure first. Then stop recording, label every attempt including failures and interventions, attach completed logs/video/configuration, and prepare/download the ZIP. Send that ZIP to Arena through your usual channel. There is no automatic Arena upload.

Refreshing or closing the browser leaves recording running. Closing the app terminal asks the recorder to finish but never stops a separately launched policy. A stale-data alert likewise does not stop the robot. The robot's emergency stop remains the operators' existing control.

## What is verified about the SDK integration

Unitree's [official G1 example](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/g1/low_level/g1_low_level_example.py) imports HG `LowState_`, subscribes to `rt/lowstate`, and initializes DDS with a domain and optional network interface. This kit uses that subscriber contract without a publisher or motion-service client. The code preserves HG message motor slots and reports the first 29 slots; operators must identify the actual G1 variant and interpret unused slots accordingly.

**Do not use that example program as a passive connectivity test:** it also switches controller mode and publishes motor commands. Use this app's read-only check. Source/API agreement does not verify firmware, network QoS, physical data correctness or policy compatibility on your robot.
