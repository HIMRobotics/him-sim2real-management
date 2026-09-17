# HIM Sim2Real Manager — 1.2.2

New to G1 policies? Read [which files to get and how to run the author’s deployment](POLICY_GUIDE.md), also linked inside the app.

**Operators: start with [START_HERE.md](START_HERE.md). Run `bash OPEN_APP.sh` to open the local control panel.**

## Get started for the session

**Mac/Linux laptop; Python 3.9+ required.** Download and unzip `HIM_SIM2REAL_MANAGER_v1.2.2.zip` from the [release page](https://github.com/HIMRobotics/him-sim2real-management/releases/latest), open a terminal in its `G1_SESSION_KIT` folder, and run:

```bash
bash OPEN_APP.sh
```

Or clone the repository:

```bash
git clone https://github.com/HIMRobotics/him-sim2real-management.git
cd him-sim2real-management
bash OPEN_APP.sh
```

This repository is private: operators need GitHub access to download it, or the owner can send them the release ZIP directly. Git is optional when using the ZIP. No web dependency installation or build is needed.

In the app: **Set up the G1 over SSH → Check SSH access → Copy recorder → Transfer policy (optional) → Open robot recorder**. Existing trusted SSH/key access and a remote Linux Python environment with Unitree SDK2 are required. The app shows the exact remote policy path; operators use their normal policy launch procedure.

Before running a policy, complete the [short stationary recording and export check](CONNECTION_GUIDE.md#two-minute-acceptance-check-before-testing-policies). A green software test does not establish compatibility with their physical robot. For each policy, create a named run, record, label the outcome, then **Prepare data for Arena → Download return ZIP**.

Need to try the interface without a robot? Run `bash OPEN_APP.sh --demo`; demo data is visibly marked synthetic and SSH operations are disabled.

**Current robot support: Unitree G1.** Additional robot integrations are planned; this release connects only to the G1’s standard Unitree HG DDS telemetry.

A portable, read-only recorder for Arena policy sessions. It runs alongside the policy and creates named `runs/` folders, portable HTML diagnostics, operator trial labels, and verified return archives in `exports/`.

The app is a small local web page served by Python’s standard library: no Node, build step, internet, account, or extra web packages. It does not take camera ownership, publish robot commands, or change network configuration. Live capture requires an existing working Unitree SDK2 Python environment. Offline/demo/report/export operations use only Python's standard library, version 3.9+.

| File | Purpose |
|---|---|
| [START_HERE.md](START_HERE.md) | Step-by-step operator instructions |
| [CONNECTION_GUIDE.md](CONNECTION_GUIDE.md) | Connect via the control laptop or onboard Linux/SSH; verify one stationary run |
| [REMOTE_SETUP.md](REMOTE_SETUP.md) | Guided SSH check, recorder/policy transfer and remote browser connection |
| `OPEN_APP.sh` | Open the local browser control panel |
| `app.py`, `web/` | Local-only server and dependency-free browser UI |
| [ADVANCED_TERMINAL.md](ADVANCED_TERMINAL.md) | Original terminal workflow as a fallback |
| `start.sh` | Guided run-name/setup prompts; starts passive recording |
| `kit.sh` | Doctor, record, annotate, report, attach, export, verify, recover |
| `label.sh` | Simple repeated trial-label prompts |
| [WRAPPER_AND_HOOK.md](WRAPPER_AND_HOOK.md) | Optional launch wrapper and extra policy logging |
| [WHAT_IS_CAPTURED.md](WHAT_IS_CAPTURED.md) | Exact fields, rates, outputs, measurement limits |
| [TROUBLESHOOTING.md](TROUBLESHOOTING.md) | SDK, networking, empty commands, interruptions |
| [ARENA_RECEIVING.md](ARENA_RECEIVING.md) | Receiving, verifying, interpreting, and comparing runs |
| [VALIDATION.md](VALIDATION.md) | Tests actually run and hardware checks remaining |
| `examples/demo-run/` | Clearly synthetic example of the delivered run/report |

From a Mac or Linux laptop, choose **Set up the G1 over SSH** to check existing SSH access, copy the recorder, transfer a policy file with checksum verification, and open the robot recorder through a managed tunnel. An existing trusted SSH host and unlocked key/agent are required. The remote Linux Python needs the Unitree SDK. The app transfers policies but never executes them.

Try a non-robot demo:

```bash
./kit.sh record --demo --name software-check --seconds 3
```

The run is permanently marked `synthetic_demo`. This tests file creation and reporting, not SDK connectivity. Never include it as a measured robot result.

Run the included tests:

```bash
python3 -m unittest discover -s tests -v
```

Existing project references: `g1_table_tennis_rally/DEPLOYMENT_DATA.md` describes the older controller-specific RGB-D recorder. `him-sports-eval` provides formal task-specific evaluation and review. This kit is standalone; it does not start either controller or automatically translate its logs into formal benchmark observations.

The app adds one outer `*_RETURN.zip` around the verified run archive and checksum, so operators send one file. Browser refresh does not interrupt collection. Only one app instance and one app-managed recording are allowed per kit folder. Live robot collection needs the existing SDK and a successful current connection check.
