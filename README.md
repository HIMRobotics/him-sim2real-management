# G1 Session Kit — 1.2.0

**Operators: start with [START_HERE.md](START_HERE.md). Run `bash OPEN_APP.sh` to open the local control panel.**

A portable, read-only recorder for two unknown Arena policies or any G1 policy exposing standard Unitree HG DDS telemetry. It runs alongside the policy and creates named `runs/` folders, portable HTML diagnostics, operator trial labels, and verified return archives in `exports/`.

The app is a small local web page served by Python’s standard library: no Node, build step, internet, account, or extra web packages. It does not take camera ownership, publish robot commands, or change network configuration. Live capture requires an existing working Unitree SDK2 Python environment. Offline/demo/report/export operations use only Python's standard library, version 3.9+.

| File | Purpose |
|---|---|
| [START_HERE.md](START_HERE.md) | Tonight's step-by-step operator instructions |
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
