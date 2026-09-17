# Laptop → G1: guided setup and file transfer

Run `bash OPEN_APP.sh` on a Mac or Linux laptop. Choose **Set up the G1 over SSH**. Native Windows hosting is not supported; a Windows browser can still view an app running on a Linux computer via the manual SSH instructions.

1. Enter the existing SSH destination: `username@robot-host`, `username@robot-IP`, or an SSH config alias. Enter the remote Python interpreter that already has Unitree SDK2 (for example, its full virtual-environment path). The kit requires Python 3.9+.
2. Click **Check SSH access**. This verifies SSH, the remote Linux/Python version and whether the SDK imports. This is NOT yet a live robot-state check.
3. Click **Copy recorder to G1**. Only recorder code/docs are copied into a new folder under the remote user's `~/.local/share/arena-g1-kit/releases/`. Existing installations and data are not overwritten. The transfer is SHA-256 checked. No packages or system services are installed.
4. Optionally select one policy file or archive and click **Transfer selected policy**. The file is copied unchanged into a unique folder under `~/.local/share/arena-g1-kit/policies/`. The app shows its verified path and checksum. It does not unpack it, install dependencies, load it into a controller, or execute it. Transfer does not establish that it is compatible with the G1.
5. Click **Open robot recorder**. The app starts only its own recorder UI on the remote computer and creates the SSH tunnel automatically. Click the resulting link to open the robot's recorder in a new tab.
6. In that tab, choose the robot-facing interface and click **Check connection**. Require live G1 state, then run the short stationary recording check. During policy tests, use the operators' normal launch/stop procedure for the transferred policy. Record, label and download the return ZIP from the robot tab; the browser saves it to your laptop.

Keep the laptop app terminal open. It maintains the SSH sessions. Browser refresh does not end those sessions. **Close recorder connection** closes the remote recorder app and tunnel; stop and save the run in the robot tab first. If SSH/app connectivity is lost, the supervisor attempts to finish the remote recorder; inspect the retained run on reconnection. It never signals or stops a separately running policy. A network partition or hard failure can leave incomplete data; use the existing recovery path rather than assuming the run closed cleanly.

## SSH prerequisite — once per laptop/account

The app uses the installed OpenSSH client with `BatchMode=yes` and `StrictHostKeyChecking=yes`. It never asks for, saves or sends an SSH password from the web page and never edits trusted-host files or keys. Existing unlocked SSH keys/agent or an already authenticated connection must work. A password-only SSH setup is not automated by this release.

If the check fails, open a normal terminal and run the command shown by the app. Verify the host fingerprint with the robot operator and complete the normal SSH/key setup. Unlock the existing key/agent if needed. Do not bypass host-key verification to make a connection error disappear. Then retry the app's check.

The interface accepts DNS names, IPv4 addresses and SSH aliases; use an alias for an IPv6 target. An empty SSH port field uses your SSH configuration/default port. The remote Python value must be `python3`, a versioned `python3.x`, or an absolute interpreter path. The helper does not activate arbitrary shell environments; use the correct interpreter and any existing remote environment setup needed by DDS.

## Boundaries

All remote operations are fixed, visible actions: probe the environment, stream and verify files, run this recorder app, and forward its local port. There is no arbitrary command box, robot power/motion-mode control, automatic policy execution, Arena download, or automatic Arena upload. This deliberate scope lets the operators keep their established policy and safety procedures.

Software tests exercise the fixed remote scripts and transfer checks locally; they do not establish an SSH connection or SDK stream from a physical G1. Verify that full path on the target hardware before the session.
