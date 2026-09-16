# Optional wrapper and extra policy data

Use separate terminals from START_HERE.md for unknown policies. The optional wrapper is for a known, foreground, noninteractive policy launch command whose normal interruption behavior has already been checked by its operators.

## Launch a policy after recording is ready

Run from the policy's usual working directory. Use the full path to this kit:

```bash
/path/to/G1_SESSION_KIT/kit.sh record \
  --name policy-a-test-01 --policy "Arena submission/version" \
  --operator "names" --robot "G1 identity" --interface eth0 \
  --artifact /path/to/policy-config.yaml \
  -- /path/to/policy/python /path/to/policy/run.py --its-normal-arguments
```

Use your actual paths/interface and normal policy arguments. The collector's Python comes from `PYTHON` or `python3`; the policy may use an entirely different environment specified in the command.

- No policy starts unless the collector receives valid live G1 state within the startup timeout.
- The policy runs in the caller's working directory, with its inherited environment plus `G1_RUN_DIR` and `G1_POLICY_LOG`.
- Arguments after `--` go to the policy unchanged. Shell pipelines/redirects are not interpreted; put complex launch logic in an existing operator-owned script.
- The policy's stdout/stderr stay on the terminal. They are not automatically saved; attach the policy's own log afterward.
- The collector ends when the foreground policy exits and records its return code. Wrapper failure returns a nonzero exit status. No automatic policy retries occur.
- Ctrl+C requests SIGINT for the wrapped policy process group and closes the recorder. This is **not** an emergency stop or proof the robot has stopped. Use the policy's established stop procedure. A policy that ignores SIGINT is left running and flagged; it is never force-killed by this kit.
- Disk or collector failure does not terminate the policy automatically. Watch the status terminal. The recorder is not a robot safety supervisor.
- Background launchers, detached services, interactive input, and containers whose child lifetime differs from the policy are outside the wrapper contract. Use separate-terminal mode instead.
- A duration limit (`--seconds`) is allowed only without a wrapped policy.

Artifacts passed with repeated `--artifact` are copied and hashed before collection; attach additional completed files afterward. Include the exact checkpoint/configuration/calibration to support reproducibility. Optional file hashes do not authenticate which software ran.

## Extra events from a policy

Robot DDS provides state even when a policy cannot be modified. If its maintainers can add logging, copy/import `policy_hook.py` and call it from their **existing logging worker**, outside the time-critical motion loop:

```python
from policy_hook import log_event

log_event("inference", trial_id="001", duration_ms=3.2, policy_step=42)
log_event("task_observation", trial_id="001", detected=True, position_m=[0.2, 0.1, 0.9])
log_event("task_result", trial_id="001", outcome="failure", reason="missed_target")
```

These values illustrate the format; do not insert them as real measurements. The helper returns `False` if disabled, a value cannot be serialized, an event exceeds 32 KiB, or writing fails. Inspect that return value in the policy's logger. Each call does filesystem I/O; it is not a realtime-safe callback. Arrays should be converted to ordinary lists.

The wrapper sets `G1_POLICY_LOG` automatically. In separate-terminal mode, export it in the **policy terminal**, using the actual run folder:

```bash
export G1_RUN_DIR="/full/RUN/DIRECTORY"
export G1_POLICY_LOG="$G1_RUN_DIR/policy/events.jsonl"
```

Use one policy-event writer per run and stop it before export. Hook events are retained and counted in the report; the kit does not interpret arbitrary user fields as validated evaluation metrics. Monotonic clocks are comparable only on the same host/boot. External recordings need an independently established clock mapping.

Policy-independent scores require task definitions and external truth. Do not infer a successful task from joint motion, command publication, or a clean process exit.
