# Policy examples

These model bundles and command files exercise the shared policy submission
interface in simulation and on hardware. The shared example image contains the
provided policy adapters. Other policies supply their own image and entrypoint
following the same [submission contract](../containers/INSTRUCTIONS.md).

| Directory | Contents |
| --- | --- |
| [policies/walker](policies/walker/) | Official NVIDIA Decoupled WBC Walk/Balance checkpoints and contract |
| [policies/stairs](policies/stairs/) | Legacy arena stair model, separate from the NVIDIA walker |
| [policies/balance](policies/balance/) | GR00T standing-balance model |
| [commands/walker](commands/walker/) | Stand, circle, and walk-then-stop velocity commands |

The shared client and image for `walker`, `stairs`, `balance`, and `holosoma`
live in [containers/examples](../containers/examples/README.md).
The simulator itself lives in `containers/sim/`.

## Run an example

From the repository root:

```bash
./submit.sh --mode sim --example walker --local --seconds 10
./submit.sh --mode sim --example balance --local --seconds 10
./submit.sh --mode sim --example walker --command examples/commands/walker/circle.yaml --view
./submit.sh --mode sim --example walker --command examples/commands/walker/forward_stop.yaml --seconds 7 --view
```

The local examples run through submission checks and may report a violation.
`--example` selects a policy and, without `--local`, builds its shared container
image. `--command FILE` supplies a velocity schedule to that policy. See the
[hardware operator guide](../robot_safety/HARDWARE_ADAPTER.md#operator-commands)
for `./submit.sh --mode real --example walker --command FILE`, startup balance,
operator handoff, and stopping.

`--command` alone does not select a package or a container. It plays a file under `commands/walker/` on the cached walker. The report `policy` field is that file name without `.yaml`: `circle.yaml` records `circle`, and `forward_stop.yaml` records `forward_stop`.

`--example walker --command FILE` is the same velocity file inside the walker container. The report `policy` field stays `walker`.

The direct command path bypasses submission checks and reports its active
runtime checks separately.

## Shared container

`containers/examples/Dockerfile` packages the shared client, which reads
`/safety/policy_state` and publishes `/safety/precheck`. The host owns the motor
output path. Simulation uses an isolated network, and hardware uses the managed
host safety service.

Generated images and build contexts belong under `.cache/`. Follow
[build and export](../containers/examples/README.md#build-and-export-the-shared-image)
to save the shared image as a submission archive without starting a policy.
