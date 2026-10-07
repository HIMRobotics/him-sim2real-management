# Simulation examples

These are policy fixtures and example clients for testing the simulation and
submission interfaces. Their Dockerfiles are specific test images, not a
general-purpose policy deployment image.

| Directory | Contents |
| --- | --- |
| [policies/walker](policies/walker/) | Official NVIDIA Decoupled WBC Walk/Balance checkpoints and contract |
| [policies/stairs](policies/stairs/) | Legacy arena stair model, separate from the NVIDIA walker |
| [policies/balance](policies/balance/) | GR00T standing-balance model |
| [commands/walker](commands/walker/) | Stand, circle, and walk-then-stop velocity commands |
| [submissions/walker](submissions/walker/) | Example walker container, ROS client, and build script |

The shared simulation-only clients for `sim.sh --example walker`, `stairs`, and
`balance` live in [containers/examples](../containers/examples/README.md).
The simulator itself lives in `containers/sim/`.

## Run an example

From the repository root:

```bash
./sim.sh --example walker --local --seconds 10
./sim.sh --example balance --local --seconds 10
./sim.sh --command examples/commands/walker/circle.yaml --view
./sim.sh --command examples/commands/walker/forward_stop.yaml --seconds 7 --view
./sim.sh --example walker --command examples/commands/walker/forward_stop.yaml --seconds 7 --view

The local examples run through submission checks and may report a violation.
`--example` selects a policy package under `policies/` and, without `--local`, builds `g1-sim-example-<name>:local`. Its report `policy` field is `walker`, `stairs`, or `balance`.

`--command` alone does not select a package or a container. It plays a file under `commands/walker/` on the cached walker. The report `policy` field is that file name without `.yaml`: `circle.yaml` records `circle`, and `forward_stop.yaml` records `forward_stop`.

`--example walker --command FILE` is the same velocity file inside the walker container. The report `policy` field stays `walker`.

The local examples run through submission checks and may report a violation.
The direct command path bypasses submission checks and labels runtime monitoring

## Container roles

`submissions/walker/Dockerfile` packages the walker, SDK telemetry subscriber,
and ROS command publisher to test a saved submission archive in MuJoCo. Its
observations depend on simulated base velocity, so it is not a validated
physical-robot deployment.

`containers/examples/Dockerfile` packages the simulation-only example clients.
Those clients require the isolated simulator network and consume `/sim/state`.

General policy deployment currently defines an image contract: the policy author
provides an image containing their dependencies and entrypoint, and the host
launcher runs it. Neither example Dockerfile is a universal deployment recipe.
See [the submission contract](../containers/INSTRUCTIONS.md).

Generated images and build contexts belong under `.cache/`. The walker build
writes `.cache/walker_submission/submission.tar`. Copy the chosen archive into
`containers/upload/` when testing the upload workflow.
