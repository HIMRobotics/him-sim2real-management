# Simulation examples

These are policy fixtures and example clients for testing the simulation and
submission interfaces. Their Dockerfiles are specific test images, not a
general-purpose policy deployment image.

| Directory | Contents |
| --- | --- |
| [policies/walker](policies/walker/) | Walker ONNX graph, external weights, and observation/action specification |
| [policies/balance](policies/balance/) | GR00T standing-balance model |
| [commands/walker](commands/walker/) | Stand and circle velocity commands |
| [scenes](scenes/) | MuJoCo scenes with a floor, a step, or a fixed obstacle |
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
./sim.sh --example walker --local --seconds 10 --scene examples/scenes/step.xml
```

## Scenes

`scenes/` has host MuJoCo XML files for `--scene`. Each one includes the bundled G1, names the ground `safety_floor`, and adds only fixed geoms, so the `stand` keyframe stays valid.

| File | Contents |
| --- | --- |
| [scenes/flat.xml](scenes/flat.xml) | Flat floor |
| [scenes/step.xml](scenes/step.xml) | 10 cm step, 1.5 m in front of the spawn |
| [scenes/obstacle.xml](scenes/obstacle.xml) | Fixed box in the forward path |

These paths are relative to `examples/scenes/`. Keep the files there so the G1 include and meshes resolve. Copy one and edit the box geoms to try another object. A free body changes the position layout, so its `stand` keyframe has to include that object's pose too. See [Submit a MuJoCo environment](../containers/SIM_RUN.md#submit-a-mujoco-environment).

The local examples run through submission checks and may report a violation.
The direct circle path bypasses submission checks and labels runtime monitoring
as partial. See [simulation instructions](../containers/SIM_RUN.md).

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
