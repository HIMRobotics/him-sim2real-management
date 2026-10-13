# Submission template

This directory is a complete submission with no weights. It already follows
every rule in [INSTRUCTIONS.md](../INSTRUCTIONS.md), and as shipped it simply
holds the robot's starting pose. The
[quick start](../INSTRUCTIONS.md#quick-start) walks through copying it,
building it, and testing it. This page describes what is inside.

| File | Edit it? | What it does |
| --- | --- | --- |
| `policy.py` | Yes | Your controller, plus the `KP` and `KD` gains it requests |
| `assets/` | Yes | Weights and config files, available in the image at `/submission/assets` |
| `requirements.txt` | Yes | Python packages to install. Uncomment `onnxruntime` for ONNX models |
| `node.py` | No | The ROS side: reads state, arms, publishes commands, and stops on abort |
| `entrypoint.sh` | Rarely | The start script. It sources ROS and runs `node.py` |
| `Dockerfile` | Rarely | Builds the image from everything above |
| `build.sh` | Rarely | Copies in the message package, builds the image, and saves `submission.tar` |

## Writing `policy.py`

`node.py` creates your `Policy` once at startup, passing it the path of the
`assets/` directory. After that it calls `Policy.act(state)` once for every new
observation, about 50 times a second, so each call has to finish within 20 ms.
`state` is the JSON described in
[Read the robot's state](../INSTRUCTIONS.md#2-read-the-robots-state), and `act`
returns 29 position targets in radians, in the
[joint order](../INSTRUCTIONS.md#joint-order). `node.py` fills in everything
else in the command, including the IDs, zero velocities and torques, and your
`KP` and `KD`.

The image runs Python 3.8, where the newest ONNX Runtime is 1.16.3. That
version loads ONNX files up to IR version 9, so if your export is newer, load
it with `onnx`, set `model.ir_version = 8`, and save it again.

The build calls `Policy(assets).act(...)` once on an all-zero state. That way a
missing import or a wrong output shape fails while you build, not on the robot.

If you copy this directory outside the repository, set `HIM_REPO` to the
repository path when running `build.sh` so it can find the message package.
