# Running someone else's G1 policy

**Start with the author's deployment instructions, not the file extension.** HIM Sim2Real Manager copies files and records data. It does not convert models or supply a universal policy runner. For your first physical session, have the G1 operator handle robot setup and controls while you handle recording.

## Recorder, policy, runner: three different things

- **Recorder:** included in this app. Click **Copy recorder to G1**; you do not choose a recorder file. It listens to robot telemetry (joint/body measurements) alongside the controller.
- **Policy/model:** the learned behavior. It needs the exact inputs and output interpretation used during training.
- **Runner + configuration:** the program that reads robot state, calls the model and sends commands; its configuration defines joint order, gains, timing and scaling. A model alone is not a complete deployment.

## Which file do I choose?

| What you received | What it means / what to do |
|---|---|
| Author-provided `.zip` or `.tar.gz` deployment bundle | Best handoff when several files belong together. Include the exported model, matching configuration, runner/version, dependencies and README with launch/stop instructions. Transfer it unchanged; this app does not extract it. |
| `.pt` or `.pth` | Could be an exported model OR a training checkpoint. Ask which. Unitree RL Gym's Python runner specifically calls `torch.jit.load`: it needs a compatible TorchScript export, not an arbitrary checkpoint. Renaming the file does not convert it. |
| `.onnx` | An exported model for a compatible ONNX runner. Keep companion files such as `.onnx.data` when supplied, plus its configuration and any reference motion/assets. Do not pass it to the RL Gym TorchScript runner. |
| `.yaml`, `.yml`, `.json` | Usually settings, not the learned policy. Keep them with the model and use the author's runner. |
| `.py`, `.sh`, executable, container or repository link | A custom deployment workflow. Get its exact version, required environment, working directory, and start/stop procedure. A repository link must be downloaded using the author's instructions before choosing a local file. |
| Only a model or a simulator checkpoint, no instructions | Ask the author for the deployment bundle below. Do not select a runner by guessing the extension. |

The picker transfers one nonempty file at a time, up to 16 GiB, with disk-space checks. It does not validate model contents or G1 compatibility. A successful checksum means the bytes arrived intact. Keep multi-file policies in one author-prepared archive so the directory structure stays together. Leave this upload empty if the operator already has the policy installed.

## Ask each Arena policy author for this

Copy this request into your usual conversation:

> Please send the real-G1 deployment bundle: exported model and all companion files; exact G1 variant and controlled joints; runner repository + commit/version; matching configuration; dependencies and required computer/OS; launch command with working directory; controller/start/stop procedure; required sensors/reference motions; and the simulation test command and result. Please say whether this exact bundle has run on physical G1 hardware. If you sent a training checkpoint, include the correct export procedure.

A 29-joint robot does not imply a 29-output policy. Joint mappings, observation order/history, action scaling, gains, control rate and sensor assumptions must match the author’s deployment. Do not change these just to make dimensions fit. The official RL Gym G1 configuration, for example, has a 12-action policy while separately setting arm/waist targets. [Unitree G1 configuration](https://github.com/unitreerobotics/unitree_rl_gym/blob/main/deploy/deploy_real/configs/g1.yaml).

## Choose the workflow the author actually used

### A. Author confirms Unitree RL Gym compatibility

Use their installed `unitree_rl_gym` checkout and environment. Put their matching configuration in its `deploy/deploy_real/configs/` folder. In that configuration, set `policy_path` to the actual exported TorchScript file, keeping all other policy-specific settings supplied by the author. Run the author’s matching MuJoCo sim2sim configuration first; simulation and real configurations live in different folders.

After the G1 operator completes physical preparation, the launch form **from the `unitree_rl_gym` repository root** is:

```bash
python deploy/deploy_real/deploy_real.py YOUR_ROBOT_INTERFACE author_g1.yaml
```

Replace the interface with the robot-facing network adapter on the computer running the runner, and `author_g1.yaml` with the supplied real-robot configuration filename. This command controls motors; it is not a connection test. Do not substitute the stock `g1.yaml` for an unknown policy. [Official runner source](https://github.com/unitreerobotics/unitree_rl_gym/blob/main/deploy/deploy_real/deploy_real.py) and [RL Gym workflow](https://github.com/unitreerobotics/unitree_rl_gym).

For this specific example, Unitree describes a supported/suspended robot, zero-torque state and debug mode before starting the controller. Its sequence uses Start to move to the default pose and A to begin policy control. Exiting with Select or Ctrl+C enters damping; the robot can fall. Have the operator manage support and the stop procedure. These controls are not universal across runners. Follow the full [physical deployment guide](https://github.com/unitreerobotics/unitree_rl_gym/blob/main/deploy/deploy_real/README.md), including its setup video.

### B. Author uses Unitree RL Lab or another ONNX deployment

Use that project's own runner, configuration and exported folder structure. Unitree RL Lab's documented G1-29DoF launch form, **from its built `deploy/robots/g1_29dof/build` directory**, is:

```bash
./g1_ctrl --network YOUR_ROBOT_INTERFACE
```

It selects policies through its configuration rather than accepting any uploaded model as a command-line argument. The operator must arrange controller ownership; Unitree says its onboard controller must be closed before this real-robot runner controls the robot. Do not guess a process to kill. Use the installed version's instructions and controls. [Official RL Lab deployment](https://github.com/unitreerobotics/unitree_rl_lab#deploy) and [policy selection configuration](https://github.com/unitreerobotics/unitree_rl_lab/blob/main/deploy/robots/g1_29dof/config/config.yaml).

Some Unitree ONNX exports also include a separate weights file, `policy.onnx.data`; send it with `policy.onnx` when present. [Official MuJoCo Lab deployment](https://github.com/unitreerobotics/unitree_rl_mjlab#4-real-deployment).

### C. Custom runner or unknown origin

Use the author's documented launch command in the environment and working directory they specify. The container example is a walker simulation test in [examples/submissions/walker/](examples/submissions/walker/). GR00T balance is a separate model in [examples/policies/balance/](examples/policies/balance/). The walker ONNX contract the checker still understands is described in [ONNX_BUNDLE.md](ONNX_BUNDLE.md). Neither file is a robot launcher. If the author's launch command is missing, the handoff is incomplete. A simulator upload to Arena does not, by itself, establish physical-G1 deployability. This manager records standard G1 telemetry independently of the policy format; it does not automatically adapt an Arena submission.

## Session order in this app

1. **Prepare:** identify the real G1 variant, policy bundle and runner with the operator. Confirm its simulation result, physical setup and stop procedure. Use their existing network/SDK setup; do not guess an IP or interface.
2. **Transfer:** click **Copy recorder to G1**. Optional: choose the author's bundle and **Transfer selected policy**. Give the displayed path to the operator; they extract/place the files according to its README. The transfer folder is not automatically the runner's working directory.
3. **Check:** open the robot recorder, select its robot-facing interface, check live state and complete the short stationary recording/export in CONNECTION_GUIDE.md. This verifies recording, not policy compatibility.
4. **Record:** name the run, enter the Arena policy ID, and put runner version/configuration and exact launch command in the notes. Start recording and wait for **RECORDING**; the operator then starts the policy using its own workflow.
5. **Return:** the operator stops the policy and secures the robot first. Stop recording, label the attempt, attach configuration, runner logs and available video, then **Prepare data for Arena → Download return ZIP**. A transferred policy is not automatically an attachment; include its checksum/path in notes and attach the model only if needed. Keep failures too.

Do not run two motor-commanding policy runners simultaneously. This recorder only subscribes to telemetry and can run alongside the selected controller. **Stop recording never stops the robot.**

Source links reviewed September 16, 2026. Commands are documented upstream examples, not hardware tests of these two unknown policies. Match the actual installed runner version before use.
