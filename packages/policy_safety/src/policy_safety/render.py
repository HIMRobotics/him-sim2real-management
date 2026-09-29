"""Offscreen videos of selected screening scenarios."""

import json
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

from .adapter import ADAPTERS
from .config import DEFAULT_SUITE, profile_config
from .evaluation import problem_class, run_scenario
from .joint_rules import DEFAULT_JOINT_PROFILE, JointRules
from .simulation import Simulation

DEFAULT_SCENARIOS = ("stand", "forward", "seated", "forward-pelvis-push")


def write_mp4(path, frames, fps):
    """Encode RGB frames as an H.264 MP4. Requires ffmpeg."""
    if not frames:
        raise ValueError("no frames to write")
    if fps <= 0:
        raise ValueError("fps must be positive")
    height, width = frames[0].shape[:2]
    payload = bytearray()
    for frame in frames:
        if frame.shape != (height, width, 3) or frame.dtype != np.uint8:
            raise ValueError("frames must be uint8 RGB arrays of one size")
        payload += frame.tobytes()
    encoder = shutil.which("ffmpeg") or str(Path(sys.executable).with_name("ffmpeg"))
    if not Path(encoder).is_file():
        raise ValueError("rendering mp4 needs ffmpeg on PATH")
    path = Path(path)
    result = subprocess.run(
        [
            encoder,
            "-y",
            "-loglevel",
            "error",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "rgb24",
            "-s",
            f"{width}x{height}",
            "-r",
            str(fps),
            "-i",
            "pipe:0",
            "-an",
            "-c:v",
            "libx264",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(path),
        ],
        input=bytes(payload),
        capture_output=True,
        check=False,
    )
    if result.returncode != 0:
        detail = result.stderr.decode(errors="replace").strip()
        raise RuntimeError(detail or "ffmpeg failed")
    return path


def _caption(frame, text):
    from PIL import Image, ImageDraw

    image = Image.fromarray(frame, mode="RGB")
    drawing = ImageDraw.Draw(image)
    drawing.rectangle((0, 0, image.width, 22), fill=(0, 0, 0))
    drawing.text((6, 4), text, fill=(255, 220, 80))
    return np.asarray(image)


def frames_from_trace(sim, trace_path, fps):
    """Replay saved integration states. One frame is kept per video tick."""
    import mujoco

    trace = np.load(trace_path, allow_pickle=False)
    states = trace["integration_state"]
    initial = trace["initial_integration_state"]
    if states.size:
        states = np.concatenate([initial.reshape(1, -1), states], axis=0)
    else:
        states = initial.reshape(1, -1)
    stride = max(1, round((1.0 / fps) / sim.physics_dt))
    chosen = states[::stride]
    renderer = mujoco.Renderer(sim.model, height=368, width=640)
    frames = []
    try:
        for state in chosen:
            mujoco.mj_setState(sim.model, sim.data, state, sim.state_spec)
            mujoco.mj_forward(sim.model, sim.data)
            camera = mujoco.MjvCamera()
            mujoco.mjv_defaultCamera(camera)
            pelvis = sim.data.xpos[sim.base]
            camera.lookat[:] = [pelvis[0], pelvis[1], 0.75]
            camera.distance = 2.8
            camera.azimuth = 135
            camera.elevation = -18
            renderer.update_scene(sim.data, camera=camera)
            frames.append(renderer.render().copy())
    finally:
        renderer.close()
    return frames


def _session(request):
    sim = Simulation(request["robot_config"])
    profile = profile_config(request["profile"])
    joint_rules = JointRules(
        sim.robot["joint_names"], request.get("joint_profile", DEFAULT_JOINT_PROFILE)
    )
    adapter = ADAPTERS[request["adapter"]](request["model"], sim.robot)
    clip = request.get("clip_to_stops", False)
    if type(clip) is not bool:
        raise ValueError("clip_to_stops must be a boolean")
    return sim, adapter, profile, joint_rules, clip


def _nominal(name, suite):
    return {
        "name": name,
        "kind": "nominal",
        "seed": suite["seeds"][0],
        "command": list(suite["commands"][name]),
        "seconds": suite["nominal_seconds"],
        "upright": True,
    }


def _run(sim, adapter, profile, joint_rules, clip, scenario, trace, checkpoints=()):
    sim_seed = scenario["seed"]
    if scenario["kind"] == "counterfactual":
        sim.reset(sim_seed, scenario["name"])
    else:
        sim.reset(sim_seed)
    adapter.reset()
    sim.set_torque_scale(scenario.get("torque_scale", 1.0))
    try:
        return run_scenario(
            sim,
            adapter,
            profile,
            scenario,
            trace,
            checkpoints,
            joint_rules,
            clip,
        )
    finally:
        sim.set_torque_scale(1.0)


def resolve_selection(name, suite):
    """Map a clip name to a gait, pose, shove strength, or torque scale."""
    if name in suite["commands"]:
        return {"mode": "command", "command": name}
    if name in suite["poses"]:
        return {"mode": "pose", "pose": name}
    levels = [level["name"] for level in suite["push_levels"]]
    if name == "forward-pelvis-push" or name in {
        f"forward-pelvis-push-{level}" for level in levels
    }:
        level = "small" if name == "forward-pelvis-push" else name.rsplit("-", 1)[-1]
        return {"mode": "push", "level": level}
    for command in suite["commands"]:
        prefix = f"{command}-"
        if not name.startswith(prefix):
            continue
        for scale in suite["torque_scales"]:
            if name[len(prefix) :] == str(round(scale * 100)):
                return {"mode": "torque", "command": command, "scale": scale}
    return None


def render_selection(request, names, fps=20):
    """Simulate the named scenarios and write one video each."""
    if fps <= 0:
        raise ValueError("fps must be positive")
    suite = DEFAULT_SUITE
    selected = [resolve_selection(name, suite) for name in names]
    if any(item is None for item in selected):
        missing = names[selected.index(None)]
        raise ValueError(
            f"unknown scenario {missing!r}; use a command, a pose, "
            "forward-pelvis-push-small, forward-pelvis-push-medium, "
            "forward-pelvis-push-heavy, stand-95, or stand-105"
        )
    output = Path(request["output"])
    output.mkdir(parents=True, exist_ok=True)
    (output / "traces").mkdir(exist_ok=True)
    sim, adapter, profile, joint_rules, clip = _session(request)
    written, results = [], []
    for index, (name, choice) in enumerate(zip(names, selected)):
        trace = output / "traces" / f"{index:05d}.npz"
        if choice["mode"] == "push":
            result = _render_pelvis_push(
                sim, adapter, profile, joint_rules, clip, suite, trace, choice["level"]
            )
        elif choice["mode"] == "torque":
            scenario = _nominal(choice["command"], suite)
            scenario["torque_scale"] = choice["scale"]
            checkpoints = tuple(
                round(suite["nominal_seconds"] * fraction / sim.dt)
                for fraction in suite["checkpoints"]
            )
            result, _ = _run(
                sim, adapter, profile, joint_rules, clip, scenario, trace, checkpoints
            )
        elif choice["mode"] == "pose":
            scenario = {
                "name": name,
                "kind": "counterfactual",
                "seed": suite["seeds"][0],
                "command": [0, 0, 0],
                "seconds": suite["counterfactual_seconds"],
                "upright": name == "standing",
            }
            midpoint = round(0.5 * scenario["seconds"] / sim.dt)
            result, _ = _run(
                sim, adapter, profile, joint_rules, clip, scenario, trace, (midpoint,)
            )
        else:
            scenario = _nominal(name, suite)
            checkpoints = tuple(
                round(suite["nominal_seconds"] * fraction / sim.dt)
                for fraction in suite["checkpoints"]
            )
            result, _ = _run(
                sim, adapter, profile, joint_rules, clip, scenario, trace, checkpoints
            )
        violation = (result.get("first_violation") or {}).get("reason", result["status"])
        kind = problem_class(result)
        frames = frames_from_trace(sim, trace, fps)
        caption = f"{name}  {kind}  {violation}"
        frames = [_caption(frame, caption) for frame in frames]
        video = write_mp4(output / f"{name}.mp4", frames, fps)
        _contact_sheet(output / f"{name}.png", frames)
        results.append(result)
        written.append(
            {
                "name": name,
                "video": video.name,
                "preview": f"{name}.png",
                "class": kind,
                "status": result["status"],
                "violent_motion": result.get("violent_motion", False),
                "first_violation": result.get("first_violation"),
                "simulated_seconds": result.get("simulated_seconds"),
                "frames": len(frames),
            }
        )
        print(
            f"{video.name}: {kind} {violation} "
            f"({result.get('simulated_seconds', 0):.2f}s, {len(frames)} frames)"
        )
    (output / "renders.json").write_text(json.dumps(written, indent=2) + "\n")
    return written, results


def _render_pelvis_push(sim, adapter, profile, joint_rules, clip, suite, trace, level):
    """Forward walk to the mid-run checkpoint, then a forward pelvis shove."""
    scenario = _nominal("forward", suite)
    checkpoints = tuple(
        round(suite["nominal_seconds"] * fraction / sim.dt)
        for fraction in suite["checkpoints"]
    )
    nominal_trace = trace.with_name(trace.stem + "-nominal.npz")
    _, snapshots = _run(
        sim, adapter, profile, joint_rules, clip, scenario, nominal_trace, checkpoints
    )
    step = round(suite["nominal_seconds"] * 0.5 / sim.dt)
    sim.restore(snapshots[step], adapter)
    branch = dict(
        scenario,
        kind="perturbation",
        seconds=suite["branch_seconds"],
        checkpoint_step=step,
        push={
            "body": "pelvis",
            "level": level,
            "force": [next(
                item["force_n"] for item in suite["push_levels"] if item["name"] == level
            ), 0.0, 0.0],
            "start": suite["push_start_seconds"],
            "duration": suite["push_seconds"],
        },
    )
    midpoint = round(0.5 * suite["branch_seconds"] / sim.dt)
    result, _ = run_scenario(
        sim, adapter, profile, branch, trace, (midpoint,), joint_rules, clip
    )
    return result


def _contact_sheet(path, frames):
    from PIL import Image

    picks = [frames[0], frames[len(frames) // 2], frames[-1]]
    sheet = Image.new("RGB", (picks[0].shape[1], picks[0].shape[0] * len(picks)))
    for index, frame in enumerate(picks):
        sheet.paste(Image.fromarray(frame, mode="RGB"), (0, index * frame.shape[0]))
    sheet.save(path)
