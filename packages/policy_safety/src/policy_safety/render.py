"""Offscreen videos of selected screening scenarios."""

import json
import shutil
import subprocess
import sys
from functools import partial
from pathlib import Path
from tempfile import TemporaryFile

import numpy as np

from .config import DEFAULT_SUITE, positive
from .evaluation import load_session, problem_class, run_scenario


def write_mp4(path, frames, fps):
    """Encode RGB frames as an H.264 MP4. Requires ffmpeg."""
    if not frames:
        raise ValueError("no frames to write")
    positive(fps, "fps")
    height, width = frames[0].shape[:2]
    if any(
        frame.shape != (height, width, 3) or frame.dtype != np.uint8 for frame in frames
    ):
        raise ValueError("frames must be uint8 RGB arrays of one size")
    encoder = shutil.which("ffmpeg") or str(Path(sys.executable).with_name("ffmpeg"))
    if not Path(encoder).is_file():
        raise ValueError("rendering mp4 needs ffmpeg on PATH")
    path = Path(path)
    args = [
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
    ]
    with (
        TemporaryFile() as errors,
        subprocess.Popen(args, stdin=subprocess.PIPE, stderr=errors) as process,
    ):
        try:
            with process.stdin:
                for frame in frames:
                    process.stdin.write(frame.tobytes())
        except BrokenPipeError:
            pass
        except BaseException:
            process.kill()
            raise
        if process.wait():
            errors.seek(0)
            raise RuntimeError(
                errors.read().decode(errors="replace").strip() or "ffmpeg failed"
            )
    return path


def frames_from_trace(sim, trace_path, fps, caption=""):
    """Replay saved integration states. One frame is kept per video tick."""
    import mujoco
    from PIL import Image, ImageDraw

    stride = max(1, round((1.0 / fps) / sim.physics_dt))
    with np.load(trace_path, allow_pickle=False) as trace:
        chosen = [
            trace["initial_integration_state"],
            *trace["integration_state"][stride - 1 :: stride].copy(),
        ]
    renderer = mujoco.Renderer(sim.model, height=368, width=640)
    frames = []
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.distance, camera.azimuth, camera.elevation = 2.8, 135, -18
    try:
        for state in chosen:
            mujoco.mj_setState(sim.model, sim.data, state, sim.state_spec)
            mujoco.mj_forward(sim.model, sim.data)
            pelvis = sim.data.xpos[sim.base]
            camera.lookat[:] = [pelvis[0], pelvis[1], 0.75]
            renderer.update_scene(sim.data, camera=camera)
            frame = Image.fromarray(renderer.render(), mode="RGB")
            if caption:
                drawing = ImageDraw.Draw(frame)
                drawing.rectangle((0, 0, frame.width, 22), fill=(0, 0, 0))
                drawing.text((6, 4), caption, fill=(255, 220, 80))
            frames.append(np.asarray(frame))
    finally:
        renderer.close()
    return frames


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
        for scale in suite["torque_scales"]:
            if name == f"{command}-{round(scale * 100)}":
                return {"mode": "torque", "command": command, "scale": scale}
    return None


def render_selection(request, names, fps=20):
    """Simulate the named scenarios and write one video each."""
    positive(fps, "fps")
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
    sim, adapter, profile, joint_rules, _files = load_session(request)
    run = partial(
        run_scenario,
        sim,
        adapter,
        profile,
        joint_rules=joint_rules,
        continue_after_violation=True,
    )
    midpoint = round(0.5 * suite["nominal_seconds"] / sim.dt)
    forward_snapshot = None
    written, results = [], []
    for index, (name, choice) in enumerate(zip(names, selected)):
        trace = output / "traces" / f"{index:05d}.npz"
        pose = choice["mode"] == "pose"
        command = choice.get("command", "forward")
        scenario = {
            "name": name if pose else command,
            "kind": "counterfactual" if pose else "nominal",
            "seed": suite["seeds"][0],
            "command": [0, 0, 0] if pose else list(suite["commands"][command]),
            "seconds": suite["counterfactual_seconds" if pose else "nominal_seconds"],
            "upright": name == "standing" if pose else True,
        }
        sim.reset(scenario["seed"], name if pose else "standing")
        adapter.reset()
        if choice["mode"] == "torque":
            scenario["torque_scale"] = choice["scale"]
        elif choice["mode"] == "push":
            if forward_snapshot is None:
                _, snapshots = run(
                    scenario, trace.with_name(trace.stem + "-nominal.npz"), (midpoint,)
                )
                forward_snapshot = snapshots[midpoint]
            sim.restore(forward_snapshot, adapter)
            level = next(
                item for item in suite["push_levels"] if item["name"] == choice["level"]
            )
            scenario.update(
                kind="perturbation",
                seconds=suite["branch_seconds"],
                checkpoint_step=midpoint,
                push={
                    "body": "pelvis",
                    "level": level["name"],
                    "force": [level["force_n"], 0, 0],
                    "start": suite["push_start_seconds"],
                    "duration": suite["push_seconds"],
                },
            )
        checkpoints = (
            (midpoint,) if choice == {"mode": "command", "command": "forward"} else ()
        )
        result, snapshots = run(scenario, trace, checkpoints)
        if checkpoints:
            forward_snapshot = snapshots.get(midpoint)
        violation = (result.get("first_violation") or {}).get(
            "reason", result["status"]
        )
        kind = problem_class(result)
        frames = frames_from_trace(sim, trace, fps, f"{name}  {kind}  {violation}")
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


def _contact_sheet(path, frames):
    from PIL import Image

    picks = [frames[0], frames[len(frames) // 2], frames[-1]]
    sheet = Image.new("RGB", (picks[0].shape[1], picks[0].shape[0] * len(picks)))
    for index, frame in enumerate(picks):
        sheet.paste(Image.fromarray(frame, mode="RGB"), (0, index * frame.shape[0]))
    sheet.save(path)
