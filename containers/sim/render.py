"""Render a standard simulation trace to an annotated MP4."""

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "robot_safety"), str(ROOT / "packages/policy_safety/src")]

import mujoco
import numpy as np
from PIL import Image, ImageDraw
from policy_safety.render import write_mp4
from view import describe_startup


def render(trace_path, output, fps=12.5):
    """Replay recorded poses using the model embedded in the trace."""
    if not np.isfinite(fps) or fps <= 0:
        raise ValueError("fps must be finite and positive")
    with np.load(trace_path, allow_pickle=False) as trace:
        model = mujoco.MjModel.from_binary_path("trace.mjb", assets={"trace.mjb": trace["model"].tobytes()})
        times, poses = trace["time"], trace["qpos"]
        startup, stops = json.loads(str(trace["startup"])), json.loads(str(trace["stop"]))
    data = mujoco.MjData(model)
    base = model.body("pelvis").id
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.distance, camera.azimuth, camera.elevation = 2.8, 135, -18
    frames = []
    with mujoco.Renderer(model, width=400, height=240) as renderer:
        for at in np.arange(times[0], times[-1] + 1e-9, 1 / fps):
            index = min(np.searchsorted(times, at, side="right") - 1, len(times) - 1)
            data.qpos[:] = poses[index]
            mujoco.mj_forward(model, data)
            camera.lookat[:] = [*data.xpos[base, :2], 0.6]
            renderer.update_scene(data, camera=camera)
            renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
            renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
            frame = Image.fromarray(renderer.render()).resize((800, 480))
            draw = ImageDraw.Draw(frame)
            draw.rectangle((0, 0, 800, 45), fill=(18, 24, 34))
            draw.text((10, 5), f"{at:.2f} s  {describe_startup(startup[index])}", fill="white")
            if stops[index]:
                draw.text((10, 25), stops[index]["reason"], fill="orange")
            frames.append(np.asarray(frame).copy())
    return write_mp4(output, frames, fps)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--fps", type=float, default=12.5)
    args = parser.parse_args()
    print(render(args.trace, args.output, args.fps))
