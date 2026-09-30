#!/usr/bin/env python3
"""Run parkour v4 evaluation, causality check, and result.mp4 with HUD slow-mo."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from parkour_env import CONTROL_DT, FrameMeta, evaluate_episode
from policy import NeutralPolicy, ScriptedParkourPolicy

ROOT = Path(__file__).resolve().parent
DEFAULT_SEED = 0
DEFAULT_HORIZON = 20.0
ARTIFACTS = ROOT / "artifacts"


def _font(size: int) -> ImageFont.ImageFont:
    for path in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationMono-Bold.ttf",
    ):
        p = Path(path)
        if p.exists():
            return ImageFont.truetype(str(p), size=size)
    return ImageFont.load_default()


def overlay_hud(frame: np.ndarray, meta: FrameMeta, title: str = "") -> np.ndarray:
    img = Image.fromarray(frame.copy())
    draw = ImageDraw.Draw(img, "RGBA")
    w, h = img.size
    panel_h = 110
    draw.rectangle([(0, 0), (w, panel_h)], fill=(8, 12, 22, 200))
    font = _font(15)
    font_sm = _font(12)
    px, py, pz = meta.pelvis_xyz
    stairs = meta.contact_stairs
    stair_str = " ".join(f"{k[-1]}={stairs.get(k, 0)}" for k in sorted(stairs))
    lines = [
        title or f"PHASE  {meta.phase}",
        f"t={meta.time:5.2f}s   pelvis  x={px:5.2f}  y={py:5.2f}  z={pz:5.2f}",
        f"foot stairs [{stair_str}]  distinct={meta.distinct_stair_foot_contacts}",
        (
            f"feet top={meta.contact_top_landing} finish={meta.contact_finish_pad}  "
            f"stood_top={meta.stood_on_top} air_gap={meta.aerial_gap} upright={meta.upright}"
        ),
        f"success={meta.success} fell={meta.fell} | FREE-BASE · pretrained+scripted · no kinematic assist",
    ]
    y = 4
    for i, line in enumerate(lines):
        fnt = font if i == 0 else font_sm
        draw.text((10, y), line, fill=(230, 240, 255, 255), font=fnt)
        y += 19 if i == 0 else 17
    if meta.success:
        draw.rectangle([(w - 118, 8), (w - 8, 36)], fill=(20, 140, 60, 220))
        draw.text((w - 108, 12), "SUCCESS", fill=(255, 255, 255, 255), font=font_sm)
    return np.asarray(img.convert("RGB"))


def make_title_card(size: tuple[int, int], text_lines: list[str], n: int = 20) -> list[np.ndarray]:
    w, h = size
    font = _font(22)
    font_sm = _font(15)
    frames = []
    for _ in range(n):
        img = Image.new("RGB", (w, h), (10, 14, 24))
        draw = ImageDraw.Draw(img)
        y = h // 2 - 48
        for i, line in enumerate(text_lines):
            fnt = font if i == 0 else font_sm
            bbox = draw.textbbox((0, 0), line, font=fnt)
            tw = bbox[2] - bbox[0]
            draw.text(((w - tw) // 2, y), line, fill=(220, 230, 245), font=fnt)
            y += 32 if i == 0 else 24
        frames.append(np.asarray(img))
    return frames


def build_result_video(
    frames: list[np.ndarray],
    metas: list[FrameMeta],
    path: Path,
    fps: float,
    slow_factor: int = 4,
) -> dict:
    if not frames or len(frames) != len(metas):
        raise RuntimeError(f"frame/meta mismatch: {len(frames)} vs {len(metas)}")

    h, w = frames[0].shape[:2]
    realtime: list[np.ndarray] = []
    for fr, meta in zip(frames, metas):
        realtime.append(overlay_hud(fr, meta, title=f"REAL-TIME  |  {meta.phase}"))

    n = len(frames)
    i0 = max(0, int(n * 0.08))
    i1 = min(n, int(n * 0.92))
    if i1 - i0 < 10:
        i0, i1 = 0, n
    slowmo: list[np.ndarray] = []
    for fr, meta in zip(frames[i0:i1], metas[i0:i1]):
        hud = overlay_hud(fr, meta, title=f"SLOW-MO ×{slow_factor}  |  {meta.phase}")
        for _ in range(slow_factor):
            slowmo.append(hud)

    bridge = make_title_card(
        (w, h),
        [
            "SLOW-MOTION REPLAY + HUD",
            "phase · foot-stair contacts · aerial gap · finish",
            "editorial overlay — physics unchanged",
        ],
        n=max(12, int(fps * 0.7)),
    )
    intro = make_title_card(
        (w, h),
        [
            "Parkour v4 — Real Physics Stairs + Gap",
            "Unitree G1 · FREE-BASE · pretrained walker + scripted",
            "kinematic assist REMOVED · not hardware-ready",
        ],
        n=max(15, int(fps * 0.9)),
    )

    all_frames = intro + realtime + bridge + slowmo
    path.parent.mkdir(parents=True, exist_ok=True)
    imageio.mimsave(path, all_frames, fps=fps, codec="libx264", quality=8)

    ARTIFACTS.mkdir(parents=True, exist_ok=True)
    mid_idx = len(intro) + len(realtime) + len(bridge) + len(slowmo) // 2
    mid_idx = min(mid_idx, len(all_frames) - 1)
    mid_path = ARTIFACTS / "midframe.png"
    Image.fromarray(all_frames[mid_idx]).save(mid_path)

    reader = imageio.get_reader(path)
    meta_v = reader.get_meta_data()
    nframes = reader.count_frames()
    reader.close()
    duration = nframes / float(fps)
    return {
        "path": str(path),
        "frames": int(nframes),
        "fps": float(fps),
        "duration_s": float(duration),
        "resolution": f"{w}x{h}",
        "recorded_sim_frames": len(frames),
        "realtime_frames": len(realtime),
        "slowmo_frames": len(slowmo),
        "midframe": str(mid_path),
    }


def run_causality(seed: int, horizon: float) -> dict:
    scripted = evaluate_episode(ScriptedParkourPolicy(), seed=seed, horizon_s=horizon, record=False)
    neutral = evaluate_episode(NeutralPolicy(), seed=seed, horizon_s=horizon, record=False)
    report = {
        "seed": seed,
        "horizon_s": horizon,
        "scripted": {
            "success": scripted.success,
            "call_count": scripted.call_count,
            "action_min": scripted.action_min,
            "action_max": scripted.action_max,
            "max_abs_action": scripted.max_abs_action,
            "contact_stairs": scripted.contact_stairs,
            "distinct_stair_foot_contacts": scripted.distinct_stair_foot_contacts,
            "contact_top_landing": scripted.contact_top_landing,
            "contact_finish_pad": scripted.contact_finish_pad,
            "stood_on_top": scripted.stood_on_top,
            "aerial_gap": scripted.aerial_gap,
            "joint_knee_range": scripted.joint_knee_range,
            "root_qpos_writes_after_reset": scripted.root_qpos_writes_after_reset,
            "total_platform_contacts": scripted.total_platform_contacts,
            "final_pelvis_xyz": scripted.final_pelvis_xyz,
            "min_pelvis_z": scripted.min_pelvis_z,
            "fell": scripted.fell,
        },
        "neutral": {
            "success": neutral.success,
            "call_count": neutral.call_count,
            "action_min": neutral.action_min,
            "action_max": neutral.action_max,
            "max_abs_action": neutral.max_abs_action,
            "total_platform_contacts": neutral.total_platform_contacts,
            "distinct_stair_foot_contacts": neutral.distinct_stair_foot_contacts,
            "stood_on_top": neutral.stood_on_top,
            "aerial_gap": neutral.aerial_gap,
            "final_pelvis_xyz": neutral.final_pelvis_xyz,
            "fell": neutral.fell,
        },
        "pass": bool(scripted.success and (not neutral.success)),
    }
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="G1 parkour runner v4 (free-base)")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument("--horizon", type=float, default=DEFAULT_HORIZON)
    parser.add_argument("--video", type=Path, default=ROOT / "result.mp4")
    parser.add_argument("--skip-video", action="store_true")
    parser.add_argument("--causality-only", action="store_true")
    args = parser.parse_args(argv)

    print("=== Causality evaluation ===")
    report = run_causality(args.seed, args.horizon)
    print(json.dumps(report, indent=2))
    if not report["pass"]:
        print("CAUSALITY FAILED", file=sys.stderr)
        return 1
    print("CAUSALITY PASS")

    if args.causality_only or args.skip_video:
        return 0

    print("=== Recording successful scripted episode (HUD + slow-mo) ===")
    render_every = 2
    fps = (1.0 / CONTROL_DT) / render_every
    result = evaluate_episode(
        ScriptedParkourPolicy(),
        seed=args.seed,
        horizon_s=args.horizon,
        record=True,
        render_every=render_every,
    )
    if not result.success:
        print("Recording episode failed success metric", file=sys.stderr)
        print(
            json.dumps(
                {
                    "contact_stairs": result.contact_stairs,
                    "distinct": result.distinct_stair_foot_contacts,
                    "top": result.contact_top_landing,
                    "finish": result.contact_finish_pad,
                    "stood_on_top": result.stood_on_top,
                    "aerial_gap": result.aerial_gap,
                    "final_pelvis": result.final_pelvis_xyz,
                    "fell": result.fell,
                },
                indent=2,
            ),
            file=sys.stderr,
        )
        return 1
    info = build_result_video(result.frames, result.frame_meta, args.video, fps=fps)
    print(
        json.dumps(
            {
                "episode": {
                    "success": result.success,
                    "call_count": result.call_count,
                    "contact_stairs": result.contact_stairs,
                    "distinct_stair_foot_contacts": result.distinct_stair_foot_contacts,
                    "contact_top_landing": result.contact_top_landing,
                    "contact_finish_pad": result.contact_finish_pad,
                    "stood_on_top": result.stood_on_top,
                    "aerial_gap": result.aerial_gap,
                    "joint_knee_range": result.joint_knee_range,
                    "root_qpos_writes_after_reset": result.root_qpos_writes_after_reset,
                    "final_pelvis_xyz": result.final_pelvis_xyz,
                },
                "video": info,
            },
            indent=2,
        )
    )
    print(f"Wrote {args.video}")
    print(f"Wrote {info['midframe']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
