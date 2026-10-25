"""Write a browser replay of the recorded MuJoCo scene."""

import json
import shutil
import struct
import sys
import uuid
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "robot_safety"))

from hardware_constants import G1_29_JOINT_NAMES
from plant import load_model
from viewer_assets import install

PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<title>G1 sim replay</title>
<style>
  html, body { margin: 0; height: 100%; background: #f4f4f4; overflow: hidden; }
  #status { position: absolute; left: 12px; top: 12px; max-width: calc(54vw - 50px); }
  #hud { font: 14px sans-serif; color: #222; }
  #start {
    position: absolute; left: 50%; top: 18%; transform: translateX(-50%);
    font: 600 18px sans-serif; padding: 12px 28px; border: 0; border-radius: 8px;
    background: #1a1a1a; color: #fff; cursor: pointer;
  }
  #start:disabled { background: #888; cursor: default; }
  #playback { position: absolute; bottom: 18px; right: 12px; z-index: 2;
    display: flex; gap: 8px; }
  #playback button { padding: 9px 12px; border: 1px solid #ccd1d5; border-radius: 6px;
    background: #fff; color: #263238; font: 14px system-ui, sans-serif; cursor: pointer; }
  #playback button:disabled { opacity: 0.5; cursor: default; }
  #thrash { position: absolute; right: 12px; top: 12px; width: min(450px, 46vw);
    background: #fffffff0; border: 1px solid #d7dce0; border-radius: 10px;
    padding: 14px; font: 13px system-ui, sans-serif; color: #263238; }
  #thrash h2 { margin: 0 0 8px; font-size: 17px; }
  #thrash-status { font-weight: 600; margin-bottom: 8px; }
  #thrash-peak { font-size: 19px; font-variant-numeric: tabular-nums; margin: 8px 0; }
  #thrash small { display: block; line-height: 1.5; margin-top: 8px; }
  .joint-readings { max-height: 245px; overflow: auto; }
  #thrash table { width: 100%; border-collapse: collapse; font-variant-numeric: tabular-nums; }
  #thrash th, #thrash td { padding: 5px 3px; text-align: right; border-bottom: 1px solid #eee; }
  #thrash th { position: sticky; top: 0; background: #fff; font-size: 11px; }
  #thrash th:first-child, #thrash td:first-child { text-align: left; }
  #thrash tr.flagged { background: #fff0da; }
  #stop, #warn { position: absolute; left: 12px; max-width: min(670px, 75vw);
    border-radius: 6px; padding: 12px 16px; font: 14px system-ui, sans-serif; white-space: pre-line; }
  #stop { bottom: 18px; background: #fff4eeed; border-left: 5px solid #b54a22; color: #682811; }
  #warn { bottom: 18px; background: #fff8e8; border-left: 5px solid #c48a12; color: #6a4b08; }
  #warn.raised { bottom: 108px; }
  #startup { margin-top: 10px; background: #eef5fdf0; border-left: 5px solid #1565c0;
    color: #0d3c73; border-radius: 6px; padding: 10px 16px;
    font: 600 15px system-ui, sans-serif; font-variant-numeric: tabular-nums; }
</style>
</head>
<body>
<div id="status">
  <div id="hud">loading</div>
  <div id="startup" role="status" hidden></div>
</div>
<div id="warn" role="status" hidden></div>
<div id="stop" role="status" hidden></div>
<div id="playback" aria-label="Replay controls">
  <button id="go-start" type="button" title="Jump to the first frame and pause (Home)" aria-keyshortcuts="Home">Go to start · Home</button>
  <button id="restart-replay" type="button" title="Replay from the beginning (R)" aria-keyshortcuts="R">Restart replay · R</button>
</div>
<aside id="thrash">
  <h2>Thrash readings</h2>
  <div id="thrash-status">Waiting for telemetry</div>
  <div id="thrash-peak"></div>
  <div class="joint-readings"><table>
    <thead><tr><th>Joint</th><th>Osc. RMS</th><th>Motion RMS</th><th>Weight</th><th>Energy share</th></tr></thead>
    <tbody id="thrash-joints"></tbody>
  </table></div>
  <small id="thrash-thresholds"></small>
  <small id="thrash-event"></small>
  <small>Whole-body score is the weighted RMS of joint oscillations. Weights express relative motion risk,
  not measured inertia. Energy share shows each joint's contribution. Space pauses, arrows step frames.
  Home goes to the first frame. R restarts playback without rerunning the simulation.</small>
</aside>
<script type="importmap">
{ "imports": { "three": "./three.module.js" } }
</script>
<script type="module">
import * as THREE from "three";
import { OrbitControls } from "./OrbitControls.js";
import { STLLoader } from "./STLLoader.js";

const motion = await (await fetch("motion.json")).json();
const hud = document.getElementById("hud");
motion.thrash ||= [];
motion.stop ||= [];
motion.warning ||= [];
motion.startup ||= [];
function showStartup(index) {
  const text = motion.startup[index] || "";
  const panel = document.getElementById("startup");
  panel.hidden = !text;
  panel.textContent = text;
}
function showWarning(index) {
  const warning = motion.warning[index] || "";
  const panel = document.getElementById("warn");
  const listed = String(motion.summary || "").includes("Limit warning");
  panel.hidden = !warning || listed;
  panel.classList.toggle("raised", !document.getElementById("stop").hidden);
  if (!panel.hidden) panel.textContent = warning;
}
function showStop(index) {
  const stop = motion.stop[index] || motion.run_stop;
  const panel = document.getElementById("stop");
  panel.hidden = !stop;
  if (!stop) return;
  const before = stop.time_secs != null && motion.time[index] < stop.time_secs;
  panel.textContent = stop.description + (before ? `\nThis frame precedes the stop.` : "");
}
const readings = document.getElementById("thrash-joints");
const thrashRows = (motion.joint_names || []).map(() => {
  const row = readings.insertRow();
  for (let i = 0; i < 5; i++) row.insertCell();
  return row;
});
function showThrash(index) {
  const sample = motion.thrash[index];
  const status = document.getElementById("thrash-status");
  const peak = document.getElementById("thrash-peak");
  const thresholds = document.getElementById("thrash-thresholds");
  const event = document.getElementById("thrash-event");
  const metrics = sample?.metrics || {};
  const available = sample?.mode !== "off" && metrics.weighted_rms != null;
  thrashRows.forEach(row => { row.hidden = !available; });
  peak.textContent = "";
  thresholds.textContent = "";
  event.textContent = sample?.event ? `First detection: ${sample.event.reason}` : "No detection recorded so far";
  status.style.color = metrics.reason ? "#a64b00" : "#263238";
  status.textContent = !sample ? "No telemetry for this frame" : sample.mode === "off"
    ? "Detector off" : `${sample.mode === "monitor" ? "Monitor · records without stopping" : "Enforce · stops on detection"} · ${metrics.reason || "no current detection"}`;
  if (!available) {
    if (sample && sample.mode !== "off") peak.textContent = "Warming up";
    if (!sample) event.textContent = "";
    return;
  }
  const contribution = metrics.energy_share;
  const order = contribution.map((_, i) => i).sort((a, b) => contribution[b] - contribution[a]);
  peak.textContent = `Whole-body oscillation: ${metrics.weighted_rms.toFixed(3)} / ${metrics.weighted_rms_limit.toFixed(2)} · ${(1000 * metrics.persistence_elapsed_secs).toFixed(0)} ms`;
  order.forEach((joint, slot) => {
    const row = thrashRows[slot];
    const values = [motion.joint_names[joint].replaceAll("_", " "),
      metrics.oscillatory_rms[joint].toFixed(3), metrics.activity_rms[joint].toFixed(3),
      metrics.joint_weights[joint].toFixed(2), `${(100 * metrics.energy_share[joint]).toFixed(1)}%`];
    values.forEach((value, column) => { row.cells[column].textContent = value; });
    row.classList.toggle("flagged", metrics.joint_indices.includes(joint));
  });
  const t = sample.thresholds;
  if (t) thresholds.textContent = `Stop threshold: whole-body RMS ≥ ${t.weighted_rms_limit.toFixed(2)} for ${1000 * t.persistence_secs} ms continuously. Timer resets below threshold.`;
  if (sample.control_path === "direct") thresholds.textContent += " Direct command: submission-command checks bypassed.";
}
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xf4f4f4);
scene.add(new THREE.AmbientLight(0xffffff, 0.65));
const sun = new THREE.DirectionalLight(0xffffff, 0.9);
sun.position.set(2, -2, 4);
scene.add(sun);
const grid = new THREE.GridHelper(8, 16, 0xbbbbbb, 0xdddddd);
grid.rotation.x = Math.PI / 2;
scene.add(grid);
let trailGeom = new THREE.BufferGeometry();
const trail = new THREE.Line(trailGeom, new THREE.LineBasicMaterial({ color: 0x1565c0 }));
scene.add(trail);
trail.frustumCulled = false;
const camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.05, 50);
camera.up.set(0, 0, 1);
camera.position.set(2.2, -2.2, 1.4);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 0.8);
controls.update();

const loader = new STLLoader();
const parts = [];
async function geometryFor(geom) {
  const s = geom.size;
  let geometry;
  switch (geom.type) {
    case "mesh": return await loader.loadAsync(geom.file);
    case "plane": return new THREE.PlaneGeometry(2 * (s[0] || 10), 2 * (s[1] || 10));
    case "box": return new THREE.BoxGeometry(2 * s[0], 2 * s[1], 2 * s[2]);
    case "sphere": return new THREE.SphereGeometry(s[0], 24, 16);
    case "ellipsoid":
      geometry = new THREE.SphereGeometry(1, 24, 16);
      geometry.scale(s[0], s[1], s[2]);
      return geometry;
    case "capsule": geometry = new THREE.CapsuleGeometry(s[0], 2 * s[1], 8, 24); break;
    case "cylinder": geometry = new THREE.CylinderGeometry(s[0], s[0], 2 * s[1], 24); break;
    default: throw new Error(`Unsupported geometry: ${geom.type}`);
  }
  geometry.rotateX(Math.PI / 2);
  return geometry;
}
for (const geom of motion.geoms) {
  const geometry = await geometryFor(geom);
  const material = new THREE.MeshStandardMaterial({
    color: new THREE.Color(...geom.color), opacity: geom.opacity,
    transparent: geom.opacity < 1, side: THREE.DoubleSide, metalness: 0.15, roughness: 0.6
  });
  const mesh = new THREE.Mesh(geometry, material);
  mesh.matrixAutoUpdate = false;
  scene.add(mesh);
  parts.push(mesh);
}

const goStartButton = document.getElementById("go-start");
const restartButton = document.getElementById("restart-replay");
let frame = 0;
let mode = motion.live ? "idle" : "replay";
let paused = false;
function show(index) {
  goStartButton.disabled = restartButton.disabled = mode === "idle" || motion.frames < 2;
  if (!motion.frames) return;
  frame = (index + motion.frames) % motion.frames;
  const focus = motion.focus[frame];
  if (mode === "idle") controls.target.set(focus[0], focus[1], focus[2]);
  parts.forEach((mesh, i) => {
    const pose = motion.geoms[i].poses[frame];
    mesh.matrix.set(
      pose[0], pose[1], pose[2], pose[9],
      pose[3], pose[4], pose[5], pose[10],
      pose[6], pose[7], pose[8], pose[11],
      0, 0, 0, 1
    );
  });
  trailGeom.setDrawRange(0, frame + 1);
  const playback = mode === "idle" ? "ready" : paused ? "paused" : mode === "live" ? "live" : "playing";
  hud.textContent = `${motion.summary}\n${playback}  ${motion.time[frame].toFixed(2)} s  frame ${frame + 1} / ${motion.frames}`;
  hud.style.whiteSpace = "pre-line";
  showThrash(frame);
  showStop(frame);
  showWarning(frame);
  showStartup(frame);
}
let trailCount = 0;
function updateTrail() {
  let attribute = trailGeom.getAttribute("position");
  if (!attribute || attribute.count < motion.frames) {
    const capacity = Math.max(motion.frames, attribute ? attribute.count * 2 : 1);
    const positions = new Float32Array(capacity * 3);
    if (attribute) positions.set(attribute.array);
    trailGeom.dispose();
    trailGeom = new THREE.BufferGeometry();
    trail.geometry = trailGeom;
    attribute = new THREE.BufferAttribute(positions, 3);
    trailGeom.setAttribute("position", attribute);
  }
  for (let i = trailCount; i < motion.frames; i++) {
    attribute.setXYZ(i, motion.focus[i][0], motion.focus[i][1], 0.02);
  }
  if (motion.frames > trailCount) {
    attribute.addUpdateRange(trailCount * 3, (motion.frames - trailCount) * 3);
    attribute.needsUpdate = true;
  }
  trailCount = motion.frames;
  trailGeom.setDrawRange(0, Math.min(frame + 1, motion.frames));
}
updateTrail();
function remember(frames, replace) {
  if (replace) {
    trailCount = 0;
    motion.time = [];
    motion.focus = [];
    motion.thrash = [];
    motion.stop = [];
    motion.warning = [];
    motion.startup = [];
    for (const geom of motion.geoms) geom.poses = [];
  }
  for (const item of frames) {
    motion.time.push(item.time);
    motion.focus.push(item.focus);
    motion.thrash.push(item.thrash || null);
    motion.stop.push(item.stop || null);
    motion.warning.push(item.warning || "");
    motion.startup.push(item.startup || "");
    item.poses.forEach((pose, index) => motion.geoms[index].poses.push(pose));
  }
  motion.frames = motion.time.length;
  updateTrail();
}
if (motion.live) {
  const start = document.createElement("button");
  start.id = "start";
  start.textContent = "Start";
  document.body.appendChild(start);
  let synced = false;
  start.addEventListener("click", async () => {
    // A tab left open from an earlier run still shows that run's page.
    try {
      const current = await (await fetch("/frames?after=0", {cache: "no-store"})).json();
      if (current.page_id && current.page_id !== motion.page_id) {
        location.reload();
        return;
      }
    } catch (error) {}
    start.disabled = true;
    start.textContent = "Starting";
    motion.summary = "Starting the policy";
    show(0);
    const response = await fetch("/start", {method: "POST"});
    if (!response.ok) {
      motion.summary = "Start failed";
      start.disabled = false;
      start.textContent = "Start";
      show(0);
      return;
    }
    const poll = async () => {
      let body;
      try {
        const after = synced ? motion.frames : 0;
        const response = await fetch("/frames?after=" + after);
        if (!response.ok) throw new Error("frames");
        body = await response.json();
      } catch (error) {
        setTimeout(poll, 200);
        return;
      }
      if (body.reset || !synced) {
        if (body.frames.length) {
          remember(body.frames, true);
          synced = true;
        }
      } else if (body.frames.length) {
        remember(body.frames, false);
      }
      if (synced) {
        if (mode === "idle") {
          mode = "live";
          frame = 0;
          last = performance.now();
          show(0);
        }
        start.remove();
      }
      if (body.summary) motion.summary = body.summary;
      if (body.stop) motion.run_stop = body.stop;
      if (body.done) mode = "replay";
      if (motion.frames) show(frame);
      if (body.done && !synced) start.remove();
      if (!body.done) setTimeout(poll, 50);
    };
    poll();
  });
}
function goToStart(play = false) {
  if (mode === "idle" || motion.frames < 2) return;
  paused = !play;
  last = performance.now();
  show(0);
}
goStartButton.addEventListener("click", () => goToStart());
restartButton.addEventListener("click", () => goToStart(true));
window.addEventListener("keydown", (event) => {
  if (event.ctrlKey || event.metaKey || event.altKey || event.target?.isContentEditable
      || /^(INPUT|TEXTAREA|SELECT)$/.test(event.target?.tagName || "")) return;
  if (event.code === "Home" || event.code === "KeyR") {
    event.preventDefault();
    if (!event.repeat) goToStart(event.code === "KeyR");
  } else if (event.code === "Space") { paused = !paused; event.preventDefault(); }
  else if (event.code === "ArrowLeft") { paused = true; show(frame - 1); event.preventDefault(); }
  else if (event.code === "ArrowRight") { paused = true; show(frame + 1); event.preventDefault(); }
});
window.addEventListener("resize", () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});
show(0);
let last = performance.now();
function tick(now) {
  requestAnimationFrame(tick);
  if (!paused && motion.frames > 1 && mode !== "idle") {
    // Frames are 20 ms apart and the display refreshes every 8 or 17 ms, so
    // several frames can fall due at once. Carrying the remainder keeps
    // playback in real time. After a stall, skip ahead rather than race.
    last = Math.max(last, now - 100);
    let next = frame;
    while (true) {
      if (mode === "live" && next >= motion.frames - 1) { last = now; break; }
      const after = (next + 1) % motion.frames;
      const gap = 1000 * (motion.time[after] - motion.time[next]);
      const delay = after === 0 ? 0 : Number.isFinite(gap) ? Math.max(0, gap) : 20;
      if (now - last < delay) break;
      last += delay;
      next = after;
      if (next === 0) break;
    }
    if (next !== frame) show(next);
  }
  controls.update();
  renderer.render(scene, camera);
}
requestAnimationFrame(tick);
</script>
</body>
</html>
"""


def write_compiled_stl(model, mesh_id, path):
    """Write the mesh MuJoCo actually simulates, in the geom's local frame."""
    vert_adr = int(model.mesh_vertadr[mesh_id])
    vert_num = int(model.mesh_vertnum[mesh_id])
    face_adr = int(model.mesh_faceadr[mesh_id])
    face_num = int(model.mesh_facenum[mesh_id])
    vertices = np.asarray(
        model.mesh_vert[vert_adr : vert_adr + vert_num], dtype=np.float32
    )
    faces = np.asarray(model.mesh_face[face_adr : face_adr + face_num], dtype=np.int32)
    triangles = vertices[faces]
    write_triangles_stl(triangles, path)


def write_triangles_stl(triangles, path):
    """Write triangle vertices as a binary STL."""
    normals = np.cross(
        triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]
    )
    length = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = np.divide(normals, length, out=np.zeros_like(normals), where=length > 0)
    with Path(path).open("wb") as stream:
        stream.write(b"g1 sim mesh".ljust(80, b"\0"))
        stream.write(struct.pack("<I", len(triangles)))
        stream.writelines(
            struct.pack("<12fH", *normal, *triangle.reshape(-1), 0)
            for normal, triangle in zip(normals, triangles)
        )


def write_heightfield_stl(model, field_id, path):
    """Export the terrain surface from the compiled heightfield."""
    rows, cols = int(model.hfield_nrow[field_id]), int(model.hfield_ncol[field_id])
    start = int(model.hfield_adr[field_id])
    size = model.hfield_size[field_id]
    x, y = np.meshgrid(
        np.linspace(-size[0], size[0], cols), np.linspace(-size[1], size[1], rows)
    )
    z = model.hfield_data[start : start + rows * cols].reshape(rows, cols) * size[2]
    vertices = np.stack((x, y, z), axis=-1)
    a, b = vertices[:-1, :-1], vertices[:-1, 1:]
    c, d = vertices[1:, :-1], vertices[1:, 1:]
    triangles = np.concatenate(
        (
            np.stack((a, b, c), axis=-2).reshape(-1, 3, 3),
            np.stack((b, d, c), axis=-2).reshape(-1, 3, 3),
        )
    )
    write_triangles_stl(triangles, path)


def visible_geoms(model):
    """Geoms the replay draws, in page order. Group 3 collision shells are skipped."""
    base = model.body("pelvis").id
    robot_bodies = {base}
    for body_id in range(base + 1, model.nbody):
        if int(model.body_parentid[body_id]) in robot_bodies:
            robot_bodies.add(body_id)
    geoms = []
    for geom_id in range(model.ngeom):
        if (
            int(model.geom_bodyid[geom_id]) in robot_bodies
            and model.geom_group[geom_id] == 3
        ):
            continue
        material = int(model.geom_matid[geom_id])
        rgba = model.mat_rgba[material] if material >= 0 else model.geom_rgba[geom_id]
        if rgba[3] <= 0:
            continue
        kind = (
            mujoco.mjtGeom(int(model.geom_type[geom_id]))
            .name.removeprefix("mjGEOM_")
            .lower()
        )
        geoms.append((geom_id, kind, rgba.tolist()))
    return base, geoms


def pose_frame(model, data, base, geoms, qpos, time):
    """One rendered frame: pelvis focus and each geom's rotation plus position."""
    data.qpos[:] = qpos
    mujoco.mj_forward(model, data)
    focus = [round(float(value), 5) for value in data.xpos[base]]
    poses = []
    for geom_id, _kind, _rgba in geoms:
        rotation = data.geom_xmat[geom_id]
        position = data.geom_xpos[geom_id]
        poses.append(
            [round(float(value), 5) for value in rotation]
            + [round(float(value), 5) for value in position]
        )
    return {"time": float(time), "focus": focus, "poses": poses}


def write_page(dest, model, geoms, frames, summary, live=False, stop=None):
    """Write index.html, motion.json, and the mesh files the page loads."""
    assets = install()
    dest = Path(dest)
    mesh_dir = dest / "meshes"
    if dest.exists():
        shutil.rmtree(dest)
    mesh_dir.mkdir(parents=True)
    payload = {
        "frames": len(frames),
        "time": [frame["time"] for frame in frames],
        "focus": [frame["focus"] for frame in frames],
        "summary": summary,
        "live": live,
        "page_id": uuid.uuid4().hex,
        "thrash": [frame.get("thrash") for frame in frames],
        "stop": [frame.get("stop") for frame in frames],
        "startup": [frame.get("startup") or "" for frame in frames],
        "run_stop": describe_stop(stop),
        "joint_names": list(G1_29_JOINT_NAMES),
        "geoms": [],
    }
    written = set()
    for slot, (geom_id, kind, rgba) in enumerate(geoms):
        geom = {
            "name": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id),
            "type": kind,
            "size": model.geom_size[geom_id].tolist(),
            "color": rgba[:3],
            "opacity": rgba[3],
            "poses": [frame["poses"][slot] for frame in frames],
        }
        if kind in ("mesh", "hfield"):
            asset_id = int(model.geom_dataid[geom_id])
            filename = f"{kind}_{asset_id}.stl"
            if filename not in written:
                writer = write_compiled_stl if kind == "mesh" else write_heightfield_stl
                writer(model, asset_id, mesh_dir / filename)
                written.add(filename)
            geom.update(type="mesh", file=f"meshes/{filename}")
        payload["geoms"].append(geom)
    dest.joinpath("motion.json").write_text(json.dumps(payload))
    dest.joinpath("index.html").write_text(PAGE)
    for asset in assets:
        shutil.copy2(asset, dest / asset.name)
    return dest


def stop_from_report(report):
    """Read current or legacy stop metadata."""
    if report.get("stop"):
        return report["stop"]
    if report.get("reason"):
        return {"reason": report["reason"], "detail": report.get("detail"),
                "time_secs": report.get("stop_time_seconds")}
    return None


def describe_startup(startup):
    """Banner text for startup balance and the hand-off to the policy."""
    if startup and startup["phase"] in ("recovery", "damping"):
        return "Holosoma recovery active" if startup["phase"] == "recovery" else "Hardware safety: damping"
    if not startup or startup["phase"] == "fault":
        return ""
    elapsed, handoff = startup["elapsed_secs"], startup["handoff_at_secs"]
    if handoff is None:
        left = startup["launch_secs"] - elapsed
        if left > 0:
            return (f"Startup balance: {elapsed:.1f} s in, {left:.1f} s left "
                    "before the policy takes over")
        return f"Startup balance: {elapsed:.1f} s in, waiting for the robot to settle"
    blending = elapsed - handoff
    if blending < startup["blend_secs"]:
        return f"Handing over to the policy: {blending:.1f} of {startup['blend_secs']:.1f} s"
    return f"Policy in control since {handoff:.1f} s"


def describe_stop(stop):
    """Attach a readable cause without changing recorded fault data."""
    if not stop:
        return None
    reason = stop["reason"]
    labels = {
        "joint_velocity_limit": "Measured joint velocity (q dot) exceeded limit",
        "initial_joint_target_velocity_limit": "Initial target velocity exceeded limit",
        "joint_torque_limit": "Torque demand or measured torque exceeded limit",
        "joint_feedforward_torque_limit": "Feedforward torque exceeded limit",
        "base_angular_speed_limit": "Base angular speed exceeded limit",
        "fall": "Standing fall check triggered",
        "nonfinite_simulation": "Simulation produced non-finite state",
        "invalid_command": "Malformed command or command sanity bound exceeded",
        "thrashing_energy": "Sustained whole-body oscillation detected",
    }
    when = stop.get("time_secs")
    lines = ["Policy stopped" + (f" at {when:.3f} s" if when is not None else ""),
             f"{labels.get(reason, reason.replace('_', ' '))} [{reason}]"]
    detail = stop.get("detail")
    if isinstance(detail, dict):
        joint = detail.get("joint")
        index = detail.get("joint_index")
        if joint is None and type(index) is int and 0 <= index < len(G1_29_JOINT_NAMES):
            joint = G1_29_JOINT_NAMES[index]
        if joint is not None:
            if reason == "base_angular_speed_limit" and str(joint) in ("0", "1", "2"):
                lines.append("Base axis: " + "xyz"[int(joint)])
            else:
                lines.append("Joint: " + str(joint))
        indices = detail.get("joint_indices", [])
        if indices:
            lines.append("Joints: " + ", ".join(
                G1_29_JOINT_NAMES[i] if type(i) is int and 0 <= i < len(G1_29_JOINT_NAMES)
                else str(i) for i in indices))
        for key, label in (("value", "Value"), ("limit", "Limit"), ("height", "Height")):
            if key in detail:
                value = detail[key]
                value = f"{value:.4g}" if type(value) in (int, float) else str(value)
                lines.append(f"{label}: {value} {detail.get('units', 'm' if key == 'height' else '')}".strip())
        for key in ("field", "detail", "torque_kind", "rejection", "bounds"):
            if key in detail:
                lines.append(f"{key.replace('_', ' ').capitalize()}: {detail[key]}")
        if reason == "thrashing_energy":
            if "weighted_rms" in detail:
                lines.append(f"Whole-body RMS: {detail['weighted_rms']:.3f} / "
                             f"{detail['weighted_rms_limit']:.3f}, sustained "
                             f"{1000 * detail['persistence_elapsed_secs']:.0f} ms")
            for index in indices:
                values = []
                for key, label in (("oscillatory_rms", "osc RMS"), ("activity_rms", "motion RMS"),
                                   ("joint_weights", "weight"), ("energy_share", "energy fraction")):
                    samples = detail.get(key, [])
                    if type(index) is int and 0 <= index < len(samples):
                        values.append(f"{label}={samples[index]:.3f}")
                if values:
                    lines.append(f"Joint {index}: " + ", ".join(values))
    elif detail:
        lines.append(str(detail))
    return dict(stop, description="\n".join(lines))


def limit_warning_text(warnings):
    """One line naming the worst limit exceedance. The run was not stopped for it."""
    items = []
    for joint, fields in (warnings or {}).items():
        if not isinstance(fields, dict):
            continue
        for item in fields.get("limits", ()):
            items.append((joint, item))
    if not items:
        return ""
    joint, item = max(items, key=lambda pair: (
        pair[1]["value"] / pair[1]["limit"] if pair[1]["limit"] else pair[1]["value"]))
    kind = item.get("torque_kind")
    kind_text = f" {kind}" if kind else ""
    extra = f" and {len(items) - 1} more" if len(items) > 1 else ""
    return (f"Limit warning: {joint}{kind_text} {item['value']:.4g} > "
            f"{item['limit']:.4g} {item['units']}{extra}. Run continued.")


def report_summary(report, safety_checks=True):
    """HUD text for a finished run."""
    summary = f"{report.get('policy') or 'Simulation'} | Safety checks: "
    if report.get("control_path") == "direct":
        summary += "PARTIAL" if report.get("safety_checks", safety_checks) else "OFF"
        summary += " (direct control, submission checks bypassed)"
    else:
        summary += "ON" if report.get("safety_checks", safety_checks) else "OFF"
    summary += " | " + str(report.get("status", ""))
    if "thrash" in report:
        thrash = report["thrash"]
        summary += " | Thrash: " + thrash["mode"]
        if thrash.get("event"):
            summary += " (" + thrash["event"]["reason"] + ")"
    if report.get("reason"):
        summary += "\n" + describe_stop(stop_from_report(report))["description"]
        if report.get("host_stall"):
            summary += (f"\nInconclusive: the simulator was stalled for "
                        f"{report['host_stall']['longest_gap_secs']:.2f} s at the stop. "
                        "Free memory on the host and rerun.")
    elif report.get("status") == "no_command":
        summary += "\nNo policy commands were accepted before the run ended."
    warning = limit_warning_text(report.get("warnings"))
    if warning:
        summary += "\n" + warning
    motion = report.get("motion", {})
    if "fell" in motion:
        summary += "\nFall recorded: " + ("yes" if motion["fell"] else "no")
    return summary


def export_idle(dest, pose="standing", scene=None):
    """Write the start page: the robot at its initial pose, waiting for Start."""
    from plant import Plant

    plant = Plant(pose, scene=scene or None, check_fall=False)
    base, geoms = visible_geoms(plant.model)
    frame = pose_frame(
        plant.model, plant.data, base, geoms, plant.data.qpos.copy(), 0.0
    )
    return write_page(
        dest,
        plant.model,
        geoms,
        [frame],
        "Press Start to run the policy.",
        live=True,
    )


def export_replay(trace_path, dest):
    """Build dest/index.html, motion.json, and the STL files the page loads."""
    trace_path, dest = Path(trace_path), Path(dest)
    with np.load(trace_path, allow_pickle=False) as trace:
        qpos = trace["qpos"]
        times = trace["time"]
        thrash = json.loads(str(trace["thrash"])) if "thrash" in trace else [None] * len(times)
        stops = json.loads(str(trace["stop"])) if "stop" in trace else [None] * len(times)
        startups = (json.loads(str(trace["startup"])) if "startup" in trace
                    else [None] * len(times))
        safety_checks = (
            bool(trace["safety_checks"]) if "safety_checks" in trace else True
        )
        metadata = {"safety_checks": safety_checks}
        if "control_path" in trace:
            metadata["control_path"] = str(trace["control_path"])
        if "checks" in trace:
            metadata["checks"] = json.loads(str(trace["checks"]))
        if "model" in trace:
            model = mujoco.MjModel.from_binary_path(
                "scene.mjb", {"scene.mjb": trace["model"].tobytes()}
            )
        else:
            model = load_model()
    if qpos.ndim != 2 or qpos.shape[1] != model.nq or not len(qpos):
        raise ValueError("trace poses do not match the recorded model")
    base, geoms = visible_geoms(model)
    data = mujoco.MjData(model)
    frames = [
        pose_frame(model, data, base, geoms, pose, float(time))
        for pose, time in zip(qpos, times)
    ]
    if not len(thrash) == len(stops) == len(startups) == len(frames):
        raise ValueError("telemetry samples do not match trace frames")
    for frame, telemetry, stop, startup in zip(frames, thrash, stops, startups):
        frame["thrash"] = telemetry
        frame["stop"] = describe_stop(stop)
        frame["startup"] = describe_startup(startup)
    summary = report_summary(metadata, safety_checks)
    report_path = trace_path.parent / "report.json"
    if report_path.is_file():
        metadata = json.loads(report_path.read_text())
        summary = report_summary(metadata, safety_checks)
    return write_page(dest, model, geoms, frames, summary, live=False, stop=stop_from_report(metadata))


def main():
    if sys.argv[1:2] == ["--idle"]:
        dest = Path(sys.argv[2])
        pose = "standing"
        scene = None
        args = sys.argv[3:]
        while args:
            if args[0] == "--pose" and len(args) > 1:
                pose = args[1]
                args = args[2:]
            elif args[0] == "--scene" and len(args) > 1:
                scene = args[1] or None
                args = args[2:]
            else:
                print("Usage: view.py --idle DEST [--pose NAME] [--scene XML]", file=sys.stderr)
                return 2
        export_idle(dest, pose, scene)
        print(dest / "index.html")
        return 0
    trace = (
        Path(sys.argv[1])
        if len(sys.argv) > 1
        else Path(__file__).resolve().parent / "output" / "trace.npz"
    )
    dest = Path(sys.argv[2]) if len(sys.argv) > 2 else trace.parent / "view"
    if not trace.is_file():
        print(f"No trace at {trace}. Run ./sim.sh first.", file=sys.stderr)
        return 1
    export_replay(trace, dest)
    print(dest / "index.html")
    return 0


if __name__ == "__main__":
    sys.exit(main())
