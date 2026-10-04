"""Write a browser replay of the recorded MuJoCo scene."""

import json
import shutil
import struct
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "robot_safety"))

from plant import load_model

PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<title>G1 sim replay</title>
<style>
  html, body { margin: 0; height: 100%; background: #f4f4f4; overflow: hidden; }
  #hud { position: absolute; left: 12px; top: 12px; font: 14px sans-serif; color: #222; }
  #start {
    position: absolute; left: 50%; top: 18%; transform: translateX(-50%);
    font: 600 18px sans-serif; padding: 12px 28px; border: 0; border-radius: 8px;
    background: #1a1a1a; color: #fff; cursor: pointer;
  }
  #start:disabled { background: #888; cursor: default; }
</style>
</head>
<body>
<div id="hud">loading</div>
<script type="importmap">
{ "imports": { "three": "./three.module.js" } }
</script>
<script type="module">
import * as THREE from "three";
import { OrbitControls } from "./OrbitControls.js";
import { STLLoader } from "./STLLoader.js";

const motion = await (await fetch("motion.json")).json();
const hud = document.getElementById("hud");
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

let frame = 0;
let mode = motion.live ? "idle" : "replay";
let paused = false;
function show(index) {
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
  const playback = mode === "idle" ? "ready" : mode === "live" ? "live" : (paused ? "paused" : "playing");
  hud.textContent = `${motion.summary}\n${playback}  ${motion.time[frame].toFixed(2)} s  frame ${frame + 1} / ${motion.frames}`;
  hud.style.whiteSpace = "pre-line";
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
    for (const geom of motion.geoms) geom.poses = [];
  }
  for (const item of frames) {
    motion.time.push(item.time);
    motion.focus.push(item.focus);
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
      if (body.done) mode = "replay";
      if (!body.done) setTimeout(poll, 50);
    };
    poll();
  });
}
window.addEventListener("keydown", (event) => {
  if (event.code === "Space") { paused = !paused; event.preventDefault(); }
  else if (event.code === "ArrowLeft") { paused = true; show(frame - 1); }
  else if (event.code === "ArrowRight") { paused = true; show(frame + 1); }
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
    const caughtUp = mode === "live" && frame >= motion.frames - 1;
    const next = caughtUp ? frame : (frame + 1) % motion.frames;
    const delay = next !== frame && motion.time[next] !== undefined
      ? Math.max(0, 1000 * (motion.time[next] - motion.time[frame]))
      : 20;
    if (!caughtUp && now - last >= delay) { last = now; show(next); }
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


def write_page(dest, model, geoms, frames, summary, live=False):
    """Write index.html, motion.json, and the mesh files the page loads."""
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
    for name in ("three.module.js", "OrbitControls.js", "STLLoader.js"):
        shutil.copy2(Path(__file__).resolve().parent / "web" / name, dest / name)
    return dest


def report_summary(report, safety_checks=True):
    """HUD text for a finished run."""
    summary = f"{report.get('policy') or 'Simulation'} | Safety checks: "
    if report.get("control_path") == "direct":
        summary += "PARTIAL" if report.get("safety_checks", safety_checks) else "OFF"
        summary += " (direct control, submission checks bypassed)"
    else:
        summary += "ON" if report.get("safety_checks", safety_checks) else "OFF"
    summary += " | " + str(report.get("status", ""))
    if report.get("reason"):
        summary += "\nStopped accepting commands: " + report["reason"]
    motion = report.get("motion", {})
    if "fell" in motion:
        summary += "\nFall recorded: " + ("yes" if motion["fell"] else "no")
    if "stairs" in motion:
        stairs = motion["stairs"]
        summary += (
            f" | Stair treads: {stairs['distinct_treads']}/3"
            f" | Finish upright: {stairs['finished_upright']}"
        )
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
    summary = report_summary(metadata, safety_checks)
    report_path = trace_path.parent / "report.json"
    if report_path.is_file():
        summary = report_summary(json.loads(report_path.read_text()), safety_checks)
    return write_page(dest, model, geoms, frames, summary, live=False)


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
