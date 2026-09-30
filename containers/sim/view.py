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
let paused = false;
function show(index) {
  frame = (index + motion.frames) % motion.frames;
  const focus = motion.focus[frame];
  controls.target.set(focus[0], focus[1], focus[2]);
  parts.forEach((mesh, i) => {
    const pose = motion.geoms[i].poses[frame];
    mesh.matrix.set(
      pose[0], pose[1], pose[2], pose[9],
      pose[3], pose[4], pose[5], pose[10],
      pose[6], pose[7], pose[8], pose[11],
      0, 0, 0, 1
    );
  });
  const playback = paused ? "paused" : "playing";
  hud.textContent = `${motion.summary}\n${playback}  ${motion.time[frame].toFixed(2)} s  frame ${frame + 1} / ${motion.frames}`;
  hud.style.whiteSpace = "pre-line";
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
  const next = (frame + 1) % motion.frames;
  const delay = next ? 1000 * (motion.time[next] - motion.time[frame]) : 20;
  if (!paused && now - last >= delay) { last = now; show(next); }
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


def export_replay(trace_path, dest):
    """Build dest/index.html, motion.json, and the STL files the page loads."""
    trace_path, dest = Path(trace_path), Path(dest)
    with np.load(trace_path, allow_pickle=False) as trace:
        qpos = trace["qpos"]
        times = trace["time"]
        safety_checks = (
            bool(trace["safety_checks"]) if "safety_checks" in trace else True
        )
        if "model" in trace:
            model = mujoco.MjModel.from_binary_path(
                "scene.mjb", {"scene.mjb": trace["model"].tobytes()}
            )
        else:
            model = load_model()
    if qpos.ndim != 2 or qpos.shape[1] != model.nq or not len(qpos):
        raise ValueError("trace poses do not match the recorded model")
    data = mujoco.MjData(model)
    geoms = []
    base = model.body("pelvis").id
    robot_bodies = {base}
    for body_id in range(base + 1, model.nbody):
        if int(model.body_parentid[body_id]) in robot_bodies:
            robot_bodies.add(body_id)
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
    poses = [[] for _ in geoms]
    focus = []
    for frame in qpos:
        data.qpos[:] = frame
        mujoco.mj_forward(model, data)
        focus.append([round(float(v), 5) for v in data.xpos[base]])
        for slot, (geom_id, _kind, _rgba) in enumerate(geoms):
            rotation = data.geom_xmat[geom_id]
            position = data.geom_xpos[geom_id]
            poses[slot].append(
                [round(float(v), 5) for v in rotation]
                + [round(float(v), 5) for v in position]
            )
    mesh_dir = dest / "meshes"
    if dest.exists():
        shutil.rmtree(dest)
    mesh_dir.mkdir(parents=True)
    summary = "Safety checks: " + ("ON" if safety_checks else "OFF")
    report_path = trace_path.parent / "report.json"
    if report_path.is_file():
        report = json.loads(report_path.read_text())
        summary = f"{report.get('policy') or 'Simulation'} | {summary} | {report.get('status', '')}"
        if report.get("reason"):
            summary += "\nStopped accepting commands: " + report["reason"]
        motion = report.get("motion", {})
        if "fell" in motion:
            summary += "\nFall recorded: " + ("yes" if motion["fell"] else "no")
        if "stairs" in motion:
            stairs = motion["stairs"]
            summary += f" | Stair treads: {stairs['distinct_treads']}/3 | Finish upright: {stairs['finished_upright']}"
    payload = {
        "frames": len(qpos),
        "time": times.tolist(),
        "focus": focus,
        "summary": summary,
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
            "poses": poses[slot],
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


def main():
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
