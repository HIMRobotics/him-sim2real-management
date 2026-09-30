"""Write a browser replay of a container sim. The page is a flat floor and the G1.

Mouse orbit, right-drag pan, scroll zoom. Space pauses. Arrow keys step one frame.
The MuJoCo desktop window is not used: this machine's display has no GLX context.
"""

import json
import shutil
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "robot_safety"))

from plant import load_model

ROBOT_XML = (
    Path(__file__).resolve().parents[2]
    / "packages/policy_safety/src/policy_safety/robots/g1/g1_robot.xml"
)

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
for (const geom of motion.geoms) {
  const geometry = await loader.loadAsync(geom.file);
  const material = new THREE.MeshStandardMaterial({ color: geom.color, metalness: 0.15, roughness: 0.6 });
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
  hud.textContent = paused ? `paused  frame ${frame + 1} / ${motion.frames}` : `frame ${frame + 1} / ${motion.frames}`;
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
  if (!paused && now - last > 20) { last = now; show(frame + 1); }
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
    vertices = np.asarray(model.mesh_vert[vert_adr : vert_adr + vert_num], dtype=np.float32)
    faces = np.asarray(model.mesh_face[face_adr : face_adr + face_num], dtype=np.int32)
    triangles = vertices[faces]
    normals = np.cross(
        triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]
    )
    length = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = np.divide(normals, length, out=np.zeros_like(normals), where=length > 0)
    with Path(path).open("wb") as stream:
        stream.write(b"g1 sim mesh".ljust(80, b"\0"))
        stream.write(struct.pack("<I", int(face_num)))
        for normal, triangle in zip(normals, triangles):
            stream.write(struct.pack("<12fH", *normal, *triangle.reshape(-1), 0))


def visual_colors():
    """Colors in the same order as the visual mesh geoms."""
    colors = []
    for geom in ET.parse(ROBOT_XML).iter("geom"):
        mesh = geom.get("mesh")
        if geom.get("class") == "visual" and mesh:
            color = "#404040" if geom.get("material") == "black" else "#b8b8b8"
            colors.append((mesh, color))
    return colors


def export_replay(trace_path, dest):
    """Build dest/index.html, motion.json, and the STL files the page loads."""
    trace_path, dest = Path(trace_path), Path(dest)
    qpos = np.load(trace_path)["qpos"]
    model = load_model()
    data = mujoco.MjData(model)
    colors = {name: color for name, color in visual_colors()}
    geoms = []
    for geom_id in range(model.ngeom):
        if int(model.geom_group[geom_id]) != 2:
            continue
        if int(model.geom_type[geom_id]) != int(mujoco.mjtGeom.mjGEOM_MESH):
            continue
        mesh_id = int(model.geom_dataid[geom_id])
        mesh_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mesh_id)
        geoms.append((geom_id, mesh_id, mesh_name, colors.get(mesh_name, "#b8b8b8")))
    poses = [[] for _ in geoms]
    focus = []
    base = model.body("pelvis").id
    for frame in qpos:
        data.qpos[:] = frame
        mujoco.mj_forward(model, data)
        focus.append([round(float(v), 5) for v in data.xpos[base]])
        for slot, (geom_id, _mesh_id, _mesh_name, _color) in enumerate(geoms):
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
    payload = {"frames": int(len(qpos)), "focus": focus, "geoms": []}
    written = set()
    for slot, (_geom_id, mesh_id, mesh_name, color) in enumerate(geoms):
        filename = f"{mesh_name}.stl"
        if mesh_id not in written:
            write_compiled_stl(model, mesh_id, mesh_dir / filename)
            written.add(mesh_id)
        payload["geoms"].append(
            {"file": f"meshes/{filename}", "color": color, "poses": poses[slot]}
        )
    dest.joinpath("motion.json").write_text(json.dumps(payload))
    dest.joinpath("index.html").write_text(PAGE)
    for name in ("three.module.js", "OrbitControls.js", "STLLoader.js"):
        shutil.copy2(Path(__file__).resolve().parent / "web" / name, dest / name)
    return dest


def main():
    trace = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parent / "output" / "trace.npz"
    dest = Path(sys.argv[2]) if len(sys.argv) > 2 else trace.parent / "view"
    if not trace.is_file():
        print(f"No trace at {trace}. Run ./sim.sh first.", file=sys.stderr)
        return 1
    export_replay(trace, dest)
    print(dest / "index.html")
    return 0


if __name__ == "__main__":
    sys.exit(main())
