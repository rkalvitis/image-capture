#!/usr/bin/env python3
"""3D inspector: Panda arm skeleton + phone mount STL at each waypoint.

For every waypoint of a block CSV, draws the arm's kinematic skeleton
(forward kinematics from the logged joints - verified to reproduce the
logged flange pose exactly), the phone-mount STL rigidly attached to
panda_link8 (mm mesh, z -8 mm, yaw -90 deg - the attach constants from
robo_arm/panda_semicircle_motion.py), the corrected ultra-wide lens point,
its calibrated optical axis, and the beetle/turntable for context. A
waypoint selector highlights one pose at a time; everything is in the
robot base (lab) frame.

Usage:
  PYTHONPATH=scripts pixi run python scripts/plot_arm_poses.py \\
      datasets/phone_20260826_113325 [--csv manual_poses_20260826_112820.csv]
"""

import argparse
import csv as csvmod
import json
import os

import numpy as np
import yaml

# Franka Panda modified-DH (Craig): (a_{i-1}, d_i, alpha_{i-1}); flange last
DH = [
    (0.0, 0.333, 0.0),
    (0.0, 0.0, -np.pi / 2),
    (0.0, 0.316, np.pi / 2),
    (0.0825, 0.0, np.pi / 2),
    (-0.0825, 0.384, -np.pi / 2),
    (0.0, 0.0, np.pi / 2),
    (0.088, 0.0, np.pi / 2),
    (0.0, 0.107, 0.0),
]

HOLDER_Z_OFFSET = -0.008
HOLDER_YAW_DEG = -90.0
HOLDER_SCALE = 0.001


def dh_T(a, d, alpha, theta):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st, 0, a],
        [st * ca, ct * ca, -sa, -d * sa],
        [st * sa, ct * sa, ca, d * ca],
        [0, 0, 0, 1]])


def fk_chain(joints):
    T = np.eye(4)
    origins = [T[:3, 3].copy()]
    for i, (a, d, al) in enumerate(DH):
        T = T @ dh_T(a, d, al, joints[i] if i < 7 else 0.0)
        origins.append(T[:3, 3].copy())
    return T, np.array(origins)


def load_stl_decimated(path, grid_mm=2.0):
    """Binary STL -> (verts[N,3] m in link8 frame, faces[M,3]) via vertex
    clustering on a grid (fast, no deps)."""
    raw = open(path, 'rb').read()
    n = int.from_bytes(raw[80:84], 'little')
    rec = np.frombuffer(raw[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
    tris = rec[:, 12:48].copy().view('<f4').reshape(n * 3, 3).astype(np.float64)
    keys = np.round(tris / grid_mm).astype(np.int64)
    uniq, inv = np.unique(keys, axis=0, return_inverse=True)
    # representative vertex per cluster = mean
    verts = np.zeros((len(uniq), 3))
    counts = np.bincount(inv, minlength=len(uniq)).astype(float)
    for k in range(3):
        verts[:, k] = np.bincount(inv, weights=tris[:, k], minlength=len(uniq)) / counts
    faces = inv.reshape(n, 3)
    good = (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
    faces = np.unique(np.sort(faces[good], axis=1), axis=0)
    # into link8 frame: mm->m, yaw -90, z offset
    verts *= HOLDER_SCALE
    a = np.radians(HOLDER_YAW_DEG)
    Rz = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
    verts = verts @ Rz.T
    verts[:, 2] += HOLDER_Z_OFFSET
    return verts, faces


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('dataset')
    ap.add_argument('--csv', default='manual_poses_20260826_112820.csv')
    ap.add_argument('-o', '--out', default=None)
    args = ap.parse_args()
    ds = args.dataset.rstrip('/')
    reg = yaml.safe_load(open(os.path.join(ds, 'registration.yaml')))
    lens_fl = np.array(reg['lens_offset_in_flange_corrected_m'])
    cam_axis_fl = np.array(reg['camera_axis_solution']['optical_axis_in_flange'])
    beetle = reg['object_center_robot_on_axis']
    axis_pt = np.array(reg['turntable_axis_robot']['point'])
    axis_dir = np.array(reg['turntable_axis_robot']['dir'])
    axis_dir = axis_dir / np.linalg.norm(axis_dir)

    rows = list(csvmod.DictReader(open(os.path.join(ds, 'phone_poses_robot', args.csv))))
    wps = []
    for w, r in enumerate(rows):
        joints = [float(r[f'joint{k}']) for k in range(1, 8)]
        T, origins = fk_chain(joints)
        lens = T[:3, 3] + T[:3, :3] @ lens_fl
        axis_w = T[:3, :3] @ cam_axis_fl
        wps.append({
            'w': w,
            'skeleton': np.round(origins, 4).tolist(),
            'T': np.round(T, 6).tolist(),
            'lens': np.round(lens, 4).tolist(),
            'look': np.round(axis_w / np.linalg.norm(axis_w), 4).tolist(),
        })

    verts, faces = load_stl_decimated(
        os.path.join(ds, 'phone_poses_robot', 'phone_mount_conf3.stl'))
    print(f'mount mesh decimated: {len(verts)} verts, {len(faces)} faces')

    u = np.cross(axis_dir, [1, 0, 0]); u /= np.linalg.norm(u)
    v = np.cross(axis_dir, u)
    th = np.linspace(0, 2 * np.pi, 61)
    platter = axis_pt[None, :] + 0.07 * (np.cos(th)[:, None] * u + np.sin(th)[:, None] * v)

    data = {
        'wps': wps,
        'mesh_v': np.round(verts, 4).tolist(),
        'mesh_f': faces.tolist(),
        'beetle': beetle,
        'axis_base': np.round(axis_pt, 4).tolist(),
        'platter': np.round(platter, 4).tolist(),
        'csv': args.csv,
    }
    out = args.out or os.path.join(ds, 'arm_poses.html')
    with open(out, 'w') as f:
        f.write(HTML_HEAD + '\nconst DATA = ' + json.dumps(data) + ';\n' + HTML_TAIL)
    print('wrote', out)


HTML_HEAD = r"""<title>Panda Waypoint Inspector</title>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans:wght@400;500;600&family=IBM+Plex+Mono:wght@400;500&display=swap">
<style>
  :root {
    color-scheme: light;
    --surface: #fcfcfb; --panel: #ffffffee; --panel-border: #e4e2dc;
    --ink: #0b0b0b; --ink-2: #52514e; --ink-3: #8a887f;
    --accent: #eb6834; --rig: #6b6963; --grid: #e8e6e0;
    --arm: #2a78d6; --armdim: #b9cfe8; --mount: #7f9db9;
    --chip-bg: #f3f2ee;
  }
  @media (prefers-color-scheme: dark) {
    :root:not([data-theme="light"]) {
      color-scheme: dark;
      --surface: #1a1a19; --panel: #232322ee; --panel-border: #3a3936;
      --ink: #ffffff; --ink-2: #c3c2b7; --ink-3: #8b897f;
      --accent: #f0784a; --rig: #a09e94; --grid: #31302d;
      --arm: #3987e5; --armdim: #34455c; --mount: #5d7d9c;
      --chip-bg: #2b2b29;
    }
  }
  :root[data-theme="dark"] {
    color-scheme: dark;
    --surface: #1a1a19; --panel: #232322ee; --panel-border: #3a3936;
    --ink: #ffffff; --ink-2: #c3c2b7; --ink-3: #8b897f;
    --accent: #f0784a; --rig: #a09e94; --grid: #31302d;
    --arm: #3987e5; --armdim: #34455c; --mount: #5d7d9c;
    --chip-bg: #2b2b29;
  }
  * { margin: 0; box-sizing: border-box; }
  html, body { height: 100%; }
  body { background: var(--surface); color: var(--ink);
         font-family: "IBM Plex Sans", "Helvetica Neue", Arial, sans-serif;
         overflow: hidden; }
  #plot { position: fixed; inset: 0; }
  .panel { position: fixed; top: 16px; left: 16px;
           background: var(--panel); border: 1px solid var(--panel-border);
           border-radius: 10px; padding: 14px 16px 12px; max-width: 300px;
           backdrop-filter: blur(6px);
           display: flex; flex-direction: column; gap: 10px; z-index: 5; }
  h1 { font-size: 15px; font-weight: 600; }
  .sub { font-size: 12px; color: var(--ink-2); line-height: 1.45; }
  .wpsel { display: grid; grid-template-columns: repeat(9, 1fr); gap: 4px; }
  .wpsel button { border: 1px solid var(--panel-border); border-radius: 6px;
                  background: transparent; color: var(--ink-2);
                  font: 500 12px "IBM Plex Mono", monospace;
                  padding: 5px 0; cursor: pointer; }
  .wpsel button.on { background: var(--chip-bg); color: var(--ink);
                     font-weight: 600; border-color: var(--arm); }
  .wpsel button:focus-visible { outline: 2px solid var(--arm); }
  .stats { font: 400 11px "IBM Plex Mono", monospace; color: var(--ink-2);
           line-height: 1.7; }
  .stats b { color: var(--ink); font-weight: 500; }
  .hint { font-size: 11px; color: var(--ink-3); }
</style>
<div id="plot" aria-label="3D view of the robot arm and phone mount at each waypoint"></div>
<div class="panel">
  <div>
    <h1>Panda waypoint inspector</h1>
    <div class="sub">Arm skeleton from the logged joints (FK reproduces the
    logged flange pose exactly), mount STL on link8, corrected lens point,
    calibrated optical axis. Robot base frame, block 1 CSV.</div>
  </div>
  <div class="wpsel" id="wpsel" role="group" aria-label="Waypoint"></div>
  <div class="stats" id="stats"></div>
  <div class="hint">0 = init hold; 1&ndash;8 = photo shots. Faint skeletons +
  lens dots show all other waypoints for context. Drag to orbit.</div>
</div>
<script src="https://cdnjs.cloudflare.com/ajax/libs/plotly.js/2.35.3/plotly.min.js"></script>
<script>
"use strict";
"""

HTML_TAIL = r"""
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
let W = 5;   // start at shot 5 - the suspicious one

function applyT(T, v) {
  return [
    T[0][0]*v[0]+T[0][1]*v[1]+T[0][2]*v[2]+T[0][3],
    T[1][0]*v[0]+T[1][1]*v[1]+T[1][2]*v[2]+T[1][3],
    T[2][0]*v[0]+T[2][1]*v[1]+T[2][2]*v[2]+T[2][3]];
}

function traces() {
  const out = [];
  const bt = DATA.beetle;
  // table + platter + rod
  out.push({ type: "mesh3d",
    x: [-0.05, 0.55, 0.55, -0.05], y: [-0.1, -0.1, 0.45, 0.45], z: [0,0,0,0],
    i: [0,0], j: [1,2], k: [2,3], color: css("--grid"), opacity: 0.4,
    hoverinfo: "skip", name: "table", showlegend: false });
  out.push({ type: "scatter3d", mode: "lines",
    x: DATA.platter.map(p=>p[0]), y: DATA.platter.map(p=>p[1]),
    z: DATA.platter.map(p=>p[2]),
    line: { color: css("--rig"), width: 2 }, hoverinfo: "skip",
    name: "stage", showlegend: false });
  out.push({ type: "scatter3d", mode: "lines",
    x: [DATA.axis_base[0], bt[0]], y: [DATA.axis_base[1], bt[1]],
    z: [DATA.axis_base[2], bt[2]],
    line: { color: css("--rig"), width: 4 }, hoverinfo: "skip",
    name: "rod", showlegend: false });
  out.push({ type: "scatter3d", mode: "markers",
    x: [bt[0]], y: [bt[1]], z: [bt[2]],
    marker: { size: 8, color: css("--accent"), symbol: "diamond" },
    name: "popillia", hovertemplate: "<b>popillia</b><extra></extra>" });

  // faint skeletons + lens dots of all waypoints
  const fx = [], fy = [], fz = [];
  for (const wp of DATA.wps) {
    if (wp.w === W) continue;
    for (const p of wp.skeleton) { fx.push(p[0]); fy.push(p[1]); fz.push(p[2]); }
    fx.push(null); fy.push(null); fz.push(null);
  }
  out.push({ type: "scatter3d", mode: "lines",
    x: fx, y: fy, z: fz, line: { color: css("--armdim"), width: 3 },
    hoverinfo: "skip", name: "other wps", showlegend: false });
  const others = DATA.wps.filter(w => w.w !== W);
  out.push({ type: "scatter3d", mode: "markers+text",
    x: others.map(w=>w.lens[0]), y: others.map(w=>w.lens[1]),
    z: others.map(w=>w.lens[2]),
    marker: { size: 4, color: css("--armdim") },
    text: others.map(w=>String(w.w)), textposition: "top center",
    textfont: { family: "IBM Plex Mono, monospace", size: 10, color: css("--ink-3") },
    hovertemplate: "lens wp %{text}<extra></extra>",
    name: "other lenses", showlegend: false });

  // selected waypoint: bold skeleton, joints, mount mesh, lens, look axis
  const wp = DATA.wps[W];
  const sk = wp.skeleton;
  out.push({ type: "scatter3d", mode: "lines+markers",
    x: sk.map(p=>p[0]), y: sk.map(p=>p[1]), z: sk.map(p=>p[2]),
    line: { color: css("--arm"), width: 8 },
    marker: { size: 4.5, color: css("--arm") },
    name: "arm wp " + W,
    hovertemplate: "joint origin<extra></extra>" });
  const mv = DATA.mesh_v.map(v => applyT(wp.T, v));
  out.push({ type: "mesh3d",
    x: mv.map(p=>p[0]), y: mv.map(p=>p[1]), z: mv.map(p=>p[2]),
    i: DATA.mesh_f.map(f=>f[0]), j: DATA.mesh_f.map(f=>f[1]),
    k: DATA.mesh_f.map(f=>f[2]),
    color: css("--mount"), opacity: 0.55, flatshading: true,
    lighting: { ambient: 0.55, diffuse: 0.7, specular: 0.05 },
    hoverinfo: "skip", name: "mount+phone", showlegend: false });
  out.push({ type: "scatter3d", mode: "markers",
    x: [wp.lens[0]], y: [wp.lens[1]], z: [wp.lens[2]],
    marker: { size: 7, color: css("--accent") },
    name: "lens", hovertemplate: "ultra-wide lens (corrected)<extra></extra>" });
  const L = 0.05;
  out.push({ type: "scatter3d", mode: "lines",
    x: [wp.lens[0], wp.lens[0]+L*wp.look[0]],
    y: [wp.lens[1], wp.lens[1]+L*wp.look[1]],
    z: [wp.lens[2], wp.lens[2]+L*wp.look[2]],
    line: { color: css("--accent"), width: 5 },
    name: "optical axis", hoverinfo: "skip", showlegend: false });
  return out;
}

function layout() {
  const ax = t => ({
    title: { text: t, font: { family: "IBM Plex Mono, monospace", size: 11 } },
    color: css("--ink-2"), gridcolor: css("--grid"),
    zerolinecolor: css("--grid"), backgroundcolor: "rgba(0,0,0,0)",
    tickfont: { family: "IBM Plex Mono, monospace", size: 9 } });
  return {
    paper_bgcolor: css("--surface"), margin: { l: 0, r: 0, t: 0, b: 0 },
    scene: { aspectmode: "data", xaxis: ax("x (m)"), yaxis: ax("y (m)"),
             zaxis: ax("z (m)"), bgcolor: "rgba(0,0,0,0)",
             camera: { eye: { x: 1.1, y: -1.3, z: 0.55 } } },
    showlegend: false,
    hoverlabel: { bgcolor: css("--panel"), bordercolor: css("--panel-border"),
                  font: { family: "IBM Plex Mono, monospace", size: 11,
                          color: css("--ink") } } };
}

function stats() {
  const wp = DATA.wps[W];
  const bt = DATA.beetle;
  const d = Math.hypot(wp.lens[0]-bt[0], wp.lens[1]-bt[1], wp.lens[2]-bt[2]);
  const tb = [(bt[0]-wp.lens[0])/d, (bt[1]-wp.lens[1])/d, (bt[2]-wp.lens[2])/d];
  const ang = Math.acos(Math.max(-1, Math.min(1,
    tb[0]*wp.look[0]+tb[1]*wp.look[1]+tb[2]*wp.look[2]))) * 180/Math.PI;
  document.getElementById("stats").innerHTML =
    `lens <b>(${wp.lens.map(v=>v.toFixed(3)).join(", ")})</b> m<br>` +
    `lens&ndash;beetle <b>${(d*1000).toFixed(1)} mm</b> · ` +
    `beetle <b>${ang.toFixed(1)}&deg;</b> off optical axis`;
}

function render(keep) {
  const lay = layout();
  const gd = document.getElementById("plot");
  if (keep && gd._fullLayout && gd._fullLayout.scene)
    lay.scene.camera = gd._fullLayout.scene.camera;
  Plotly.react("plot", traces(), lay, { displaylogo: false, responsive: true });
}

const sel = document.getElementById("wpsel");
for (let w = 0; w < DATA.wps.length; w++) {
  const b = document.createElement("button");
  b.textContent = w;
  b.classList.toggle("on", w === W);
  b.addEventListener("click", () => {
    W = w;
    for (const bb of sel.children) bb.classList.toggle("on", Number(bb.textContent) === W);
    stats(); render(true);
  });
  sel.appendChild(b);
}
stats(); render(false);
matchMedia("(prefers-color-scheme: dark)").addEventListener("change", () => render(true));
new MutationObserver(() => render(true))
  .observe(document.documentElement, { attributes: true, attributeFilter: ["data-theme"] });
</script>
"""

if __name__ == '__main__':
    main()
