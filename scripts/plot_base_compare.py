#!/usr/bin/env python3
"""A/B view: the arm rendered under two robot-base hypotheses.

Same scene style as plot_arm_poses.py (skeleton + mount STL + beetle/stage,
waypoint selector), but the arm is drawn TWICE at every waypoint:

  blue    = the current joint-solve base pose (the reference)
  magenta = the per-hold AVERAGE base pose (positions + quaternion-mean
            rotation over the clean holds)

Both are placed relative to the same fixed mocap world (beetle, rod, stage),
so the offset between the two arms IS the base-pose disagreement - the
~0.9 deg AX=XB orientation-chain bias leveraged into ~18 mm.

Usage:
  PYTHONPATH=scripts pixi run python scripts/plot_base_compare.py \\
      datasets/phone_20260826_113325 [-o base_compare.html]
"""

import argparse
import json
import os

import numpy as np
import yaml

from link_robot_mocap import load_run_csv, mat_quat, quat_mat, rot_angle_deg
from plot_arm_poses import fk_chain, load_stl_decimated
from plot_base_estimates import gather, joint_solve


def average_T_moc_rob(holds, T_fl_body):
    Ts = []
    for b, w, Tb, Tfl in holds:
        Ts.append(Tb @ np.linalg.inv(T_fl_body) @ np.linalg.inv(Tfl))
    qs = np.array([mat_quat(T[:3, :3]) for T in Ts])
    qs[qs @ qs[0] < 0] *= -1
    qm = qs.mean(0)
    qm /= np.linalg.norm(qm)
    T = np.eye(4)
    T[:3, :3] = quat_mat(qm)
    T[:3, 3] = np.mean([T[:3, 3] for T in Ts], axis=0)
    return T


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('dataset')
    ap.add_argument('--csv', default='manual_poses_20260826_112820.csv')
    ap.add_argument('-o', '--out', default=None)
    args = ap.parse_args()
    ds = args.dataset.rstrip('/')
    reg = yaml.safe_load(open(os.path.join(ds, 'registration.yaml')))
    lens_fl = np.array(reg['lens_offset_in_flange_corrected_m'])
    beetle = reg['camera_axis_solution']['beetle_visual_center_robot_m']
    axis_pt = np.array(reg['turntable_axis_robot']['point'])
    axis_dir = np.array(reg['turntable_axis_robot']['dir'])
    axis_dir = axis_dir / np.linalg.norm(axis_dir)
    Rx = np.array(reg['R_flange_from_body'])
    r = np.array(reg['marker_body_origin_in_flange_m'])
    T_fl_body = np.eye(4)
    T_fl_body[:3, :3] = Rx
    T_fl_body[:3, 3] = r
    R_rm = np.array(reg['T_robot_from_mocap']['rotation_matrix'])
    t_rm = np.array(reg['T_robot_from_mocap']['translation_m'])
    T_moc_rob_cur = np.eye(4)
    T_moc_rob_cur[:3, :3] = R_rm.T
    T_moc_rob_cur[:3, 3] = -R_rm.T @ t_rm

    holds = gather(ds, gate=True)
    T_moc_rob_avg = average_T_moc_rob(holds, T_fl_body)
    # canvas = current robot frame; avg-arm points map through dT
    dT = np.linalg.inv(T_moc_rob_cur) @ T_moc_rob_avg
    d_ang = rot_angle_deg(dT[:3, :3])
    d_t = np.linalg.norm(dT[:3, 3]) * 1000
    print(f'base disagreement: |dt| {d_t:.1f} mm at the base, '
          f'rotation {d_ang:.2f} deg')

    rows_T, _ = load_run_csv(os.path.join(ds, 'phone_poses_robot', args.csv))
    import csv as csvmod
    rows = list(csvmod.DictReader(
        open(os.path.join(ds, 'phone_poses_robot', args.csv))))
    wps = []
    for w, row in enumerate(rows):
        joints = [float(row[f'joint{k}']) for k in range(1, 8)]
        T, origins = fk_chain(joints)
        sk_cur = origins
        sk_avg = (dT[:3, :3] @ origins.T).T + dT[:3, 3]
        lens_cur = T[:3, 3] + T[:3, :3] @ lens_fl
        T_avg = dT @ T
        wps.append({
            'w': w,
            'sk_cur': np.round(sk_cur, 4).tolist(),
            'sk_avg': np.round(sk_avg, 4).tolist(),
            'T_cur': np.round(T, 6).tolist(),
            'T_avg': np.round(T_avg, 6).tolist(),
            'lens_cur': np.round(lens_cur, 4).tolist(),
            'lens_avg': np.round((dT[:3, :3] @ lens_cur + dT[:3, 3]), 4).tolist(),
        })

    verts, faces = load_stl_decimated(
        os.path.join(ds, 'phone_poses_robot', 'phone_mount_conf3.stl'),
        grid_mm=3.0)
    u = np.cross(axis_dir, [1, 0, 0])
    u /= np.linalg.norm(u)
    v = np.cross(axis_dir, u)
    th = np.linspace(0, 2 * np.pi, 61)
    platter = axis_pt[None, :] + 0.07 * (np.cos(th)[:, None] * u
                                         + np.sin(th)[:, None] * v)
    data = {
        'wps': wps,
        'mesh_v': np.round(verts, 4).tolist(),
        'mesh_f': faces.tolist(),
        'beetle': beetle,
        'axis_base': np.round(axis_pt, 4).tolist(),
        'platter': np.round(platter, 4).tolist(),
        'd_t_mm': round(float(d_t), 2),
        'd_ang_deg': round(float(d_ang), 3),
    }
    out = args.out or os.path.join(ds, 'base_compare.html')
    with open(out, 'w') as f:
        f.write(HTML_HEAD + '\nconst DATA = ' + json.dumps(data) + ';\n'
                + HTML_TAIL)
    print('wrote', out)


HTML_HEAD = r"""<title>Base Shift Inspector</title>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans:wght@400;500;600&family=IBM+Plex+Mono:wght@400;500&display=swap">
<style>
  :root {
    color-scheme: light;
    --surface: #fcfcfb; --panel: #ffffffee; --panel-border: #e4e2dc;
    --ink: #0b0b0b; --ink-2: #52514e; --ink-3: #8a887f;
    --accent: #eb6834; --rig: #6b6963; --grid: #e8e6e0;
    --cur: #2a78d6; --avg: #e87ba4; --chip-bg: #f3f2ee;
  }
  @media (prefers-color-scheme: dark) {
    :root:not([data-theme="light"]) {
      color-scheme: dark;
      --surface: #1a1a19; --panel: #232322ee; --panel-border: #3a3936;
      --ink: #ffffff; --ink-2: #c3c2b7; --ink-3: #8b897f;
      --accent: #f0784a; --rig: #a09e94; --grid: #31302d;
      --cur: #3987e5; --avg: #d55181; --chip-bg: #2b2b29;
    }
  }
  :root[data-theme="dark"] {
    color-scheme: dark;
    --surface: #1a1a19; --panel: #232322ee; --panel-border: #3a3936;
    --ink: #ffffff; --ink-2: #c3c2b7; --ink-3: #8b897f;
    --accent: #f0784a; --rig: #a09e94; --grid: #31302d;
    --cur: #3987e5; --avg: #d55181; --chip-bg: #2b2b29;
  }
  * { margin: 0; box-sizing: border-box; }
  html, body { height: 100%; }
  body { background: var(--surface); color: var(--ink);
         font-family: "IBM Plex Sans", "Helvetica Neue", Arial, sans-serif;
         overflow: hidden; }
  #plot { position: fixed; inset: 0; }
  .panel { position: fixed; top: 16px; left: 16px;
           background: var(--panel); border: 1px solid var(--panel-border);
           border-radius: 10px; padding: 14px 16px 12px; max-width: 310px;
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
                     font-weight: 600; border-color: var(--cur); }
  .stats { font: 400 11px "IBM Plex Mono", monospace; color: var(--ink-2);
           line-height: 1.7; }
  .stats b { color: var(--ink); font-weight: 500; }
  .hint { font-size: 11px; color: var(--ink-3); }
</style>
<div id="plot" aria-label="Arm rendered under two base-pose hypotheses"></div>
<div class="panel">
  <div>
    <h1>Base shift inspector</h1>
    <div class="sub"><span style="color:var(--cur)">&#9632;</span> joint-solve
    base (reference) vs <span style="color:var(--avg)">&#9632;</span>
    per-hold-average base &mdash; the same waypoint, the same fixed beetle
    and stage. The gap between the two arms IS the base-pose
    disagreement.</div>
  </div>
  <div class="wpsel" id="wpsel" role="group" aria-label="Waypoint"></div>
  <div class="stats" id="stats"></div>
  <div class="hint">Drag to orbit. The magenta arm carries the ~0.9&deg;
  orientation-chain bias; the blue one is the position-only solve.</div>
</div>
<script src="https://cdnjs.cloudflare.com/ajax/libs/plotly.js/2.35.3/plotly.min.js"></script>
<script>
"use strict";
"""

HTML_TAIL = r"""
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
let W = 1;

function applyT(T, v) {
  return [
    T[0][0]*v[0]+T[0][1]*v[1]+T[0][2]*v[2]+T[0][3],
    T[1][0]*v[0]+T[1][1]*v[1]+T[1][2]*v[2]+T[1][3],
    T[2][0]*v[0]+T[2][1]*v[1]+T[2][2]*v[2]+T[2][3]];
}

function armTraces(wp, which, color, name) {
  const sk = which === "cur" ? wp.sk_cur : wp.sk_avg;
  const T = which === "cur" ? wp.T_cur : wp.T_avg;
  const lens = which === "cur" ? wp.lens_cur : wp.lens_avg;
  const out = [];
  out.push({ type: "scatter3d", mode: "lines+markers",
    x: sk.map(p=>p[0]), y: sk.map(p=>p[1]), z: sk.map(p=>p[2]),
    line: { color: color, width: 7 },
    marker: { size: 4, color: color },
    name: name, hovertemplate: name + "<extra></extra>" });
  const mv = DATA.mesh_v.map(v => applyT(T, v));
  out.push({ type: "mesh3d",
    x: mv.map(p=>p[0]), y: mv.map(p=>p[1]), z: mv.map(p=>p[2]),
    i: DATA.mesh_f.map(f=>f[0]), j: DATA.mesh_f.map(f=>f[1]),
    k: DATA.mesh_f.map(f=>f[2]),
    color: color, opacity: 0.35, flatshading: true,
    lighting: { ambient: 0.6, diffuse: 0.65, specular: 0.05 },
    hoverinfo: "skip", name: name + " mount", showlegend: false });
  out.push({ type: "scatter3d", mode: "markers",
    x: [lens[0]], y: [lens[1]], z: [lens[2]],
    marker: { size: 6, color: color, symbol: "circle",
              line: { color: css("--surface"), width: 1 } },
    name: name + " lens", showlegend: false,
    hovertemplate: name + " lens<extra></extra>" });
  return out;
}

function traces() {
  const out = [];
  const bt = DATA.beetle;
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
  const wp = DATA.wps[W];
  out.push(...armTraces(wp, "cur", css("--cur"), "joint solve"));
  out.push(...armTraces(wp, "avg", css("--avg"), "per-hold average"));
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
    showlegend: true,
    legend: { x: 1, xanchor: "right", y: 0.02, yanchor: "bottom",
              bgcolor: css("--panel"), bordercolor: css("--panel-border"),
              borderwidth: 1,
              font: { family: "IBM Plex Sans, sans-serif", size: 11,
                      color: css("--ink-2") } },
    hoverlabel: { bgcolor: css("--panel"), bordercolor: css("--panel-border"),
                  font: { family: "IBM Plex Mono, monospace", size: 11,
                          color: css("--ink") } } };
}

function stats() {
  const wp = DATA.wps[W];
  const d = Math.hypot(wp.lens_cur[0]-wp.lens_avg[0],
                       wp.lens_cur[1]-wp.lens_avg[1],
                       wp.lens_cur[2]-wp.lens_avg[2]);
  document.getElementById("stats").innerHTML =
    `base shift <b>${DATA.d_t_mm.toFixed(1)} mm</b> · ` +
    `rotation <b>${DATA.d_ang_deg.toFixed(2)}&deg;</b><br>` +
    `lens shift at wp ${W}: <b>${(d*1000).toFixed(1)} mm</b>`;
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
    for (const bb of sel.children)
      bb.classList.toggle("on", Number(bb.textContent) === W);
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
