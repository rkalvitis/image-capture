#!/usr/bin/env python3
"""Rotatable 3D view: the Motive `phone` body frame on the mount STL.

Shows, in the panda_link8 (flange) frame:
  * the phone-mount+phone STL,
  * the phone body's ORIGIN (= marker centroid if the body was created with
    Motive's default pivot) and its x/y/z AXES, both from the August AX=XB
    solve (registration.yaml: marker_body_origin_in_flange_m,
    R_flange_from_body),
  * the ultra-wide lens point,
  * the 4 individual marker balls IF registration.yaml contains
    `phone_markers_in_body` (list of [x,y,z] in the CURRENT body frame,
    exported from Motive's asset pane). The August bag stores only the body
    pose, never marker positions, and the July rigid_bodies.yaml describes a
    SUPERSEDED body definition - it is deliberately not used.

Usage:
  PYTHONPATH=scripts pixi run python scripts/plot_body_on_mount.py \\
      datasets/phone_20260826_113325 [-o body_on_mount.html]
"""

import argparse
import json
import os

import numpy as np
import yaml

from plot_arm_poses import load_stl_decimated

HTML_HEAD = r"""<title>Phone Body on the Mount</title>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans:wght@400;500;600&family=IBM+Plex+Mono:wght@400;500&display=swap">
<style>
  :root {
    color-scheme: light;
    --surface: #fcfcfb; --panel: #ffffffee; --panel-border: #e4e2dc;
    --ink: #0b0b0b; --ink-2: #52514e; --ink-3: #8a887f;
    --accent: #eb6834; --mount: #7f9db9; --grid: #e8e6e0;
    --ax-x: #e34948; --ax-y: #008300; --ax-z: #2a78d6;
    --marker: #eda100; --chip-bg: #f3f2ee;
  }
  @media (prefers-color-scheme: dark) {
    :root:not([data-theme="light"]) {
      color-scheme: dark;
      --surface: #1a1a19; --panel: #232322ee; --panel-border: #3a3936;
      --ink: #ffffff; --ink-2: #c3c2b7; --ink-3: #8b897f;
      --accent: #f0784a; --mount: #5d7d9c; --grid: #31302d;
      --ax-x: #e66767; --ax-y: #35a935; --ax-z: #3987e5;
      --marker: #c98500; --chip-bg: #2b2b29;
    }
  }
  :root[data-theme="dark"] {
    color-scheme: dark;
    --surface: #1a1a19; --panel: #232322ee; --panel-border: #3a3936;
    --ink: #ffffff; --ink-2: #c3c2b7; --ink-3: #8b897f;
    --accent: #f0784a; --mount: #5d7d9c; --grid: #31302d;
    --ax-x: #e66767; --ax-y: #35a935; --ax-z: #3987e5;
    --marker: #c98500; --chip-bg: #2b2b29;
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
  .stats { font: 400 11px "IBM Plex Mono", monospace; color: var(--ink-2);
           line-height: 1.7; }
  .stats b { color: var(--ink); font-weight: 500; }
  .hint { font-size: 11px; color: var(--ink-3); }
</style>
<div id="plot" aria-label="Phone marker body frame on the mount STL"></div>
<div class="panel">
  <div>
    <h1>Phone body on the mount</h1>
    <div class="sub">The Motive <em>phone</em> body frame placed on
    phone_mount_conf3.stl (panda_link8 frame), from this session&rsquo;s
    AX=XB solve. Drag to orbit.</div>
  </div>
  <div class="stats" id="stats"></div>
  <div class="hint" id="hint"></div>
</div>
<script src="https://cdnjs.cloudflare.com/ajax/libs/plotly.js/2.35.3/plotly.min.js"></script>
<script>
"use strict";
"""

HTML_TAIL = r"""
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();

function traces() {
  const out = [];
  const mv = DATA.mesh_v;
  out.push({ type: "mesh3d",
    x: mv.map(p=>p[0]), y: mv.map(p=>p[1]), z: mv.map(p=>p[2]),
    i: DATA.mesh_f.map(f=>f[0]), j: DATA.mesh_f.map(f=>f[1]),
    k: DATA.mesh_f.map(f=>f[2]),
    color: css("--mount"), opacity: 0.45, flatshading: true,
    lighting: { ambient: 0.55, diffuse: 0.7, specular: 0.05 },
    hoverinfo: "skip", name: "mount+phone" });

  const o = DATA.origin;
  out.push({ type: "scatter3d", mode: "markers",
    x: [o[0]], y: [o[1]], z: [o[2]],
    marker: { size: 9, color: css("--ink"), symbol: "diamond" },
    name: "body origin (marker centroid)",
    hovertemplate: "<b>body origin</b> = marker centroid<br>" +
      `(${o.map(v=>(v*1000).toFixed(1)).join(", ")}) mm in link8<extra></extra>` });

  const L = 0.05;
  const axnames = ["x", "y", "z"], axcss = ["--ax-x", "--ax-y", "--ax-z"];
  for (let a = 0; a < 3; a++) {
    const d = DATA.axes[a];
    out.push({ type: "scatter3d", mode: "lines+text",
      x: [o[0], o[0]+L*d[0]], y: [o[1], o[1]+L*d[1]], z: [o[2], o[2]+L*d[2]],
      line: { color: css(axcss[a]), width: 6 },
      text: ["", "body " + axnames[a]], textposition: "top center",
      textfont: { family: "IBM Plex Mono, monospace", size: 11,
                  color: css(axcss[a]) },
      name: "body " + axnames[a], hoverinfo: "skip" });
  }

  out.push({ type: "scatter3d", mode: "markers",
    x: [DATA.lens[0]], y: [DATA.lens[1]], z: [DATA.lens[2]],
    marker: { size: 7, color: css("--accent") },
    name: "ultra-wide lens",
    hovertemplate: "<b>ultra-wide lens</b> (mesh ring)<extra></extra>" });

  if (DATA.handeye) {
    const h = DATA.handeye;
    out.push({ type: "scatter3d", mode: "markers",
      x: [h.t[0]], y: [h.t[1]], z: [h.t[2]],
      marker: { size: 8, color: "#d55181", symbol: "x" },
      name: "hand-eye camera (board run)",
      hovertemplate: "<b>hand-eye camera</b> via the stored body&rarr;flange " +
        "chain<br>" + h.date + "<extra></extra>" });
    if (h.axis) {
      const L = 0.05;
      out.push({ type: "scatter3d", mode: "lines",
        x: [h.t[0], h.t[0]+L*h.axis[0]],
        y: [h.t[1], h.t[1]+L*h.axis[1]],
        z: [h.t[2], h.t[2]+L*h.axis[2]],
        line: { color: "#d55181", width: 5 },
        name: "hand-eye optical axis", hoverinfo: "skip" });
    }
  }
  if (DATA.markers.length) {
    out.push({ type: "scatter3d", mode: "markers+text",
      x: DATA.markers.map(m=>m[0]), y: DATA.markers.map(m=>m[1]),
      z: DATA.markers.map(m=>m[2]),
      marker: { size: 10, color: css("--marker"),
                line: { color: css("--surface"), width: 1.5 } },
      text: DATA.markers.map((m,i)=>"m"+(i+1)), textposition: "top center",
      textfont: { family: "IBM Plex Mono, monospace", size: 11,
                  color: css("--ink-2") },
      name: "markers (Motive export)",
      hovertemplate: "<b>marker %{text}</b><extra></extra>" });
  }
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
    scene: { aspectmode: "data",
             xaxis: ax("x (m)"), yaxis: ax("y (m)"), zaxis: ax("z (m)"),
             bgcolor: "rgba(0,0,0,0)",
             camera: { eye: { x: 1.3, y: 1.0, z: 0.7 } } },
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

const o = DATA.origin, l = DATA.lens;
const d = Math.hypot(o[0]-l[0], o[1]-l[1], o[2]-l[2]);
document.getElementById("stats").innerHTML =
  `origin <b>(${o.map(v=>(v*1000).toFixed(1)).join(", ")}) mm</b> in link8<br>` +
  `origin&ndash;lens <b>${(d*1000).toFixed(1)} mm</b>`;
document.getElementById("hint").innerHTML = DATA.markers.length
  ? "Orange balls = markers from the Motive asset export."
  : "Individual marker balls are NOT in the recorded data (the bag carries " +
    "only the body pose; the July marker file describes a superseded body " +
    "definition). Export the current body&rsquo;s marker positions from " +
    "Motive&rsquo;s asset pane into registration.yaml " +
    "(<code>phone_markers_in_body</code>) and rerun the script to draw them.";

function render() {
  Plotly.react("plot", traces(), layout(),
    { displaylogo: false, responsive: true });
}
render();
matchMedia("(prefers-color-scheme: dark)").addEventListener("change", render);
new MutationObserver(render)
  .observe(document.documentElement, { attributes: true, attributeFilter: ["data-theme"] });
</script>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('dataset')
    ap.add_argument('-o', '--out', default=None)
    ap.add_argument('--rigid-bodies', default=None,
                    help='rigid_bodies.yaml freshly produced by '
                         '`pixi run dump-bodies` AGAINST THE CURRENT Motive '
                         'body definition; its bodies.phone.markers are '
                         'drawn. Do NOT pass a stale file - the July one '
                         'describes a superseded body.')
    args = ap.parse_args()
    ds = args.dataset.rstrip('/')
    reg = yaml.safe_load(open(os.path.join(ds, 'registration.yaml')))
    r = np.array(reg['marker_body_origin_in_flange_m'])
    Rx = np.array(reg['R_flange_from_body'])
    lens = np.array(reg['lens_offset_in_flange_corrected_m'])
    markers_body = reg.get('phone_markers_in_body', [])
    handeye = None
    he_path = 'calibration/handeye_robot.yaml'
    if os.path.exists(he_path):
        he = yaml.safe_load(open(he_path))
        br = he.get('T_flange_from_body_bridged')
        if br:
            # NEW body: camera-bridged transform supersedes the old chain
            Rx = np.array(br['rotation_matrix'])
            r = np.array(br['translation_m'])
            rbfile = 'calibration/rigid_bodies.yaml'
            if os.path.exists(rbfile):
                rb = yaml.safe_load(open(rbfile))
                markers_body = [m['position_m']
                                for m in rb['bodies']['phone']['markers']]
            cb = he['camera_in_body']
            cam_fl = Rx @ np.array(cb['translation_m']) + r
            from link_robot_mocap import quat_mat as _qm
            ax_fl = Rx @ (_qm(cb['quaternion_xyzw']) @ np.array([0, 0, 1.0]))
            handeye = {'t': cam_fl.tolist(), 'axis': ax_fl.tolist(),
                       'date': he.get('date')}
        else:
            cf = he.get('camera_in_flange')
            if cf:
                handeye = {
                    't': cf['translation_m'],
                    'axis': he.get('optical_axis_in_flange'),
                    'date': he.get('date'),
                }
    if args.rigid_bodies:
        rb = yaml.safe_load(open(args.rigid_bodies))
        markers_body = [m['position_m']
                        for m in rb['bodies']['phone']['markers']]
        print(f"markers from {args.rigid_bodies} (dated {rb.get('date')}) - "
              f"make sure this dump is NEWER than the current body definition")
    markers = [(Rx @ np.array(m) + r).tolist() for m in markers_body]

    verts, faces = load_stl_decimated(
        os.path.join(ds, 'phone_poses_robot', 'phone_mount_conf3.stl'))
    data = {
        'mesh_v': np.round(verts, 4).tolist(),
        'mesh_f': faces.tolist(),
        'origin': np.round(r, 5).tolist(),
        'axes': np.round(Rx.T, 5).tolist()[0:3] and
                [np.round(Rx[:, k], 5).tolist() for k in range(3)],
        'lens': np.round(lens, 5).tolist(),
        'markers': [np.round(np.array(m), 5).tolist() for m in markers],
        'handeye': handeye,
    }
    out = args.out or os.path.join(ds, 'body_on_mount.html')
    with open(out, 'w') as f:
        f.write(HTML_HEAD + '\nconst DATA = ' + json.dumps(data) + ';\n'
                + HTML_TAIL)
    print(f'wrote {out} ({len(markers)} markers drawn)')


if __name__ == '__main__':
    main()
