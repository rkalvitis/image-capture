#!/usr/bin/env python3
"""Rotatable 3D dome of the ring-protocol dataset camera positions.

Reads <dataset>/camera_poses.csv (build_ring_poses.py) and renders every
camera in the INSECT (popillia body) frame - the frame the splat will be
trained in - with a lab-frame toggle. Rings colored by recording order;
hover shows image, ring, insect yaw, sharpness.

Usage:
  PYTHONPATH=scripts pixi run python scripts/plot_ring_dome.py \\
      datasets/aug31_20deg_9vertical [-o ring_dome.html]
"""

import argparse
import csv
import json
import os

import numpy as np
from link_robot_mocap import quat_mat

HTML_HEAD = r"""<title>Ring Capture Dome</title>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans:wght@400;500;600&family=IBM+Plex+Mono:wght@400;500&display=swap">
<style>
  :root {
    color-scheme: light;
    --surface: #fcfcfb; --panel: #ffffffee; --panel-border: #e4e2dc;
    --ink: #0b0b0b; --ink-2: #52514e; --ink-3: #8a887f;
    --accent: #eb6834; --rig: #6b6963; --grid: #e8e6e0; --chip-bg: #f3f2ee;
  }
  @media (prefers-color-scheme: dark) {
    :root:not([data-theme="light"]) {
      color-scheme: dark;
      --surface: #1a1a19; --panel: #232322ee; --panel-border: #3a3936;
      --ink: #ffffff; --ink-2: #c3c2b7; --ink-3: #8b897f;
      --accent: #f0784a; --rig: #a09e94; --grid: #31302d; --chip-bg: #2b2b29;
    }
  }
  :root[data-theme="dark"] {
    color-scheme: dark;
    --surface: #1a1a19; --panel: #232322ee; --panel-border: #3a3936;
    --ink: #ffffff; --ink-2: #c3c2b7; --ink-3: #8b897f;
    --accent: #f0784a; --rig: #a09e94; --grid: #31302d; --chip-bg: #2b2b29;
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
  .toggle { display: flex; border: 1px solid var(--panel-border);
            border-radius: 7px; overflow: hidden; }
  .toggle button { flex: 1; border: 0; padding: 6px 4px;
                   background: transparent; color: var(--ink-2);
                   font: 500 12px "IBM Plex Sans", sans-serif; cursor: pointer; }
  .toggle button.on { background: var(--chip-bg); color: var(--ink);
                      font-weight: 600; }
  .stats { display: flex; flex-wrap: wrap; gap: 5px; }
  .chip { background: var(--chip-bg); border-radius: 6px; padding: 4px 8px;
          font: 400 11px "IBM Plex Mono", monospace; color: var(--ink-2); }
  .chip b { color: var(--ink); font-weight: 500; }
  .hint { font-size: 11px; color: var(--ink-3); }
</style>
<div id="plot" aria-label="Ring dataset camera positions in 3D"></div>
<div class="panel">
  <div>
    <h1>Ring capture dome</h1>
    <div class="sub">Every camera in the INSECT frame (mocap-measured, 10&deg;
    steps): full rings at descending heights. Toggle to the lab frame to see
    the cameras parked per height while the insect spins.</div>
  </div>
  <div class="toggle" role="group" aria-label="Frame">
    <button id="b-ins" class="on">Insect frame</button>
    <button id="b-lab">Lab frame</button>
  </div>
  <div class="stats" id="chips"></div>
  <div class="hint">Hover a dot: image · ring · insect yaw · sharpness
  (low = defocused; the pose-8 ring is scored per image). Click legend rings
  to hide/show.</div>
</div>
<script src="https://cdnjs.cloudflare.com/ajax/libs/plotly.js/2.35.3/plotly.min.js"></script>
<script>
"use strict";
"""

HTML_TAIL = r"""
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
let FRAME = "ins";

// 10-step single-hue ramp (recording order = height order, mostly)
const RAMP = ["#c7ddf6","#a8c9ef","#8ab5e8","#6ba0e0","#4f8cd8","#3a78cd",
              "#2a64bd","#1f51a6","#163f8a","#0e2f6d"];
const RAMP_D = ["#cfe2f7","#b3d0f1","#97bdeb","#7caae4","#6497dd","#4f85d5",
                "#3d73ca","#2f62bb","#2451a8","#1b4190"];

function traces() {
  const dark = matchMedia("(prefers-color-scheme: dark)").matches &&
    document.documentElement.getAttribute("data-theme") !== "light" ||
    document.documentElement.getAttribute("data-theme") === "dark";
  const ramp = dark ? RAMP_D : RAMP;
  const out = [];
  const key = FRAME === "ins" ? "ins" : "lab";
  const rings = [...new Set(DATA.cams.map(c => c.ring))].sort((a,b)=>a-b);
  for (const g of rings) {
    const cs = DATA.cams.filter(c => c.ring === g);
    out.push({ type: "scatter3d", mode: "markers",
      x: cs.map(c=>c[key][0]), y: cs.map(c=>c[key][1]), z: cs.map(c=>c[key][2]),
      marker: { size: 4.5, color: ramp[(g-1) % ramp.length],
                line: { color: css("--surface"), width: 0.5 } },
      name: "pose " + (g-1) + " (" + cs.length + ")",
      text: cs.map(c => c.img + " · pose " + (g-1) + " · yaw " +
                   c.yaw.toFixed(0) + "° · sharp " + c.sharp.toFixed(0)),
      hovertemplate: "%{text}<extra></extra>" });
  }
  const I = DATA.insect, RL = DATA.rod_lo;
  out.push({ type: "scatter3d", mode: "markers",
    x: [I[0]], y: [I[1]], z: [I[2]],
    marker: { size: 9, color: css("--accent"), symbol: "diamond" },
    name: "insect (on the fitted axis)",
    hovertemplate: "<b>insect</b> on the circle-fit rotation axis" +
      "<extra></extra>" });
  out.push({ type: "scatter3d", mode: "lines",
    x: [RL[0], I[0]], y: [RL[1], I[1]], z: [RL[2], I[2]],
    line: { color: css("--rig"), width: 4 },
    name: "rod (true axis)", showlegend: false, hoverinfo: "skip" });
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
             camera: { eye: { x: 1.2, y: -1.2, z: 0.5 } } },
    showlegend: true,
    legend: { x: 1, xanchor: "right", y: 0.02, yanchor: "bottom",
              bgcolor: css("--panel"), bordercolor: css("--panel-border"),
              borderwidth: 1,
              font: { family: "IBM Plex Sans, sans-serif", size: 10,
                      color: css("--ink-2") } },
    hoverlabel: { bgcolor: css("--panel"), bordercolor: css("--panel-border"),
                  font: { family: "IBM Plex Mono, monospace", size: 11,
                          color: css("--ink") } } };
}

function render(keep) {
  const lay = layout();
  const gd = document.getElementById("plot");
  if (keep && gd._fullLayout && gd._fullLayout.scene)
    lay.scene.camera = gd._fullLayout.scene.camera;
  Plotly.react("plot", traces(), lay, { displaylogo: false, responsive: true });
}

document.getElementById("chips").innerHTML =
  `<span class="chip">photos <b>${DATA.cams.length}</b></span>` +
  `<span class="chip">rings <b>${new Set(DATA.cams.map(c=>c.ring)).size}</b></span>` +
  `<span class="chip">step <b>−10.0&deg;</b></span>` +
  `<span class="chip">clock <b>${DATA.offset_s.toFixed(2)} s</b></span>`;

for (const [id, fr] of [["b-ins","ins"],["b-lab","lab"]]) {
  document.getElementById(id).addEventListener("click", () => {
    FRAME = fr;
    document.getElementById("b-ins").classList.toggle("on", fr==="ins");
    document.getElementById("b-lab").classList.toggle("on", fr==="lab");
    render(true);
  });
}
render(false);
matchMedia("(prefers-color-scheme: dark)").addEventListener("change", () => render(true));
new MutationObserver(() => render(true))
  .observe(document.documentElement, { attributes: true, attributeFilter: ["data-theme"] });
</script>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('dataset')
    ap.add_argument('-o', '--out', default=None)
    args = ap.parse_args()
    ds = args.dataset.rstrip('/')
    rows = list(csv.DictReader(open(os.path.join(ds, 'camera_poses.csv'))))
    cams = []
    for r in rows:
        cams.append({
            'img': r['image'],
            'ring': int(r['ring']),
            'ins': [float(r['cam_x']), float(r['cam_y']), float(r['cam_z'])],
            'lab': [float(v) for v in r['phone_world'].split()],
            'yaw': float(r['popillia_yaw_deg']),
            'sharp': float(r['sharpness']),
        })
    # insect position: ON the fitted rotation axis (the body origin is the
    # marker centroid, which orbits the axis - insect_axis.yaml from
    # build_ring_poses --axis-only), at the axis point nearest the camera
    # cloud centroid (where the dome aims)
    import yaml
    ax = yaml.safe_load(open(os.path.join(ds, 'insect_axis.yaml')))
    a_pt = np.array(ax['axis_point_insect_frame_m'])
    a_dir = np.array(ax['axis_dir_insect_frame'])
    a_dir = a_dir / np.linalg.norm(a_dir)
    cloud = np.mean([c['ins'] for c in cams], axis=0)
    h = float((cloud - a_pt) @ a_dir)
    insect = a_pt + h * a_dir
    rod_lo = a_pt + (h - 0.45) * a_dir
    data = {'cams': cams, 'offset_s': -0.15,
            'insect': np.round(insect, 4).tolist(),
            'rod_lo': np.round(rod_lo, 4).tolist()}
    out = args.out or os.path.join(ds, 'ring_dome.html')
    with open(out, 'w') as f:
        f.write(HTML_HEAD + '\nconst DATA = ' + json.dumps(data) + ';\n'
                + HTML_TAIL)
    print('wrote', out, f'({len(cams)} cameras)')


if __name__ == '__main__':
    main()
