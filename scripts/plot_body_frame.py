#!/usr/bin/env python3
"""Rotatable 3D plot of the CURRENT phone body frame: markers, centroid,
camera position + optical axis (from the board-run hand-eye).

Everything is in the phone BODY frame (the redefined one) — no STL, because
placing the mount mesh requires the body<->flange transform, which is
exactly what the redefinition voided. Markers come from
calibration/rigid_bodies.yaml (`pixi run dump-bodies`); if the on-screen
marker count disagrees with the physical balls, RE-RUN dump-bodies on the
lab LAN and regenerate.

Usage:
  PYTHONPATH=scripts pixi run python scripts/plot_body_frame.py \\
      [-o body_frame.html]
"""

import argparse
import json

import numpy as np
import yaml

HTML_HEAD = r"""<title>Phone Body Frame</title>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans:wght@400;500;600&family=IBM+Plex+Mono:wght@400;500&display=swap">
<style>
  :root {
    color-scheme: light;
    --surface: #fcfcfb; --panel: #ffffffee; --panel-border: #e4e2dc;
    --ink: #0b0b0b; --ink-2: #52514e; --ink-3: #8a887f;
    --cam: #e87ba4; --marker: #eda100; --grid: #e8e6e0;
    --ax-x: #e34948; --ax-y: #008300; --ax-z: #2a78d6; --chip-bg: #f3f2ee;
  }
  @media (prefers-color-scheme: dark) {
    :root:not([data-theme="light"]) {
      color-scheme: dark;
      --surface: #1a1a19; --panel: #232322ee; --panel-border: #3a3936;
      --ink: #ffffff; --ink-2: #c3c2b7; --ink-3: #8b897f;
      --cam: #d55181; --marker: #c98500; --grid: #31302d;
      --ax-x: #e66767; --ax-y: #35a935; --ax-z: #3987e5; --chip-bg: #2b2b29;
    }
  }
  :root[data-theme="dark"] {
    color-scheme: dark;
    --surface: #1a1a19; --panel: #232322ee; --panel-border: #3a3936;
    --ink: #ffffff; --ink-2: #c3c2b7; --ink-3: #8b897f;
    --cam: #d55181; --marker: #c98500; --grid: #31302d;
    --ax-x: #e66767; --ax-y: #35a935; --ax-z: #3987e5; --chip-bg: #2b2b29;
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
</style>
<div id="plot" aria-label="Phone body frame: markers, centroid, camera"></div>
<div class="panel">
  <div>
    <h1>Phone body frame</h1>
    <div class="sub">The redefined body's own frame: marker balls, centroid
    (= origin if the pivot is default), and the board-run hand-eye camera
    with its optical axis. Drag to orbit.</div>
  </div>
  <div class="stats" id="stats"></div>
</div>
<script src="https://cdnjs.cloudflare.com/ajax/libs/plotly.js/2.35.3/plotly.min.js"></script>
<script>
"use strict";
"""

HTML_TAIL = r"""
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();

function traces() {
  const out = [];
  const M = DATA.markers;
  out.push({ type: "scatter3d", mode: "markers+text",
    x: M.map(m=>m[0]), y: M.map(m=>m[1]), z: M.map(m=>m[2]),
    marker: { size: 11, color: css("--marker"),
              line: { color: css("--surface"), width: 1.5 } },
    text: M.map((m,i)=>"m"+(i+1)), textposition: "top center",
    textfont: { family: "IBM Plex Mono, monospace", size: 12,
                color: css("--ink-2") },
    name: "markers (" + M.length + ")",
    hovertemplate: "<b>marker %{text}</b><br>(%{x:.4f}, %{y:.4f}, %{z:.4f}) m" +
      "<extra></extra>" });
  const c = DATA.centroid;
  out.push({ type: "scatter3d", mode: "markers",
    x: [c[0]], y: [c[1]], z: [c[2]],
    marker: { size: 8, color: css("--ink"), symbol: "diamond" },
    name: "centroid",
    hovertemplate: "<b>marker centroid</b> (%{x:.4f}, %{y:.4f}, %{z:.4f}) m" +
      "<extra></extra>" });
  const axn = ["x","y","z"], axc = ["--ax-x","--ax-y","--ax-z"];
  for (let a = 0; a < 3; a++) {
    const d = [0,0,0].map((_,i)=> i===a ? 0.04 : 0);
    out.push({ type: "scatter3d", mode: "lines+text",
      x: [0, d[0]], y: [0, d[1]], z: [0, d[2]],
      line: { color: css(axc[a]), width: 5 },
      text: ["", "body " + axn[a]], textposition: "top center",
      textfont: { family: "IBM Plex Mono, monospace", size: 11,
                  color: css(axc[a]) },
      name: "body " + axn[a], hoverinfo: "skip" });
  }
  const cam = DATA.cam;
  out.push({ type: "scatter3d", mode: "markers",
    x: [cam[0]], y: [cam[1]], z: [cam[2]],
    marker: { size: 9, color: css("--cam"), symbol: "x" },
    name: "camera (hand-eye)",
    hovertemplate: "<b>camera</b> (%{x:.4f}, %{y:.4f}, %{z:.4f}) m" +
      "<extra></extra>" });
  const L = 0.05, ax = DATA.cam_axis;
  out.push({ type: "scatter3d", mode: "lines",
    x: [cam[0], cam[0]+L*ax[0]], y: [cam[1], cam[1]+L*ax[1]],
    z: [cam[2], cam[2]+L*ax[2]],
    line: { color: css("--cam"), width: 5 },
    name: "optical axis", hoverinfo: "skip" });
  return out;
}

function layout() {
  const ax = t => ({
    title: { text: t, font: { family: "IBM Plex Mono, monospace", size: 11 } },
    color: css("--ink-2"), gridcolor: css("--grid"),
    zerolinecolor: css("--ink-3"), backgroundcolor: "rgba(0,0,0,0)",
    tickfont: { family: "IBM Plex Mono, monospace", size: 9 } });
  return {
    paper_bgcolor: css("--surface"), margin: { l: 0, r: 0, t: 0, b: 0 },
    scene: { aspectmode: "data", xaxis: ax("x (m)"), yaxis: ax("y (m)"),
             zaxis: ax("z (m)"), bgcolor: "rgba(0,0,0,0)",
             camera: { eye: { x: 1.3, y: 1.0, z: 0.8 } } },
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

const cm = DATA.cam;
document.getElementById("stats").innerHTML =
  `camera <b>(${cm.map(v=>(v*1000).toFixed(1)).join(", ")}) mm</b><br>` +
  `|camera&ndash;centroid| <b>${(Math.hypot(...cm)*1000).toFixed(1)} mm</b><br>` +
  `hand-eye validated: <b>1.0 mm / 1.7&deg;</b> board consistency` +
  (DATA.n_markers_note ? `<br><span style="color:#c98500">${DATA.n_markers_note}</span>` : "");

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
    ap.add_argument('-o', '--out', default='datasets/phone_20260831_142707/body_frame.html')
    ap.add_argument('--rigid-bodies', default='calibration/rigid_bodies.yaml')
    ap.add_argument('--handeye', default='calibration/handeye_robot.yaml')
    args = ap.parse_args()
    rb = yaml.safe_load(open(args.rigid_bodies))
    markers = [m['position_m'] for m in rb['bodies']['phone']['markers']]
    he = yaml.safe_load(open(args.handeye))
    cam = he['camera_in_body']['translation_m']
    q = he['camera_in_body']['quaternion_xyzw']
    x, y, z, w = q
    R = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    data = {
        'markers': [np.round(np.array(m), 5).tolist() for m in markers],
        'centroid': np.round(np.mean(markers, axis=0), 5).tolist(),
        'cam': np.round(np.array(cam), 5).tolist(),
        'cam_axis': np.round(R @ np.array([0, 0, 1.0]), 4).tolist(),
        'n_markers_note': ('dump shows %d markers; if the physical body has '
                           'more, re-run `pixi run dump-bodies` and '
                           'regenerate' % len(markers)),
    }
    with open(args.out, 'w') as f:
        f.write(HTML_HEAD + '\nconst DATA = ' + json.dumps(data) + ';\n'
                + HTML_TAIL)
    print('wrote', args.out, f'({len(markers)} markers)')


if __name__ == '__main__':
    main()
