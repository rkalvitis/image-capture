#!/usr/bin/env python3
"""3D comparison of COLMAP-refined camera positions vs the guidance priors.

Reads the prior poses (rebuilt from registration.yaml + the CSVs, same math
as colmap_guided.py) and the sim(3)-aligned refined model
(<dataset>/colmap/sparse_refined/0/images.txt), and writes an interactive
HTML page: open circles = prior, solid dots = COLMAP, a line per camera
showing the correction, with x1/x5/x20 displacement exaggeration.

Usage:
  PYTHONPATH=scripts pixi run python scripts/plot_pose_audit.py \\
      datasets/phone_20260826_113325 [-o pose_audit.html]
"""

import argparse
import json
import os

import numpy as np
import yaml

from colmap_guided import build_poses, read_images_txt

HTML_HEAD = r"""<title>Dome Pose Audit</title>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans:wght@400;500;600&family=IBM+Plex+Mono:wght@400;500&display=swap">
<style>
  :root {
    color-scheme: light;
    --surface: #fcfcfb; --panel: #ffffffee; --panel-border: #e4e2dc;
    --ink: #0b0b0b; --ink-2: #52514e; --ink-3: #8a887f;
    --accent: #eb6834; --rig: #6b6963; --grid: #e8e6e0;
    --b1: #a8c9ef; --b2: #7fb0e8; --b3: #4f92de;
    --b4: #2a78d6; --b5: #1d5cab; --b6: #133f79;
    --chip-bg: #f3f2ee;
  }
  @media (prefers-color-scheme: dark) {
    :root:not([data-theme="light"]) {
      color-scheme: dark;
      --surface: #1a1a19; --panel: #232322ee; --panel-border: #3a3936;
      --ink: #ffffff; --ink-2: #c3c2b7; --ink-3: #8b897f;
      --accent: #f0784a; --rig: #a09e94; --grid: #31302d;
      --b1: #b7d3f3; --b2: #8ebced; --b3: #62a0e8;
      --b4: #3987e5; --b5: #578fd0; --b6: #7ea6d8;
      --chip-bg: #2b2b29;
    }
  }
  :root[data-theme="dark"] {
    color-scheme: dark;
    --surface: #1a1a19; --panel: #232322ee; --panel-border: #3a3936;
    --ink: #ffffff; --ink-2: #c3c2b7; --ink-3: #8b897f;
    --accent: #f0784a; --rig: #a09e94; --grid: #31302d;
    --b1: #b7d3f3; --b2: #8ebced; --b3: #62a0e8;
    --b4: #3987e5; --b5: #578fd0; --b6: #7ea6d8;
    --chip-bg: #2b2b29;
  }
  * { margin: 0; box-sizing: border-box; }
  html, body { height: 100%; }
  body {
    background: var(--surface); color: var(--ink);
    font-family: "IBM Plex Sans", "Helvetica Neue", Arial, sans-serif;
    overflow: hidden;
  }
  #plot { position: fixed; inset: 0; }
  .panel {
    position: fixed; top: 16px; left: 16px;
    background: var(--panel); border: 1px solid var(--panel-border);
    border-radius: 10px; padding: 14px 16px 12px; max-width: 300px;
    backdrop-filter: blur(6px);
    display: flex; flex-direction: column; gap: 10px; z-index: 5;
  }
  h1 { font-size: 15px; font-weight: 600; }
  .sub { font-size: 12px; color: var(--ink-2); line-height: 1.45; }
  .toggle { display: flex; border: 1px solid var(--panel-border);
            border-radius: 7px; overflow: hidden; }
  .toggle button {
    flex: 1; border: 0; padding: 6px 4px; background: transparent;
    color: var(--ink-2); font: 500 12px "IBM Plex Sans", sans-serif;
    cursor: pointer;
  }
  .toggle button.on { background: var(--chip-bg); color: var(--ink);
                      font-weight: 600; }
  .toggle button:focus-visible { outline: 2px solid var(--b4);
                                 outline-offset: -2px; }
  .stats { display: flex; flex-wrap: wrap; gap: 5px; }
  .chip { background: var(--chip-bg); border-radius: 6px; padding: 4px 8px;
          font: 400 11px "IBM Plex Mono", monospace; color: var(--ink-2); }
  .chip b { color: var(--ink); font-weight: 500; }
  .hint { font-size: 11px; color: var(--ink-3); }
</style>
<div id="plot" aria-label="3D comparison of prior and COLMAP camera positions"></div>
<div class="panel">
  <div>
    <h1>Dome pose audit</h1>
    <div class="sub">Open circles = guidance priors (robot CSV chain).
    Solid dots = COLMAP bundle-adjusted, sim(3)-aligned back to the beetle
    frame. Lines show each camera&rsquo;s correction.</div>
  </div>
  <div class="toggle" role="group" aria-label="Displacement exaggeration">
    <button data-k="1" class="on">&Delta;&times;1</button>
    <button data-k="5">&Delta;&times;5</button>
    <button data-k="20">&Delta;&times;20</button>
  </div>
  <div class="stats" id="chips"></div>
  <div class="hint">Corrections are millimetres on a ~10 cm dome &mdash; use
  the exaggeration to see the pattern. Hover any dot for the image name and
  the correction in mm.</div>
</div>
<script src="https://cdnjs.cloudflare.com/ajax/libs/plotly.js/2.35.3/plotly.min.js"></script>
<script>
"use strict";
"""

HTML_TAIL = r"""
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
let K = 1;

function traces() {
  const C = [css("--b1"), css("--b2"), css("--b3"), css("--b4"), css("--b5"), css("--b6")];
  const out = [];
  for (let b = 1; b <= 6; b++) {
    const cams = DATA.cams.filter(c => c.block === b);
    const disp = cams.map(c => [
      c.prior[0] + K * (c.colmap[0] - c.prior[0]),
      c.prior[1] + K * (c.colmap[1] - c.prior[1]),
      c.prior[2] + K * (c.colmap[2] - c.prior[2])]);
    out.push({
      type: "scatter3d", mode: "markers",
      x: cams.map(c => c.prior[0]), y: cams.map(c => c.prior[1]),
      z: cams.map(c => c.prior[2]),
      marker: { size: 6, color: "rgba(0,0,0,0)",
                line: { color: C[b - 1], width: 2 }, symbol: "circle" },
      name: "block " + b + " prior", legendgroup: "b" + b,
      text: cams.map(c => c.img),
      hovertemplate: "<b>%{text}</b> prior<extra></extra>",
    });
    out.push({
      type: "scatter3d", mode: "markers",
      x: disp.map(p => p[0]), y: disp.map(p => p[1]), z: disp.map(p => p[2]),
      marker: { size: 5.5, color: C[b - 1] },
      name: "block " + b + " colmap", legendgroup: "b" + b, showlegend: false,
      text: cams.map(c => c.img + " — " + c.mm.toFixed(1) + " mm"),
      hovertemplate: "<b>%{text}</b> (COLMAP)<extra></extra>",
    });
    const lx = [], ly = [], lz = [];
    cams.forEach((c, i) => {
      lx.push(c.prior[0], disp[i][0], null);
      ly.push(c.prior[1], disp[i][1], null);
      lz.push(c.prior[2], disp[i][2], null);
    });
    out.push({
      type: "scatter3d", mode: "lines",
      x: lx, y: ly, z: lz,
      line: { color: C[b - 1], width: 3 },
      legendgroup: "b" + b, showlegend: false, hoverinfo: "skip",
      name: "corr " + b,
    });
  }
  out.push({
    type: "scatter3d", mode: "markers",
    x: [DATA.beetle[0]], y: [DATA.beetle[1]], z: [DATA.beetle[2]],
    marker: { size: 9, color: css("--accent"), symbol: "diamond",
              line: { color: css("--surface"), width: 1.5 } },
    name: "popillia",
    hovertemplate: "<b>popillia</b><extra></extra>",
  });
  return out;
}

function layout() {
  const ax = t => ({
    title: { text: t, font: { family: "IBM Plex Mono, monospace", size: 11 } },
    color: css("--ink-2"), gridcolor: css("--grid"),
    zerolinecolor: css("--grid"), backgroundcolor: "rgba(0,0,0,0)",
    tickfont: { family: "IBM Plex Mono, monospace", size: 9 },
  });
  return {
    paper_bgcolor: css("--surface"),
    margin: { l: 0, r: 0, t: 0, b: 0 },
    scene: { aspectmode: "data", xaxis: ax("x (m)"), yaxis: ax("y (m)"),
             zaxis: ax("z (m)"), bgcolor: "rgba(0,0,0,0)",
             camera: { eye: { x: 0.9, y: -1.4, z: 0.5 } } },
    showlegend: true,
    legend: { x: 1, xanchor: "right", y: 0.02, yanchor: "bottom",
              bgcolor: css("--panel"), bordercolor: css("--panel-border"),
              borderwidth: 1,
              font: { family: "IBM Plex Sans, sans-serif", size: 11,
                      color: css("--ink-2") } },
    hoverlabel: { bgcolor: css("--panel"), bordercolor: css("--panel-border"),
                  font: { family: "IBM Plex Mono, monospace", size: 11,
                          color: css("--ink") } },
  };
}

function render(keep) {
  const lay = layout();
  const gd = document.getElementById("plot");
  if (keep && gd._fullLayout && gd._fullLayout.scene) {
    lay.scene.camera = gd._fullLayout.scene.camera;
  }
  Plotly.react("plot", traces(), lay,
    { displaylogo: false, responsive: true });
}

const mm = DATA.cams.map(c => c.mm).sort((a, b) => a - b);
const mean = DATA.cams.reduce((s, c) => s + c.mm, 0) / DATA.cams.length;
document.getElementById("chips").innerHTML =
  `<span class="chip">mean <b>${mean.toFixed(2)} mm</b></span>` +
  `<span class="chip">median <b>${mm[Math.floor(mm.length / 2)].toFixed(2)} mm</b></span>` +
  `<span class="chip">max <b>${mm[mm.length - 1].toFixed(1)} mm</b></span>` +
  `<span class="chip">scale <b>${DATA.scale.toFixed(4)}</b></span>`;

for (const btn of document.querySelectorAll(".toggle button")) {
  btn.addEventListener("click", () => {
    K = Number(btn.dataset.k);
    for (const b of document.querySelectorAll(".toggle button")) {
      b.classList.toggle("on", b === btn);
    }
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
    ap.add_argument('-o', '--out', default=None,
                    help='output HTML (default <dataset>/pose_audit.html)')
    args = ap.parse_args()
    ds = args.dataset.rstrip('/')

    priors, _, _ = build_poses(ds)
    refined = read_images_txt(os.path.join(ds, 'colmap', 'sparse_refined',
                                           '0', 'images.txt'))
    reg = yaml.safe_load(open(os.path.join(ds, 'registration.yaml')))
    beetle = reg['object_center_robot_on_axis']

    csvs_n = 8   # shots per block
    names = sorted(priors)
    cams = []
    for i, n in enumerate(names):
        if n not in refined:
            continue
        Cp = priors[n][1]
        Cr = refined[n][1]
        cams.append({
            'img': n.replace('.jpg', ''),
            'block': i // csvs_n + 1,
            'prior': np.round(Cp, 5).tolist(),
            'colmap': np.round(Cr, 5).tolist(),
            'mm': float(np.linalg.norm(Cr - Cp) * 1000),
        })
    # the sim(3) scale that was applied is recorded here only informatively:
    # re-derive it from the two centred point sets
    A = np.array([c['colmap'] for c in cams])
    B = np.array([c['prior'] for c in cams])
    scale_info = float(np.sqrt(((B - B.mean(0)) ** 2).sum()
                               / ((A - A.mean(0)) ** 2).sum()))
    data = {'cams': cams, 'beetle': beetle, 'scale': scale_info}

    out = args.out or os.path.join(ds, 'pose_audit.html')
    with open(out, 'w') as f:
        f.write(HTML_HEAD + '\nconst DATA = ' + json.dumps(data) + ';\n'
                + HTML_TAIL)
    mm = sorted(c['mm'] for c in cams)
    print(f'wrote {out}: {len(cams)} cameras, mean {np.mean(mm):.2f} mm, '
          f'median {mm[len(mm)//2]:.2f} mm, max {mm[-1]:.1f} mm')


if __name__ == '__main__':
    main()
