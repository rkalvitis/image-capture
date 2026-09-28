#!/usr/bin/env python3
"""Full rig assembly in the phone BODY frame (31.08 session only + STLs).

Places, in the marker-body frame:
  * the mount BASE mesh ('phone mount base only.stl', same mesh frame as
    phone_mount_conf3: mm, DIN flange origin) via the session-only
    body<->flange chain (camera_in_body + the STL 45-deg cradle constraint),
  * the iPhone 15 Pro model, posed so its ultra-wide ring sits exactly at
    the hand-eye camera position, back normal along the optical axis,
    phone top opposite sensor +x,
  * the 5 markers: m2/m5 directly on the base, m1/m4 on wooden stick
    extensions (drawn as sticks to the base), m3 on the PHONE CASE
    (validated against the phone model, not the base),
  * the camera + optical axis.

Usage:
  PYTHONPATH=scripts pixi run python scripts/plot_rig_assembly.py \\
      [-o rig_assembly.html]
"""

import argparse
import json

import numpy as np
import yaml

DEFAULT_DS = 'datasets/aug31_20deg_9vertical'


def quat_mat(q):
    x, y, z, w = np.asarray(q, float)
    n = np.linalg.norm([x, y, z, w])
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load_stl(path, grid_mm=2.5):
    raw = open(path, 'rb').read()
    n = int.from_bytes(raw[80:84], 'little')
    rec = np.frombuffer(raw[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
    tris = rec[:, 12:48].copy().view('<f4').reshape(n * 3, 3).astype(np.float64)
    keys = np.round(tris / grid_mm).astype(np.int64)
    uniq, inv = np.unique(keys, axis=0, return_inverse=True)
    verts = np.zeros((len(uniq), 3))
    counts = np.bincount(inv, minlength=len(uniq)).astype(float)
    for k in range(3):
        verts[:, k] = np.bincount(inv, weights=tris[:, k],
                                  minlength=len(uniq)) / counts
    faces = inv.reshape(n, 3)
    good = ((faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2])
            & (faces[:, 0] != faces[:, 2]))
    faces = np.unique(np.sort(faces[good], axis=1), axis=0)
    return verts * 0.001, faces   # -> metres, native mesh frame


def main():
    import os
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('dataset', nargs='?', default=DEFAULT_DS,
                    help='dataset folder containing calibrations/rig_transforms.yaml')
    ap.add_argument('-o', '--out', default=None)
    args = ap.parse_args()
    ds = args.dataset.rstrip('/')
    cal = os.path.join(ds, 'calibrations')
    rt = yaml.safe_load(open(os.path.join(cal, 'rig_transforms.yaml')))

    # EVERYTHING below comes from rig_transforms.yaml (+ the dataset meshes)
    R_bc = np.array(rt['T_body_from_cam']['rotation_matrix'])
    p_b = np.array(rt['T_body_from_cam']['translation_m'])
    R_fb = np.array(rt['T_flange_from_body']['rotation_matrix'])
    t_fb = np.array(rt['T_flange_from_body']['translation_m'])
    R_bm = np.array(rt['T_body_from_phone_model']['rotation_matrix'])
    t_bm = np.array(rt['T_body_from_phone_model']['translation_m'])
    M = np.array([rt['markers_in_body_m'][f'm{i}']['position'] for i in range(1, 6)])
    b2l = rt['base_mesh_to_link8']

    bv, bf = load_stl(os.path.join(cal, rt['meshes']['base']))
    a = np.radians(b2l['yaw_deg'])
    Rz = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0],
                   [0, 0, 1]])
    bv = bv @ Rz.T
    bv[:, 2] += b2l['z_offset_m']
    bv_body = (R_fb.T @ (bv - t_fb).T).T

    pv, pf = load_stl(os.path.join(cal, rt['meshes']['phone_model']))
    pv_body = (R_bm @ pv.T).T + t_bm

    # sticks: m1/m4 to nearest base vertex
    sticks = []
    for mi in (0, 3):
        d = np.linalg.norm(bv_body - M[mi], axis=1)
        sticks.append([M[mi].tolist(), bv_body[np.argmin(d)].tolist()])

    data = {
        'base_v': np.round(bv_body, 4).tolist(), 'base_f': bf.tolist(),
        'phone_v': np.round(pv_body, 4).tolist(), 'phone_f': pf.tolist(),
        'markers': np.round(M, 5).tolist(),
        'sticks': sticks,
        'cam': np.round(p_b, 5).tolist(),
        'cam_axis': np.round(R_bc @ np.array([0, 0, 1.0]), 4).tolist(),
    }
    out = args.out or os.path.join(ds, 'rig_assembly.html')
    with open(out, 'w') as f:
        f.write(HTML_HEAD + '\nconst DATA = ' + json.dumps(data) + ';\n'
                + HTML_TAIL)
    print('wrote', out,
          f'(base {len(bf)} tris, phone {len(pf)} tris)')


HTML_HEAD = r"""<title>Rig Assembly</title>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans:wght@400;500;600&family=IBM+Plex+Mono:wght@400;500&display=swap">
<style>
  :root {
    color-scheme: light;
    --surface: #fcfcfb; --panel: #ffffffee; --panel-border: #e4e2dc;
    --ink: #0b0b0b; --ink-2: #52514e; --ink-3: #8a887f;
    --base: #7f9db9; --phone: #52514e; --marker: #eda100;
    --cam: #e87ba4; --grid: #e8e6e0; --chip-bg: #f3f2ee;
  }
  @media (prefers-color-scheme: dark) {
    :root:not([data-theme="light"]) {
      color-scheme: dark;
      --surface: #1a1a19; --panel: #232322ee; --panel-border: #3a3936;
      --ink: #ffffff; --ink-2: #c3c2b7; --ink-3: #8b897f;
      --base: #5d7d9c; --phone: #8b897f; --marker: #c98500;
      --cam: #d55181; --grid: #31302d; --chip-bg: #2b2b29;
    }
  }
  :root[data-theme="dark"] {
    color-scheme: dark;
    --surface: #1a1a19; --panel: #232322ee; --panel-border: #3a3936;
    --ink: #ffffff; --ink-2: #c3c2b7; --ink-3: #8b897f;
    --base: #5d7d9c; --phone: #8b897f; --marker: #c98500;
    --cam: #d55181; --grid: #31302d; --chip-bg: #2b2b29;
  }
  * { margin: 0; box-sizing: border-box; }
  html, body { height: 100%; }
  body { background: var(--surface); color: var(--ink);
         font-family: "IBM Plex Sans", "Helvetica Neue", Arial, sans-serif;
         overflow: hidden; }
  #plot { position: fixed; inset: 0; }
  .panel { position: fixed; top: 16px; left: 16px;
           background: var(--panel); border: 1px solid var(--panel-border);
           border-radius: 10px; padding: 14px 16px 12px; max-width: 320px;
           backdrop-filter: blur(6px);
           display: flex; flex-direction: column; gap: 10px; z-index: 5; }
  h1 { font-size: 15px; font-weight: 600; }
  .sub { font-size: 12px; color: var(--ink-2); line-height: 1.45; }
  .hint { font-size: 11px; color: var(--ink-3); }
</style>
<div id="plot" aria-label="Full rig assembly in the body frame"></div>
<div class="panel">
  <div>
    <h1>Rig assembly</h1>
    <div class="sub">Body frame, 31.08 session + STL geometry only: mount
    base, iPhone model seated on the estimated lens position, all 5 balls
    (m2/m5 on the base, m1/m4 on wooden-stick extensions, m3 on the phone
    case), camera + optical axis.</div>
  </div>
  <div class="hint">Verify: every ball on its physical perch, sticks
  pointing at their glue spots, m3 on the case. Drag to orbit.</div>
</div>
<script src="https://cdnjs.cloudflare.com/ajax/libs/plotly.js/2.35.3/plotly.min.js"></script>
<script>
"use strict";
"""

HTML_TAIL = r"""
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();

function mesh(v, f, color, opacity, name) {
  return { type: "mesh3d",
    x: v.map(p=>p[0]), y: v.map(p=>p[1]), z: v.map(p=>p[2]),
    i: f.map(t=>t[0]), j: f.map(t=>t[1]), k: f.map(t=>t[2]),
    color: color, opacity: opacity, flatshading: true,
    lighting: { ambient: 0.55, diffuse: 0.7, specular: 0.05 },
    hoverinfo: "skip", name: name, showlegend: true };
}

function traces() {
  const out = [];
  out.push(mesh(DATA.base_v, DATA.base_f, css("--base"), 0.5, "mount base"));
  out.push(mesh(DATA.phone_v, DATA.phone_f, css("--phone"), 0.55, "iPhone (cased)"));
  const M = DATA.markers;
  const lbl = ["m1 (stick)", "m2 (base)", "m3 (PHONE case)", "m4 (stick)", "m5 (base)"];
  out.push({ type: "scatter3d", mode: "markers+text",
    x: M.map(m=>m[0]), y: M.map(m=>m[1]), z: M.map(m=>m[2]),
    marker: { size: 10, color: css("--marker"),
              line: { color: css("--surface"), width: 1.5 } },
    text: lbl.map((_,i)=>"m"+(i+1)), textposition: "top center",
    textfont: { family: "IBM Plex Mono, monospace", size: 12,
                color: css("--ink-2") },
    name: "markers",
    customdata: lbl,
    hovertemplate: "<b>%{customdata}</b><br>(%{x:.4f}, %{y:.4f}, %{z:.4f}) m" +
      "<extra></extra>" });
  for (const s of DATA.sticks) {
    out.push({ type: "scatter3d", mode: "lines",
      x: [s[0][0], s[1][0]], y: [s[0][1], s[1][1]], z: [s[0][2], s[1][2]],
      line: { color: css("--ink-3"), width: 4 },
      name: "stick", showlegend: false, hoverinfo: "skip" });
  }
  const c = DATA.cam, a = DATA.cam_axis, L = 0.06;
  out.push({ type: "scatter3d", mode: "markers",
    x: [c[0]], y: [c[1]], z: [c[2]],
    marker: { size: 8, color: css("--cam"), symbol: "x" },
    name: "camera (hand-eye)",
    hovertemplate: "<b>camera</b><extra></extra>" });
  out.push({ type: "scatter3d", mode: "lines",
    x: [c[0], c[0]+L*a[0]], y: [c[1], c[1]+L*a[1]], z: [c[2], c[2]+L*a[2]],
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


if __name__ == '__main__':
    main()
