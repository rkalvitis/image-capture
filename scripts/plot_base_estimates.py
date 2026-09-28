#!/usr/bin/env python3
"""Per-hold robot-base estimates: scatter, average, and the joint solve.

Every quality-gated mocap hold yields an independent closed-form estimate of
the robot base pose in the mocap frame:

    T_moc<-base(k) = T_moc<-body(k) . inv(T_fl<-body) . inv(T_base<-fl(k))

This script computes them all, their average, the joint least-squares solve
(current registration), and the UNGATED joint solve ("before"), and writes a
rotatable 3D plot of the base-origin estimates as mm offsets from the current
solve. Also prints the numbers.

Usage:
  PYTHONPATH=scripts pixi run python scripts/plot_base_estimates.py \\
      datasets/phone_20260826_113325 [-o base_estimates.html]
"""

import argparse
import json
import os

import numpy as np
import yaml

from link_robot_mocap import (init_marker_offset, load_bag_topic,
                              load_run_csv, match_stills_to_waypoints,
                              mat_quat, quat_mat, rot_angle_deg,
                              solve_registration, still_windows)

HTML_HEAD = r"""<title>Robot Base Estimates</title>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans:wght@400;500;600&family=IBM+Plex+Mono:wght@400;500&display=swap">
<style>
  :root {
    color-scheme: light;
    --surface: #fcfcfb; --panel: #ffffffee; --panel-border: #e4e2dc;
    --ink: #0b0b0b; --ink-2: #52514e; --ink-3: #8a887f;
    --b1: #2a78d6; --b2: #eb6834; --avg: #008300; --old: #8a887f;
    --grid: #e8e6e0; --chip-bg: #f3f2ee;
  }
  @media (prefers-color-scheme: dark) {
    :root:not([data-theme="light"]) {
      color-scheme: dark;
      --surface: #1a1a19; --panel: #232322ee; --panel-border: #3a3936;
      --ink: #ffffff; --ink-2: #c3c2b7; --ink-3: #8b897f;
      --b1: #3987e5; --b2: #d95926; --avg: #35a935; --old: #8b897f;
      --grid: #31302d; --chip-bg: #2b2b29;
    }
  }
  :root[data-theme="dark"] {
    color-scheme: dark;
    --surface: #1a1a19; --panel: #232322ee; --panel-border: #3a3936;
    --ink: #ffffff; --ink-2: #c3c2b7; --ink-3: #8b897f;
    --b1: #3987e5; --b2: #d95926; --avg: #35a935; --old: #8b897f;
    --grid: #31302d; --chip-bg: #2b2b29;
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
  .stats { font: 400 11px "IBM Plex Mono", monospace; color: var(--ink-2);
           line-height: 1.7; }
  .stats b { color: var(--ink); font-weight: 500; }
</style>
<div id="plot" aria-label="Per-hold robot base estimates in 3D"></div>
<div class="panel">
  <div>
    <h1>Robot base estimates</h1>
    <div class="sub">Base origin estimated independently from every clean
    mocap hold, shown as mm offsets from the current joint solve (origin).
    Hover a dot for its block/waypoint. Per-hold estimates also carry the
    hold&rsquo;s orientation noise &mdash; the joint solve avoids it, which
    is why it is the reference.</div>
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
  for (const b of [1, 2]) {
    const pts = DATA.holds.filter(h => h.block === b);
    out.push({ type: "scatter3d", mode: "markers",
      x: pts.map(h=>h.d[0]), y: pts.map(h=>h.d[1]), z: pts.map(h=>h.d[2]),
      marker: { size: 6, color: css("--b" + b) },
      name: "block " + b + " holds",
      text: pts.map(h=>`b${h.block} wp${h.wp}`),
      hovertemplate: "<b>%{text}</b><br>(%{x:.1f}, %{y:.1f}, %{z:.1f}) mm" +
        "<extra></extra>" });
  }
  out.push({ type: "scatter3d", mode: "markers",
    x: [DATA.avg[0]], y: [DATA.avg[1]], z: [DATA.avg[2]],
    marker: { size: 10, color: css("--avg"), symbol: "cross" },
    name: "average of holds",
    hovertemplate: "<b>average</b> (%{x:.2f}, %{y:.2f}, %{z:.2f}) mm<extra></extra>" });
  out.push({ type: "scatter3d", mode: "markers",
    x: [0], y: [0], z: [0],
    marker: { size: 10, color: css("--ink"), symbol: "diamond" },
    name: "joint solve (current)",
    hovertemplate: "<b>joint solve</b> = origin<extra></extra>" });
  out.push({ type: "scatter3d", mode: "markers",
    x: [DATA.old[0]], y: [DATA.old[1]], z: [DATA.old[2]],
    marker: { size: 9, color: css("--old"), symbol: "square" },
    name: "before (ungated solve)",
    hovertemplate: "<b>ungated solve</b> (%{x:.1f}, %{y:.1f}, %{z:.1f}) mm" +
      "<extra></extra>" });
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
    scene: { aspectmode: "data",
             xaxis: ax("dx (mm)"), yaxis: ax("dy (mm)"), zaxis: ax("dz (mm)"),
             bgcolor: "rgba(0,0,0,0)",
             camera: { eye: { x: 1.4, y: 1.1, z: 0.8 } } },
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

const n = DATA.holds.length;
const spread = Math.max(...DATA.holds.map(h=>Math.hypot(...h.d)));
const am = Math.hypot(...DATA.avg), om = Math.hypot(...DATA.old);
document.getElementById("stats").innerHTML =
  `<b>${n}</b> clean holds, spread up to <b>${spread.toFixed(1)} mm</b><br>` +
  `average vs joint solve: <b>${am.toFixed(2)} mm</b><br>` +
  `before (ungated) vs current: <b>${om.toFixed(1)} mm</b>`;

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


def gather(ds, gate):
    bag = sorted(f for f in os.listdir(ds) if f.endswith('.db3'))[0]
    t, ph = load_bag_topic(os.path.join(ds, bag), '/optitrack/phone')
    t = t - t[0]
    csvs = sorted(f for f in os.listdir(os.path.join(ds, 'phone_poses_robot'))
                  if f.startswith('manual_poses_2026'))
    holds = []   # (block, wp, T_moc_body, T_fl)
    for bi, (lo, hi) in enumerate([(0, 336), (336, 615)]):
        m = (t >= lo) & (t < hi)
        kw = dict(quality_gate=True, max_dropout=0.8, max_std_mm=1.5,
                  min_samples=60) if gate else {}
        st = still_windows(t[m], ph[m, :3], ph[m, 3:7], **kw)
        T_wp, _ = load_run_csv(os.path.join(ds, 'phone_poses_robot', csvs[bi]))
        for k, w in match_stills_to_waypoints(T_wp, st):
            _, c, q = st[k]
            Tb = np.eye(4)
            Tb[:3, :3] = quat_mat(q)
            Tb[:3, 3] = c
            holds.append((bi + 1, w, Tb, T_wp[w]))
    return holds


def joint_solve(holds):
    T_fl_all = [h[3] for h in holds]
    T_b_all = [h[2] for h in holds]
    p_moc = np.array([h[2][:3, 3] for h in holds])
    p_fl = np.array([h[3][:3, 3] for h in holds])
    R_fl = np.array([h[3][:3, :3] for h in holds])
    r0, _ = init_marker_offset(T_fl_all, T_b_all)
    R, t, r, resid = solve_registration(p_moc, p_fl, R_fl, r0)
    keep = resid < 3 * np.median(resid)
    if keep.sum() < len(keep):
        R, t, r, resid = solve_registration(p_moc[keep], p_fl[keep],
                                            R_fl[keep], r)
    # base origin in the mocap frame: T_moc<-rob translation = -R^T t... here
    # R,t map mocap->robot, so base(0,0,0 in robot) in mocap = R^T (0 - t)
    return R, t, -R.T @ t


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('dataset')
    ap.add_argument('-o', '--out', default=None)
    args = ap.parse_args()
    ds = args.dataset.rstrip('/')
    reg = yaml.safe_load(open(os.path.join(ds, 'registration.yaml')))
    Rx = np.array(reg['R_flange_from_body'])
    r = np.array(reg['marker_body_origin_in_flange_m'])
    T_fl_body = np.eye(4)
    T_fl_body[:3, :3] = Rx
    T_fl_body[:3, 3] = r

    holds = gather(ds, gate=True)
    R_cur, t_cur, base_cur = joint_solve(holds)
    holds_all = gather(ds, gate=False)
    _, _, base_old = joint_solve(holds_all)

    per_hold = []
    for b, w, Tb, Tfl in holds:
        T_moc_rob = Tb @ np.linalg.inv(T_fl_body) @ np.linalg.inv(Tfl)
        base = T_moc_rob[:3, 3]
        per_hold.append((b, w, base))
        d = (base - base_cur) * 1000
        print(f'block {b} wp {w}: base offset from joint solve '
              f'({d[0]:+6.2f}, {d[1]:+6.2f}, {d[2]:+6.2f}) mm  '
              f'|{np.linalg.norm(d):5.2f}| mm')
    avg = np.mean([p[2] for p in per_hold], axis=0)
    print(f'\naverage of {len(per_hold)} holds vs joint solve: '
          f'{np.linalg.norm(avg - base_cur)*1000:.2f} mm')
    print(f'ungated ("before") vs current joint solve: '
          f'{np.linalg.norm(base_old - base_cur)*1000:.1f} mm')

    data = {
        'holds': [{'block': b, 'wp': w,
                   'd': np.round((p - base_cur) * 1000, 2).tolist()}
                  for b, w, p in per_hold],
        'avg': np.round((avg - base_cur) * 1000, 3).tolist(),
        'old': np.round((base_old - base_cur) * 1000, 2).tolist(),
    }
    out = args.out or os.path.join(ds, 'base_estimates.html')
    with open(out, 'w') as f:
        f.write(HTML_HEAD + '\nconst DATA = ' + json.dumps(data) + ';\n'
                + HTML_TAIL)
    print(f'wrote {out}')


if __name__ == '__main__':
    main()
