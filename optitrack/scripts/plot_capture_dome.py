#!/usr/bin/env python3
"""Build the interactive 3D capture-dome plot for a turntable session.

Reads <dataset>/registration.yaml (from scripts/link_robot_mocap.py, with the
camera_axis_solution appended by scripts/solve_camera_axis.py) plus the
phone_poses_robot/*.csv block logs, and writes a self-contained HTML page:
camera position per image (lab frame and beetle frame), calibrated facing
stubs, phone outlines, the rig (rod, axis, marker circle, stage, table,
robot base). Open the file in a browser or publish it as an artifact.

Usage:
  PYTHONPATH=scripts pixi run python scripts/plot_capture_dome.py \
      datasets/phone_20260826_113325 [-o capture_dome.html]
"""

import argparse
import json
import os

import numpy as np
import yaml

from link_robot_mocap import load_run_csv

VIEWER_HEAD = r"""<title>Popillia Capture Dome</title>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans:wght@400;500;600&family=IBM+Plex+Mono:wght@400;500&display=swap">
<style>
  :root {
    color-scheme: light;
    --surface: #fcfcfb;
    --panel: #ffffffee;
    --panel-border: #e4e2dc;
    --ink: #0b0b0b;
    --ink-2: #52514e;
    --ink-3: #8a887f;
    --accent: #eb6834;
    --rig: #6b6963;
    --grid: #e8e6e0;
    --ray: #9a988f;
    --b1: #a8c9ef; --b2: #7fb0e8; --b3: #4f92de;
    --b4: #2a78d6; --b5: #1d5cab; --b6: #133f79;
    --chip-bg: #f3f2ee;
  }
  @media (prefers-color-scheme: dark) {
    :root:not([data-theme="light"]) {
      color-scheme: dark;
      --surface: #1a1a19;
      --panel: #232322ee;
      --panel-border: #3a3936;
      --ink: #ffffff;
      --ink-2: #c3c2b7;
      --ink-3: #8b897f;
      --accent: #f0784a;
      --rig: #a09e94;
      --grid: #31302d;
      --ray: #6e6c64;
      --b1: #b7d3f3; --b2: #8ebced; --b3: #62a0e8;
      --b4: #3987e5; --b5: #578fd0; --b6: #7ea6d8;
      --chip-bg: #2b2b29;
    }
  }
  :root[data-theme="dark"] {
    color-scheme: dark;
    --surface: #1a1a19;
    --panel: #232322ee;
    --panel-border: #3a3936;
    --ink: #ffffff;
    --ink-2: #c3c2b7;
    --ink-3: #8b897f;
    --accent: #f0784a;
    --rig: #a09e94;
    --grid: #31302d;
    --ray: #6e6c64;
    --b1: #b7d3f3; --b2: #8ebced; --b3: #62a0e8;
    --b4: #3987e5; --b5: #578fd0; --b6: #7ea6d8;
    --chip-bg: #2b2b29;
  }
  * { margin: 0; box-sizing: border-box; }
  html, body { height: 100%; }
  body {
    background: var(--surface);
    color: var(--ink);
    font-family: "IBM Plex Sans", "Helvetica Neue", Arial, sans-serif;
    overflow: hidden;
  }
  #plot { position: fixed; inset: 0; }
  .panel {
    position: fixed;
    top: 16px; left: 16px;
    background: var(--panel);
    border: 1px solid var(--panel-border);
    border-radius: 10px;
    padding: 14px 16px 12px;
    max-width: 290px;
    backdrop-filter: blur(6px);
    display: flex; flex-direction: column; gap: 10px;
    z-index: 5;
  }
  h1 { font-size: 15px; font-weight: 600; letter-spacing: .01em; }
  .sub { font-size: 12px; color: var(--ink-2); line-height: 1.45; }
  .toggle {
    display: flex; border: 1px solid var(--panel-border);
    border-radius: 7px; overflow: hidden;
  }
  .toggle button {
    flex: 1; border: 0; padding: 6px 4px;
    background: transparent; color: var(--ink-2);
    font: 500 12px "IBM Plex Sans", sans-serif;
    cursor: pointer;
  }
  .toggle button.on { background: var(--chip-bg); color: var(--ink); font-weight: 600; }
  .toggle button:focus-visible { outline: 2px solid var(--b4); outline-offset: -2px; }
  .blocks { display: grid; grid-template-columns: 1fr 1fr; gap: 3px 12px; }
  .bk {
    display: flex; align-items: center; gap: 7px;
    font: 400 12px "IBM Plex Mono", monospace; color: var(--ink-2);
  }
  .bk i {
    width: 10px; height: 10px; border-radius: 50%;
    flex: none; border: 1px solid var(--panel-border);
  }
  .stats { display: flex; flex-wrap: wrap; gap: 5px; }
  .chip {
    background: var(--chip-bg); border-radius: 6px;
    padding: 4px 8px; font: 400 11px "IBM Plex Mono", monospace;
    color: var(--ink-2);
  }
  .chip b { color: var(--ink); font-weight: 500; }
  .hint { font-size: 11px; color: var(--ink-3); }
  @media (max-width: 640px) {
    .panel { max-width: calc(100vw - 32px); }
  }
</style>

<div id="plot" aria-label="Rotatable 3D scene of camera positions around the beetle"></div>
<div class="panel">
  <div>
    <h1>Popillia capture dome</h1>
    <div class="sub">48 photos, 6 turntable angles &times; 8 arm poses.
    Drag to orbit, scroll to zoom, right-drag to pan.</div>
  </div>
  <div class="toggle" role="group" aria-label="Reference frame">
    <button id="btn-obj" class="on">Beetle frame</button>
    <button id="btn-lab">Lab frame</button>
  </div>
  <div class="sub" id="frame-note">Stage rotation folded into the cameras —
  the dome shows where every shot sits <em>relative to the beetle</em>.</div>
  <div class="blocks" id="legend"></div>
  <div class="stats">
    <span class="chip">reg&nbsp;RMS <b id="st-rms"></b></span>
    <span class="chip">rod <b id="st-rod"></b></span>
    <span class="chip">marker&nbsp;circle&nbsp;r <b id="st-rad"></b></span>
    <span class="chip">lens&ndash;beetle <b>28&ndash;53&nbsp;mm</b></span>
  </div>
  <div class="hint">Solid stubs on each dot = the lens optical axis, calibrated from the
  beetle&rsquo;s position in the photos themselves (2.1&deg; rms). Shots 5&ndash;7
  pivot about the lens (verified vs mocap) &mdash; turn on &ldquo;phone
  outline&rdquo; in the plot legend to see the phone body swing while the lens
  holds still. Hover a dot for the image name.</div>
</div>

<script src="https://cdnjs.cloudflare.com/ajax/libs/plotly.js/2.35.3/plotly.min.js"></script>
<script>
"""

VIEWER_TAIL = r""""use strict";

const css = name =>
  getComputedStyle(document.documentElement).getPropertyValue(name).trim();

let frame = "obj";   // "obj" | "lab"

function blockColors() {
  return [css("--b1"), css("--b2"), css("--b3"), css("--b4"), css("--b5"), css("--b6")];
}

function buildTraces() {
  const C = blockColors();
  const S = SCENE;
  const traces = [];

  // ground plane (z = 0, shared table)
  traces.push({
    type: "mesh3d",
    x: [-0.05, 0.55, 0.55, -0.05], y: [-0.1, -0.1, 0.45, 0.45], z: [0, 0, 0, 0],
    i: [0, 0], j: [1, 2], k: [2, 3],
    color: css("--grid"), opacity: 0.45,
    hoverinfo: "skip", name: "table", showlegend: true, showscale: false,
  });

  // stage platter + marker circle
  const platter = S.platter, circle = S.circle;
  traces.push({
    type: "scatter3d", mode: "lines",
    x: platter.map(p => p[0]), y: platter.map(p => p[1]), z: platter.map(p => p[2]),
    line: { color: css("--rig"), width: 2 },
    hoverinfo: "skip", name: "rotation stage", legendgroup: "stage",
  });
  traces.push({
    type: "scatter3d", mode: "lines",
    x: circle.map(p => p[0]), y: circle.map(p => p[1]), z: circle.map(p => p[2]),
    line: { color: css("--rig"), width: 4 },
    name: "marker circle", legendgroup: "stage", showlegend: false,
    hovertemplate: "popillia marker-centroid circle<br>r = " + S.circle_radius_mm +
      " mm<extra></extra>",
  });

  // rod (marker plane -> beetle) + axis continuation
  const ab = S.axis_base, bt = S.beetle, at = S.axis_top;
  traces.push({
    type: "scatter3d", mode: "lines",
    x: [ab[0], bt[0]], y: [ab[1], bt[1]], z: [ab[2], bt[2]],
    line: { color: css("--rig"), width: 5 },
    name: "rod / axis", legendgroup: "stage", showlegend: false,
    hovertemplate: "rod — " + S.rod_height_mm + " mm above the marker plane<extra></extra>",
  });
  traces.push({
    type: "scatter3d", mode: "lines",
    x: [bt[0], at[0]], y: [bt[1], at[1]], z: [bt[2], at[2]],
    line: { color: css("--rig"), width: 2, dash: "dot" },
    hoverinfo: "skip", legendgroup: "stage", showlegend: false, name: "axis",
  });

  // view rays (camera -> beetle)
  const rx = [], ry = [], rz = [];
  for (const c of S.cams) {
    const p = c[frame];
    rx.push(p[0], bt[0], null); ry.push(p[1], bt[1], null); rz.push(p[2], bt[2], null);
  }
  traces.push({
    type: "scatter3d", mode: "lines",
    x: rx, y: ry, z: rz,
    line: { color: css("--ray"), width: 1 },
    opacity: 0.35, hoverinfo: "skip", name: "rays to beetle", visible: "legendonly",
  });

  // cameras, one trace per block (+ a facing-line trace in the same legend group)
  const dirKey = frame === "obj" ? "dir_obj" : "dir_lab";
  const FACE_LEN = 0.035;   // m, roughly the working distance
  for (let b = 1; b <= 6; b++) {
    const cams = S.cams.filter(c => c.block === b);
    traces.push({
      type: "scatter3d", mode: "markers",
      x: cams.map(c => c[frame][0]),
      y: cams.map(c => c[frame][1]),
      z: cams.map(c => c[frame][2]),
      marker: {
        size: 6.5, color: C[b - 1],
        line: { color: css("--surface"), width: 1 },
        symbol: "circle",
      },
      name: "block " + b + " · " + cams[0].angle + "°",
      legendgroup: "blk" + b,
      text: cams.map(c => c.img),
      customdata: cams.map(c => [c.shot, c.angle]),
      hovertemplate: "<b>%{text}</b><br>block " + b +
        " (stage %{customdata[1]}°) · shot %{customdata[0]}<br>" +
        "x %{x:.4f}  y %{y:.4f}  z %{z:.4f} m<extra></extra>",
    });
    const fx = [], fy = [], fz = [];
    for (const c of cams) {
      const p = c[frame], d = c[dirKey];
      fx.push(p[0], p[0] + FACE_LEN * d[0], null);
      fy.push(p[1], p[1] + FACE_LEN * d[1], null);
      fz.push(p[2], p[2] + FACE_LEN * d[2], null);
    }
    traces.push({
      type: "scatter3d", mode: "lines",
      x: fx, y: fy, z: fz,
      line: { color: C[b - 1], width: 3 },
      opacity: 0.85, hoverinfo: "skip",
      legendgroup: "blk" + b, showlegend: false,
      name: "facing " + b,
    });
  }

  // phone outlines (147 x 71 mm, lens ring at the corner)
  const LONG = 0.147, WIDE = 0.0715;
  const longKey = frame === "obj" ? "body_obj" : "body_lab";
  const wideKey = frame === "obj" ? "wide_obj" : "wide_lab";
  const ox = [], oy = [], oz = [];
  for (const c of S.cams) {
    const p = c[frame], L = c[longKey], Wv = c[wideKey];
    const corners = [
      p,
      [p[0] + LONG * L[0], p[1] + LONG * L[1], p[2] + LONG * L[2]],
      [p[0] + LONG * L[0] + WIDE * Wv[0], p[1] + LONG * L[1] + WIDE * Wv[1],
       p[2] + LONG * L[2] + WIDE * Wv[2]],
      [p[0] + WIDE * Wv[0], p[1] + WIDE * Wv[1], p[2] + WIDE * Wv[2]],
      p,
    ];
    for (const q of corners) { ox.push(q[0]); oy.push(q[1]); oz.push(q[2]); }
    ox.push(null); oy.push(null); oz.push(null);
  }
  traces.push({
    type: "scatter3d", mode: "lines",
    x: ox, y: oy, z: oz,
    line: { color: css("--ray"), width: 1.5 },
    opacity: 0.5, hoverinfo: "skip",
    name: "phone outline", visible: "legendonly",
  });

  // beetle
  traces.push({
    type: "scatter3d", mode: "markers",
    x: [bt[0]], y: [bt[1]], z: [bt[2]],
    marker: { size: 9, color: css("--accent"), symbol: "diamond",
              line: { color: css("--surface"), width: 1.5 } },
    name: "popillia",
    hovertemplate: "<b>popillia</b> (on the rotation axis)<br>" +
      "x %{x:.4f}  y %{y:.4f}  z %{z:.4f} m<extra></extra>",
  });

  // robot base (lab frame only)
  traces.push({
    type: "scatter3d", mode: "markers+text",
    x: [0], y: [0], z: [0],
    marker: { size: 7, color: css("--rig"), symbol: "square" },
    text: ["Panda base"], textposition: "top center",
    textfont: { family: "IBM Plex Mono, monospace", size: 10, color: css("--ink-2") },
    name: "Panda base", visible: frame === "lab",
    hovertemplate: "robot base frame origin<extra></extra>",
  });

  return traces;
}

function layout() {
  const axis = title => ({
    title: { text: title, font: { family: "IBM Plex Mono, monospace", size: 11 } },
    color: css("--ink-2"),
    gridcolor: css("--grid"),
    zerolinecolor: css("--grid"),
    backgroundcolor: "rgba(0,0,0,0)",
    tickfont: { family: "IBM Plex Mono, monospace", size: 9 },
  });
  return {
    paper_bgcolor: css("--surface"),
    margin: { l: 0, r: 0, t: 0, b: 0 },
    scene: {
      aspectmode: "data",
      xaxis: axis("x (m)"), yaxis: axis("y (m)"), zaxis: axis("z (m)"),
      camera: { eye: { x: 0.9, y: -1.4, z: 0.35 },
                center: { x: 0, y: 0, z: -0.05 } },
      bgcolor: "rgba(0,0,0,0)",
    },
    showlegend: true,
    legend: {
      x: 1, xanchor: "right", y: 0.02, yanchor: "bottom",
      bgcolor: css("--panel"), bordercolor: css("--panel-border"), borderwidth: 1,
      font: { family: "IBM Plex Sans, sans-serif", size: 11, color: css("--ink-2") },
    },
    hoverlabel: {
      bgcolor: css("--panel"), bordercolor: css("--panel-border"),
      font: { family: "IBM Plex Mono, monospace", size: 11, color: css("--ink") },
    },
  };
}

function render(keepCamera) {
  const lay = layout();
  if (keepCamera) {
    const gd = document.getElementById("plot");
    if (gd._fullLayout && gd._fullLayout.scene) {
      lay.scene.camera = gd._fullLayout.scene.camera;
    }
  }
  Plotly.react("plot", buildTraces(), lay,
    { displaylogo: false, responsive: true,
      modeBarButtonsToRemove: ["resetCameraLastSave3d"] });
}

// legend swatches + stats in the panel
function paintPanel() {
  const C = blockColors();
  document.getElementById("legend").innerHTML = C.map((c, i) =>
    `<span class="bk"><i style="background:${c}"></i>blk ${i + 1} · ${i * 60}°</span>`
  ).join("");
  document.getElementById("st-rms").textContent = SCENE.rms_mm + " mm";
  document.getElementById("st-rod").textContent = SCENE.rod_height_mm + " mm";
  document.getElementById("st-rad").textContent = SCENE.circle_radius_mm + " mm";
}

const noteObj = "Stage rotation folded into the cameras — the dome shows where " +
  "every shot sits <em>relative to the beetle</em>.";
const noteLab = "Physical positions in the robot base frame — the arm repeats " +
  "the same 8 poses, so the 6 blocks stack on top of each other.";

function setFrame(f) {
  frame = f;
  document.getElementById("btn-obj").classList.toggle("on", f === "obj");
  document.getElementById("btn-lab").classList.toggle("on", f === "lab");
  document.getElementById("frame-note").innerHTML = f === "obj" ? noteObj : noteLab;
  render(true);
}
document.getElementById("btn-obj").addEventListener("click", () => setFrame("obj"));
document.getElementById("btn-lab").addEventListener("click", () => setFrame("lab"));

paintPanel();
render(false);

// re-render on theme change
const mq = matchMedia("(prefers-color-scheme: dark)");
mq.addEventListener("change", () => { paintPanel(); render(true); });
new MutationObserver(() => { paintPanel(); render(true); })
  .observe(document.documentElement, { attributes: true, attributeFilter: ["data-theme"] });
</script>
"""


def build_scene(ds):
    reg = yaml.safe_load(open(os.path.join(ds, "registration.yaml")))
    lens_fl = np.array(reg["lens_offset_in_flange_corrected_m"])
    axis_pt = np.array(reg["turntable_axis_robot"]["point"])
    fold = reg.get("fold_axis_robot", reg["turntable_axis_robot"])
    fold_pt = np.array(fold["point"])
    axis_dir = np.array(fold["dir"])
    axis_dir = axis_dir / np.linalg.norm(axis_dir)
    beetle = np.array(reg["object_center_robot_on_axis"])
    Rcf = np.array(reg["camera_axis_solution"]["R_cam_from_flange"])
    cam_axis_fl = np.array(reg["camera_axis_solution"]["optical_axis_in_flange"])
    long_fl = Rcf.T @ np.array([1.0, 0, 0])   # lens ring -> phone bottom
    wide_fl = Rcf.T @ np.array([0, -1.0, 0])  # lens ring -> across the phone

    def axis_rot(deg):
        a = np.radians(deg)
        k = axis_dir
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * (K @ K)

    csvs = sorted(f for f in os.listdir(os.path.join(ds, "phone_poses_robot"))
                  if f.startswith("manual_poses_2026"))
    images = sorted(i for i in os.listdir(os.path.join(ds, "images"))
                    if i.startswith("IMG"))
    cams = []
    for b, fn in enumerate(csvs):
        T_wp, _ = load_run_csv(os.path.join(ds, "phone_poses_robot", fn))
        Rb = axis_rot(b * 60.0)   # beetle frame: undo the -60 deg/step rotation
        for w in range(1, len(T_wp)):
            R_fl, p_fl = T_wp[w][:3, :3], T_wp[w][:3, 3]
            lens = p_fl + R_fl @ lens_fl
            d = R_fl @ cam_axis_fl
            L = R_fl @ long_fl
            Wd = R_fl @ wide_fl
            cams.append({
                "img": images[b * (len(T_wp) - 1) + (w - 1)],
                "block": b + 1, "shot": w, "angle": b * 60,
                "lab": np.round(lens, 4).tolist(),
                "obj": np.round(fold_pt + Rb @ (lens - fold_pt), 4).tolist(),
                "dir_lab": np.round(d, 4).tolist(),
                "dir_obj": np.round(Rb @ d, 4).tolist(),
                "body_lab": np.round(L, 4).tolist(),
                "body_obj": np.round(Rb @ L, 4).tolist(),
                "wide_lab": np.round(Wd, 4).tolist(),
                "wide_obj": np.round(Rb @ Wd, 4).tolist(),
            })

    u = np.cross(axis_dir, [1, 0, 0])
    u /= np.linalg.norm(u)
    v = np.cross(axis_dir, u)
    r = reg["turntable_axis_mocap"]["circle_radius_m"]
    th = np.linspace(0, 2 * np.pi, 73)
    ring = lambda rad: axis_pt[None, :] + rad * (np.cos(th)[:, None] * u
                                                 + np.sin(th)[:, None] * v)
    return {
        "beetle": np.round(beetle, 4).tolist(),
        "axis_top": np.round(beetle + 0.06 * axis_dir, 4).tolist(),
        "axis_base": np.round(axis_pt, 4).tolist(),
        "circle": np.round(ring(r), 4).tolist(),
        "platter": np.round(ring(0.07), 4).tolist(),
        "rod_height_mm": round(reg["object_height_above_marker_plane_m"] * 1000, 1),
        "circle_radius_mm": round(r * 1000, 1),
        "marker_plane_mm": round(reg["marker_plane_above_robot_z0_m"] * 1000, 1),
        "rms_mm": round(reg["registration_rms_m"] * 1000, 2),
        "cams": cams,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("dataset")
    ap.add_argument("-o", "--out", default=None,
                    help="output HTML path (default <dataset>/capture_dome.html)")
    args = ap.parse_args()
    ds = args.dataset.rstrip("/")
    out = args.out or os.path.join(ds, "capture_dome.html")
    scene = build_scene(ds)
    with open(out, "w") as f:
        f.write(VIEWER_HEAD + "\nconst SCENE = " + json.dumps(scene) + ";\n"
                + VIEWER_TAIL)
    print(f"wrote {out} ({len(scene['cams'])} cameras)")


if __name__ == "__main__":
    main()
