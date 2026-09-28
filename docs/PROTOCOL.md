# Protocol: coordinator <-> app <-> robot

All channels carry **newline-delimited JSON** (one object per line), UTF-8.
Every message has `"t"` = sender's monotonic-ish wall time in seconds
(float) and `"id"` = the stop index it refers to where applicable.

## Mac coordinator -> iPhone app (USB tunnel, TCP)

| message | fields | meaning |
|---|---|---|
| `hello` | `session`, `lens_positions` [floats 0..1], `settle_ms`, `exposure` {`mode`: auto \| locked \| custom, `duration_s`, `iso`}, `wb` {`mode`: auto \| locked \| custom, `temperature_k`, `tint`}; optional heat knobs `raw` (proraw \| bayer), `quality` (quality \| balanced \| speed), `frame_rate` (int), `screen_brightness` (0..1), `preview` (bool) | configure once per session; `locked` = meter once now then hold (dataset default), `auto` = continuous (bench default), `custom` = explicit values; app locks focus and replies `ready` (with `raw`, `quality`, `preview` as applied) |
| `stack` | `id`, `pose` (k), `step` (turntable), `lens_positions` [floats] (v1: one value = the per-pose focus from the YAML) | set focus, settle, fire the stack now |
| `focus` | `lens_position` (0..1 or `"auto"`) | move the lens now and hold it; app replies `focus` with the applied `lens_position`. `"auto"` = autofocus on the frame centre (the beetle; continuous AF for ≤ 4 s until the lens settles, one-shot AF does not scan at macro distance), then lock at the position found (`auto: true` in the reply) — the coordinator's `focus_mode: auto` does this at every stop and shoots the stack around the result |
| `status` | — | app replies `status` |
| `ui` | `horizontal` (bool), `clips` (bool), `fill` (bool), `preview` (bool), `screen_brightness` (0..1), `raw`, `quality`, `frame_rate` — all optional | switch the phone UI layout (horizontal = text rotated 90° for the sideways cradle position), toggle the cradle-clip overlay / preview crop, turn the live preview off, dim the screen, change the heat knobs live; app replies `ui` with the current state + `thermal` |
| `bye` | — | end of session |

## iPhone app -> Mac coordinator

| message | fields | meaning |
|---|---|---|
| `ready` | `device`, `lens`, `format`, `free_bytes`, `calib_available` (bool) | configuration applied |
| `frame` | `id`, `k` (focus index), `file`, `capture_time`, `lens_position` | sent as each frame is written (progress) |
| `done` | `id`, `frames` [ {`k`, `file`, `capture_time`, `lens_position`, `intrinsics` {`fx`,`fy`,`cx`,`cy`,`w`,`h`}, `exposure` {`duration_s`,`iso`}, `thermal`} ] | stack complete |
| `error` | `id`, `reason` | capture failed; coordinator decides to retry |
| `focus` | `lens_position` | lens settled after a `focus` command |
| `status` | `thermal`, `free_bytes`, `battery`, `charging` | on request |

`capture_time` is the phone's wall clock at exposure midpoint (from
`AVCapturePhoto.timestamp` mapped to wall time). The coordinator measures
phone-vs-Mac clock offset at `hello` by round-trip (`ping`/`pong`, offset =
midpoint difference, expect < 5 ms over USB) and logs it; EXIF is no longer
used for timing.

## Mac coordinator <-> robot PC (TCP/JSON over the lab LAN — decided)

The robot PC runs ROS 1 (`panda_moveit_ctrl`); the Mac has no access to
it. `panda_manual_poses.py` (repo `robo_arm`) gets one new param,
`_confirm_via:=socket` (default `keyboard` = unchanged behaviour), and one
optional `_cycles:=N` (repeat the whole run N times without relaunch,
default 1). With `socket`, a tiny TCP server (`_confirm_port`, default
5600) replaces the three `arc.wait_for_enter(...)` calls in run mode:

| point in the script | today | with `_confirm_via:=socket` |
|---|---|---|
| initial pose reached ("place/check the object") | Enter | send `reached {"id":0,"cycle":c,"joints":[7]}`; wait for `go` |
| each pose 1..N-1 reached ("take the photo") | Enter | send `reached {"id":k,...}`; wait for `go` |
| last pose N ("final photo, then return") | Enter | send `reached {"id":N,...}`; wait for `go`; then returns to initial, sends `done {"cycle":c}` |

Messages (newline-delimited JSON):

| direction | message | meaning |
|---|---|---|
| robot -> Mac | `hello` {`poses`: N, `cycles`: C} | on connect |
| robot -> Mac | `reached` {`id`, `cycle`, `joints`[7]} | hold reached and settled (the script's existing "pose_reached" event moment) |
| Mac -> robot | `go` {`id`} | leave this pose (photo taken and verified) |
| Mac -> robot | `abort` | finish the cycle now via the normal return-to-initial path |
| robot -> Mac | `done` {`cycle`} | back at the initial pose |
| robot -> Mac | `error` {`reason`} | motion aborted |

The script's existing event topic (`init_confirmed`, `pose_reached,k`,
`pose_left,k`, `return_start`) and CSV log stay unchanged, so the
joint-state logger and the pose CSV keep working exactly as today.

Turntable (manual, v1): `reached id 0` is the turntable moment. With
`turntable_confirm: mocap` the coordinator prints "turn the insect to step s
(yaw θ)", waits until the popillia body's mocap yaw is within ±0.5° of θ and
still for 1 s, then sends `go 0`; with `enter` it waits for Enter on the Mac
(yaw logged if available); `both` = Enter, then the yaw check. With
`pose_confirm: true` every `reached k` is followed by "lens at the stack
centre — Enter to shoot / a value 0..1 to re-centre / skip / abort" before
the `stack`. `--mocap none` runs the whole loop without OptiTrack.

## Per-frame sidecar (written by the app next to each DNG)

`IMG_<session>_<stop:04d>_<k:02d>.json` next to `IMG_<session>_<stop:04d>_<k:02d>.dng`
(k = focus index within the stop). Both are written atomically into
`Documents/<session>/` of the app (visible in the Files app under
On My iPhone > CaptureApp; nothing goes to the Photos library).

```json
{
  "file": "IMG_ring1_0001_03.dng", "session": "ring1",
  "stop": 1, "pose": 1, "step": 0, "focus_index": 3,
  "lens_position": 0.0385, "lens_position_requested": 0.0385,
  "capture_time": 1788859486.029,
  "exposure": {"duration_s": 0.0179, "iso": 241.5},
  "white_balance_gains": {"r": 1.9, "g": 1.0, "b": 1.6},
  "intrinsics": {"fx": 1823.7, "fy": 1825.3, "cx": 2008.3, "cy": 1513.7, "w": 4032, "h": 3024},
  "intrinsics_video": {"fx": 711.1, "fy": 711.7, "cx": 783.1, "cy": 589.7, "w": 1572, "h": 1178},
  "calibration": null, "gdc_enabled": true, "orientation": 6,
  "raw_format": "ProRAW 4032x3024", "is_raw": true, "zoom_factor": 1.0,
  "exif": {"ExposureTime": 0.0179, "ISOSpeedRatings": [241], "DateTimeOriginal": "...", "SubsecTimeOriginal": "..."},
  "device": "iPhone 18.7.8", "lens": "ultra-wide", "thermal": "serious"
}
```

`intrinsics` = the video-connection matrix scaled to the photo size (see
NOTES.md for the caveat); `calibration` = the photo-level
`cameraCalibrationData` when the API delivers it (never on the single
ultra-wide). Fields that the API does not provide for a given capture are
written as `null`, never omitted, so downstream code can rely on the schema.

## Coordinator outputs (the dataset folder: plan `session_dir`, else `datasets/phone_<YYYYMMDD_HHMMSS>/`)

Layout since 2026-09-15 ("stacks"; older sessions have the flat
`images/` + `sidecars/` + `merged/` layout, still read by every tool):

- `plan.json` + `plan.yaml` — the plan as run; `session.jsonl` — every event
  (`robot_initial/reached/done`, `turntable`, `focus_table`, `exposure_check`,
  `exposure_applied`, `still`, `stop_ok`, `stop_failed`, `stop_already_done`, `merge_done`, ...).
- `images/<image>/stack/<frame>.dng + .json` — pulled from the phone after
  every stop (`<image>` = `s<step:02d>_p<pose>`); `stack/mocap.csv` = all mocap
  samples of both bodies over the capture window; `stack/stack.json` = the
  merge summary (lens grid, reference frame, alignment rows, frames.csv rows).
  With `merge_delete_raw` the DNGs are removed after the merged image was
  written AND read back; the sidecars stay; the phone keeps everything.
- `images/<image>/<image>.tif` — the merged image (`merge_format`: `tif` =
  16-bit sRGB-encoded TIFF master, `tif-linear`, `png`, `jpg`) + `<image>.jpg` preview.
- `frames.csv` — one row per raw frame:
  `image, file, stop, step, pose, focus_index, lens_position, focus_centre, focus_source, fx, fy, cx, cy,
  capture_time, mocap_t0, mocap_t1, phone_jitter_mm, pop_jitter_mm,
  phone_x, phone_y, phone_z, phone_quat_xyzw, pop_x, pop_y, pop_z, pop_quat_xyzw,
  popillia_yaw_deg, cam_x, cam_y, cam_z, quat_xyzw, phone_world`.
  `focus_centre` = the stack's centre lens position and `focus_source` where it
  came from (`pose`, `auto`, `table:<image of the old dataset>`, `range`).
  `phone_*` / `pop_*` are the mocap bodies in the world frame (mean over
  `[mocap_t0, mocap_t1]`, Z-up, metres, quaternions xyzw). `cam_*` /
  `quat_xyzw` are **T_insect<-cam** (camera in the popillia frame after the
  hand-eye transform, the meaning these columns have in `camera_poses.csv`
  of the optitrack repo) — filled when the plan names `rig_transforms`,
  otherwise empty and computed later by `colmap_export.py --rig`.
  `capture_time` is on the Mac clock (phone time minus the measured offset).
- `stacks.csv` (flat layout: `merged/merged_frames.csv`) — one row per merged
  image with the columns above for the REFERENCE frame (its `fx fy cx cy`, its
  pose columns) plus `image, ref_file, ref_focus_index, ref_lens_position,
  n_frames, w, h, decoder, linear, format, fusion, align, align_max_shift_px,
  align_min_cc, seconds, raw_deleted_on_mac`; `alignment.csv` — per frame the
  breathing scale/shift predicted by the intrinsics vs measured by the alignment.
  All times are on the Mac clock: `capture_time` = phone timestamp minus the
  hello-time offset; `mocap.csv` / body poses use the driver's header stamps
  (NatNet-synced capture time, the same as in the rosbag).
- `robot/robot_poses.csv` — the joints reported at every reached pose;
  `robot/`, `calibrations/` — copies of the plan's `robot_files` /
  `calibration_files` (+ `rigid_bodies_<date>.yaml` from `dump_bodies`);
  `mocap/bag_<ts>/` — the rosbag of `record_mocap`; `exposure_check/` — the
  test frames of the session-start exposure check.
- `gs_upload/` (from `colmap_export.py`) — `images/` (8-bit sRGB PNG,
  lossless, rendered from the TIFFs), `sparse/0/{cameras, images,
  points3D}.txt` (one PINHOLE camera per image with its own fx fy cx cy),
  `camera_poses.csv` in the layout `plot_ring_dome.py` reads.
