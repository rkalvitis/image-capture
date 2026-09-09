# Protocol: coordinator <-> app <-> robot

All channels carry **newline-delimited JSON** (one object per line), UTF-8.
Every message has `"t"` = sender's monotonic-ish wall time in seconds
(float) and `"id"` = the stop index it refers to where applicable.

## Mac coordinator -> iPhone app (USB tunnel, TCP)

| message | fields | meaning |
|---|---|---|
| `hello` | `session`, `lens_positions` [floats 0..1], `settle_ms`, `exposure` {`mode`: auto \| locked \| custom, `duration_s`, `iso`}, `wb` {`mode`: auto \| locked \| custom, `temperature_k`, `tint`} | configure once per session; `locked` = meter once now then hold (dataset default), `auto` = continuous (bench default), `custom` = explicit values; app locks focus and replies `ready` |
| `stack` | `id`, `pose` (k), `step` (turntable), `lens_positions` [floats] (v1: one value = the per-pose focus from the YAML) | set focus, settle, fire the stack now |
| `focus` | `lens_position` (0..1) | move the lens now and hold it; app replies `focus` with the applied `lens_position`. (No autofocus: the focus stack 0.0→0.5 covers the working range.) |
| `status` | — | app replies `status` |
| `ui` | `horizontal` (bool), `clips` (bool), `fill` (bool), all optional | switch the phone UI layout (horizontal = text rotated 90° for the sideways cradle position), toggle the cradle-clip overlay / preview crop; app replies `ui` with the current state |
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

Turntable (manual, v1): `reached id 0` is the turntable moment. The
coordinator prints "turn the insect to step s (yaw θ)", waits until the
popillia body's mocap yaw is within ±0.5° of θ and still for 1 s, then
sends `go 0`. The operator touches only the platter.

## Per-frame sidecar (written by the app next to each DNG)

`IMG_<n>_s<stop>_f<k>.json`:

```json
{
  "file": "IMG_0001_s000_f03.dng",
  "stop": 0, "ring": 1, "step": 0, "focus_index": 3,
  "lens_position": 0.42,
  "capture_time": 1788251422.437,
  "exposure": {"duration_s": 0.01, "iso": 64},
  "intrinsics": {"fx": 1802.1, "fy": 1802.1, "cx": 2036.7, "cy": 1421.0, "w": 4032, "h": 3024},
  "distortion_lut": [ ... ], "distortion_center": [cx, cy],
  "extrinsics_ref": [[...],[...],[...]],
  "device": "iPhone 15 Pro", "lens": "ultra-wide", "thermal": "nominal"
}
```

Fields that the API does not provide for a given capture are written as
`null`, never omitted, so downstream code can rely on the schema.

## Coordinator outputs (dataset folder)

- `session.jsonl` — every message sent/received plus state transitions.
- `frames.csv` — one row per frame: `file, stop, ring, step, focus_index,
  lens_position, fx, fy, cx, cy, capture_time, mocap_t0, mocap_t1,
  phone_jitter_mm, pop_jitter_mm, phone_pose (world), popillia_pose (world)`.
  Column names for poses match `camera_poses.csv` from
  `scripts/build_ring_poses.py` so existing plots keep working.
