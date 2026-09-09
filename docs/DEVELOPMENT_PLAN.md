# Development plan: automated iPhone capture with focus stacking

Written 2026-09-03 from the discussion after the ring362 dataset. Each stage
is written to be promptable as "implement Stage N of
docs/capture_app/DEVELOPMENT_PLAN.md". Nothing here is built yet.

## 1. Goal

Capture a full insect dataset (rings × turntable steps × focus stack)
**without touching the phone or typing anything during the session**, with
every frame carrying exact intrinsics and an exact capture time, and with
the arm/turntable advancing only when the previous stop is verified good.

Success looks like: start the coordinator, walk away, come back to a
dataset folder that is ready for COLMAP/3DGS (images, masks pending,
per-frame sidecars, pose log), with no EXIF matching, no clock photo, no
AirDrop.

## 2. Why (problems this removes)

| problem today | cause | what the app does instead |
|---|---|---|
| rig disturbed between shots | shutter tap, focus slider on the phone | nothing is touched; shots triggered over the cable |
| focus stacking ~10x slower | manual slider per step, per stop | N locked lens positions fired automatically per stop |
| focal length ambiguous per image | focus breathing (fx 1976 -> ~1760 px), unknown per photo | lens positions are repeatable; Apple intrinsics logged per frame; one ChArUco run per position |
| principal point wrong in trainer | assumed image centre; calibrated cx,cy 91 px off | cx,cy known per frame from Apple, written to cameras.txt (trainer already patched to honour it) |
| photo <-> pose pairing by EXIF + clock photo | phone and Mac clocks | coordinator knows the mocap interval of every shot |
| bad shots found at COLMAP time | no live check | mocap stillness check before AND during each stack; bad stop is repeated immediately |
| transfer by AirDrop, hours | thousands of DNGs | pulled over USB-C as they are written |
| battery | long sessions | charging over the same cable |

## 3. Architecture

```
robot PC ──(TCP/JSON or ROS topic)──┐
                                    ▼
OptiTrack ──NatNet──▶ Mac COORDINATOR (python, in the pixi env)
                        │  state machine, mocap stillness check,
                        │  session log, dataset folder writer
                        ▼ USB-C tunnel (pymobiledevice3 / iproxy), JSON lines
                      iPhone APP (Swift, AVFoundation)
                        ultra-wide, locked focus stack, custom exposure,
                        ProRAW DNG + sidecar, files pulled via afc
```

Design rules:

- **The app talks only to the Mac.** It never knows about OptiTrack or the
  robot. All sequencing lives in the coordinator, where the mocap data is.
- **Wired first.** USB-C for trigger, files and power. Wi-Fi is a fallback
  only (same JSON protocol over a plain socket).
- **Everything is logged before it is needed.** Capture time, lens position,
  Apple intrinsics/extrinsics/distortion table, exposure, device thermal
  state, mocap interval — one sidecar per frame from day one, even if the
  pipeline does not use every field yet.
- **The dataset folder contract is unchanged** (see
  `docs/IPHONE_IMPLEMENTATION_PLAN.md`); the coordinator writes it directly.

## 4. Components

### 4.1 iPhone app (Swift, SwiftUI, AVFoundation)

- Opens the **physical** `builtInUltraWideCamera` (never the virtual 0.5x
  device, so iOS cannot switch lenses).
- `lockForConfiguration` once: `setFocusModeLocked(lensPosition:)`,
  `setExposureModeCustom(duration:iso:)`, white balance locked.
- `AVCapturePhotoOutput` with Apple ProRAW enabled; also enables camera
  calibration data delivery where the API allows it, and keeps a video data
  output connection with intrinsic-matrix delivery on as the per-frame
  intrinsics source.
- Listens on a TCP port; executes `stack` commands: for each lens position
  in the list, set focus, wait `settle_ms`, capture, record
  `AVCapturePhoto.timestamp` and calibration, write DNG + sidecar JSON into
  its Documents folder (file sharing enabled). Replies `done` with the
  frame list.
- Minimal UI: connection state, last command, preview, thermal state,
  free storage, a manual "stack" button for bench tests.
- No autofocus, no auto exposure, no HDR/Live Photo, no orientation
  handling (sensor-native, as today).

### 4.2 Mac coordinator (python, `scripts/capture_coordinator.py`)

- Subscribes to `/optitrack/phone` and `/optitrack/popillia` (reuses the
  running driver from `pixi run record-poses`, which also keeps the bag).
- State machine per stop: `MOVE -> REACHED -> STILL -> STACK -> VERIFY ->
  next` (see §6).
- Stillness check reuses the jitter definition from
  `scripts/build_ring_poses.py` (std of position over the window).
- Talks to the app over the USB tunnel, to the robot PC over the existing
  channel (see §7 open questions), and prompts the operator when a step is
  manual (turntable).
- Writes `session.jsonl` (one line per event) and `frames.csv` (one row per
  frame: file, stop, ring, turntable step, focus index, lens position,
  Apple fx fy cx cy, capture time, mocap interval, phone/popillia pose and
  jitter over the interval) directly into the dataset folder.
- Pulls files from the phone after each stack (`pymobiledevice3 afc pull`)
  and verifies count + size before allowing the next move. Never deletes
  on the phone (decided): deletion is a manual step after the dataset is
  confirmed.

### 4.3 Robot PC side (small addition to the existing scripts)

- `panda_manual_poses.py` already replays poses with confirm-each-pose
  and already publishes `pose_reached` / `pose_left` events; the change is
  a `_confirm_via:=socket` mode that replaces the three Enter prompts with
  `reached` / `go` messages over TCP (see PROTOCOL.md), plus `_cycles:=N`.
- The builder has no access to the robot PC: deliver the patch + a fake
  robot server; the real test is at a lab session.
- **Loop order (decided 2026-09-08):** one robot run = one turntable step.
  Outer loop turntable (manual, 10°), inner loop the recorded vertical
  poses (automatic). The initial-pose wait is the turntable moment.
- The continuous joint-state logger stays as is.

### 4.4 Turntable

- Today: rotated by hand in 10° steps. Until motorised, the coordinator
  prompts "turn to step k" and waits for the popillia body's yaw from mocap
  to reach the target (±0.5°) and be still, so no keypress is needed.
- Motorised later: same `reached/go` exchange as the arm.

## 5. Stages

### Stage 0 — Feasibility bench test (no rig, 1 day)

- Xcode project from Apple's AVCam sample; add ultra-wide selection, locked
  lens position, custom exposure, ProRAW capture.
- Shoot the same static object at 10 lens positions, twice. Acceptance:
  DNGs open in the existing pipeline; lens position repeatability visible
  (sharpness curve identical between the two runs); Apple intrinsics
  readable and change with lens position.
- Establish the USB tunnel on the Mac (`pymobiledevice3`), echo a JSON
  message both ways, pull a DNG with `afc`.

### Stage 1 — App v1: stack on command

- TCP listener, `hello`/`stack`/`done`/`status` messages (PROTOCOL.md).
- Sidecar JSON per frame with everything in §4.1.
- Manual "stack" button for bench use.
- Acceptance: 100 stacks in a row without a dropped frame or thermal
  throttling; timestamps monotonic; sidecar complete.

### Stage 2 — Coordinator v1: phone + mocap only

- Stillness gate, `stack` trigger, file pull, `frames.csv` writer.
- Turntable by prompt + mocap-yaw confirmation; arm still moved by hand
  (operator presses Enter on the Mac between rings only).
- Acceptance: one full ring captured with zero touches of the phone; every
  frame has a mocap interval with phone jitter < 0.1 mm; `frames.csv`
  drop-in compatible with `build_ring_poses.py` output.

### Stage 3 — Intrinsics per lens position

- ChArUco run at each stack lens position (board at the matching distance);
  compare `calibrate_intrinsics.py` against Apple's fx, fy, cx, cy.
- Decide: use Apple values directly, or a fitted per-position table.
- Write `calibration/intrinsics_by_lens_position.yaml`; the COLMAP export
  assigns one PINHOLE camera per lens position (fixed, not refined).
- Acceptance: COLMAP on a stack subset with fixed per-position K reaches the
  free model's reprojection error (~0.8 px) without per-image focal.

### Stage 4 — Robot handshake

- `reached`/`go` socket in `panda_manual_poses.py`; coordinator drives the
  full ring × pose loop.
- Acceptance: a complete session (10 rings) unattended except turntable
  prompts; session log shows every stop verified.

### Stage 5 — Pipeline integration

- `organize`/`match` steps read `frames.csv` from the coordinator instead of
  EXIF; `prepare_ring_gs.py` writes per-lens-position cameras.
- **Focus-stack merge runs on the Mac, never on the phone.** The app only
  writes the N raw ProRAW frames per stop; the coordinator's `frames.csv`
  groups them by `stop` and `focus_index`, and a merge step
  (`scripts/merge_focus_stacks.py`, building on the existing
  `organize_focus_stacks.py` / `match_stacks.py`) produces one all-in-focus
  image per stop. Raw stacks are kept so the merge can be redone.
- Merge rule that matters for the poses: because fx breathes with focus,
  the N frames of a stack have slightly different magnification. Align all
  frames to a fixed **reference lens position** (the same index at every
  stop) before merging; the merged image then inherits that reference
  frame's intrinsics and its `frames.csv` row, and COLMAP/3DGS use exactly
  that K. Never merge without alignment and never pick a different
  reference per stop.
- Merge tooling candidates: `align_image_stack` + `enfuse` (Hugin, free,
  scriptable, handles the scale alignment), or a Laplacian-pyramid merge
  in Python on the linear DNG data if enfuse's tone handling gets in the
  way of ProRAW linearity. Decide in Stage 5 on one real stack.
- Acceptance: a dataset goes from session end to a trained 3DGS with no
  manual matching step.

### Stage 6 (optional) — Motorised turntable

- Same handshake as the arm; removes the last prompt.

## 6. Coordinator state machine (per stop)

```
for turntable step s in 0..S-1:                      (outer loop, manual platter)
  TURN     robot sends reached(0); coordinator prompts "turn to step s";
           waits until popillia yaw == θ_s ± 0.5° and still 1 s; sends go(0)
  for pose k in 1..N:                                 (inner loop, robot)
    MOVE   wait reached(k) from the robot
    STILL  phone AND popillia jitter < 0.1 mm over 1.0 s (timeout 15 s -> warn)
    SHOOT  send stack(stop) to app; wait done (timeout = frames * 3 s)
    VERIFY mocap still over [first, last capture]? files pulled, count/size OK?
           yes -> log stop, send go(k)     no -> repeat SHOOT (max 2), then
           PAUSE and ask the operator: retry / skip / abort
  robot returns to initial and sends done(s)
```

## 7. Open questions (decide before Stage 1)

1. Robot channel: raw TCP/JSON to the robot PC, or a ROS 1 bridge? (Robot PC
   runs ROS 1 `panda_moveit_ctrl`; Mac runs ROS 2 Humble.) TCP/JSON is the
   least machinery.
2. Does `AVCapturePhoto.cameraCalibrationData` populate for single-camera
   ProRAW captures, or only the video-connection intrinsic matrix? Stage 0
   answers this; the sidecar takes whichever is available.
3. Do Apple's intrinsics describe the ProRAW DNG pixel grid exactly
   (scaling, any crop)? Stage 3 answers this against the board.
4. Stack design: how many lens positions, which range, front-to-back
   coverage of a 12 mm beetle at 40–60 mm. Pick empirically in Stage 0 with
   a sharpness sweep.
5. Cable routing: short right-angle USB-C lead strain-relieved to the mount
   so the cable itself does not move the phone.
6. Apple developer account: free (7-day installs) is enough for development;
   decide before the first real session whether a paid account is worth the
   convenience.

## 8. Risks

- **Thermal throttling** on long ProRAW sessions: log thermal state, insert
  cooling pauses in the coordinator if `serious`.
- **Lens position non-linearity / drift**: repeatability is what matters;
  verify in Stage 0 and again per session with a quick sweep.
- **iOS API limits**: no official way to disable all computational
  processing in ProRAW; accept it (the current pipeline already does).
- **USB tunnel flakiness** after phone sleep: keep the app foreground with
  idle timer disabled; coordinator reconnects with backoff.
- **Storage**: ~25 MB per ProRAW frame; 10 rings × 36 steps × 10 focus =
  3600 frames ≈ 90 GB. Pull and delete as you go; keep ≥ 20 GB free.

## 9. Estimated effort

| stage | effort |
|---|---|
| 0 feasibility | 1 day |
| 1 app v1 | 2–3 days |
| 2 coordinator v1 | 2 days |
| 3 intrinsics per position | 1 day + one board session |
| 4 robot handshake | 1 day (needs robot PC access) |
| 5 pipeline integration | 1–2 days |

## 10. Feasibility check (verified against Apple docs, 2026-09-03)

| claim | status | source |
|---|---|---|
| App can fire the shutter programmatically, no touch | **yes** — `AVCapturePhotoOutput.capturePhoto(with:delegate:)` is how every camera app shoots | Apple, "Capturing photos in RAW and Apple ProRAW formats" |
| Apple ProRAW DNG from a third-party app | **yes**, iOS 14.3+; set `isAppleProRAWEnabled` (after checking `isAppleProRAWSupported`) before `startRunning()`, session preset `.photo` (or a format with `isHighestPhotoQualitySupported`), pick the format with `isAppleProRAWPixelFormat`; `fileDataRepresentation()` is a DNG | same article; WWDC21 "Capture and process ProRAW images": "Apple ProRAW is supported on all devices, including dual wide-, dual-, and triple-camera devices" |
| Ultra-wide as the capture device | **yes** — `builtInUltraWideCamera` (iOS 13+), via `AVCaptureDevice.DiscoverySession` | Apple docs |
| Locked, repeatable focus positions | **yes** — `setFocusModeLocked(lensPosition:completionHandler:)` (iOS 8+), value 0..1, inside `lockForConfiguration()` | Apple docs |
| Remote trigger over USB-C | **yes** — `pymobiledevice3 usbmux forward <local_port> <device_port>` tunnels TCP over USB; app runs an `NWListener` | pymobiledevice3 CLI reference |
| Pull DNGs over USB without AirDrop | **yes** — `pymobiledevice3 apps afc <bundle_id> --documents` (app needs file sharing enabled in Info.plist) | pymobiledevice3 CLI reference |
| Per-frame intrinsics from Apple | **partly** — the intrinsic matrix is delivered on an `AVCaptureVideoDataOutput` connection (iOS 11+, single camera OK, `isCameraIntrinsicMatrixDeliverySupported`). Photo-level `cameraCalibrationData` (intrinsics + distortion LUT + extrinsics) is delivered **only** for virtual multi-camera devices with constituent photo delivery on and geometric distortion correction OFF — i.e. not for a plain ultra-wide ProRAW capture | Apple docs `isCameraCalibrationDataDeliverySupported` |
| App must be in the foreground | **constraint** — iOS forbids camera use in the background; keep the app foreground, idle timer disabled, phone on the cable | Apple docs, `videoDeviceNotAvailableInBackground` |

Consequences for the plan:
- Stage 0 must test ProRAW on the **physical** ultra-wide device (WWDC text says "all devices" but does not name the ultra-wide explicitly; the stock Camera app does it, so the hardware path exists).
- Intrinsics come from the video connection's per-frame matrix, captured at the moment of each shot, not from the DNG. Stage 3 validates them against ChArUco per lens position.
- If the distortion LUT is wanted, the alternative is capturing through the dual-wide virtual device with constituent delivery and GDC off; whether that combines with ProRAW is a Stage 0 question, not needed for the baseline plan (COLMAP already fits the ProRAW DNGs with a plain PINHOLE model at 0.84 px).
