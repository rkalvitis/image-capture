# Prompt: automate the insect capture session (v1, single shot per stop)

Use this file as the opening prompt for the session (or the person) that
builds this. It is self-contained; the two linked files hold the details.

**The goal is to automate the whole capture process**, not only the phone:
the arm goes to a pose, the rig is verified still, the phone shoots, the
files land in the dataset folder with their metadata, the turntable/arm
advances, and the loop repeats for all rings — with the operator doing
nothing except what is still physically manual (turning the turntable, for
now). The work has three parts, built in this order because each is
testable on its own:

- **Part A — iPhone app**: shoots on command, records everything about the
  shot, serves the files.
- **Part B — Mac coordinator**: runs the session state machine, uses the
  OptiTrack stream to decide when it is safe to shoot, pulls the files,
  writes the dataset folder.
- **Part C — robot / turntable handshake**: "reached" / "go" between the
  coordinator and the robot PC; turntable steps confirmed by mocap yaw.

**Focus stacking is out of scope for v1** — everything must be built so
that a stack is later just "the same capture routine run N times per
stop", but v1 takes ONE photo per stop. Answer the questions in §8 before
writing code.

---

## 1. What you are building and why

A small iOS app for an **iPhone 15 Pro** that takes **Apple ProRAW photos
of an insect on command from a Mac**, with **no manual input on the phone
during a session**. The phone sits in a cradle on a robot arm inside a
motion-capture lab; today every shot is a shutter tap and every tap
disturbs the rig. The Mac already knows when the arm has stopped and the
rig is still (it reads the OptiTrack stream), so the Mac must be the one
that says "shoot now".

The photos feed a COLMAP + Gaussian-splatting pipeline that is very
sensitive to camera intrinsics, so the app must also **record exactly how
each photo was taken**: locked lens position, locked exposure, Apple's
per-frame intrinsic matrix, and a precise capture timestamp.

Background reading (in this folder):
- `DEVELOPMENT_PLAN.md` — the full plan; v1 = Stage 0 + Stage 1 there,
  minus the stack loop. §10 lists what has been verified against Apple's
  docs (ProRAW from third-party apps, locked lens position, USB tunnel,
  foreground-only camera).
- `PROTOCOL.md` — the message contract with the Mac and the per-frame
  sidecar schema. Implement it as written; propose changes rather than
  silently deviating.

## 2. Scope of v1

### Part A — iPhone app. In:
- Ultra-wide physical camera (`builtInUltraWideCamera`), session preset
  `.photo` (or the format with `isHighestPhotoQualitySupported`).
- Locked focus at a lens position sent by the Mac; locked custom exposure
  and white balance sent by the Mac; all set once per session (`hello`) and
  changeable live (`focus`).
- Apple ProRAW DNG output (fallback to Bayer RAW if ProRAW is not offered
  on this camera — report which one was used).
- One capture per `stack` command (v1 treats `lens_positions` as a
  one-element list; do not special-case it away — the loop must exist,
  it just runs once).
- Per-frame sidecar JSON with everything in PROTOCOL.md; fields the API
  cannot provide are `null`, never omitted.
- TCP JSON-lines server on the phone; the Mac reaches it through a USB
  tunnel (`pymobiledevice3 usbmux forward`) or Wi-Fi (same code path).
- Files written to the app's Documents folder with file sharing enabled,
  so the Mac pulls them over USB (`pymobiledevice3 apps afc`).
- Minimal UI: connection state, last command, live preview, current lens
  position, exposure, thermal state, free storage, a manual "shoot" button
  for bench tests only.
- Keeps the screen awake and the session alive for hours (foreground only —
  iOS forbids camera use in the background).

Out (explicitly, for v1):
- Focus-stack loop with N positions and the merge (later; the code shape
  must allow it).
- Any talk to OptiTrack, ROS or the robot from the phone (the Mac does that).
- Saving to the Photos library, editing, sharing, cloud.

### Part B — Mac coordinator (`scripts/capture_coordinator.py`, python, pixi env). In:
- Subscribes to `/optitrack/phone` and `/optitrack/popillia` from the
  driver already started by `pixi run record-poses` (which also records
  the bag, as today).
- Session state machine per stop: `MOVE -> REACHED -> STILL -> SHOOT ->
  VERIFY -> next` (DEVELOPMENT_PLAN.md §6). Stillness = position std of
  both bodies < 0.1 mm over 1.0 s (same definition as
  `scripts/build_ring_poses.py`); VERIFY re-checks stillness over the
  capture interval and that the pulled file count/sizes are right; a bad
  stop is repeated (max 2) then flagged.
- Talks to the app over the USB tunnel (starts `pymobiledevice3 usbmux
  forward` itself, reconnects with backoff).
- Pulls DNG + sidecar after every stop (`pymobiledevice3 apps afc`),
  verifies, then deletes on the phone.
- Writes into the standard session folder
  (`datasets/phone_<YYYYMMDD_HHMMSS>/`): `images/`, sidecars, `session.jsonl`
  (every message and state change), `frames.csv` (one row per frame with
  stop, ring, turntable step, lens position, Apple fx fy cx cy, capture
  time, mocap interval, phone/popillia pose and jitter over the interval —
  pose columns named as in `camera_poses.csv` so existing plots work).
- Ring/pose plan read from a small YAML (rings, steps per ring, turntable
  step angle, lens position, exposure).
- Operator prompts only for what is manual (turntable step in v1).
- A tiny standalone test client (`tools/mac_client.py`, stdlib only) that
  sends `hello` / `focus` / `stack` / `status` / `ping` and prints replies,
  so Part A can be tested before Part B exists.

Out: EXIF-based matching, clock photo, `match_photos.py` — the coordinator
knows every capture interval.

### Part C — robot / turntable handshake. In:
- Robot PC: replace the "confirm each pose" keypress in
  `panda_manual_poses.py` with a socket exchange: send `reached {id,
  joints}` when the hold is settled, wait for `go {id+1}`. Joint-state
  logger unchanged.
- Coordinator: `MOVE` sends `go`, waits for `reached`, then applies the
  mocap stillness gate anyway (the robot's own "settled" is not trusted
  alone).
- Turntable (manual in v1): coordinator prints the target step and waits
  until the popillia body's yaw from mocap is within ±0.5° of it and still
  — no keypress. Motorised turntable later uses the same `reached/go`.

Out: motorised turntable, any change to the robot trajectory generation.

## 3. Hard requirements

1. **No touch during a session.** After `hello`, every capture is triggered
   by the Mac. The manual button exists for bench tests only.
2. **Deterministic camera state.** Lens position, exposure, WB locked; log
   the values actually applied (read back from the device), not the
   requested ones.
3. **Intrinsics per frame.** Enable intrinsic-matrix delivery on a video
   data output connection and record the matrix current at the moment of
   each capture (plus `intrinsicMatrixReferenceDimensions` if available).
   Also record the photo's own calibration data if the API delivers it.
4. **Timestamps.** Record the capture timestamp from the photo object mapped
   to wall-clock time with millisecond resolution; answer `ping` with the
   phone's clock so the Mac can measure the offset.
5. **Sensor-native pixels.** Do not rotate or transpose; write the DNG as
   delivered; record the EXIF orientation flag in the sidecar.
6. **Robustness.** Reconnect after cable/socket drops without losing camera
   state; a failed capture returns `error` and the session continues;
   never block the socket while capturing.
7. **Verifiable.** Every acceptance test in §7 must be runnable by someone
   with the phone, a Mac and the cable.

## 4. Suggested architecture (adjust, but say why)

```
CaptureApp/
  App/            SwiftUI entry, single screen
  Camera/         CameraController: session setup, device config, capture
                  delegate, intrinsics tap (AVCaptureVideoDataOutput)
  Control/        ControlServer: NWListener, JSON-lines codec, command
                  dispatcher, session state
  Storage/        FrameStore: file naming, DNG + sidecar writing, free space
  Model/          Codable structs mirroring PROTOCOL.md
tools/
  mac_client.py   test client: hello / focus / stack / status / ping
```

Start from Apple's AVCam sample for the capture pipeline; keep the
control server independent of the camera so it can be unit-tested with a
fake camera.

## 5. File naming and storage

- `IMG_<session>_<stop:04d>_<k:02d>.dng` + `.json` sidecar, both written
  atomically (temp name, then rename) so a pull never sees a partial file.
- Session folder `Documents/<session>/`.
- Never delete on the phone by itself; the Mac deletes after a verified
  pull. Refuse `stack` with an `error` if free space < 2 GB.

## 6. Deliverables

Part A:
- Xcode project that builds and runs on the iPhone 15 Pro with a free
  developer account (7-day signing is acceptable for now).
- `tools/mac_client.py` (python 3, stdlib only) and a README with the exact
  commands to: forward the port over USB, run a bench capture, pull files.
- A short `NOTES.md` recording what the API actually delivered on this
  phone: ProRAW yes/no on the ultra-wide, calibration data yes/no, the
  intrinsics reference dimensions, the lens position range that focuses
  on a 40–60 mm subject.

Part B:
- `scripts/capture_coordinator.py` + a `pixi run capture-session` task,
  the session-plan YAML with the ring362 protocol as the example, and a
  dry-run mode (fake phone, replayed mocap bag) that exercises the state
  machine without hardware.
- `session.jsonl` / `frames.csv` writers with a self-test.

Part C:
- The robot-PC patch (diff against `panda_manual_poses.py`) and a fake
  robot server for testing the coordinator without the arm.
- One end-to-end bench run recorded in `NOTES.md`.

## 7. Acceptance tests

1. Bench: `hello` → `ready`; `focus 0.45` → `focus` reply with the applied
   value; `stack` → one DNG + one sidecar in Documents; the Mac pulls both
   over USB; the DNG opens with the existing pipeline (`scripts/
   match_photos.py --self-test` style EXIF read works).
2. Repeatability: 20 `stack` commands at the same lens position → sharpness
   of the same target varies < 5%; sidecar lens_position identical.
3. Intrinsics: sidecar fx changes monotonically when lens_position is swept
   0.2 → 0.8 in 0.1 steps; reference dimensions recorded.
4. Endurance: 500 captures over 2 h on the cable with no drop, no thermal
   `serious` state, timestamps monotonic, no missing sidecar.
5. Recovery: unplug/replug the cable mid-session → the next `stack` succeeds
   without re-sending `hello`.
6. No-touch: a full bench session runs from `hello` to `bye` with the phone
   untouched.

Part B / C:
7. Dry run: coordinator completes a 2-ring × 6-step plan against the fake
   phone + fake robot + a replayed mocap bag, producing a valid session
   folder with `frames.csv` rows that `plot_ring_dome.py`-style tooling
   can read.
8. Stillness gate: with the real rig, tapping the mount during STILL delays
   the shot and is logged; a tap during the capture interval causes a
   repeat of that stop.
9. Lab run: one full ring (36 stops) unattended except turntable prompts;
   every frame has a mocap interval with phone jitter < 0.1 mm; file counts
   match; nothing was touched on the phone.
10. Robot: 10 consecutive poses driven by `go`/`reached` with the
    coordinator, no keyboard on the robot PC.

## 8. Decisions already taken (answered 2026-09-08) — do not re-ask

**Topology.** The phone is on THIS MacBook (USB-C only). The OptiTrack
driver, the coordinator and the phone tunnel all run on the MacBook, which
is plugged into the lab LAN. The robot arm is driven from a DIFFERENT PC on
the same LAN; the builder has NO access to that PC. Mac <-> robot PC link:
**plain TCP/JSON over the LAN** (decided by the builder; least machinery,
no ROS 1/ROS 2 bridging). The robot-side change is delivered as a patch +
spec and tested against a fake robot server; the real test happens at a
lab session.

**Session protocol (v1).** Outer loop = turntable step (MANUAL, no motor:
the operator rotates the insect 10°); inner loop = the robot's recorded
vertical poses (9 in the last session, run automatically). One robot run
= one turntable step: `initial pose -> pose 1..N -> back to initial`.
The "place/check the object" wait at the initial pose IS the turntable
moment: the coordinator confirms the new step from the popillia body's
mocap yaw (±0.5°, still) and only then sends `go`. No keypress anywhere.

**Camera.** Ultra-wide, 12 MP Apple ProRAW — confirmed available on this
phone (stock Camera app shoots it); if the third-party API does not offer
it on the physical ultra-wide, fall back to Bayer RAW DNG and report. Exposure
and WB fixed values sent by the Mac in `hello`, defaults = last session
(1/100 s, ISO 64), changeable live. Flash/torch always off.

**Apple calibration data: ALWAYS record everything the API delivers** —
per-frame intrinsic matrix (+ reference dimensions) from the video
connection, and the photo's `cameraCalibrationData` (intrinsics,
distortion LUT + centre, extrinsics, pixel size) whenever it is present.
Fields not delivered are `null`. No experimental dual-camera path in v1.

**Files.** New naming `IMG_<session>_<stop:04d>_<k:02d>.dng/.json`. The
phone NEVER deletes automatically; the coordinator pulls after every stop
and verifies, deletion is a manual step (Finder / AFC) after the dataset
is confirmed. App refuses `stack` below 2 GB free and reports it.

**Operations.** Thermal: warn only (log per frame, `status` to the Mac),
keep shooting. Screen: keep the live preview visible for now (may change).
Failure after 2 retries of a stop: pause and ask the operator
(retry / skip / abort). Signing: free Apple ID (7-day re-install).

**Focus per pose.** The lens position is NOT found live: it is decided
beforehand PER ROBOT POSE (each vertical pose has its own working
distance) and stored in the coordinator YAML as a list `lens_position[k]`.
When the robot reports `reached k`, the coordinator sends the app
`focus lens_position[k]` (or includes it in `stack`), the app settles,
then shoots. The app never runs autofocus during a session.

**Stillness thresholds** are YAML parameters (default 0.1 mm over 1 s,
both bodies) — the real values will be decided by experiment on the rig;
the coordinator must print the measured jitter at every stop so that
experiment is easy. **Turntable** steps are physically indexed, so the
±0.5° mocap-yaw confirmation applies.

**Undistortion.** Apple's geometric distortion correction stays ON; the
sidecar records the flag every frame.

**Session plan source.** The robot keeps its own recorded pose list
(`manual_poses*.csv` next to `panda_manual_poses.py`, replayed in run
mode); the coordinator YAML only holds: number of turntable steps, step
angle, camera settings, thresholds, and how many poses to expect per run.

## 9. Questions still open (decide, then record in NOTES.md)

Answer inline here; unanswered questions are decided by the builder and
recorded in `NOTES.md`.

**Devices and accounts**
1. macOS/Xcode version on this MacBook? Is Xcode installed? Bundle id?
3. iOS version on the iPhone 15 Pro? (Sets the minimum deployment target.)
4. Is the phone available for bench testing outside the lab, and how often?

**Camera settings**
9. Is 12 MP ProRAW (4032×3024) the wanted format, or should the app expose
   the format list and let the Mac choose?
**Connection**
13. Should the app require a token/handshake so a stray connection cannot
    trigger captures?

**Timing**
14. How precise must the capture timestamp be — is 5 ms clock offset (USB
    round-trip) acceptable, or do you want the Mac's time echoed into the
    sidecar as well?
15. Should the app also write the capture time into the DNG EXIF (it will by
    default), and keep `SubSecTimeOriginal` as today so old tooling works?

**Files**
18. Sidecar as separate `.json`, or also embedded into the DNG (XMP)? Both?

**UI and operations**
19. What must be visible on the phone screen during a session (preview,
    counter, last error)? Anything that must NOT be shown (bright screen
    near the specimen)?
20. Screen brightness: dim to minimum during a session? The screen faces
    away from the specimen in the cradle — confirm.
**Calibration data**
**Coordinator (Part B)**
**Robot / turntable (Part C)**
31. IP/hostname of the robot PC on the lab LAN, and a free TCP port the
    robot script may listen on (the Mac connects to it).
**Future hooks (design only, no implementation in v1)**
24. Confirm the stack contract: `lens_positions` list in `hello`, one
    `frame` message per position, one `done` per stop — as in PROTOCOL.md.
---

When the questions are answered, produce, in order: (1) a short design
note (≤ 1 page) confirming the decisions, (2) Part A: the Xcode project,
test client and README, plus `NOTES.md` with the measured facts, (3) Part
B: the coordinator with its dry-run mode, (4) Part C: the robot patch and
fake robot. Report each acceptance test in §7 as pass/fail with the
evidence. Do not start Part B before Part A's tests 1–3 pass on the real
phone; do not start Part C before Part B's dry run (test 7) passes.
