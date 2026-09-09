# Measured facts (fill in from the real phone / rig)

| item | value | date |
|---|---|---|
| macOS / Xcode version | macOS 15.7.7, Xcode 26.3 (17C529), xcodegen 2.46, pymobiledevice3 11.10.1 (pipx) | 2026-09-08 |
| iOS version, device | iPhone 15 Pro, iOS ? | |
| ProRAW offered on the physical ultra-wide in the app (`format` in `ready`) | YES — "ProRAW 4032x3024", DNG 15.4 MB, is_raw true | 2026-09-08 |
| `calib_available` (photo-level calibration data) | false (as expected: single physical camera) — sidecar `calibration` = null | 2026-09-08 |
| video intrinsics reference dimensions (w×h) and fx at lens 0.3 / 0.5 / 0.7 | 1572×1178 video frame, scaled ×(4032/1572) to the photo. Sweep (fx px @ lens): 1779@0.2, 1747@0.3, 1722@0.4, 1692@0.5, 1666@0.6, 1637@0.7, 1611@0.8; cx,cy constant ≈ (2009, 1510). CAVEAT: the scale assumes the video frame covers the same field of view as the photo — verify against ChArUco (Stage 3) | 2026-09-08 |
| scale factor video→photo confirmed against ChArUco? | | |
| lens position range that focuses a 40–60 mm subject | not measured yet (one-shot AF on a far scene returns 0.0; do the AF on the beetle at working distance to get the value) | 2026-09-08 |
| USB round trip / clock offset (`mac_client.py ping`) | 19 ms round trip, offset −30…−41 ms (phone behind Mac) | 2026-09-08 |
| ProRAW capture time per frame | ~1.9 s from `stack` to `done` (0.3 s settle + ~0.3 s focus/capture + ~1.35 s ProRAW processing) | 2026-09-08 |
| thermal state after 1 h on the cable | | |
| stillness thresholds chosen (mm / s) | | |
| robot PC host:port | | |

## Bench / acceptance test log
(see docs/APP_BUILD_PROMPT.md §7 — record pass/fail with evidence)

## Findings 2026-09-08 (bench)
- **EXIF DateTimeOriginal on ProRAW is ~0.93 s LATE** (stamped at end of
  processing): app capture_time = +0.57 s after the shoot command, EXIF =
  +1.44 s. Use the app's `capture_time` (from `AVCapturePhoto.timestamp`),
  never EXIF, for mocap windows. (The old EXIF-based pipeline was saved by
  its 3 s still windows.)
- Apple's per-frame intrinsics put the principal point at the image centre
  (2004, 1508) whereas the ChArUco calibration gave (2037, 1421). To be
  resolved in Stage 3 with a board at the lens positions used.
- fx from Apple at lens 0.45 = 1705 px (scaled); ChArUco at the macro band
  gave 1976. Different focus positions — the per-position sweep will tell.
- Battery/charging reported −1/false in the first build: battery monitoring
  was not enabled (fixed in AppState.start()).
- One-shot autofocus returned the same value every time in the first build:
  the wait loop checked `isAdjustingFocus` before the scan started (fixed).
- Autofocus (fixed build): returns 0.0 for a far scene, i.e. changes with the
  scene; use `mac_client.py focus auto` with the beetle in view to pick the
  per-pose lens positions.
- Battery/charging OK in the fixed build (0.80, charging over USB-C).

## Screen layout (2026-09-08, from phone_mount_conf3.stl)
- Cradle clips cover ~11 mm in from each long edge, 42–80 mm below the camera
  end (≈ y 240–466 pt, 50 pt deep each side, portrait). Top 40 mm and the
  whole lower half are free. App layout: preview at the camera end (only its
  side edges can be clipped), all text/buttons in the lower half. "Clips"
  button draws the zones for checking against the real cradle.
- Final layout (confirmed on the rig 2026-09-08): info panel in the top
  240 pt free zone (rows, stack fields, buttons, last log line — all fit in
  both modes, verified by screenshot), preview below it. Horizontal = UI
  rotated 270° (camera end to the right in the cradle); 90° was upside down.
  Red clip overlay confirmed correct by the user.
- Info.plist MUST contain `UILaunchScreen` — without it iOS runs the app
  letterboxed at 320×480 pt (first builds looked wrong because of this).
- Screen mirroring on the Mac: QuickTime Player → File → New Movie Recording →
  camera source = the iPhone (USB), shows the phone screen live. Screenshots
  from the terminal: `pymobiledevice3 developer dvt screenshot out.png`.
- Zoom vs focus (2026-09-08): `videoZoomFactor` stays 1.0 (= "0.5x" ultra-wide)
  at every lens position — we never change zoom. The visible "zoom" while
  focusing is focus breathing: fx goes 1779 px (lens 0.2) -> 1611 px (0.8),
  ~10% field-of-view change, inherent to the ultra-wide at macro distances
  and recorded per frame in the intrinsics. Preview sharpness confirms focus
  moves (76 at lens 0.0 vs 509 at 0.5 on the same scene).
- Hidden band: the whole 42–80 mm band (incl. the middle) is unusable on the
  real cradle; layout = panel (top 240 pt) / black band (226 pt) / preview.
- Thermal throttle in the app: 3 s pause between frames at "serious", capture
  paused (max 5 min) at "critical"; coordinator stack timeout allows for it.
- Files per stack: 40 frames = 80 files (DNG + JSON sidecar each).
