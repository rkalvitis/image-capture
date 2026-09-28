# Automated capture — planning docs

Plan, protocol and decisions for the app + coordinator + robot handshake
implemented in this repo (see the top-level README.md for the code layout
and how to run it). Planning started 2026-09-03; v1 implemented and
bench-tested 2026-09-08; merge, COLMAP export and the streaming merge
(Stage 5/7) added 2026-09-09.

| file | what it is |
|---|---|
| `DEVELOPMENT_PLAN.md` | Goals, architecture, staged build list with acceptance criteria, risks, open questions. Start here. |
| `PROTOCOL.md` | The message contract between Mac coordinator, phone app and robot PC, plus the per-frame sidecar format. |
| `DATASET_RUNBOOK.md` | The full dataset session of 2026-09-16 (`plan.sep15.yaml`): one command, the manual inputs, resume, what to watch, after the session. |
| `MAIN_TEST_RUNBOOK.md` | The 2026-09-09 main test (9 stacks, no mocap) — done. |
| `APP_BUILD_PROMPT.md` | Ready-to-use prompt for building v1 of the whole automation — app, Mac coordinator, robot/turntable handshake (single shot per stop, focus stacking deferred) — with the open development questions to answer first. |

Related context in the optitrack repo (`~/Documents/repos/optitrack`):

- `docs/IPHONE_CAPTURE_PLAN.md`, `docs/IPHONE_IMPLEMENTATION_PLAN.md` — the
  manual pipeline this replaces.
- `datasets/aug31_20deg_9vertical/datasetRecord_31_08.md` — the session
  runbook the coordinator automates.
- `scripts/build_ring_poses.py`, `scripts/match_photos.py` — the pose
  pairing logic the coordinator makes unnecessary (timestamps become known).
- The robot script being patched lives in `~/Documents/repos/robo_arm`.

Why this exists (2026-09-02/03 findings): every manual touch disturbs the
rig; focus stacking makes a manual session ~10x slower; the ultra-wide's
focal length breathes with focus (fx 1976 -> ~1760 px) and the trainer
needs exact intrinsics including the principal point. An app with
programmatic lens positions and Apple's per-frame calibration solves all
three at the source.
