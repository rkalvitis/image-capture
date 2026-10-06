# OneDrive upload during a session (rclone)

Implemented 2026-09-29. The Ubuntu robot PC has 51 GB free; a session produces hundreds of
GB of ProRAW plus tens of GB of masters, and the Mac is not in the lab. So the coordinator
uploads every finished stop to OneDrive-USI while the session runs, verifies the cloud
copy, and only then deletes the local copy and the frames on the phone.

    plan:  upload_remote: "onedrive:SUPSI/<dataset>"     (the start prompt asks for it too; a bare
           name there means onedrive:SUPSI/<name> — every dataset lives under OneDrive-USI/SUPSI)
           upload_what: all | masters | raw
           upload_delete_local: true      upload_delete_phone: true
    code:  coordinator/capture_coordinator/upload_worker.py  (tests: tests/test_upload.py,
           tests/test_dry_run.py `upload` tests, tests/fake_rclone.py stands in for rclone)

## One-time setup

rclone talks to OneDrive with an OAuth token. The token is created on a machine with a
browser (the Mac, once) and pasted into the PC's rclone config; the PC never signs in.

**Status 2026-09-29: done on the Mac.** The token was created, the remote `onedrive` exists in
`~/.config/rclone/rclone.conf` on the Mac (drive `Documenti`, business, 897 GB free), a forced
refresh worked, an hourly `rclone lsd` check runs on the Mac for 30 h
(`~/.config/rclone/refresh_check.log`) to prove the refresh survives the 24 h mark, and the
speed test gave 13.7 MB/s from the Mac's network (a stop ≈ 62 s). What is left for the PC is
step 2b below: copy the config file.

1. **Mac** (rclone 1.75 installed with brew on 2026-09-29):

       rclone authorize "onedrive"

   A browser window opens: sign in with the USI account, accept. The terminal prints
   `Paste the following into your remote machine --->` followed by a JSON token. Copy it.
   If the USI tenant blocks the consent ("need admin approval"), stop here: the fallback is
   an anonymous "anyone with the link can edit" folder with a browser upload, which the
   coordinator cannot drive (see NOTES / the 2026-09-28 discussion).

2. **PC**: install rclone, then EITHER (2b, prepared) copy the Mac's config file — from the Mac on
   the lab switch:

       curl https://rclone.org/install.sh | sudo bash                       # on the PC
       ssh leon@192.168.201.158 'mkdir -p ~/.config/rclone && chmod 700 ~/.config/rclone'   # from the Mac
       scp ~/.config/rclone/rclone.conf leon@192.168.201.158:~/.config/rclone/
       ssh leon@192.168.201.158 'chmod 600 ~/.config/rclone/rclone.conf && rclone lsd onedrive:SUPSI'

   (both machines then hold the same refresh token; Entra keeps the older refresh tokens valid
   when rclone rotates them, and the Mac's copy can be revoked at any time), OR (2a) create the
   remote by hand with the pasted token:

       rclone config
         n) New remote   name> onedrive   Storage> onedrive
         client_id / client_secret: Enter (blank)   region> global
         Edit advanced config? n
         Use web browser to automatically authenticate? n      <- headless
         config_token> <paste the JSON from the Mac>
         Type of connection> onedrive (OneDrive Personal or Business)
         choose the drive it lists, confirm, y) Yes this is OK, q) Quit

   Check: `rclone lsd onedrive:` lists the top-level folders; `rclone about onedrive:`
   the quota. The config lives in `~/.config/rclone/rclone.conf` on the PC: it grants
   access to the whole OneDrive account. Keep it to the `leon` user (`chmod 600`); revoke
   it any time at https://myapps.microsoft.com (rclone under "Apps") or by deleting it.

3. **Measure the upload speed** before the first real session:

       bash tools/upload_speedtest.sh onedrive:SUPSI/speedtest 300

   A 50-frame stop is ~750 MB raw + ~60 MB master. At 10 MB/s that is ~80 s per stop,
   about the capture time of a stop, so the upload keeps up; at 3 MB/s it does not and
   `merge_max_pending` (5 stops on disk) throttles the capture, or you set
   `upload_what: masters` and keep the raw on the phone for later.

**Token lifetime.** The access token lasts ~1.6 h; rclone refreshes it automatically with the
refresh token (proven 2026-09-29). Entra refresh tokens live 90 days by default with a sliding
window; the token's `xms_pftexp` claim sits 24 h after each access token's expiry and moves with
every refresh, so it is not a hard 24 h limit. The 30 h check on the Mac is the empirical proof
that a 12 h session is covered; if it ever shows FAILED, run `rclone authorize` again on the day
of the session and copy the config to the PC (the pre-flight at the session start also tests it).

## Cloud layout (= the existing dataset folders in OneDrive-USI/SUPSI, user 2026-09-29)

    <remote>/raw/<frame>.dng + <frame>.json     every raw frame, flat, phone file names (IMG_<session>_<stop>_<k>)
    <remote>/stacked/<image>.tif                one master per stop, s<step>_p<pose>
    <remote>/previews/<image>.jpg               JPEG previews of the masters
    <remote>/stack_meta/<image>.json            per-stop merge report (stack.json: reference frame, alignment, sharpness)
    <remote>/poses/                             stops.csv, turntable.csv, robot_poses.csv, mocap/<image>.csv, robot_runs/, mocap_bag/
    <remote>/robot/ calibrations/ exposure_check/ + plan.yaml/json, frames.csv, stacks.csv, alignment.csv, session.jsonl, summary.json, SUMMARY.md

Locally the session keeps `images/<image>/stack/` (the merge and `--resume` need it); only the cloud is flat.

## What happens per stop

Two phases when the merge runs inline (since 2026-09-29 afternoon; before that the whole stop
waited for the merge, which is why the first files appeared only minutes after the first stop):

- **raw phase, right after the pull** (parallel to the merge): `stack/` (DNGs + sidecars) is
  copied and checked; with `upload_delete_phone` the phone copy is deleted then. Nothing local
  is deleted, the merge still reads the raw. Event `stop_raw_uploaded`.
- **final phase, after the merge**: copy + check of the full `upload_what` set (the master and
  preview join; the raw is only re-checked, it is already there) → local deletion as
  configured. Event `stop_uploaded`.

1. capture → pull → raw phase; merge → final phase.
2. `rclone copy images/<image>/stack <remote>/raw` (rules: `*.dng`, `*.json`, not stack.json /
   mocap.csv), `rclone copy images/<image> <remote>/stacked` (`/*.tif`), `... <remote>/previews`
   (`/*.jpg`), `rclone copyto stack/stack.json <remote>/stack_meta/<image>.json`, per
   `upload_what`. 4 parallel transfers, rclone retries.
3. `rclone check --one-way` of the same set: every local file must exist in the cloud with
   the same hash (OneDrive QuickXorHash, computed locally). Exit 0 = **verified**.
4. `upload_delete_local`: `true` = the whole local `images/<image>/` folder goes (PC, no disk);
   `raw` = only the raw frames go and master, preview, sidecars, stack.json, mocap.csv stay (Mac,
   the dataset folder is kept, 2026-09-30); `false` = nothing deleted. `--resume` needs frames.csv only.
5. `upload_delete_phone`: the stop's DNG + JSON files are removed from the phone through
   AFC (house arrest, the same connection the pull uses; `pull_worker.py delete`). Only
   allowed when the raw frames were uploaded (`upload_what raw | all`, plan.py refuses
   `masters` + delete_phone, because the raw would then exist nowhere).
6. `session.jsonl` events `stop_uploaded` (files, bytes, seconds, deleted_local,
   deleted_phone) / `stop_upload_retry` / `stop_upload_failed` / `phone_deleted`;
   `summary.json` → `results.stops_uploaded`, `stops_still_local`, `uploaded_gb`,
   `upload_mb_s`, `files_deleted_on_phone`.

Failures: copy or check failing → 3 attempts (20 s apart) → `stop_upload_failed`, the stop
stays on the PC and on the phone, the session goes on. A failed **merge** → the stop is
uploaded as it is but nothing is deleted (a re-merge needs the raw). No rclone / no token /
tenant blocked → the coordinator refuses to start (pre-flight `rclone mkdir` + `lsd`).

`upload_delete_session: true` (unattended runs): at the very end, after the bag recorder and the
tunnel are stopped, the final sync is verified with `rclone check` and the WHOLE session folder on
the PC is deleted, provided every captured stop was uploaded and nothing is left under `images/`.
Otherwise it stays and the reason is printed (`session folder KEPT (...)`).

Every ~3 minutes and at the end the session-level files are synced too: plan, frames.csv,
session.jsonl, summary.json, SUMMARY.md, stacks/alignment csv, `calibrations/`, `robot/`,
`exposure_check/`, **`poses/`**, and at the end `mocap/` → `<remote>/poses/mocap_bag/`.

## The `poses/` folder (user, 2026-09-29)

All pose outputs in one place, locally in `<session>/poses/` and in `<remote>/poses/`
(`capture_coordinator/poses_collect.py`, refreshed after every stop):

| file | content |
|---|---|
| `stops.csv` | one row per stop: step, robot pose, image, capture window, platter commanded/reported deg, popillia yaw, phone + popillia body poses (world, mean over the window), cam_* if a rig file was given, the 7 joints reached |
| `turntable.csv` | every turntable event (commanded deg, reported deg, yaw, how confirmed) |
| `robot_poses.csv` | copy of `robot/robot_poses.csv` (joints per stop from the robot) |
| `mocap/<image>.csv` | the per-stop mocap samples of both bodies |
| `robot_runs/` | the robot scripts' own files matched by the plan's `poses_collect` globs and newer than the session start: `manual_poses_<ts>.csv` (panda_manual_poses.py run log: joints, end-effector pose, lens position per waypoint) and `joint_states_<ts>.db` (record_joint_states.py) |
| `mocap_bag/` | cloud only: the rosbag of the whole session |

Regenerate by hand: `python -m capture_coordinator.poses_collect <session_dir>`.

## Disk budget on the PC

With `upload_delete_local` the session folder holds at most `merge_max_pending`
(`upload_max_pending`) stops that are still merging or uploading: 5 × (750 MB + 60 MB)
≈ 4 GB, plus the sidecars. The start prompt prints the session's full size and the free
space of the chosen folder.
