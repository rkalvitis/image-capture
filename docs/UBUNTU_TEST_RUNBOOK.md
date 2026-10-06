# Ubuntu test run: 9 vertical poses, 1 rotation, OneDrive upload (prepared 2026-09-29)

Goal (no motor: the NEMA 17 board has not arrived, the platter is turned by hand; here 1 rotation
means no turning at all): prove the whole chain on the robot PC alone: coordinator ↔ robot script (socket
confirm) ↔ phone (USB) ↔ mocap bag ↔ inline merge ↔ OneDrive upload with verified deletion
on the PC and on the phone ↔ `poses/` in the cloud. Plan: `coordinator/plan.ubuntu_test.yaml`
(9 stops × 50 frames ≈ 7 GB raw + 0.5 GB masters). The user runs every command on the PC;
paste the terminal output back.

## 0. On the Mac (done 2026-09-29 unless noted)

- App rebuilt and reinstalled with a fresh 7-day profile, developer trusted, launches. Done.
- `rclone` installed (brew), token created, remote `onedrive` configured and tested on the Mac
  (13.7 MB/s upload, refresh works, 30 h refresh check running). Done.
- Sync the repo to the PC (Mac on the lab switch, USB-Ethernet):

      bash tools/sync_to_pc.sh            # -> leon@192.168.201.158:~/shared_ws/roberts_ws/image-capture/

## 1. On the PC, one time

    cd ~/shared_ws/roberts_ws/image-capture
    bash tools/ubuntu_info.sh                              # paste the output (first time on this PC)
    curl -fsSL https://pixi.sh/install.sh | bash           # new shell afterwards
    cd optitrack && pixi install && pixi run build && cd ..   # ~10 min, ROS 2 Humble + rawpy/opencv + pymobiledevice3
    cd optitrack && pixi run record-mock & sleep 10; kill %1; cd ..    # a bag without hardware = the pipeline works
    curl https://rclone.org/install.sh | sudo bash         # rclone
    # then FROM THE MAC: scp ~/.config/rclone/rclone.conf leon@192.168.201.158:~/.config/rclone/  (docs/ONEDRIVE.md 2b)
    chmod 600 ~/.config/rclone/rclone.conf; rclone lsd onedrive:SUPSI     # must list dataset_17_09_2026, ...
    bash tools/upload_speedtest.sh onedrive:SUPSI/speedtest 300     # MB/s -> decides whether raw upload keeps up
    pixi run --manifest-path optitrack/pixi.toml python tests/test_dry_run.py     # 22 tests, ~2 min, no hardware
    pixi run --manifest-path optitrack/pixi.toml python tests/test_upload.py      # 11 tests

Phone: plug in, tap Trust, `pixi run --manifest-path optitrack/pixi.toml pymobiledevice3
usbmux list` shows it. For `launch_app: true` start, in its own terminal and leave it:

    sudo ~/shared_ws/roberts_ws/image-capture/optitrack/.pixi/envs/default/bin/pymobiledevice3 remote tunneld

(without it: tap the app icon once before starting; nothing else changes).

Check in the plan `poses_collect`: the folder where the ROBOT scripts write
`manual_poses_<ts>.csv` / `joint_states_<ts>.db` on this PC (the plan lists both
`~/shared_ws/SynthVLA/src/panda_moveit_ctrl/scripts/` and `~/shared_ws/roberts_ws/robot/`;
wrong globs just match nothing).

## 2. The run (three terminals on the PC's desktop session, not over plain ssh)

Terminal A, ROS 1 (as before the main test): move_group up, then
`rosrun panda_moveit_ctrl record_joint_states.py` (optional but wanted for `poses/robot_runs/`).

Terminal B, ROS 1: the robot run, one cycle, waiting for the coordinator on port 5600:

    bash run_cycles.sh 1 _confirm_port:=5600 _object_radius:=0.04 _rod_radius:=0.01

Terminal C, the coordinator:

    cd ~/shared_ws/roberts_ws/image-capture
    bash tools/run_coordinator.sh --plan coordinator/plan.ubuntu_test.yaml

It asks, Enter = the plan's value:

1. `Number of insect rotations per revolution [1]` → Enter. (With the motor plan it also
   prints the microstep resolution and the worst rounding error: every stop is an
   absolute target rounded to the nearest microstep, so a non-integer 360/N never
   accumulates; N values that are exact are listed.)
2. `Session folder [~/sessions/ubuntu_test_<date>]` → Enter or a path; it prints the
   session's size and the free space there.
3. `Upload to OneDrive [onedrive:SUPSI/ubuntu_test_2026-09-29]` → Enter, or `-` for none.
   Then the pre-flight: the remote must answer (`rclone mkdir` + `lsd`), free quota printed.

Then as on the Mac: tunnel, app, dump-bodies, mocap bag start; the mocap check
(`/optitrack/phone`, `/optitrack/popillia` receiving — Motive streaming); the robot run
starts and reaches the initial pose; **Enter = insect placed**; 9 stops; the status line
shows `pulls / merges / uploads pending`; at the end nothing is asked (`stop_confirm: false`,
unattended): the merges and uploads drain, the bag and tunnel stop, the final sync sends the
mocap bag and is verified, the session folder `~/sessions/ubuntu_test_...` is deleted
(`upload_delete_session: true`), the command exits. The last lines to expect:
`final sync (mocap bag, session files) -> onedrive:...: ok (verified)` and
`session folder deleted (everything is in the cloud)`; a `session folder KEPT (...)` line names
what was not uploaded.

Expected log lines per stop: `[stop k s00_pk] pulled 50 frames`, then `[upload] stop k s00_pk: raw
100 files, 750 MB in ~60 s (12 MB/s) -> onedrive:..., verified (master follows after the merge);
deleted 100 on the phone`, later `[merge] stop k: ok` and `[upload] stop k s00_pk: 102 files, 810 MB
checked ... verified; deleted 52 local`. The merge takes a few minutes per stop (3 in parallel). Pause any time with `p` + Enter, Ctrl-C once, or the PAUSE file.

## 3. After the run, check and paste

- `cat <session>/SUMMARY.md` → `9 / 9 stops`, `9 merged masters`, `9 uploaded to onedrive:...`,
  `stops_still_local 0`, `files_deleted_on_phone 900`, upload MB/s.
- `rclone lsd onedrive:SUPSI/ubuntu_test_2026-09-29` → `raw` (450 dng + 450 json, flat), `stacked`
  (9 tif), `previews`, `stack_meta`, `poses` (with `mocap_bag/`), `robot`, `calibrations`;
  `rclone size onedrive:SUPSI/ubuntu_test_2026-09-29`.
- `ls <session>/images/` → empty: every stop folder is deleted after its verified upload (user, 2026-09-29);
  `find <session>/images -type f | wc -l` → 0.
- `<session>/poses/stops.csv` → 9 rows with joints and body poses; `poses/robot_runs/`
  holds `manual_poses_<ts>.csv` (+ `joint_states_<ts>.db` if the recorder ran).
- Phone: `pixi run pymobiledevice3 apps ...` is not needed; the app's Documents/<session>/
  folder is empty (`python tools/pull_frames.py <session> /tmp/x` lists nothing).
- Merge speed on the i9 and the upload MB/s go into NOTES.md.

## If something fails

| symptom | what to do |
|---|---|
| `OneDrive upload: cannot reach ...` at start | token / tenant: `rclone lsd onedrive:` by hand; or answer `-` to run without upload |
| `[upload] stop k ... check failed` ×3 | the stop stays local + on the phone; `rclone copy` it later by hand; the session continues |
| `[upload] buffer full: 5 stop(s) still merging/uploading` | upload slower than capture; let it wait, or `upload_what: masters` next time |
| phone delete failed | files stay on the phone (harmless); `pull_worker.py delete` needs the same AFC access as the pull |
| app does not launch | tunneld not running as root, or tap the icon; on the Mac the profile expires every 7 days |
| robot run does not start | `_confirm_port:=5600` and `robot_host: 127.0.0.1`; the coordinator reconnects per run |
