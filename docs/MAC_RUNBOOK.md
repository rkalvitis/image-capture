# Mac run (from 2026-09-30): Mac = coordinator + focus stacker + OneDrive uploader

Decided 2026-09-30 (the PC merges too slowly): the robot PC only runs the robot script; the
Mac does the rest over the lab LAN, as in the September dataset. Plan: `coordinator/plan.mactest.yaml`
(2 rotations with the motor, p0..p8, 50 frames, ISO 80 1/100 s, blackout, upload + delete).
Settings the user confirmed: OneDrive app on the Mac keeps files cloud-only (Files On-Demand),
the lab Ethernet gives the Mac internet (same IPs as before), sessions live in
`~/Documents/repos/optitrack/datasets/<name>`; after merge + verified upload the whole
`images/<image>/` folder is deleted (raw AND masters, user 2026-09-30 afternoon), the folder keeps
only the session files, poses/ and the bags (~60 MB), the robot run logs are copied by hand after the run.

## Before the run

- Phone: charged to 100 %, airplane mode on, plugged into the Mac's USB-C, app installed (build of
  2026-09-30 with blackout + per-frame thermal; the free profile expires ~2026-10-07, rebuild then).
- Mac: on the lab switch (USB-Ethernet, 192.168.201.245), `rclone lsd onedrive:SUPSI` answers,
  Motive streaming both bodies, the Portenta on USB (`turntable_drive: portenta`; without the board
  set `turntable_drive: manual` in the plan).
- Robot PC, ROS 1 terminal: move_group up, optionally `rosrun panda_moveit_ctrl record_joint_states.py`,
  then the two runs waiting for the coordinator:

      bash run_cycles.sh 4 _confirm_port:=5600 _obstacles:=false      # rehearsal: 4 runs, no obstacle scene (user 2026-09-30)
      # (run_cycles.sh adds _execute:=true _confirm_via:=socket; `_confirm_via:=none` is the debug walk WITHOUT the coordinator)

## The run (Mac)

    cd ~/Documents/repos/image-capture
    bash tools/run_coordinator.sh --plan coordinator/plan.mactest.yaml

Questions at the start, Enter keeps the plan: rotations `2` (motor: step 180 deg, exact), session
folder `~/Documents/repos/optitrack/datasets/mactest_2026-09-30`, OneDrive `onedrive:SUPSI/mactest_2026-09-30`.
Then: tunnel, app (screen goes black), dump-bodies, mocap bag, mocap check, robot run 1 reaches the
initial pose (the coordinator zeroes the motor there, no prompt: place the insect BEFORE starting), exposure check
(Enter = keep ISO 80 1/100 s, or type an EV once), 9 stops; the motor turns 180 deg, the yaw gate
confirms it, run 2, 9 stops; the platter reverses to 0; merges and uploads drain; bag + tunnel
stop; final sync; the command exits. No Enter at the end.

Per stop in the log: `pulled 50 frames`, `[upload] ... raw 100 files ... verified ... deleted 100
on the phone`, `[merge] stop k: ok in ~90s`, `[upload] ... verified; deleted 102 local` (the whole
stop folder: raw, sidecars, master, preview).

## After the run

1. Robot run logs from the PC into the session, by hand (from the Mac):

       S=~/Documents/repos/optitrack/datasets/mactest_2026-09-30
       scp 'leon@192.168.201.158:~/shared_ws/SynthVLA/src/panda_moveit_ctrl/scripts/manual_poses_2026*.csv' $S/poses/robot_runs/
       scp 'leon@192.168.201.158:~/shared_ws/SynthVLA/src/panda_moveit_ctrl/scripts/joint_states_2026*.db' $S/poses/robot_runs/
       rclone copy $S/poses onedrive:SUPSI/mactest_2026-09-30/poses

2. Check: `cat $S/SUMMARY.md` (18/18 stops, 18 uploaded, files deleted on phone 1800),
   `rclone lsd onedrive:SUPSI/mactest_2026-09-30` (raw, stacked, previews, stack_meta, poses, robot,
   calibrations), `ls $S/images/` is empty.
3. Phone: `python3 tools/pull_frames.py <session_id> /tmp/x` lists nothing (all deleted).
