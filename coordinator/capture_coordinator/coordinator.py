"""The session state machine (docs/DEVELOPMENT_PLAN.md §6).

for turntable step s:                       (outer loop: manual platter, or the motor of hardware/turntable/)
  robot run starts -> reached(0)            initial pose = turntable moment
  TURN    Enter and/or popillia yaw == target ± tol and still       -> go(0)
          (motor: Enter once to place the insect, then GOTO s*step_deg, optional yaw gate)
  (step 0 only, optional) EXPOSURE CHECK: one test frame, statistics, EV to apply
  for pose k in 1..N:
    MOVE   wait reached(k)
    STILL  phone AND popillia jitter < threshold over window (timeout -> ask)
    SHOOT  focus(lens[k]); stack(stop)
    VERIFY still over the capture interval; files pulled; count ok
           fail -> retry (max_retries) -> ask operator: retry / skip / abort
    MERGE  (background) fuse the stack, delete the Mac copies of the raw frames
    UPLOAD (background, plan upload_remote) the raw stack goes to OneDrive right after the pull
           (parallel to the merge), the master after the merge; rclone copy + check, then delete the
           local / phone copies (upload_delete_local / upload_delete_phone) — docs/ONEDRIVE.md
    go(k)
  robot returns -> done
end: wait for the merges and uploads, Enter, stop the mocap recording and the tunnel

At start the command asks (Enter = the plan's value): the session folder, the OneDrive
destination and the number of insect rotations (the step becomes exactly 360/N deg);
--no-ask / --auto-operator take the plan as it is, --session-dir / --remote / --rotations override.

Run it with the optitrack pixi env python for the ROS 2 mocap source:
  bash tools/run_coordinator.sh --plan coordinator/plan.sep15.yaml   [--resume] [--start-step N]
"""
import argparse
import collections
import json
import platform
import socket
import subprocess
import os
import queue
import signal
import sys
import threading
import time

from .console import Console, pause_file
from .launcher import Launcher, Notifier
from .merge_worker import MergeWorker
from .mocap import yaw_deg, yaw_diff_deg
from .phone import PhoneClient
from .poses import fmt_vec, insect_from_cam, load_rig, mat_quat
from .plan import Plan
from .poses_collect import collect_poses
from .robot import RobotClient
from .session import SessionWriter
from .summary import write_summary
from .turntable import TurntableError
from .upload_worker import UploadWorker


class Abort(Exception):
    pass


class Progress:
    """Terminal progress bar for the frames of the current stack + the session.
    Writes with \r on one line; a final newline when the stack is done."""

    def __init__(self, out=None, width=30):
        self.out = out or sys.stdout
        self.width = width
        self.total_stacks = 0
        self.done_stacks = 0
        self._t0 = None

    def start(self, stop_id, pose, n_frames):
        self._t0, self._n, self._label = time.time(), n_frames, f"stop {stop_id} pose {pose}"
        self.frame(0)

    def frame(self, k, lens=None):
        n = max(1, self._n)
        filled = int(self.width * k / n)
        bar = "#" * filled + "-" * (self.width - filled)
        el = time.time() - self._t0
        eta = (el / k * (n - k)) if k else 0.0
        extra = f" lens {lens:.4f}" if lens is not None else ""
        sess = f" | stacks {self.done_stacks}/{self.total_stacks}" if self.total_stacks else ""
        self.out.write(f"\r  {self._label}: [{bar}] {k:3d}/{n} frames{extra}  {el:4.0f}s, ~{eta:3.0f}s left{sess}   ")
        self.out.flush()

    def finish(self, ok=True):
        self.done_stacks += 1 if ok else 0
        self.out.write("\n")
        self.out.flush()


class Coordinator:
    def __init__(self, plan, phone, robot_factory, mocap, puller, writer,
                 operator=None, on_turntable_prompt=None, log=print, start_step=0, turntable=None):
        self.plan, self.phone, self.robot_factory = plan, phone, robot_factory
        self.mocap, self.puller, self.writer = mocap, puller, writer
        self.operator = operator or self._ask_operator
        self.on_turntable_prompt = on_turntable_prompt
        self.turntable = turntable                                   # None = manual platter (Enter)
        self._tt_started = False                                     # first rotation of this run done (position set)
        self._tt_yaw_sign = None                                     # +1/-1: sense of the popillia yaw per motor degree
        self._tt_at_zero = False                                     # rotations-per-pose: platter known to sit at step 0
        self._pre_turn = None                                        # (step, thread, result) of a motor move started early
        self._exposure_checked = False
        self.log = log
        self.start_step = int(start_step)
        self.stop_id = writer.max_stop + 1 if writer.resumed else 0
        self.yaw0 = writer.yaw0 if writer.resumed else None
        self.exposure = getattr(writer, "exposure", None)          # applied exposure of a resumed session
        self.notifier = Notifier(enabled=plan.notify, repeat_s=plan.notify_repeat_s, voice=plan.notify_voice, log=log)
        self.pause_requested = False                                 # set by the console ('p'), Ctrl-C, or the PAUSE file
        self.console = None
        # the pull + bookkeeping of every captured stop run in this background thread (never blocks the capture loop)
        self._final_q = queue.Queue()
        self._final_pending = 0
        self._final_lock = threading.Lock()
        self._final_thr = threading.Thread(target=self._finalize_loop, daemon=True, name="finalize")
        self._final_thr.start()
        self._moved_t = 0.0                                          # time of the last movement (robot reached / insect turned)
        self._thermal_last = "nominal"                               # worst thermal state reported in the last stack
        self._t_session = time.time()                                # robot run files older than this are not ours
        self.stats = {"stops_ok": 0, "retries": 0, "skipped": 0, "already_done": 0}
        self.focus_table = plan.load_focus_table()
        if self.focus_table:
            self.log(f"focus table: {sum(len(v) for v in self.focus_table.values())} entries for poses "
                     f"{sorted(self.focus_table)} from {plan.focus_table}")
        self.T_body_cam = None
        if plan.rig_transforms and os.path.exists(plan.rig_transforms):
            self.T_body_cam = load_rig(plan.rig_transforms)
            self.log(f"camera poses: T_body<-cam from {plan.rig_transforms}")
        elif plan.rig_transforms:
            self.log(f"WARNING rig_transforms not found: {plan.rig_transforms} -> no cam_* columns; "
                     f"compute them later with colmap_export.py --rig")
        self.progress = Progress(out=sys.stdout) if log is print else Progress(out=open(os.devnull, "w"))
        self.progress.total_stacks = plan.turntable_steps * (plan.poses + (1 if plan.shoot_initial_pose else 0))
        self.progress.done_stacks = len(writer.done)
        if writer.resumed:
            self.log(f"RESUME: {len(writer.done)} stops already captured, next stop id {self.stop_id}, "
                     f"starting at turntable step {self.start_step}" + (f", yaw0 {self.yaw0:.2f}" if self.yaw0 is not None else ""))
        self.uploader = None
        self._stop_files = {}                                        # stop -> (image, phone file names) for the upload
        if plan.upload_remote:
            self.uploader = UploadWorker(writer.dir, plan.upload_remote, what=plan.upload_what,
                                         delete_local=plan.upload_delete_local, delete_phone=plan.upload_delete_phone,
                                         puller=puller, phone_session=writer.session_id, rclone=plan.upload_rclone,
                                         transfers=plan.upload_transfers, bwlimit=plan.upload_bwlimit,
                                         timeout_s=plan.upload_timeout_s, log=self.log, event=writer.event)
            self.log(f"upload: {plan.upload_what} -> {plan.upload_remote} after every stop; delete local = "
                     f"{plan.upload_delete_local}, delete on the phone = {plan.upload_delete_phone}")
        self.merger = None
        if plan.merge_inline:
            extra = ["--fusion", plan.merge_fusion, "--format", plan.merge_format, "--mask", plan.merge_mask]
            if plan.merge_index_maps:
                extra.append("--index-maps")
            self.merger = MergeWorker(writer.dir, plan.merge_ref(), python=plan.merge_python or None,
                                      delete_raw=plan.merge_delete_raw, log=self.log, extra_args=extra,
                                      jobs=plan.merge_jobs, on_done=self._on_merge_done if self.uploader else None)
            self.log(f"inline merge: {plan.merge_fusion} fusion, {plan.merge_format}, {plan.merge_jobs} job(s), "
                     f"ref index {plan.merge_ref()}, delete raw on the Mac = {plan.merge_delete_raw}")

    # -- operator ------------------------------------------------------
    @staticmethod
    def _ask_operator(question, choices):
        """Interactive prompt. choices ['enter', ...]: an empty line = 'enter';
        a number 0..1 is returned as-is when 'lens' is among the choices, any
        number when 'ev' is."""
        while True:
            ans = input(f"{question} [{'/'.join(choices)}] ").strip().lower()
            if ans == "" and "enter" in choices:
                return "enter"
            if "lens" in choices or "ev" in choices:
                try:
                    v = float(ans)
                    if "ev" in choices or 0.0 <= v <= 1.0:
                        return v
                except ValueError:
                    pass
            for c in choices:
                if ans == c or (ans and c.startswith(ans)):
                    return c

    def _ask(self, question, choices, spoken=None):
        """Operator prompt with the attention call (sound + speech + banner, repeated) until answered."""
        self.notifier.start(spoken or question.strip().lstrip(">").strip())
        try:
            return self.operator(question, choices)
        finally:
            self.notifier.stop()

    def _thresholds(self):
        return {"phone": self.plan.still_phone_mm, "popillia": self.plan.still_popillia_mm}

    # -- pause / resume --------------------------------------------------
    def request_pause(self):
        self.pause_requested = True

    def status_line(self):
        pend = self.merger.pending if self.merger else 0
        up = f", uploads pending {self.uploader.pending}" if self.uploader else ""
        return (f"stops ok {self.stats['stops_ok']}, next stop id {self.stop_id}, step {getattr(self, '_current_step', '-')}, "
                f"pulls pending {self._final_pending}, merges pending {pend}{up}" + (", PAUSE requested" if self.pause_requested else ""))

    def _on_merge_done(self, stop, ok):
        """MergeWorker callback (merge thread): hand the stop to the upload worker."""
        image, names = self._stop_files.get(stop, (None, []))
        if image is None:
            self.log(f"[upload] stop {stop}: no image name recorded — not uploaded")
            return
        self.uploader.submit(stop, image, names, merged_ok=ok)

    def _buffer_pending(self):
        """Stops whose files are still on this disk waiting for the merge and/or the upload."""
        return (self.merger.pending if self.merger else 0) + (self.uploader.pending if self.uploader else 0)

    # -- background finalize (pull + rows + merge hand-off) ------------------
    def _finalize_async(self, pend):
        if pend is None:
            return
        with self._final_lock:
            self._final_pending += 1
        self._final_q.put(pend)

    def _finalize_loop(self):
        while True:
            pend = self._final_q.get()
            if pend is None:
                return
            try:
                self._finalize(pend)
            except Exception as err:                       # noqa: BLE001
                self.log(f"[stop {pend.get('stop')}] finalize failed: {err}")
                self.writer.event("stop_finalize_failed", stop=pend.get("stop"), reason=str(err))
            finally:
                with self._final_lock:
                    self._final_pending -= 1

    def _drain_finalize(self, log_every=15.0):
        last = time.time()
        while True:
            with self._final_lock:
                n = self._final_pending
            if n == 0:
                return
            if time.time() - last > log_every:
                self.log(f"[pull] waiting for {n} stop(s) still being pulled/recorded...")
                last = time.time()
            time.sleep(0.2)

    def _check_pause(self, where):
        """Checkpoint: pause here if requested (console 'p', Ctrl-C, or <session>/PAUSE).
        The arm holds its pose, the phone idles, merges continue. Enter resumes."""
        pf = pause_file(self.writer.dir)
        if self.console is not None and self.console.pause_requested:
            self.pause_requested = True
        if not self.pause_requested and not os.path.exists(pf):
            return
        self.pause_requested = False
        if self.console is not None:
            self.console.pause_requested = False
        self.writer.event("paused", where=where, stop=self.stop_id)
        self.log(f"\n>>> PAUSED ({where}). {self.status_line()}")
        ans = self._ask(">>> Paused. Enter = resume, abort = end the session", ["enter", "abort"],
                        "Session paused. Press Enter to resume.")
        if os.path.exists(pf):
            try:
                os.remove(pf)
            except OSError:
                pass
        if ans == "abort":
            raise Abort()
        self.writer.event("resumed", where=where, stop=self.stop_id)
        self.log(">>> resumed")
        self._moved_t = time.time()                    # a long pause: treat as a fresh settle

    # -- session -------------------------------------------------------
    def _exposure_dict(self):
        p = self.plan
        if self.exposure:
            return {"mode": "custom", "duration_s": float(self.exposure["duration_s"]), "iso": float(self.exposure["iso"])}
        return {"mode": p.exposure_mode, "duration_s": p.exposure_duration_s, "iso": p.exposure_iso}

    def _hello(self):
        p = self.plan
        power = {"raw": p.phone_raw or None, "quality": p.phone_quality or None,
                 "frame_rate": p.phone_frame_rate or None,
                 "screen_brightness": p.phone_screen_brightness if p.phone_screen_brightness >= 0 else None,
                 "preview": p.phone_preview, "blackout": True if p.phone_blackout else None}
        return self.phone.hello(session=self.writer.session_id, lens_positions=p.lens_for_pose(1), settle_ms=p.settle_ms,
                                exposure=self._exposure_dict(), wb=p.wb, power=power)

    def run(self):
        p = self.plan
        self.writer.event("session_start", session=self.writer.session_id, plan=p.name, resumed=self.writer.resumed,
                          start_step=self.start_step, loop_order=p.loop_order, host=socket.gethostname(),
                          platform=platform.platform(), coordinator_commit=repo_commit(),
                          turntable_drive=self.turntable.drive if self.turntable is not None else "manual")
        self.phone.connect()
        if self.turntable is not None:
            info = self.turntable.connect()
            self.log(f"[turntable] {self.turntable.drive}: {info}")
            self.writer.event("turntable_ready", drive=self.turntable.drive, **info)
        ready = self._hello()
        offset = self.phone.ping()
        self.log(f"[phone] ready: {ready.get('device')} {ready.get('lens')} {ready.get('format')} "
                 f"(raw {ready.get('raw')}, quality {ready.get('quality')}, preview {ready.get('preview')}); "
                 f"clock offset {offset*1000:.1f} ms; calib {ready.get('calib_available')}")
        self.writer.event("phone_ready", reply=ready, clock_offset_s=offset, exposure=self._exposure_dict())
        try:
            if p.loop_order == "rotations-per-pose":
                self._pose_outer_run()
            else:
                for step in range(self.start_step, p.turntable_steps):
                    self._turntable_step(step)
                if p.turntable_steps > 1 and (self.turntable is not None and self._tt_started):
                    self._turntable_return("session")        # undo the revolution backwards (thread height)
                elif p.turntable_steps > 1 and self.turntable is None:
                    self._turntable_return("session")        # manual platter: one Enter after turning it back
        except (Abort, KeyboardInterrupt):
            self.log("session aborted by the operator")
            self.writer.event("session_abort", stats=self.stats)
        else:
            self.writer.event("session_end", stats=self.stats)
        finally:
            self._drain_finalize()                       # every captured stop pulled and recorded
            self.phone.close()
            if self.turntable is not None:
                self.turntable.close()
            if self.merger:
                self.log("waiting for the background merges to finish...")
                self.merger.drain()
                self.stats.update(self.merger.summary())
                self.writer.event("merge_done", **self.merger.summary())
                self.merger.close()
            if self.uploader:
                self.log("waiting for the background uploads to finish...")
                self.uploader.drain()
                self.stats.update(self.uploader.summary())
                self.writer.event("upload_done", **self.uploader.summary())
            self._write_summary(final=True)                # session.jsonl is flushed per event: complete already
            self._collect_poses()
            if self.uploader:
                ok = self.uploader.upload_session_files(final=False)     # mocap/ follows in main() once the bag is closed
                self.log(f"session files -> {self.plan.upload_remote}: {'ok' if ok else 'FAILED (see above)'}")
                self.uploader.close()
            self.writer.close()
        self.log(f"session {self.writer.session_id} finished: {self.stats}")
        return self.stats

    def _collect_poses(self):
        """<session>/poses/: stops.csv, turntable.csv, robot_poses.csv, mocap/<image>.csv and the robot
        script's own run files (plan poses_collect globs) — after every stop and at the end."""
        try:
            collect_poses(self.writer.dir, self.plan.poses_collect, since=self._t_session, log=self.log)
        except Exception as e:                       # never let the bookkeeping stop a session
            self.log(f"WARNING poses/ not collected: {e}")

    def _write_summary(self, final=False):
        """<session>/summary.json + SUMMARY.md (rewritten after every stop, final at the end)."""
        try:
            paths = write_summary(self.writer.dir, extra={"stats": dict(self.stats), "final": final,
                                                          "plan": self.plan.to_dict()})
            if final:
                self.log(f"summary: {paths[1]}")
        except Exception as e:                       # never let the bookkeeping stop a session
            self.log(f"WARNING summary not written: {e}")

    # -- one turntable step = one robot run ----------------------------
    def _turntable_step(self, step):
        p = self.plan
        self._current_step = step
        robot = self.robot_factory()
        self.log(f"[step {step}] waiting for the robot run to start (port {p.robot_port})...")
        robot.connect()
        if robot.poses and robot.poses != p.poses:
            self.log(f"WARNING: robot has {robot.poses} poses, plan says {p.poses} — using the robot's count")
        n_poses = robot.poses or p.poses
        msg = robot.wait_reached()
        if msg.get("type") != "reached" or int(msg.get("id", -1)) != 0:
            raise RuntimeError(f"expected reached 0 from the robot, got {msg}")
        self._moved_t = time.time()
        self.writer.event("robot_initial", step=step, joints=msg.get("joints"))
        self._wait_turntable(step)
        self._moved_t = time.time()                      # the insect was just turned / placed
        if step == 0 and p.exposure_check and not self.exposure and not self._exposure_checked:
            self._exposure_checked = True
            self._exposure_check(step, 0 if p.shoot_initial_pose else 1)
        try:
            pending = None
            if p.shoot_initial_pose:
                self.writer.robot_pose(step, 0, self.stop_id, self.writer.stack_name(step, 0), msg.get("joints"))
                self._check_pause(f"step {step} pose 0, before the stack")
                pending = self._stop_or_skip(step, 0)    # robot pose 0 = the initial pose (joint_start.csv)
            self._check_pause(f"step {step} pose 0, arm holding")
            robot.go(0)                                  # the arm leaves as soon as the frames are shot ...
            self._finalize_async(pending)                # ... pull / bookkeeping / merge hand-off in the background
            for k in range(1, n_poses + 1):
                msg = robot.wait_reached()
                if msg.get("type") == "done":
                    raise RuntimeError("robot finished the run early")
                if int(msg["id"]) != k:
                    self.log(f"WARNING: robot reached {msg['id']}, expected {k}")
                    k = int(msg["id"])
                self._moved_t = time.time()
                self.writer.event("robot_reached", step=step, pose=k, joints=msg.get("joints"))
                self.writer.robot_pose(step, k, self.stop_id, self.writer.stack_name(step, k), msg.get("joints"))
                self._check_pause(f"step {step} pose {k}, before the stack")
                pending = self._stop_or_skip(step, k)
                self._check_pause(f"step {step} pose {k}, arm holding")
                robot.go(k)
                self._finalize_async(pending)
                if k == n_poses:
                    self._start_pre_turn(step + 1)       # motor: turn NOW, while the arm goes home (user, 2026-09-30)
            robot.wait_done()
            self.writer.event("robot_done", step=step)
        except (Abort, KeyboardInterrupt):
            robot.abort()
            try:
                robot.wait_done(timeout=120)
            except Exception:
                pass
            raise
        finally:
            robot.close()

    # -- loop_order rotations-per-pose: one robot run, all rotations at every pose ---------
    def _pose_outer_run(self):
        p = self.plan
        if self.start_step:
            self.log(f"NOTE: --start-step {self.start_step} is ignored with loop_order rotations-per-pose "
                     f"(finished stops are skipped instead)")
        robot = self.robot_factory()
        self.log(f"waiting for the robot run to start (port {p.robot_port}): one run, "
                 f"{p.turntable_steps} rotations at every pose...")
        robot.connect()
        if robot.poses and robot.poses != p.poses:
            self.log(f"WARNING: robot has {robot.poses} poses, plan says {p.poses} — using the robot's count")
        n_poses = robot.poses or p.poses
        msg = robot.wait_reached()
        if msg.get("type") != "reached" or int(msg.get("id", -1)) != 0:
            raise RuntimeError(f"expected reached 0 from the robot, got {msg}")
        self._moved_t = time.time()
        self.writer.event("robot_initial", step=0, joints=msg.get("joints"))
        self._wait_turntable(0)                          # Enter: insect placed, platter at step 0 (motor: SETPOS 0)
        self._tt_at_zero = True
        self._moved_t = time.time()
        if p.exposure_check and not self.exposure and not self._exposure_checked:
            self._exposure_checked = True
            self._exposure_check(0, 0 if p.shoot_initial_pose else 1)
        try:
            if p.shoot_initial_pose:
                self._rotations_at_pose(0, msg.get("joints"))
            self._check_pause("pose 0, arm holding")
            robot.go(0)
            for k in range(1, n_poses + 1):
                msg = robot.wait_reached()
                if msg.get("type") == "done":
                    raise RuntimeError("robot finished the run early")
                if int(msg["id"]) != k:
                    self.log(f"WARNING: robot reached {msg['id']}, expected {k}")
                    k = int(msg["id"])
                self._moved_t = time.time()
                self.writer.event("robot_reached", step=0, pose=k, joints=msg.get("joints"))
                self._rotations_at_pose(k, msg.get("joints"))
                self._check_pause(f"pose {k}, arm holding")
                robot.go(k)
            robot.wait_done()
            self.writer.event("robot_done", step=0)
        except (Abort, KeyboardInterrupt):
            robot.abort()
            try:
                robot.wait_done(timeout=120)
            except Exception:
                pass
            raise
        finally:
            robot.close()

    def _rotations_at_pose(self, k, joints):
        """All turntable steps at robot pose k (arm holding), then the platter goes BACK to step 0."""
        p = self.plan
        moved = False
        for step in range(p.turntable_steps):
            self._current_step = step
            if (step, k) in self.writer.done:
                self._stop_or_skip(step, k)              # logs stop_already_done; the platter is not moved for it
                continue
            if not (step == 0 and self._tt_at_zero):
                self._wait_turntable(step)
                self._moved_t = time.time()
                self._tt_at_zero = (step == 0)
                moved = moved or step != 0
            self.writer.robot_pose(step, k, self.stop_id, self.writer.stack_name(step, k), joints)
            self._check_pause(f"pose {k} step {step}, before the stack")
            pending = self._stop_or_skip(step, k)
            self._finalize_async(pending)                # pull in the background while the platter turns
        if moved or not self._tt_at_zero:
            self._turntable_return(k)

    def _turntable_return(self, k):
        """Reverse the platter to step 0 (never a full turn forward: the insect sits on a thread
        and would creep in height over many revolutions). Called before the arm leaves pose k
        (rotations-per-pose) and, with k = "session", after the last rotation of a
        poses-per-rotation session (user, 2026-09-30: every full revolution is undone backwards)."""
        label = f"pose {k}" if not isinstance(k, str) else k
        if self.turntable is not None:
            self.log(f"[{label}] platter returning to step 0 (reverse)...")
            info = self._tt_call(lambda: self.turntable.goto(0.0), "GOTO 0 (return)")
            self.log(f"[{label}] platter back at {info.get('pos_deg', 0.0):.3f} deg ({info.get('move_s', 0):.1f} s)")
            self.writer.event("turntable_return", pose=k, drive=self.turntable.drive, yaw=self._popillia_yaw(), **info)
        else:
            what = f"Pose {k} done" if not isinstance(k, str) else "All rotations done"
            ans = self._ask(f"\n>>> {what}. Turn the insect BACK to step 0 (reverse, the same way back), then press Enter",
                            ["enter", "abort"], "All rotations done. Turn the insect back to the start, then press Enter.")
            if ans == "abort":
                raise Abort()
            self.writer.event("turntable_return", pose=k, drive="manual", confirmed="enter", yaw=self._popillia_yaw())
        self._tt_at_zero = True
        self._moved_t = time.time()

    def _stop_or_skip(self, step, k):
        """Capture the stop (with retries); returns the pending capture for _finalize, or None."""
        if (step, k) in self.writer.done:
            self.log(f"[step {step} pose {k}] already captured as {self.writer.done[(step, k)]} — skipped (resume)")
            self.writer.event("stop_already_done", step=step, pose=k, image=self.writer.done[(step, k)])
            self.stats["already_done"] += 1
            return None
        return self._stop(step, k)

    def _start_pre_turn(self, step):
        """poses-per-rotation with the motor: the platter move for `step` starts in the background
        as soon as the last stack of the previous rotation is shot; `_wait_turntable(step)`
        then only joins it (the 10 s settle counts from the end of the move)."""
        p = self.plan
        if self.turntable is None or not self._tt_started or step >= p.turntable_steps:
            return
        result = {}

        def run():
            try:
                self._motor_turntable(step)
                result["ok"] = True
            except BaseException as e:                   # Abort / TurntableError -> re-raised by the join
                result["error"] = e
        t = threading.Thread(target=run, daemon=True, name=f"pre-turn-{step}")
        t.start()
        self._pre_turn = (step, t, result)
        self.writer.event("turntable_early", step=step)
        self.log(f"[step {step}] platter turning while the arm returns (early move)")

    def _wait_turntable(self, step):
        if self._pre_turn is not None and self._pre_turn[0] == step:
            _s, t, result = self._pre_turn
            self._pre_turn = None
            t.join()
            if "error" in result:
                raise result["error"]
            return
        if self.turntable is not None:
            return self._motor_turntable(step)
        p = self.plan
        thr = self._thresholds()
        if p.turntable_confirm in ("enter", "both") or not self.mocap.enabled:
            last = self.mocap.latest("popillia") if self.mocap.enabled else None
            yaw_now = yaw_deg(last[2]) if last else None
            target = None
            if self.yaw0 is not None and yaw_now is not None:
                target = self.yaw0 + step * p.turntable_step_deg
            if step == 0 and self.yaw0 is None:
                q = (f"\n>>> Robot at the initial pose. Place/check the insect (step 0"
                     + (f", yaw now {yaw_now:.1f} deg" if yaw_now is not None else "") + "), then press Enter to start")
            else:
                q = (f"\n>>> TURN the insect to step {step} (your direction and index"
                     + (f", yaw now {yaw_now:.1f} deg" if yaw_now is not None else "") + "), then press Enter")
            spoken = ("Robot at the initial pose. Place the insect, then press Enter." if step == 0 and self.yaw0 is None
                      else f"Rotation {step}: turn the insect, then press Enter.")
            ans = self._ask(q, ["enter", "abort"], spoken)
            if ans == "abort":
                raise Abort()
            last = self.mocap.latest("popillia") if self.mocap.enabled else None
            yaw = yaw_deg(last[2]) if last else None
            if self.yaw0 is None and yaw is not None:
                self.yaw0 = yaw
            self.writer.event("turntable", step=step, yaw=yaw, target_yaw=(self.yaw0 + step * p.turntable_step_deg)
                              if self.yaw0 is not None else None, confirmed="enter")
            if p.turntable_confirm == "enter" or not self.mocap.enabled:
                return
            # both: fall through to the yaw check below
        if self.yaw0 is None:
            if p.stillness_gate:
                ok, j, _ = self.mocap.wait_still(thr, p.still_window_s, p.still_timeout_s)
            last = self.mocap.latest("popillia")
            self.yaw0 = yaw_deg(last[2]) if last else 0.0
            target = self.yaw0
            self.log(f"[step 0] reference popillia yaw = {self.yaw0:.2f} deg")
        else:
            target = self.yaw0 + step * p.turntable_step_deg
            if p.turntable_confirm == "mocap":
                self.log(f"\n>>> TURN the insect to step {step} (yaw {target:.1f} deg, "
                         f"+{p.turntable_step_deg:.0f} deg from the last step). Waiting for mocap...")
        if self.on_turntable_prompt:
            self.on_turntable_prompt(step, target)
        while True:
            t0 = time.time()
            while True:
                last = self.mocap.latest("popillia")
                yaw = yaw_deg(last[2]) if last else None
                still, j = self.mocap.is_still(thr, p.still_window_s)
                if not p.stillness_gate:
                    still = last is not None and time.time() - last[0] < 0.5     # only "samples arriving"
                if yaw is not None and abs(yaw_diff_deg(yaw, target)) <= p.turntable_yaw_tol_deg and still:
                    self.log(f"[step {step}] turntable at {yaw:.2f} deg (target {target:.1f}), still {j}")
                    self.writer.event("turntable", step=step, target_yaw=target, yaw=yaw, jitter=j)
                    return
                if time.time() - t0 > (15.0 if p.turntable_confirm == "both" else max(60.0, p.still_timeout_s * 4)):
                    break
                time.sleep(0.1)
            ans = self._ask(f"turntable not at step {step} (yaw {None if yaw is None else round(yaw, 2)}, "
                            f"target {target:.1f}, still {still}) — turn/adjust, then",
                            ["retry", "accept", "abort"], f"Turntable not at step {step}. Adjust the insect.")
            if ans == "accept":
                self.writer.event("turntable", step=step, target_yaw=target, yaw=yaw, jitter=j, accepted=True)
                return
            if ans == "abort":
                raise Abort()

    def _popillia_yaw(self):
        last = self.mocap.latest("popillia") if self.mocap.enabled else None
        return yaw_deg(last[2]) if last else None

    def _motor_turntable(self, step):
        """Motorised platter: no operator input at all — the platter is zeroed at the first step
        (the insect was placed before the run), then absolute GOTO per step; `turntable_confirm` mocap|both waits (bounded) for the popillia
        yaw to reach the expected value and RECORDS the outcome — it never asks the operator."""
        p = self.plan
        target_deg = step * p.turntable_step_deg
        if not self._tt_started:
            # no prompt (user, 2026-09-30): the insect is placed before the run starts; the platter's current
            # position becomes stop `step` (ZERO) the moment the robot reaches its initial pose
            info = self._tt_call(lambda: self.turntable.set_position(target_deg), f"SETPOS {target_deg:g}")
            self._tt_started = True
            yaw = self._popillia_yaw()
            if self.yaw0 is None and yaw is not None:
                self.yaw0 = yaw
            self.log(f"[step {step}] platter position set to {target_deg:g} deg (no prompt)"
                     + (f", popillia yaw {yaw:.2f}" if yaw is not None else ""))
            self.writer.event("turntable", step=step, yaw=yaw, target_yaw=(self.yaw0 + step * p.turntable_step_deg)
                              if self.yaw0 is not None else None, confirmed="auto", drive=self.turntable.drive,
                              commanded_deg=target_deg, **info)
            return
        yaw_before = self._popillia_yaw()
        while True:
            self.log(f"[step {step}] turning the platter to {target_deg:g} deg...")
            info = self._tt_call(lambda: self.turntable.goto(target_deg), f"GOTO {target_deg:g}")
            self._moved_t = time.time()
            self.log(f"[step {step}] platter at {info.get('pos_deg', target_deg):.3f} deg ({info.get('move_s', 0):.1f} s)")
            gate = p.turntable_confirm in ("mocap", "both") and self.mocap.enabled
            expect = None
            if self.yaw0 is not None:
                if self._tt_yaw_sign is None and yaw_before is not None:
                    # learn the sense of the yaw per platter degree from this first move
                    time.sleep(min(1.0, max(0.2, p.still_window_s)))
                    y = self._popillia_yaw()
                    if y is not None:
                        d = yaw_diff_deg(y, yaw_before)
                        if abs(d) > p.turntable_yaw_tol_deg:
                            self._tt_yaw_sign = 1 if d > 0 else -1
                            self.log(f"[step {step}] popillia yaw moved {d:+.2f} deg for +{p.turntable_step_deg:g} platter deg "
                                     f"-> yaw sign {self._tt_yaw_sign:+d}")
                if self.on_turntable_prompt and not gate:
                    self.on_turntable_prompt(step, self.yaw0 + (self._tt_yaw_sign or 1) * target_deg)
                expect = self.yaw0 + (self._tt_yaw_sign or 1) * target_deg
            if self.on_turntable_prompt and gate:
                self.on_turntable_prompt(step, expect if expect is not None else target_deg)
            if not gate:
                yaw = self._popillia_yaw()
                self.writer.event("turntable", step=step, yaw=yaw, target_yaw=expect, confirmed="motor",
                                  drive=self.turntable.drive, commanded_deg=target_deg, **info)
                return
            # yaw gate (mocap | both): the insect body must sit at the expected yaw and be still
            thr = self._thresholds()
            t0 = time.time()
            yaw, still, j = None, False, None
            while time.time() - t0 < max(15.0, p.still_timeout_s):
                yaw = self._popillia_yaw()
                still, j = self.mocap.is_still(thr, p.still_window_s)
                if not p.stillness_gate:
                    last = self.mocap.latest("popillia")
                    still = last is not None and time.time() - last[0] < 0.5
                if yaw is not None and expect is not None and abs(yaw_diff_deg(yaw, expect)) <= p.turntable_yaw_tol_deg and still:
                    self.log(f"[step {step}] turntable at yaw {yaw:.2f} deg (expected {expect:.1f}), still {j}")
                    self.writer.event("turntable", step=step, target_yaw=expect, yaw=yaw, jitter=j, confirmed="motor+mocap",
                                      drive=self.turntable.drive, commanded_deg=target_deg, **info)
                    return
                time.sleep(0.1)
            # the motor did the rotation: never stop to ask (user, 2026-09-30) — the yaw is recorded as it is
            self.log(f"[step {step}] WARNING popillia yaw {None if yaw is None else round(yaw, 2)} deg, expected "
                     f"{None if expect is None else round(expect, 1)} (still {still}) — continuing, recorded as is")
            self.writer.event("turntable", step=step, target_yaw=expect, yaw=yaw, jitter=j, accepted=True, auto=True,
                              drive=self.turntable.drive, commanded_deg=target_deg, **info)
            return

    def _tt_call(self, fn, what):
        """Run a turntable command; on a serial error ask retry / abort (never silently continue)."""
        while True:
            try:
                return fn() or {}
            except TurntableError as e:
                self.log(f"[turntable] ERROR {what}: {e}")
                self.writer.event("turntable_error", command=what, error=str(e))
                ans = self._ask(f"turntable {what} failed: {e}. Check the USB/board/power, then",
                                ["retry", "abort"], "Turntable error. Check the board.")
                if ans == "abort":
                    raise Abort()
                try:
                    self.turntable.close(); self.turntable.connect()
                except TurntableError as e2:
                    self.log(f"[turntable] reconnect failed: {e2}")

    # -- exposure check (step 0) ---------------------------------------
    def _exposure_check(self, step, k):
        """One test frame at the first stop's init focus; print the statistics
        of +0 .. +2 EV; apply the chosen EV as a custom exposure; repeat until
        the operator accepts (Enter with 0 / 'keep')."""
        p = self.plan
        try:
            from .exposure import apply_ev, auto_ev, exposure_stats, format_stats
            from .stack_merge import decode_dng
        except ImportError as err:
            self.log(f"exposure check skipped (needs opencv/rawpy: {err})")
            return
        if p.focus_mode == "table":
            centre, source, _ = self._focus_from_table(k)
        else:
            centre, source = p.init_focus(k), p.focus_mode
        out_dir = os.path.join(self.writer.dir, "exposure_check")
        auto = p.exposure_check_ev == "auto"
        suggest = 0.0 if auto else float(p.exposure_check_ev)
        n = 0
        while True:
            n += 1
            self._settle_after_move()
            self.phone.focus(centre)
            done = self.phone.stack(9000 + n, k, step, [centre])
            f = done["frames"][0]
            names = [f["file"], os.path.splitext(f["file"])[0] + ".json"]
            pulled = self.puller.pull(self.writer.session_id, names, out_dir, out_dir)
            exp = f.get("exposure") or {}
            try:
                lin, _, _ = decode_dng(pulled[0])
                st = exposure_stats(lin)
            except Exception as err:                       # noqa: BLE001
                self.log(f"exposure check: cannot analyse {pulled[0]}: {err}")
                return
            self.log(f"\n[exposure check {n}] test frame {f['file']} at lens {centre:.4f} ({source}): "
                     f"1/{1.0 / max(float(exp.get('duration_s', 0.01)), 1e-6):.0f} s, ISO {float(exp.get('iso', 0)):.0f}")
            self.log(format_stats(st))
            if auto and n == 1:
                suggest = auto_ev(st)
                self.log(f"  auto: {suggest:+.1f} EV puts the white foam (frame p99) at 0.85 sRGB")
            self.writer.event("exposure_check", n=n, file=f["file"], exposure=exp, stats={kk: v for kk, v in st.items() if kk != "rows"},
                              rows=st["rows"], suggested_ev=suggest)
            ans = self._ask(f">>> EV to apply on top of this exposure (Enter = {suggest:+.1f}, 0 = keep as is, "
                            f"or type a value)", ["enter", "ev", "keep", "abort"], "Exposure check: choose the EV.")
            if ans == "abort":
                raise Abort()
            ev = suggest if ans == "enter" else (0.0 if ans == "keep" else float(ans))
            if abs(ev) < 1e-6:
                self.writer.event("exposure_accepted", n=n, exposure=exp)
                return
            d, iso = apply_ev(float(exp.get("duration_s", 0.01)), float(exp.get("iso", 100)), ev,
                              max_duration_s=p.exposure_max_duration_s)
            self.exposure = {"duration_s": d, "iso": iso, "ev_total": (self.exposure or {}).get("ev_total", 0.0) + ev}
            self.log(f"applying {ev:+.2f} EV -> custom exposure 1/{1.0 / d:.0f} s, ISO {iso:.0f}")
            self.writer.event("exposure_applied", n=n, ev=ev, exposure=self.exposure)
            self._hello()
            suggest = 0.0                                   # next round: Enter = accept

    # -- one stop ------------------------------------------------------
    def _stop(self, step, k):
        p = self.plan
        attempt = 0
        while True:
            attempt += 1
            try:
                pending = self._capture(step, k, attempt)
                self.stop_id += 1
                return pending
            except Abort:
                raise
            except Exception as err:
                if "skipped by the operator" in str(err):
                    self.stats["skipped"] += 1
                    self.writer.event("stop_skipped", stop=self.stop_id, step=step, pose=k)
                    self.stop_id += 1
                    return None
                self.log(f"[stop {self.stop_id}] attempt {attempt} failed: {err}")
                self.writer.event("stop_failed", stop=self.stop_id, step=step, pose=k, attempt=attempt, reason=str(err))
                if attempt <= p.max_retries:
                    self.stats["retries"] += 1
                    continue
                ans = self._ask(f"stop {self.stop_id} (step {step}, pose {k}) failed {attempt} times: {err}",
                                ["retry", "skip", "abort"], f"Stop {self.stop_id} failed. Retry, skip or abort.")
                if ans == "retry":
                    attempt = 0
                    continue
                if ans == "skip":
                    self.stats["skipped"] += 1
                    self.writer.event("stop_skipped", stop=self.stop_id, step=step, pose=k)
                    self.stop_id += 1
                    return None
                raise Abort()

    def _settle_after_move(self):
        remaining = float(self.plan.settle_after_move_s) - (time.time() - self._moved_t)
        if remaining > 0:
            self.log(f"[stop {self.stop_id}] settling {remaining:.1f}s after the last movement "
                     f"(settle_after_move_s {self.plan.settle_after_move_s:g})")
            time.sleep(remaining)
            self.writer.event("settled", stop=self.stop_id, waited_s=round(remaining, 2))

    def _thermal_wait(self):
        """plan thermal_wait_s: before every stack ask the phone for its thermal state; if it is
        serious/critical, idle (poll every 5 s, up to thermal_wait_s) until it is fair/nominal again.
        Idle cooling is cheaper than the app's in-stack throttle (3 s per frame = 200 s stacks)."""
        max_s = float(self.plan.thermal_wait_s)
        if max_s <= 0:
            return
        try:
            st = self.phone.status()
            state = str(st.get("thermal", self._thermal_last))
        except Exception:                                # noqa: BLE001
            return
        # one status per stop -> thermal + battery in the log (iOS suspends charging when hot: the battery
        # fell 90 -> 80 % in the 16-stack bench of 2026-09-30 although the cable was connected)
        self.writer.event("phone_status", stop=self.stop_id, thermal=state, battery=st.get("battery"),
                          charging=st.get("charging"), free_bytes=st.get("free_bytes"))
        before = state
        if state not in ("serious", "critical"):
            self._thermal_last = state
            return
        t0 = time.time()
        self.log(f"[stop {self.stop_id}] phone thermal {state} -> cooling before the stack (up to {max_s:.0f} s)...")
        while time.time() - t0 < max_s:
            time.sleep(5.0)
            try:
                state = str(self.phone.status().get("thermal", state))
            except Exception:                            # noqa: BLE001
                break
            if state not in ("serious", "critical"):
                break
        waited = time.time() - t0
        self.log(f"[stop {self.stop_id}] thermal now {state} after {waited:.0f} s")
        self.writer.event("thermal_wait", stop=self.stop_id, before=before, after=state, waited_s=round(waited, 1))
        self._thermal_last = state

    def _focus_from_table(self, k):
        """Stack centre for robot pose k from the focus table: the entry with the
        nearest popillia yaw (mocap now); falls back to the pose median / plan value."""
        entries = self.focus_table.get(k, [])
        if self.plan.focus_table_yaw is not None:
            yaw = float(self.plan.focus_table_yaw)               # fixed yaw from the plan (e.g. the old session's first rotation)
        else:
            last = self.mocap.latest("popillia")
            yaw = yaw_deg(last[2]) if last else None
        if not entries:
            centre = self.plan.init_focus(k)
            return centre, "plan", {"reason": "no table entries for this pose", "yaw": yaw}
        with_yaw = [e for e in entries if e[0] is not None]
        if yaw is None or not with_yaw:
            # no mocap: the old image at the same rotation step (old session order within the pose),
            # ASSUMING the insect starts at the same orientation as in the old session
            ordered = sorted(entries, key=lambda e: e[3])
            step = getattr(self, "_current_step", 0)
            if step < len(ordered):
                e = ordered[step]
                return float(e[1]), f"table-step:{e[2]}", {"reason": "no yaw -> same rotation step as the old session",
                                                             "image": e[2], "old_index": step}
            centre = float(sorted(e[1] for e in entries)[len(entries) // 2])
            return centre, "table-median", {"reason": "no yaw, step beyond the old session", "yaw": yaw}
        best = min(with_yaw, key=lambda e: abs(yaw_diff_deg(yaw, float(e[0]))))
        # re-shots at the same yaw (duplicates in the old dataset): take the latest one
        same = [e for e in with_yaw if abs(yaw_diff_deg(float(e[0]), float(best[0]))) < 1.0]
        best = max(same, key=lambda e: e[3])
        d = yaw_diff_deg(yaw, float(best[0]))
        info = {"image": best[2], "table_yaw": best[0], "yaw": round(yaw, 2), "yaw_diff_deg": round(d, 2),
                "candidates": len(same)}
        if abs(d) > self.plan.turntable_step_deg:
            self.log(f"WARNING: nearest focus-table entry for pose {k} is {d:.1f} deg away in yaw")
        return float(best[1]), f"table:{best[2]}", info

    def _capture(self, step, k, attempt):
        p = self.plan
        thr = self._thresholds()
        image = self.writer.stack_name(step, k)
        lens = p.lens_for_pose(k)
        if p.focus_mode == "pose" or len(lens) == 1:
            centre = p.init_focus(k)                             # the init focus from the plan
        else:
            centre = lens[len(lens) // 2]                        # range: no init focus, the middle frame is the reference
        source = p.focus_mode
        # SETTLE: a fixed minimum idle time after the last movement (the arm's move swings the popillia)
        self._settle_after_move()
        self._thermal_wait()                                         # plan thermal_wait_s (after a hot stack)
        # STILL (only with stillness_gate; otherwise the jitter is logged and nothing waits or fails)
        if p.stillness_gate:
            ok, j, waited = self.mocap.wait_still(thr, p.still_window_s, p.still_timeout_s)
        else:
            _, j = self.mocap.is_still(thr, p.still_window_s)
            ok, waited = True, 0.0
        self.writer.event("still", stop=self.stop_id, jitter=j, waited_s=round(waited, 2), ok=ok, gate=p.stillness_gate)
        self.log(f"[stop {self.stop_id} {image}] step {step} pose {k} attempt {attempt}: jitter {j}"
                 + (f" after {waited:.1f}s" if p.stillness_gate else " (gate off)"))
        if not ok:
            raise RuntimeError(f"rig not still after {p.still_timeout_s}s: {j}")
        # SHOOT
        if not self.mocap.enabled:
            ok = True
        if p.focus_mode == "auto" and p.frames_per_stop > 1:
            centre = self.phone.focus_auto()             # AF on the centre (the beetle), stack around it
            lens, source = p.lens_around(centre), "auto"
            self.writer.event("autofocus", stop=self.stop_id, lens_position=centre, stack=[lens[0], lens[-1]])
            self.log(f"[stop {self.stop_id}] autofocus {centre:.4f} -> stack {lens[0]}..{lens[-1]} ({len(lens)} frames)")
        elif p.focus_mode == "table":
            centre, source, info = self._focus_from_table(k)
            lens = p.lens_around(centre) if p.frames_per_stop > 1 else [centre]
            self.writer.event("focus_table", stop=self.stop_id, pose=k, lens_position=centre, **info)
            self.log(f"[stop {self.stop_id}] focus table: {source} -> {centre:.4f} (yaw diff {info.get('yaw_diff_deg')}) "
                     f"-> stack {lens[0]}..{lens[-1]} ({len(lens)} frames)")
        if p.pose_confirm:
            self.phone.focus(centre)                     # lens at the stack centre so the operator can judge the focus
            while True:
                ans = self._ask(f"\n>>> step {step} pose {k}: lens at {centre:.4f} ({source}). "
                                f"Enter = shoot the stack, or type a lens value 0..1 to re-centre",
                                ["enter", "lens", "skip", "abort"], f"Pose {k}: check the focus, then press Enter.")
                if ans == "abort":
                    raise Abort()
                if ans == "skip":
                    raise RuntimeError("skipped by the operator at the focus check")
                if isinstance(ans, float):
                    centre, source = ans, "operator"
                    lens = p.lens_around(centre) if p.frames_per_stop > 1 else [centre]
                    self.phone.focus(centre)
                    self.log(f"[stop {self.stop_id}] centre overridden -> {centre:.4f}, stack {lens[0]}..{lens[-1]}")
                    continue
                break
            self.writer.event("pose_confirmed", stop=self.stop_id, pose=k, centre=centre, source=source)
        applied = self.phone.focus(lens[0])
        t_shoot0 = time.time()
        self.progress.start(self.stop_id, k, len(lens))
        try:
            done = self.phone.stack(self.stop_id, k, step, lens,
                                    on_frame=lambda m: self.progress.frame(int(m.get("k", 0)) + 1, m.get("lens_position")))
        finally:
            self.progress.finish(ok=False)
        t_shoot1 = time.time()
        frames = done.get("frames", [])
        if len(frames) != len(lens):
            raise RuntimeError(f"phone returned {len(frames)} frames, expected {len(lens)}")
        # VERIFY stillness over the capture interval (phone clock -> mac clock)
        off = self.phone.clock_offset_s
        ct = [float(f["capture_time"]) - off for f in frames]
        t0, t1 = min(ct) - 0.2, max(ct) + 0.2
        if t1 < t_shoot0 - 5 or t0 > t_shoot1 + 5:
            self.log(f"WARNING: capture times {ct} far from the shoot window {t_shoot0:.2f}-{t_shoot1:.2f}; using the window")
            t0, t1 = t_shoot0, t_shoot1
        time.sleep(max(0.0, (t1 + 0.1) - time.time()))
        jitters = {b: self.mocap.jitter_mm(b, t0, t1)[0] for b in thr}
        if self.mocap.enabled and p.stillness_gate:
            for b, thr_mm in thr.items():
                if jitters[b] is None or jitters[b] > thr_mm:
                    raise RuntimeError(f"moved during capture: {jitters}")
        thermal = collections.Counter(str(f.get("thermal", "?")) for f in frames)
        order = ["nominal", "fair", "serious", "critical"]
        self._thermal_last = max(thermal, key=lambda s: order.index(s) if s in order else -1) if thermal else "nominal"
        self.writer.event("captured", stop=self.stop_id, step=step, pose=k, image=image, n=len(frames), jitter=jitters,
                          t0=round(t0, 3), t1=round(t1, 3), duration_s=round(t_shoot1 - t_shoot0, 3),
                          thermal=dict(thermal))
        # the frames are on the phone; the robot may leave now — pull + bookkeeping happen in _finalize
        return {"stop": self.stop_id, "step": step, "pose": k, "image": image, "frames": frames, "t0": t0, "t1": t1,
                "jitters": jitters, "centre": centre, "source": source, "applied": applied, "off": off}

    def _finalize(self, pend):
        """Pull the captured frames (while the arm moves), record the rows, hand the stop to the merge."""
        if pend is None:
            return
        p = self.plan
        stop, step, k, image, frames = pend["stop"], pend["step"], pend["pose"], pend["image"], pend["frames"]
        t0, t1, jitters, centre, source, applied, off = (pend[x] for x in ("t0", "t1", "jitters", "centre", "source", "applied", "off"))
        # PULL into images/<image>/stack/ — but only when the local buffer has room: at most
        # merge_max_pending stacks (x 50 frames) waiting for the merge (+ upload) on this disk; the
        # phone keeps the frames meanwhile. Applies when something deletes the local copies.
        max_pending = p.upload_max_pending or p.merge_max_pending
        if self.uploader and p.upload_delete_local and max_pending > 0:
            waited = self.uploader.wait_for_room(max_pending, extra_pending=lambda: self.merger.pending if self.merger else 0,
                                                 log=self.log)
            if waited > 0:
                self.writer.event("merge_buffer_wait", stop=stop, seconds=round(waited, 1))
        elif self.merger and p.merge_delete_raw and p.merge_max_pending > 0:
            waited = self.merger.wait_for_room(p.merge_max_pending, log=self.log)
            if waited > 0:
                self.writer.event("merge_buffer_wait", stop=stop, seconds=round(waited, 1))
        stack_dir = self.writer.stack_dir(image)
        names = []
        for f in frames:
            names.append(f["file"])
            names.append(os.path.splitext(f["file"])[0] + ".json")
        t_pull = time.time()
        for attempt in range(1, 4):
            try:
                pulled = self.puller.pull(self.writer.session_id, names, stack_dir, stack_dir)
                for path in pulled:
                    if not os.path.exists(path) or os.path.getsize(path) < 64:
                        raise RuntimeError(f"pulled file missing/too small: {path}")
                break
            except Exception as err:                       # noqa: BLE001
                self.log(f"[stop {stop}] pull attempt {attempt} failed: {err}")
                if attempt == 3:
                    self.writer.event("stop_pull_failed", stop=stop, step=step, pose=k, image=image, files=names, reason=str(err))
                    self.stats["skipped"] += 1
                    self.log(f"[stop {stop} {image}] NOT recorded — the frames stay on the phone (session "
                             f"{self.writer.session_id}); pull them later with tools/pull_frames.py, or --resume redoes the stop")
                    return
                time.sleep(2.0)
        self.log(f"[stop {stop} {image}] pulled {len(frames)} frames in {time.time() - t_pull:.1f}s (arm moving meanwhile)")
        # RECORD
        cam_p, cam_q = self.mocap.mean_pose("phone", t0, t1)
        pop_p, pop_q = self.mocap.mean_pose("popillia", t0, t1)
        if self.mocap.enabled:
            samples = {b: self.mocap.window(b, t0, t1) for b in ("phone", "popillia")}
            if any(samples.values()):
                self.writer.mocap_samples(image, samples)
        cam_cols = {}
        if self.T_body_cam is not None and cam_p is not None and pop_p is not None:
            T_ic = insect_from_cam(cam_p, cam_q, pop_p, pop_q, self.T_body_cam)
            cam_cols = {"cam_x": round(float(T_ic[0, 3]), 6), "cam_y": round(float(T_ic[1, 3]), 6),
                        "cam_z": round(float(T_ic[2, 3]), 6), "quat_xyzw": fmt_vec(mat_quat(T_ic[:3, :3]))}
        for f in frames:
            intr = f.get("intrinsics") or {}
            row = {
                "image": image, "file": f["file"], "stop": stop, "step": step, "pose": k,
                "focus_index": f.get("k", 0), "lens_position": f.get("lens_position", applied),
                "focus_centre": round(float(centre), 4), "focus_source": source,
                "fx": intr.get("fx"), "fy": intr.get("fy"), "cx": intr.get("cx"), "cy": intr.get("cy"),
                "capture_time": float(f["capture_time"]) - off, "mocap_t0": t0, "mocap_t1": t1,
                "phone_jitter_mm": jitters.get("phone"), "pop_jitter_mm": jitters.get("popillia"),
            }
            if cam_p is not None:
                row.update({"phone_x": round(float(cam_p[0]), 6), "phone_y": round(float(cam_p[1]), 6),
                            "phone_z": round(float(cam_p[2]), 6), "phone_quat_xyzw": fmt_vec(cam_q),
                            "phone_world": fmt_vec(cam_p, 4)})
            if pop_p is not None:
                row.update({"pop_x": round(float(pop_p[0]), 6), "pop_y": round(float(pop_p[1]), 6),
                            "pop_z": round(float(pop_p[2]), 6), "pop_quat_xyzw": fmt_vec(pop_q),
                            "popillia_yaw_deg": round(yaw_deg(pop_q), 3)})
            row.update(cam_cols)
            self.writer.frame(row)
        self.writer.event("stop_ok", stop=stop, step=step, pose=k, image=image, files=names, jitter=jitters)
        self.stats["stops_ok"] += 1
        self.progress.done_stacks += 1
        self._stop_files[stop] = (image, list(names))
        if self.merger:
            if self.uploader and p.upload_what in ("raw", "all"):
                self.uploader.submit(stop, image, names, phase="raw")     # raw up NOW, in parallel with the merge
            self.merger.submit(stop)                         # -> _on_merge_done -> final upload (master + check)
        elif self.uploader:
            self.uploader.submit(stop, image, names, merged_ok=None)
        self._write_summary()
        self._collect_poses()


# ---------------------------------------------------------------------------
def repo_commit():
    """Short git commit of this coordinator (for the session summary), "" if unknown."""
    try:
        root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        out = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=root, capture_output=True, text=True, timeout=5)
        dirty = subprocess.run(["git", "status", "--porcelain", "--untracked-files=no"], cwd=root,
                               capture_output=True, text=True, timeout=5).stdout.strip()
        return out.stdout.strip() + ("+dirty" if dirty else "") if out.returncode == 0 else ""
    except Exception:
        return ""


def wait_for_mocap(mocap, bodies=("phone", "popillia"), timeout=30.0, log=print, measure_s=2.0):
    """Start-up check: every body must deliver samples. Prints one line per
    body (topic, rate) and returns the list of bodies NOT received."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        missing = [b for b in bodies if mocap.latest(b) is None]
        if not missing:
            break
        time.sleep(0.5)
    c0 = dict(mocap.counts)
    time.sleep(measure_s)
    missing = []
    for b in bodies:
        n = mocap.counts.get(b, 0) - c0.get(b, 0)
        last = mocap.latest(b)
        if last is None or n == 0:
            log(f"[mocap] /optitrack/{b}: NOT RECEIVING (subscribed, no samples)")
            missing.append(b)
        else:
            age = time.time() - last[0]
            log(f"[mocap] /optitrack/{b}: receiving, {n / measure_s:.0f} Hz, last sample {age:.2f}s ago, "
                f"pos {fmt_vec(last[1], 3)}")
    return missing


def _free_bytes(path):
    """Free bytes of the file system that will hold `path` (the nearest existing parent)."""
    p = os.path.abspath(os.path.expanduser(path))
    while p and not os.path.exists(p):
        parent = os.path.dirname(p)
        if parent == p:
            break
        p = parent
    try:
        st = os.statvfs(p)
        return st.f_bavail * st.f_frsize
    except OSError:
        return None


def describe_precision(plan):
    pr = plan.turntable_precision()
    if pr["exact"]:
        return f"exact: {plan.turntable_step_deg / pr['deg_per_microstep']:.0f} microsteps per step"
    return (f"{pr['deg_per_microstep']:.4f} deg per microstep, worst rounding {pr['max_error_deg']:.4f} deg "
            f"at any stop, never accumulating")


def ask_rotations(plan):
    """How many insect rotations per revolution: the step becomes exactly 360/N deg. With the
    motor the microstep resolution and the worst rounding error are printed (the board rounds
    every ABSOLUTE target, so the error never accumulates); N values that are an integer number
    of microsteps are listed."""
    motor = plan.turntable_drive != "manual"
    while True:
        ans = input(f"\n>>> Number of insect rotations per revolution (1 = no rotation, 36 = every 10 deg) "
                    f"[{plan.turntable_steps}]: ").strip()
        if not ans:
            plan.set_turntable_steps(plan.turntable_steps)
            break
        try:
            plan.set_turntable_steps(int(ans))
            break
        except ValueError as err:
            print(f"   {err}")
    if not motor:
        print(f"   manual platter: you turn the insect by {plan.turntable_step_deg:g} deg per step and press Enter "
              f"(the motor is planned, not built: turntable_drive serial)")
    if motor:
        pr = plan.turntable_precision()
        mspr = int(round(pr["microsteps_per_rev"]))
        exact = [n for n in range(1, 361) if mspr % n == 0]
        print(f"   motor: {mspr} microsteps per platter revolution -> {describe_precision(plan)}")
        if not pr["exact"]:
            print(f"   exact step counts with this motor/microstepping: {', '.join(str(n) for n in exact if n <= 100)}")


def ask_session_dir(plan, out_root, default, resume=False, ask=True):
    """Where the session data goes. Shows the size the plan needs and the free space."""
    default = default or os.path.join(out_root, f"{plan.name}_" + time.strftime("%Y%m%d"))
    raw_b, master_b = plan.storage_estimate()
    keep = []
    deleting = (plan.upload_remote and plan.upload_delete_local) or (plan.merge_inline and plan.merge_delete_raw)
    if plan.merge_inline and plan.merge_delete_raw and plan.upload_what == "masters" or (plan.merge_inline and plan.merge_delete_raw and not plan.upload_remote):
        keep.append(f"raw only until merged (<= {plan.merge_max_pending} stops on disk)")
    elif plan.upload_remote and plan.upload_delete_local:
        keep.append(f"raw + masters only until uploaded (<= {plan.upload_max_pending or plan.merge_max_pending} stops on disk)")
    else:
        keep.append("everything stays on this disk")
    print(f"\nsession size: {plan.stops_planned()} stops -> ~{raw_b / 1e9:.0f} GB raw"
          + (f" + ~{master_b / 1e9:.0f} GB masters" if master_b else "") + f"; {'; '.join(keep)}")
    while True:
        if ask:
            ans = input(f">>> Session folder (the dataset itself) [{default}]: ").strip()
            path = os.path.expandvars(os.path.expanduser(ans)) if ans else default
        else:
            path = default
        path = os.path.abspath(path)
        holds = os.path.exists(os.path.join(path, "session.jsonl")) or os.path.exists(os.path.join(path, "frames.csv"))
        if holds and not resume:
            print(f"   {path} already holds a session: start with --resume to continue it, or choose another folder")
            if not ask:
                sys.exit(1)
            continue
        if resume and not holds:
            print(f"   --resume but {path} holds no session yet (a new one starts there)")
        free = _free_bytes(path)
        need = (0 if not deleting else 0) if deleting else raw_b + master_b
        if free is not None:
            print(f"   free space there: {free / 1e9:.0f} GB" + (f" (needs ~{need / 1e9:.0f} GB without deletion)" if need else ""))
            if need and free < need and ask:
                if input("   not enough space for the whole session. Continue anyway? [y/N] ").strip().lower() not in ("y", "yes"):
                    continue
        return path


ONEDRIVE_DATASETS = "onedrive:SUPSI"        # every dataset lives under OneDrive-USI/SUPSI (user, 2026-09-29)


def ask_remote(plan):
    default = plan.upload_remote or "-"
    ans = input(f">>> Upload to OneDrive during the session (folder name under {ONEDRIVE_DATASETS}/, "
                f"or a full rclone remote:path, '-' = none) [{default}]: ").strip()
    if not ans:
        ans = default
    if ans == "-":
        return ""
    if ":" not in ans:                                  # bare name -> under SUPSI
        ans = f"{ONEDRIVE_DATASETS}/{ans.strip('/')}"
    return ans


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--plan", required=True)
    ap.add_argument("--mocap", choices=["ros2", "synthetic", "none"], default="ros2",
                    help="none = no OptiTrack: no stillness gate / poses (robot + phone communication tests)")
    ap.add_argument("--puller", choices=["usb", "local"], default="usb")
    ap.add_argument("--fake-phone-dir", help="with --puller local: the fake phone's Documents dir")
    ap.add_argument("--tunnel", action="store_true", help="start `pymobiledevice3 usbmux forward` for the phone port")
    ap.add_argument("--auto-operator", choices=["retry", "skip", "abort", "accept"],
                    help="answer every operator prompt automatically (tests)")
    ap.add_argument("--output-root", help="override plan.output_root")
    ap.add_argument("--session-dir", help="override plan.session_dir (the dataset folder itself); also skips the folder question")
    ap.add_argument("--remote", help="override plan.upload_remote (rclone remote:path; '-' = no upload); skips that question")
    ap.add_argument("--rotations", type=int, help="override plan.turntable_steps (step = 360/N deg); skips that question")
    ap.add_argument("--no-ask", action="store_true", help="take the plan's folder / remote / rotations without asking")
    ap.add_argument("--resume", action="store_true", help="continue an existing session folder (captured stops are skipped)")
    ap.add_argument("--start-step", type=int, default=0, help="first turntable step to run (resume)")
    ap.add_argument("--stack-start", type=float, help="override plan.focus_stack_start")
    ap.add_argument("--stack-end", type=float, help="override plan.focus_stack_end")
    ap.add_argument("--stack-frames", type=int, help="override plan.frames_per_stop (1 = single frame)")
    a = ap.parse_args(argv)

    plan = Plan.load(a.plan)
    if a.stack_start is not None: plan.focus_stack_start = a.stack_start
    if a.stack_end is not None: plan.focus_stack_end = a.stack_end
    if a.stack_frames is not None: plan.frames_per_stop = a.stack_frames
    print(f"focus stack: {plan.frames_per_stop} frame(s) per stop, lens {plan.focus_stack_start}→{plan.focus_stack_end}")
    out_root = a.output_root or plan.output_root
    session_dir = a.session_dir or plan.session_dir or None
    ask = not (a.no_ask or a.auto_operator) and sys.stdin.isatty()
    # -- start questions: rotations, session folder, OneDrive destination (Enter = the plan's value)
    if a.rotations is not None:
        plan.set_turntable_steps(a.rotations)
    elif ask and not a.resume:
        ask_rotations(plan)
    print(f"insect rotations: {plan.turntable_steps} x {plan.turntable_step_deg:g} deg"
          + (f" ({describe_precision(plan)})" if plan.turntable_drive != "manual" else "")
          + f"; {plan.stops_planned()} stops x {plan.frames_per_stop} frames")
    if session_dir is None or (ask and not a.session_dir):
        session_dir = ask_session_dir(plan, out_root, session_dir, resume=a.resume, ask=ask)
    if a.remote is not None:
        plan.upload_remote = "" if a.remote.strip() in ("-", "") else a.remote.strip()
    elif ask:
        plan.upload_remote = ask_remote(plan)
    plan.validate_upload()
    uploader_check = None
    if plan.upload_remote:
        uploader_check = UploadWorker(session_dir or out_root, plan.upload_remote, what=plan.upload_what, rclone=plan.upload_rclone)
        try:
            uploader_check.check_remote()
        except RuntimeError as err:
            uploader_check.close()
            sys.exit(f"OneDrive upload: {err}")
        free, total, _ = uploader_check.about()
        uploader_check.close()
        print(f"upload: {plan.upload_what} -> {plan.upload_remote} (reachable"
              + (f", {free / 1e9:.0f} GB free of {total / 1e9:.0f}" if free is not None and total else "") + ")")
    else:
        print("upload: none (everything stays in the session folder" + ("" if plan.merge_delete_raw else " and on the phone") + ")")
    writer = SessionWriter(out_root, name=plan.name, session_dir=session_dir, resume=a.resume)
    print(f"session folder: {writer.dir}" + (" (resumed)" if writer.resumed else ""))
    writer.save_plan(plan.to_dict(), a.plan)
    for src in plan.calibration_files:
        dst = writer.copy_in(src, "calibrations")
        print(f"  calibration: {src} -> {dst}" if dst else f"  WARNING calibration file missing: {src}")
    for src in plan.robot_files:
        dst = writer.copy_in(src, "robot")
        print(f"  robot file: {src} -> {dst}" if dst else f"  WARNING robot file missing: {src}")

    launcher = Launcher(plan, writer.dir)
    if a.tunnel:
        plan.start_tunnel = True
    launcher.start()
    operator_for_prompts = (lambda q, choices: (print(f"{q} -> auto '{a.auto_operator}'"),
                                                a.auto_operator if a.auto_operator in choices else choices[0])[1]) \
        if a.auto_operator else None

    if a.mocap == "ros2":
        from .mocap import Ros2MocapSource
        mocap = Ros2MocapSource()
        print("[mocap] subscribed to /optitrack/phone and /optitrack/popillia — checking that samples arrive...")
        while True:
            missing = wait_for_mocap(mocap, ("phone", "popillia"))
            if not missing:
                break
            ans = (operator_for_prompts or Coordinator._ask_operator)(
                f">>> mocap NOT receiving {missing}: Motive streaming? bodies tracked (markers visible)? "
                f"driver running (record_mocap)? Fix it, then", ["retry", "abort"])
            if ans == "abort":
                launcher.stop(); mocap.close()
                sys.exit("aborted: no mocap samples for " + ", ".join(missing))
    elif a.mocap == "none":
        from .mocap import NoMocapSource
        mocap = NoMocapSource()
        print("NO MOCAP: stillness gate off, no poses recorded, turntable by Enter")
    else:
        from .mocap import SyntheticMocapSource
        mocap = SyntheticMocapSource()

    phone = PhoneClient(plan.phone_host, plan.phone_port)
    phone.on_reconnect = launcher.restart_tunnel           # USB pulled and replugged: new forwarder, then reconnect + hello
    if a.puller == "usb":
        from .pull import UsbPuller
        puller = UsbPuller(plan.phone_bundle_id)
    else:
        from .pull import LocalDirPuller
        puller = LocalDirPuller(a.fake_phone_dir)

    console = None
    if a.auto_operator:
        operator = lambda q, choices: (print(f"{q} -> auto '{a.auto_operator}'"), a.auto_operator if a.auto_operator in choices else choices[0])[1]
    else:
        console = Console()
        operator = console.ask if console.interactive else None
    on_turn = None
    if a.mocap == "synthetic":
        on_turn = lambda step, target: mocap.set_target_yaw(target)   # the "operator" turns the platter

    from .turntable import make_turntable
    turntable = make_turntable(plan)
    if turntable is not None:
        print(f"turntable: {plan.turntable_drive} ({plan.turntable_port}), {plan.turntable_step_deg:g} deg per step, "
              f"{plan.turntable_steps_per_rev * plan.turntable_microsteps * plan.turntable_gear_ratio:.0f} microsteps/rev")
    print(f"loop order: {plan.loop_order} = {plan.loop_order_text()}")
    launcher.start_robot(1 if plan.loop_order == "rotations-per-pose" else plan.turntable_steps - a.start_step)
    coord = Coordinator(plan, phone, lambda: RobotClient(plan.robot_host, plan.robot_port),
                        mocap, puller, writer, operator=operator, on_turntable_prompt=on_turn, start_step=a.start_step,
                        turntable=turntable)
    if console is not None:
        coord.console = console
        console.status_fn = coord.status_line

    def on_sigint(signum, frame):
        # first Ctrl-C = pause at the next checkpoint; the second one aborts (KeyboardInterrupt)
        coord.request_pause()
        signal.signal(signal.SIGINT, signal.default_int_handler)
        print("\n>>> Ctrl-C: pausing at the next checkpoint (after the running stack). Ctrl-C again = abort.")
    signal.signal(signal.SIGINT, on_sigint)
    print("pause at any time: type p + Enter, or Ctrl-C once, or `touch " + pause_file(writer.dir) + "`")
    stats = {}
    try:
        stats = coord.run()
    finally:
        if plan.stop_confirm and launcher.procs:
            coord.notifier.start("Session finished. Press Enter to stop the recording.")
            try:
                ask = operator or Coordinator._ask_operator
                ask(f"\n>>> Session finished ({launcher.status()}). Press Enter to stop the mocap recording / tunnel", ["enter"])
            finally:
                coord.notifier.stop()
        launcher.stop()
        mocap.close()
        if plan.upload_remote:
            # the bag recorder is stopped now: the final sync sends mocap/ (-> poses/mocap_bag/) and the last summary
            final = UploadWorker(writer.dir, plan.upload_remote, what=plan.upload_what, rclone=plan.upload_rclone,
                                 transfers=plan.upload_transfers, bwlimit=plan.upload_bwlimit, timeout_s=plan.upload_timeout_s)
            try:
                ok = final.upload_session_files(final=True, check=plan.upload_delete_session)
                print(f"final sync (mocap bag, session files) -> {plan.upload_remote}: {'ok' if ok else 'FAILED'}"
                      + (" (verified)" if ok and plan.upload_delete_session else ""))
            finally:
                final.close()
            if plan.upload_delete_session:
                delete_session_folder(writer.dir, stats, final_ok=ok)


def delete_session_folder(session_dir, stats, final_ok):
    """plan upload_delete_session: the whole session folder goes when (1) the final sync was verified,
    (2) every captured stop was uploaded and none failed, (3) nothing is left under images/.
    Otherwise the folder stays and the reason is printed (user, 2026-09-29: once everything is in
    OneDrive the local copy is not needed)."""
    import shutil
    left = sum(len(fs) for _r, _d, fs in os.walk(os.path.join(session_dir, "images")))
    why = []
    if not final_ok:
        why.append("final sync not verified")
    if stats.get("uploaded_failed"):
        why.append(f"{stats['uploaded_failed']} stop upload(s) failed")
    if stats.get("stops_ok", 0) != stats.get("uploaded_ok", 0):
        why.append(f"{stats.get('stops_ok', 0)} stops captured but {stats.get('uploaded_ok', 0)} uploaded")
    if left:
        why.append(f"{left} file(s) still under images/")
    if why:
        print(f"session folder KEPT ({'; '.join(why)}): {session_dir}")
        return False
    try:
        shutil.rmtree(session_dir)
    except OSError as e:
        print(f"session folder could not be deleted: {e}")
        return False
    print(f"session folder deleted (everything is in the cloud): {session_dir}")
    return True


if __name__ == "__main__":
    main()
