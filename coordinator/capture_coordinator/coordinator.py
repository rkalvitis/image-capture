"""The session state machine (docs/DEVELOPMENT_PLAN.md §6).

for turntable step s:                       (outer loop, manual platter)
  robot run starts -> reached(0)            initial pose = turntable moment
  TURN    wait until popillia yaw == target ± tol and still      -> go(0)
  for pose k in 1..N:
    MOVE   wait reached(k)
    STILL  phone AND popillia jitter < threshold over window (timeout -> ask)
    SHOOT  focus(lens[k]); stack(stop)
    VERIFY still over the capture interval; files pulled; count ok
           fail -> retry (max_retries) -> ask operator: retry / skip / abort
    go(k)
  robot returns -> done

Run it with the optitrack pixi env python for the ROS 2 mocap source:
  pixi run --manifest-path ~/Documents/repos/optitrack/pixi.toml \
      python -m capture_coordinator.coordinator --plan plan.yaml
"""
import argparse
import json
import os
import subprocess
import sys
import time

from .mocap import yaw_deg, yaw_diff_deg
from .phone import PhoneClient
from .plan import Plan
from .robot import RobotClient
from .session import SessionWriter


class Abort(Exception):
    pass


class Coordinator:
    def __init__(self, plan, phone, robot_factory, mocap, puller, writer,
                 operator=None, on_turntable_prompt=None, log=print):
        self.plan, self.phone, self.robot_factory = plan, phone, robot_factory
        self.mocap, self.puller, self.writer = mocap, puller, writer
        self.operator = operator or self._ask_operator
        self.on_turntable_prompt = on_turntable_prompt
        self.log = log
        self.stop_id = 0
        self.yaw0 = None
        self.stats = {"stops_ok": 0, "retries": 0, "skipped": 0}

    # -- operator ------------------------------------------------------
    @staticmethod
    def _ask_operator(question, choices):
        while True:
            ans = input(f"{question} [{'/'.join(choices)}] ").strip().lower()
            for c in choices:
                if ans == c or (ans and c.startswith(ans)):
                    return c

    def _thresholds(self):
        return {"phone": self.plan.still_phone_mm, "popillia": self.plan.still_popillia_mm}

    # -- session -------------------------------------------------------
    def run(self):
        p = self.plan
        self.writer.save_plan(p.to_dict())
        self.writer.event("session_start", session=self.writer.session_id, plan=p.name)
        self.phone.connect()
        ready = self.phone.hello(session=self.writer.session_id,
                                 lens_positions=p.lens_for_pose(1), settle_ms=p.settle_ms,
                                 exposure={"mode": p.exposure_mode, "duration_s": p.exposure_duration_s, "iso": p.exposure_iso},
                                 wb=p.wb)
        offset = self.phone.ping()
        self.log(f"[phone] ready: {ready.get('device')} {ready.get('lens')} {ready.get('format')}; "
                 f"clock offset {offset*1000:.1f} ms; calib {ready.get('calib_available')}")
        self.writer.event("phone_ready", reply=ready, clock_offset_s=offset)
        try:
            for step in range(p.turntable_steps):
                self._turntable_step(step)
        except Abort:
            self.log("session aborted by the operator")
            self.writer.event("session_abort", stats=self.stats)
        else:
            self.writer.event("session_end", stats=self.stats)
        finally:
            self.phone.close()
            self.writer.close()
        self.log(f"session {self.writer.session_id} finished: {self.stats}")
        return self.stats

    # -- one turntable step = one robot run ----------------------------
    def _turntable_step(self, step):
        p = self.plan
        robot = self.robot_factory()
        self.log(f"[step {step}] waiting for the robot run to start (port {p.robot_port})...")
        robot.connect()
        if robot.poses and robot.poses != p.poses:
            self.log(f"WARNING: robot has {robot.poses} poses, plan says {p.poses} — using the robot's count")
        n_poses = robot.poses or p.poses
        msg = robot.wait_reached()
        if msg.get("type") != "reached" or int(msg.get("id", -1)) != 0:
            raise RuntimeError(f"expected reached 0 from the robot, got {msg}")
        self.writer.event("robot_initial", step=step)
        self._wait_turntable(step)
        robot.go(0)
        try:
            for k in range(1, n_poses + 1):
                msg = robot.wait_reached()
                if msg.get("type") == "done":
                    raise RuntimeError("robot finished the run early")
                if int(msg["id"]) != k:
                    self.log(f"WARNING: robot reached {msg['id']}, expected {k}")
                    k = int(msg["id"])
                self.writer.event("robot_reached", step=step, pose=k, joints=msg.get("joints"))
                self._stop(step, k)
                robot.go(k)
            robot.wait_done()
            self.writer.event("robot_done", step=step)
        except Abort:
            robot.abort()
            try:
                robot.wait_done(timeout=120)
            except Exception:
                pass
            raise
        finally:
            robot.close()

    def _wait_turntable(self, step):
        p = self.plan
        thr = self._thresholds()
        if self.yaw0 is None:
            ok, j, _ = self.mocap.wait_still(thr, p.still_window_s, p.still_timeout_s)
            last = self.mocap.latest("popillia")
            self.yaw0 = yaw_deg(last[2]) if last else 0.0
            target = self.yaw0
            self.log(f"[step 0] reference popillia yaw = {self.yaw0:.2f} deg")
        else:
            target = self.yaw0 + step * p.turntable_step_deg
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
                if yaw is not None and abs(yaw_diff_deg(yaw, target)) <= p.turntable_yaw_tol_deg and still:
                    self.log(f"[step {step}] turntable at {yaw:.2f} deg (target {target:.1f}), still {j}")
                    self.writer.event("turntable", step=step, target_yaw=target, yaw=yaw, jitter=j)
                    return
                if time.time() - t0 > max(60.0, p.still_timeout_s * 4):
                    break
                time.sleep(0.1)
            ans = self.operator(f"turntable not at step {step} (yaw {yaw}, target {target:.1f}, still {still})",
                                ["retry", "accept", "abort"])
            if ans == "accept":
                self.writer.event("turntable", step=step, target_yaw=target, yaw=yaw, jitter=j, accepted=True)
                return
            if ans == "abort":
                raise Abort()

    # -- one stop ------------------------------------------------------
    def _stop(self, step, k):
        p = self.plan
        attempt = 0
        while True:
            attempt += 1
            try:
                self._capture(step, k, attempt)
                self.stats["stops_ok"] += 1
                return
            except Abort:
                raise
            except Exception as err:
                self.log(f"[stop {self.stop_id}] attempt {attempt} failed: {err}")
                self.writer.event("stop_failed", stop=self.stop_id, step=step, pose=k, attempt=attempt, reason=str(err))
                if attempt <= p.max_retries:
                    self.stats["retries"] += 1
                    continue
                ans = self.operator(f"stop {self.stop_id} (step {step}, pose {k}) failed {attempt} times: {err}",
                                    ["retry", "skip", "abort"])
                if ans == "retry":
                    attempt = 0
                    continue
                if ans == "skip":
                    self.stats["skipped"] += 1
                    self.writer.event("stop_skipped", stop=self.stop_id, step=step, pose=k)
                    self.stop_id += 1
                    return
                raise Abort()

    def _capture(self, step, k, attempt):
        p = self.plan
        thr = self._thresholds()
        lens = p.lens_for_pose(k)
        # STILL
        ok, j, waited = self.mocap.wait_still(thr, p.still_window_s, p.still_timeout_s)
        self.writer.event("still", stop=self.stop_id, jitter=j, waited_s=round(waited, 2), ok=ok)
        self.log(f"[stop {self.stop_id}] step {step} pose {k} attempt {attempt}: jitter {j} after {waited:.1f}s")
        if not ok:
            raise RuntimeError(f"rig not still after {p.still_timeout_s}s: {j}")
        # SHOOT
        applied = self.phone.focus(lens[0])
        t_shoot0 = time.time()
        done = self.phone.stack(self.stop_id, k, step, lens)
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
        for b, thr_mm in thr.items():
            if jitters[b] is None or jitters[b] > thr_mm:
                raise RuntimeError(f"moved during capture: {jitters}")
        # PULL
        names = []
        for f in frames:
            names.append(f["file"])
            names.append(os.path.splitext(f["file"])[0] + ".json")
        pulled = self.puller.pull(self.writer.session_id, names, self.writer.images_dir, self.writer.sidecar_dir)
        for path in pulled:
            if not os.path.exists(path) or os.path.getsize(path) < 64:
                raise RuntimeError(f"pulled file missing/too small: {path}")
        # RECORD
        cam_p, cam_q = self.mocap.mean_pose("phone", t0, t1)
        pop_p, pop_q = self.mocap.mean_pose("popillia", t0, t1)
        for f in frames:
            intr = f.get("intrinsics") or {}
            self.writer.frame({
                "file": f["file"], "stop": self.stop_id, "step": step, "pose": k,
                "focus_index": f.get("k", 0), "lens_position": f.get("lens_position", applied),
                "fx": intr.get("fx"), "fy": intr.get("fy"), "cx": intr.get("cx"), "cy": intr.get("cy"),
                "capture_time": float(f["capture_time"]) - off, "mocap_t0": t0, "mocap_t1": t1,
                "phone_jitter_mm": jitters.get("phone"), "pop_jitter_mm": jitters.get("popillia"),
                "cam_x": cam_p[0], "cam_y": cam_p[1], "cam_z": cam_p[2],
                "quat_xyzw": " ".join(f"{v:.6f}" for v in cam_q),
                "popillia_yaw_deg": round(yaw_deg(pop_q), 3),
                "pop_x": pop_p[0], "pop_y": pop_p[1], "pop_z": pop_p[2],
                "pop_quat_xyzw": " ".join(f"{v:.6f}" for v in pop_q),
                "phone_world": " ".join(f"{v:.4f}" for v in cam_p),
            })
        self.writer.event("stop_ok", stop=self.stop_id, step=step, pose=k, files=names, jitter=jitters)
        self.stop_id += 1


# ---------------------------------------------------------------------------
def start_usb_tunnel(local_port, device_port, log=print):
    cmd = ["pymobiledevice3", "usbmux", "forward", str(local_port), str(device_port)]
    log("starting USB tunnel: " + " ".join(cmd))
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)
    if proc.poll() is not None:
        raise RuntimeError("pymobiledevice3 usbmux forward exited immediately (installed? phone plugged in and trusted?)")
    return proc


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--plan", required=True)
    ap.add_argument("--mocap", choices=["ros2", "synthetic"], default="ros2")
    ap.add_argument("--puller", choices=["usb", "local"], default="usb")
    ap.add_argument("--fake-phone-dir", help="with --puller local: the fake phone's Documents dir")
    ap.add_argument("--tunnel", action="store_true", help="start `pymobiledevice3 usbmux forward` for the phone port")
    ap.add_argument("--auto-operator", choices=["retry", "skip", "abort", "accept"],
                    help="answer every operator prompt automatically (tests)")
    ap.add_argument("--output-root", help="override plan.output_root")
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
    writer = SessionWriter(out_root, name=plan.name)
    print(f"session folder: {writer.dir}")

    if a.mocap == "ros2":
        from .mocap import Ros2MocapSource
        mocap = Ros2MocapSource()
    else:
        from .mocap import SyntheticMocapSource
        mocap = SyntheticMocapSource()

    tunnel = start_usb_tunnel(plan.phone_port, plan.phone_port) if a.tunnel else None
    phone = PhoneClient(plan.phone_host, plan.phone_port)
    if a.puller == "usb":
        from .pull import UsbPuller
        puller = UsbPuller(plan.phone_bundle_id)
    else:
        from .pull import LocalDirPuller
        puller = LocalDirPuller(a.fake_phone_dir)

    operator = (lambda q, choices: (print(f"{q} -> auto '{a.auto_operator}'"), a.auto_operator if a.auto_operator in choices else choices[0])[1]) if a.auto_operator else None
    on_turn = None
    if a.mocap == "synthetic":
        on_turn = lambda step, target: mocap.set_target_yaw(target)   # the "operator" turns the platter

    coord = Coordinator(plan, phone, lambda: RobotClient(plan.robot_host, plan.robot_port),
                        mocap, puller, writer, operator=operator, on_turntable_prompt=on_turn)
    try:
        coord.run()
    finally:
        mocap.close()
        if tunnel:
            tunnel.terminate()


if __name__ == "__main__":
    main()
