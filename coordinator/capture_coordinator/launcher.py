"""Start / stop everything around the coordinator (plan keys start_tunnel,
launch_app, record_mocap, dump_bodies, robot_start_cmd) so that ONE command
runs the whole session:

    bash tools/run_coordinator.sh --plan coordinator/plan.sep15.yaml

- USB tunnel: `pymobiledevice3 usbmux forward <port> <port>`
- app: `xcrun devicectl device process launch --device <id> --terminate-existing <bundle>`
- mocap: `pixi run record-poses` in the optitrack repo (driver + rosbag),
  RECORD_DIR=<session>/mocap, RECORD_BODIES="phone popillia"; stopped with
  SIGINT at the end so the bag closes cleanly
- rigid bodies: `pixi run dump-bodies -- --out <session>/calibrations/rigid_bodies_<date>.yaml`
- robot: an optional shell command ({cycles} = remaining turntable steps),
  e.g. an ssh line that runs run_cycles.sh on the robot PC (needs an ssh key)
"""
import datetime
import os
import shutil
import signal
import subprocess
import threading
import time


class Notifier:
    """macOS attention call while the coordinator waits for the operator:
    a sound (afplay), a spoken message (say) and a notification banner
    (osascript), repeated every `repeat_s` seconds until stop()."""
    SOUND = "/System/Library/Sounds/Glass.aiff"

    def __init__(self, enabled=True, repeat_s=45.0, voice="", log=print):
        self.enabled = enabled and shutil.which("say") is not None
        self.repeat_s, self.voice, self.log = float(repeat_s), voice, log
        self._stop = threading.Event()
        self._thr = None

    def _once(self, text, banner):
        try:
            if os.path.exists(self.SOUND):
                subprocess.run(["afplay", self.SOUND], timeout=5, capture_output=True)
            if banner:
                subprocess.run(["osascript", "-e", 'display notification "%s" with title "Capture coordinator"'
                                % text.replace('"', "'")], timeout=5, capture_output=True)
            cmd = ["say"] + (["-v", self.voice] if self.voice else []) + [text]
            subprocess.run(cmd, timeout=30, capture_output=True)
        except (OSError, subprocess.TimeoutExpired):
            pass

    def start(self, text):
        """Fire now and repeat in the background until stop()."""
        if not self.enabled:
            return
        self.stop()
        self._stop = threading.Event()
        stop = self._stop

        def loop():
            first = True
            while not stop.is_set():
                self._once(text, banner=first)
                first = False
                if self.repeat_s <= 0 or stop.wait(self.repeat_s):
                    return
        self._thr = threading.Thread(target=loop, daemon=True)
        self._thr.start()

    def stop(self):
        if self._thr is not None:
            self._stop.set()
            self._thr = None


class Launcher:
    def __init__(self, plan, session_dir, log=print):
        self.plan, self.session_dir, self.log = plan, session_dir, log
        self.procs = {}
        self.optitrack = os.path.expanduser(plan.optitrack_repo)

    def _pixi(self, *args):
        return ["pixi", "run", "--manifest-path", os.path.join(self.optitrack, "pixi.toml"), *args]

    def start_tunnel(self):
        cmd = ["pymobiledevice3", "usbmux", "forward", str(self.plan.phone_port), str(self.plan.phone_port)]
        self.log("[launcher] USB tunnel: " + " ".join(cmd))
        p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1.0)
        if p.poll() is not None:
            raise RuntimeError("pymobiledevice3 usbmux forward exited immediately (installed? phone plugged in and trusted?)")
        self.procs["tunnel"] = p
        return p

    def restart_tunnel(self):
        """After a USB unplug/replug: kill the forwarder and start a new one (the phone's on_reconnect hook)."""
        if not self.plan.start_tunnel:
            return
        p = self.procs.pop("tunnel", None)
        if p is not None and p.poll() is None:
            try:
                p.terminate(); p.wait(timeout=5)
            except (subprocess.TimeoutExpired, OSError):
                p.kill()
        try:
            self.start_tunnel()
        except RuntimeError as err:
            self.log(f"[launcher] tunnel restart: {err}")

    def launch_app(self):
        if not self.plan.phone_device_id:
            self.log("[launcher] launch_app: no phone_device_id in the plan — skipped")
            return
        cmd = ["xcrun", "devicectl", "device", "process", "launch", "--device", self.plan.phone_device_id,
               "--terminate-existing", self.plan.phone_bundle_id]
        self.log("[launcher] app: " + " ".join(cmd))
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
        if res.returncode != 0:
            self.log(f"[launcher] app launch failed: {(res.stderr or res.stdout).strip().splitlines()[-1:]}")
        else:
            time.sleep(2.0)

    def record_mocap(self):
        bag_dir = os.path.join(self.session_dir, "mocap", "bag_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
        env = dict(os.environ)
        env["RECORD_DIR"] = bag_dir
        env["RECORD_BODIES"] = self.plan.mocap_bodies
        cmd = self._pixi("record-poses")
        self.log(f"[launcher] mocap driver + bag -> {bag_dir}")
        p = subprocess.Popen(cmd, cwd=self.optitrack, env=env, stdout=open(os.path.join(self.session_dir, "mocap", "record.log"), "a"),
                             stderr=subprocess.STDOUT, start_new_session=True)
        self.procs["mocap"] = p
        return p

    def dump_bodies(self):
        out = os.path.join(self.session_dir, "calibrations",
                           "rigid_bodies_" + datetime.datetime.now().strftime("%Y%m%d") + ".yaml")
        cmd = self._pixi("dump-bodies", "--", "--out", out)
        self.log("[launcher] rigid bodies -> " + out)
        try:
            res = subprocess.run(cmd, cwd=self.optitrack, capture_output=True, text=True, timeout=60)
            if res.returncode != 0:
                self.log(f"[launcher] dump-bodies failed (Motive streaming?): {(res.stderr or res.stdout).strip().splitlines()[-1:]}")
        except (subprocess.TimeoutExpired, OSError) as err:
            self.log(f"[launcher] dump-bodies failed: {err}")

    def start_robot(self, cycles):
        cmd = self.plan.robot_start_cmd
        if not cmd:
            return None
        cmd = cmd.replace("{cycles}", str(int(cycles)))
        self.log("[launcher] robot: " + cmd)
        p = subprocess.Popen(cmd, shell=True, stdout=open(os.path.join(self.session_dir, "robot", "robot_start.log"), "a"),
                             stderr=subprocess.STDOUT, start_new_session=True)
        self.procs["robot"] = p
        return p

    def start(self):
        p = self.plan
        if p.start_tunnel:
            self.start_tunnel()
        if p.launch_app:
            self.launch_app()
        if p.dump_bodies:
            self.dump_bodies()
        if p.record_mocap:
            self.record_mocap()

    def stop(self):
        for name in ("mocap", "tunnel"):
            p = self.procs.pop(name, None)
            if p is None or p.poll() is not None:
                continue
            self.log(f"[launcher] stopping {name}...")
            try:
                if name == "mocap":
                    os.killpg(os.getpgid(p.pid), signal.SIGINT)      # ros2 launch + bag record close on SIGINT
                    p.wait(timeout=30)
                else:
                    p.terminate(); p.wait(timeout=5)
            except (subprocess.TimeoutExpired, ProcessLookupError, OSError):
                try:
                    p.kill()
                except OSError:
                    pass
        # the robot command (if any) ends by itself after its cycles
        self.procs.pop("robot", None)

    def status(self):
        return {k: ("running" if v.poll() is None else f"exit {v.returncode}") for k, v in self.procs.items()}
