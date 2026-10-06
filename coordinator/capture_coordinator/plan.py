"""Session plan (YAML)."""
import dataclasses
import os
import time
from typing import List

import yaml

# <repo>/coordinator/capture_coordinator/plan.py -> <repo>
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def expand_path(v):
    """`~` and `$VAR` / `${VAR}` in plan paths, so one plan file can serve the Mac and the
    Ubuntu PC (e.g. session_dir: $DATASETS/sep15_10deg_9vertical_FS)."""
    return os.path.expandvars(os.path.expanduser(v)) if v else v


@dataclasses.dataclass
class Plan:
    name: str = "session"
    subject: str = ""                       # what is being photographed (species, specimen id) -> summary
    notes: str = ""                         # free text for the session summary (setup, lighting, purpose)
    loop_order: str = "poses-per-rotation"  # poses-per-rotation = turntable outer loop: all robot poses at one insect
                                            #   rotation, then turn the insect, repeat (36 robot runs)
                                            # rotations-per-pose = robot pose outer loop: at every robot pose all
                                            #   turntable steps, then the platter RETURNS to step 0 (reverse, so the
                                            #   insect's height on the thread does not creep), then the next pose
                                            #   (ONE robot run; needs the motorised turntable to make sense)
    turntable_steps: int = 36
    turntable_step_deg: float = 10.0
    turntable_yaw_tol_deg: float = 0.5
    # motorised platter (hardware/turntable/): NEMA 17 + A4988 on a USB-serial board
    turntable_drive: str = "manual"         # manual = operator turns + Enter | portenta = the user's Portenta H7 stepper
                                            # controller (repo phd_roberts_thesis_stepper_motor_control) | serial = the older
                                            # Arduino sketch in hardware/turntable/ | fake = dry run
    turntable_port: str = "auto"            # auto = first USB serial port, else /dev/ttyACM0, /dev/cu.usbmodem...
    turntable_baud: int = 115200
    turntable_steps_per_rev: int = 200      # motor full steps per revolution (17HS4401: 1.8 deg)
    turntable_microsteps: int = 16          # A4988 MS1..3
    turntable_gear_ratio: float = 1.0       # motor revolutions per platter revolution (1 = direct drive)
    turntable_direction: int = 1            # -1 flips the DIR sense
    turntable_speed_deg_s: float = 20.0
    turntable_accel_deg_s2: float = 40.0
    turntable_hold: bool = True             # driver enabled between moves (holds the platter)
    poses: int = 9                          # robot poses per run (must match the robot's pose file)
    shoot_initial_pose: bool = False        # also shoot at the initial pose (robot pose 0, joint_start.csv) before go(0)
    lens_position_initial: float = 0.3      # init focus of robot pose 0 (shoot_initial_pose)
    lens_position: List[float] = dataclasses.field(default_factory=lambda: [0.45] * 9)  # per pose k=1..N
    frames_per_stop: int = 1                # 1 = single frame at lens_position[k]; >1 = focus stack
    focus_mode: str = "range"               # range = linspace(start, end); pose = stack around lens_position[k];
                                            # table = stack around the focus_table entry of the same pose with the nearest
                                            # popillia yaw (tools/focus_from_dataset.py); auto = AF on the centre at every stop
    focus_table: str = ""                   # yaml from tools/focus_from_dataset.py (focus_mode table)
    focus_table_yaw: float = None           # table: use the entry nearest THIS popillia yaw for every stop (instead of the
                                            # live mocap yaw), e.g. 176.48 = the first rotation of the aug31 session
    focus_stack_start: float = 0.0          # fixed: lens positions linspace(start, end, frames_per_stop)
    focus_stack_end: float = 0.5
    focus_stack_step: float = 0.0           # pose/table/auto: >0 = centre ± (n-1)/2 steps; 0 = uniform grid over
                                            # [focus_stack_start, focus_stack_end] with frames_per_stop frames, shifted so
                                            # that the centre (init focus) is EXACTLY one of the frames
    thermal_wait_s: float = 0.0             # >0: if the phone reported thermal "serious"/"critical" during the last stack,
                                            # wait (idle, up to this many seconds) before the next stack until it is "fair"
                                            # again — cooling idle beats the app's 3 s-per-frame throttle (200 s stacks)
    settle_ms: int = 300                    # lens settle on the phone between frames
    settle_after_move_s: float = 10.0       # minimum idle time after ANY movement (robot pose reached, insect turned)
                                            # before the first frame of the stack: the arm's move swings the popillia
                                            # (user, 2026-09-15); the mocap stillness gate runs after it on top
    exposure_mode: str = "locked"           # auto | locked (meter at hello, then hold) | custom
    exposure_duration_s: float = 0.01       # custom only
    exposure_iso: int = 64                  # custom only
    wb: dict = dataclasses.field(default_factory=lambda: {"mode": "locked"})   # auto | locked | custom {temperature_k, tint}
    turntable_confirm: str = "mocap"       # mocap = wait for the popillia yaw (±tol, still); enter = operator presses Enter on the Mac
                                            # after turning (yaw logged if mocap runs); both = Enter first, then the yaw check
    pose_confirm: bool = False              # at every robot pose: lens set to the stack centre, then wait for Enter on the Mac
                                            # (type a lens value 0..1 to override the centre; 'skip' / 'abort' also accepted)
    stillness_gate: bool = True             # False (dataset, 2026-09-16): never wait for / fail on mocap jitter — the
                                            # popillia body's tracking noise at rest is 0.1-0.2 mm; jitter is still logged
    still_window_s: float = 1.0
    still_phone_mm: float = 0.1
    still_popillia_mm: float = 0.1
    still_timeout_s: float = 15.0
    max_retries: int = 2
    # phone heat reduction (sent in hello; "" / None = leave the app's default)
    phone_raw: str = ""                     # proraw | bayer (bayer = no on-phone processing, demosaic on the Mac)
    phone_quality: str = ""                 # quality | balanced | speed (ProRAW processing effort)
    phone_frame_rate: int = 0               # cap the preview/intrinsics frame rate (0 = default 30)
    phone_screen_brightness: float = -1.0   # 0..1 (-1 = leave)
    phone_preview: bool = True              # live preview on the phone screen
    phone_blackout: bool = False            # dataset sessions: the phone screen is plain black (nothing drawn, brightness 0);
                                            # long-press 2 s on the phone or `bye` at the end brings the UI back
    phone_host: str = "127.0.0.1"
    phone_port: int = 5601
    phone_bundle_id: str = "ch.idsia.capture"
    robot_host: str = "192.168.201.10"
    robot_port: int = 5600
    output_root: str = "datasets"
    rig_transforms: str = ""              # rig_transforms.yaml (T_body_from_cam) -> camera pose columns in frames.csv
    merge_inline: bool = False              # merge each stack on the Mac during the session (background worker)
    merge_ref_index: int = -1               # reference focus index for the merge (-1 = per stop the frame where the beetle is sharpest)
    merge_delete_raw: bool = False          # after a verified merge delete that stop's DNGs from the MAC (never the phone)
    merge_python: str = ""                  # interpreter with rawpy+cv2 (+dtcwt) for the merge worker ("" = this one)
    merge_mask: str = "threshold"           # beetle mask that picks the merge reference: threshold | sam3 (SAM 3 "insect",
                                            # optitrack `masking` env, ~15 s per stop, one at a time)
    merge_fusion: str = "pyramid"           # pyramid (A, Laplacian pyramid) | wavelet (B, complex wavelets, Forster 2004)
    merge_format: str = "tif"               # tif = 16-bit sRGB TIFF master | tif-linear | png | jpg (+ a JPEG preview always)
    merge_jobs: int = 2                     # stops merged in parallel during the session (wavelet: 3-4)
    merge_index_maps: bool = False          # also write which frame won per pixel (stack/<image>_index.png)
    merge_max_pending: int = 5              # raw stacks allowed on the Mac at once (5 x 50 = 250 frames): the next pull
                                            # from the phone waits until a merge has finished and deleted its raw frames
    # cloud upload during the session (rclone -> OneDrive; docs/ONEDRIVE.md). The start prompt of the
    # coordinator asks for the remote too ("-" = none); --remote overrides it
    upload_remote: str = ""                 # rclone "remote:path", e.g. onedrive:SUPSI/popillia_2026-10-01 ("" = no upload)
    upload_what: str = "all"                # masters = merged image + preview + sidecars | raw = DNGs + sidecars | all
    upload_delete_local: object = False     # false | true | raw — after `rclone check` passed: true = the whole local
                                            # images/<image>/ folder goes (PC, no disk); raw = only the raw frames go, the
                                            # master + preview + sidecars stay (Mac: the dataset folder is kept)
    upload_delete_phone: bool = False       # ... and the stop's DNG + sidecar from the PHONE (only with upload_what raw | all)
    upload_rclone: str = "rclone"           # rclone binary (path)
    upload_transfers: int = 4               # parallel file transfers per stop
    upload_bwlimit: str = ""                # rclone --bwlimit, e.g. "8M" (bytes/s) or "08:00,4M 18:00,off"
    upload_timeout_s: int = 7200            # per rclone call
    upload_max_pending: int = 0             # stops allowed on this disk waiting for merge+upload before the next pull waits
                                            # (0 = merge_max_pending)
    upload_delete_session: bool = False     # at the very end, after the final sync passed `rclone check` and every captured
                                            # stop was uploaded: delete the whole session folder on this computer
    # session folder / dataset
    session_dir: str = ""                   # the dataset folder itself ("" = output_root/phone_<timestamp>)
    calibration_files: List[str] = dataclasses.field(default_factory=list)   # copied into <session>/calibrations/
    robot_files: List[str] = dataclasses.field(default_factory=list)         # copied into <session>/robot/
    poses_collect: List[str] = dataclasses.field(default_factory=list)       # globs of the ROBOT script's outputs on this PC
                                            # (manual_poses_*.csv run logs, joint_states_*.db) -> <session>/poses/robot_runs/;
                                            # <session>/poses/ = stops.csv, turntable.csv, robot_poses.csv, mocap/<image>.csv
    # session-start exposure check (one prompt at step 0: a test frame, statistics, EV to apply)
    exposure_check: bool = False
    exposure_check_ev: object = "auto"      # Enter at the check applies this EV: "auto" = the EV that puts the white foam
                                            # (frame p99) at 0.85 sRGB, measured on the test frame (2026-09-30); or a number
    exposure_max_duration_s: float = 0.1    # the rig is static: shutter up to this, then the ISO rises
    # start/stop everything from the coordinator (tools/run_coordinator.sh --plan ...)
    start_tunnel: bool = False              # pymobiledevice3 usbmux forward for phone_port
    launch_app: bool = False                # bring the app to the foreground: macOS xcrun devicectl, Linux pymobiledevice3 dvt launch
    app_launch_cmd: str = ""                # override: shell command, {bundle} / {device} substituted ("" = per platform)
    phone_device_id: str = ""               # CoreDevice id for devicectl (macOS only; Linux uses the bundle id)
    record_mocap: bool = False              # `pixi run record-poses` in the optitrack repo, bag -> <session>/mocap/
    mocap_bodies: str = "phone popillia"    # RECORD_BODIES for the bag
    dump_bodies: bool = False               # `pixi run dump-bodies` -> <session>/calibrations/rigid_bodies_<date>.yaml
    optitrack_repo: str = ""                # folder with the OptiTrack pixi.toml; "" = <this repo>/optitrack (since 2026-09-28)
    robot_start_cmd: str = ""               # optional shell command that starts the robot runs, {cycles} substituted
    stop_confirm: bool = True               # at the end: wait for Enter before the recording/tunnel are stopped
    notify: bool = False                    # sound + spoken message + banner whenever the operator is needed
                                            # (macOS: afplay/say/osascript; Linux: paplay|aplay, spd-say|espeak-ng, notify-send)
    notify_repeat_s: float = 45.0           # repeat the sound/speech every N s until the prompt is answered (0 = once)
    notify_voice: str = ""                  # `say -v <voice>` / `spd-say -y <voice>` / `espeak-ng -v <voice>` ("" = default)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            d = yaml.safe_load(f) or {}
        unknown = set(d) - {f.name for f in dataclasses.fields(cls)}
        if unknown:
            raise ValueError(f"unknown plan keys: {sorted(unknown)}")
        if d.get("focus_mode") == "fixed":
            d["focus_mode"] = "range"                  # old name
        today = time.strftime("%Y-%m-%d")
        for key in ("name", "session_dir", "upload_remote", "notes"):   # {date} = the day the plan is loaded
            if isinstance(d.get(key), str) and "{date}" in d[key]:
                d[key] = d[key].replace("{date}", today)
        p = cls(**d)
        if p.turntable_confirm not in ("mocap", "enter", "both"):
            raise ValueError("turntable_confirm must be mocap | enter | both")
        if p.loop_order not in ("poses-per-rotation", "rotations-per-pose"):
            raise ValueError("loop_order must be poses-per-rotation | rotations-per-pose")
        if p.turntable_drive not in ("manual", "portenta", "serial", "fake"):
            raise ValueError("turntable_drive must be manual | portenta | serial | fake")
        if p.turntable_direction not in (1, -1):
            raise ValueError("turntable_direction must be 1 | -1")
        if p.focus_mode not in ("range", "pose", "table", "auto"):
            raise ValueError(f"focus_mode must be range | pose | table | auto, not {p.focus_mode!r}")
        if p.merge_mask not in ("threshold", "sam3"):
            raise ValueError("merge_mask must be threshold | sam3")
        if p.merge_fusion not in ("pyramid", "wavelet"):
            raise ValueError("merge_fusion must be pyramid | wavelet")
        if p.merge_format not in ("tif", "tif-linear", "png", "jpg"):
            raise ValueError("merge_format must be tif | tif-linear | png | jpg")
        p.validate_upload()
        if p.exposure_check_ev != "auto":
            try:
                p.exposure_check_ev = float(p.exposure_check_ev)
            except (TypeError, ValueError):
                raise ValueError("exposure_check_ev must be a number or auto")
        for key in ("session_dir", "output_root", "optitrack_repo", "rig_transforms", "focus_table"):
            v = getattr(p, key)
            if v:
                setattr(p, key, expand_path(v))
        p.calibration_files = [expand_path(v) for v in p.calibration_files]
        p.robot_files = [expand_path(v) for v in p.robot_files]
        p.poses_collect = [expand_path(v) for v in p.poses_collect]
        if not p.optitrack_repo:
            p.optitrack_repo = os.path.join(REPO_ROOT, "optitrack")
        elif not os.path.isabs(p.optitrack_repo):
            p.optitrack_repo = os.path.join(REPO_ROOT, p.optitrack_repo)     # relative = inside this repo
        if p.focus_mode == "table" and not p.focus_table:
            raise ValueError("focus_mode table needs focus_table (tools/focus_from_dataset.py --yaml)")
        if len(p.lens_position) == 1 and p.poses > 1:
            p.lens_position = p.lens_position * p.poses
        if len(p.lens_position) != p.poses:
            raise ValueError(f"lens_position has {len(p.lens_position)} values, poses = {p.poses}")
        return p

    def validate_upload(self):
        if self.upload_what not in ("masters", "raw", "all"):
            raise ValueError("upload_what must be masters | raw | all")
        if self.upload_delete_local not in (True, False, "raw"):
            raise ValueError("upload_delete_local must be true | false | raw")
        if self.upload_delete_local == "raw" and self.upload_what not in ("raw", "all"):
            raise ValueError("upload_delete_local: raw needs the raw frames in the upload (upload_what raw | all)")
        if not self.upload_remote:
            return
        if ":" not in self.upload_remote:
            raise ValueError(f"upload_remote must be an rclone 'remote:path', got {self.upload_remote!r}")
        if self.upload_delete_session and not (self.upload_delete_local is True and self.upload_what == "all"):
            raise ValueError("upload_delete_session needs upload_what: all and upload_delete_local: true "
                             "(otherwise files that exist nowhere else would be deleted)")
        if self.upload_delete_phone and self.upload_what == "masters":
            raise ValueError("upload_delete_phone needs the raw frames in the upload: upload_what raw | all "
                             "(with masters only, the DNGs would exist nowhere after the phone delete)")
        if self.upload_what in ("raw", "all") and self.merge_inline and self.merge_delete_raw:
            raise ValueError("with upload_what raw | all the UPLOAD worker deletes the local raw frames after the verified "
                             "upload: set merge_delete_raw: false (and upload_delete_local: true)")

    def set_turntable_steps(self, n):
        """N insect rotations per revolution: the step becomes exactly 360/N deg (the motor rounds
        each ABSOLUTE target to the nearest microstep, so nothing accumulates). Returns the
        microstep resolution and the largest rounding error of any stop, in degrees."""
        n = int(n)
        if n < 1:
            raise ValueError("turntable_steps must be >= 1")
        self.turntable_steps = n
        self.turntable_step_deg = 360.0 / n
        return self.turntable_precision()

    def turntable_precision(self):
        """{'deg_per_microstep', 'max_error_deg', 'exact'} for the current step / motor settings."""
        mspr = float(self.turntable_steps_per_rev) * float(self.turntable_microsteps) * float(self.turntable_gear_ratio)
        deg_us = 360.0 / mspr
        worst = 0.0
        for s in range(int(self.turntable_steps)):
            target = s * self.turntable_step_deg
            steps = round(target / deg_us)
            worst = max(worst, abs(steps * deg_us - target))
        return {"deg_per_microstep": deg_us, "max_error_deg": worst, "exact": worst < 1e-9, "microsteps_per_rev": mspr}

    def stops_planned(self):
        return int(self.turntable_steps) * (int(self.poses) + (1 if self.shoot_initial_pose else 0))

    def storage_estimate(self, raw_mb=15.0, master_mb=60.0):
        """Rough bytes of a full session: (raw, masters). ProRAW DNG ~9-15 MB, 16-bit LZW TIFF ~60 MB (2026-09)."""
        stops = self.stops_planned()
        raw = stops * int(self.frames_per_stop) * raw_mb * 1e6
        masters = stops * master_mb * 1e6 if self.merge_inline else 0.0
        return raw, masters

    def loop_order_text(self):
        n = self.poses + (1 if self.shoot_initial_pose else 0)
        if self.loop_order == "rotations-per-pose":
            return (f"robot pose outer loop: at each of the {n} robot poses all {self.turntable_steps} insect rotations "
                    f"({self.turntable_step_deg:g} deg each), platter returned to step 0 (reverse) before the next pose; "
                    f"one robot run")
        return (f"insect rotation outer loop: all {n} robot poses at one insect rotation, then the insect is turned by "
                f"{self.turntable_step_deg:g} deg; {self.turntable_steps} rotations = {self.turntable_steps} robot runs")

    def merge_ref(self):
        """-1 = per stop the frame that carries the init focus (focus_centre)."""
        return self.merge_ref_index

    def init_focus(self, k):
        """Plan init focus of robot pose k (0 = the initial pose, 1..N = the pose file)."""
        return float(self.lens_position_initial if k == 0 else self.lens_position[k - 1])

    def lens_for_pose(self, k):
        """Lens positions for robot pose k (0 = initial pose, 1..N): one value, the stack
        around the init focus (focus_mode pose), or the fixed range."""
        if self.frames_per_stop <= 1:
            return [self.init_focus(k)]
        if self.focus_mode == "pose":
            return self.lens_around(self.init_focus(k))
        n = int(self.frames_per_stop)
        a, b = float(self.focus_stack_start), float(self.focus_stack_end)
        return [round(a + (b - a) * i / (n - 1), 4) for i in range(n)]

    def load_focus_table(self):
        """{robot_pose: [(yaw_deg, lens, image), ...]} from focus_table (or {} if none)."""
        if not self.focus_table:
            return {}
        with open(self.focus_table) as f:
            t = yaml.safe_load(f)
        table = {}
        for e in t.get("table", []):
            table.setdefault(int(e["robot_pose"]), []).append((e.get("yaw_deg"), float(e["lens"]), e.get("image"), float(e.get("t") or 0)))
        return table

    def lens_around(self, centre):
        """frames_per_stop lens positions containing `centre` (the init focus) exactly.
        focus_stack_step > 0: centre ± (n-1)/2 steps (odd n -> centre is the middle frame).
        focus_stack_step == 0: the uniform grid start..end (step (end-start)/(n-1)) shifted
        by less than one step so that one frame equals `centre`; the grid start is
        kept >= 0 and the end <= 1 (then the shift can reach a full step)."""
        n = max(1, int(self.frames_per_stop))
        centre = float(centre)
        if n == 1:
            return [round(centre, 6)]
        step = float(self.focus_stack_step)
        if step > 0:
            return [round(min(1.0, max(0.0, centre + step * (i - (n - 1) / 2.0))), 6) for i in range(n)]
        a, b = float(self.focus_stack_start), float(self.focus_stack_end)
        step = (b - a) / (n - 1)
        k0 = int(round((centre - a) / step))                      # index of the frame that becomes `centre`
        start = centre - k0 * step
        while start < -1e-9 and k0 > 0:
            k0 -= 1; start = centre - k0 * step
        while start + (n - 1) * step > 1.0 + 1e-9 and k0 < n - 1:
            k0 += 1; start = centre - k0 * step
        k0 = min(max(k0, 0), n - 1)
        vals = [centre + (i - k0) * step for i in range(n)]
        vals[k0] = centre
        return [round(min(1.0, max(0.0, v)), 6) for v in vals]

    def centre_index(self, lens, centre):
        """Index of the frame that carries the init focus (closest to centre)."""
        return min(range(len(lens)), key=lambda i: abs(lens[i] - centre))

    def to_dict(self):
        return dataclasses.asdict(self)
