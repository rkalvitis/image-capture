"""Session plan (YAML)."""
import dataclasses
import os
from typing import List

import yaml


@dataclasses.dataclass
class Plan:
    name: str = "session"
    turntable_steps: int = 36
    turntable_step_deg: float = 10.0
    turntable_yaw_tol_deg: float = 0.5
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
    phone_host: str = "127.0.0.1"
    phone_port: int = 5601
    phone_bundle_id: str = "ch.idsia.capture"
    robot_host: str = "192.168.201.10"
    robot_port: int = 5600
    output_root: str = "datasets"
    rig_transforms: str = ""              # rig_transforms.yaml (T_body_from_cam) -> camera pose columns in frames.csv
    merge_inline: bool = False              # merge each stack on the Mac during the session (background worker)
    merge_ref_index: int = -1               # reference focus index for the merge (-1 = the init-focus frame of each stop)
    merge_delete_raw: bool = False          # after a verified merge delete that stop's DNGs from the MAC (never the phone)
    merge_python: str = ""                  # interpreter with rawpy+cv2 (+dtcwt) for the merge worker ("" = this one)
    merge_fusion: str = "pyramid"           # pyramid (A, Laplacian pyramid) | wavelet (B, complex wavelets, Forster 2004)
    merge_format: str = "tif"               # tif = 16-bit sRGB TIFF master | tif-linear | png | jpg (+ a JPEG preview always)
    merge_jobs: int = 2                     # stops merged in parallel during the session (wavelet: 3-4)
    merge_index_maps: bool = False          # also write which frame won per pixel (stack/<image>_index.png)
    merge_max_pending: int = 5              # raw stacks allowed on the Mac at once (5 x 50 = 250 frames): the next pull
                                            # from the phone waits until a merge has finished and deleted its raw frames
    # session folder / dataset
    session_dir: str = ""                   # the dataset folder itself ("" = output_root/phone_<timestamp>)
    calibration_files: List[str] = dataclasses.field(default_factory=list)   # copied into <session>/calibrations/
    robot_files: List[str] = dataclasses.field(default_factory=list)         # copied into <session>/robot/
    # session-start exposure check (one prompt at step 0: a test frame, statistics, EV to apply)
    exposure_check: bool = False
    exposure_check_ev: float = 1.5          # suggested EV correction on top of the metered exposure
    exposure_max_duration_s: float = 0.1    # the rig is static: shutter up to this, then the ISO rises
    # start/stop everything from the coordinator (tools/run_coordinator.sh --plan ...)
    start_tunnel: bool = False              # pymobiledevice3 usbmux forward for phone_port
    launch_app: bool = False                # xcrun devicectl ... launch the app on the phone
    phone_device_id: str = ""               # CoreDevice id for devicectl
    record_mocap: bool = False              # `pixi run record-poses` in the optitrack repo, bag -> <session>/mocap/
    mocap_bodies: str = "phone popillia"    # RECORD_BODIES for the bag
    dump_bodies: bool = False               # `pixi run dump-bodies` -> <session>/calibrations/rigid_bodies_<date>.yaml
    optitrack_repo: str = "~/Documents/repos/optitrack"
    robot_start_cmd: str = ""               # optional shell command that starts the robot runs, {cycles} substituted
    stop_confirm: bool = True               # at the end: wait for Enter before the recording/tunnel are stopped
    notify: bool = False                    # macOS sound + spoken message + banner whenever the operator is needed
    notify_repeat_s: float = 45.0           # repeat the sound/speech every N s until the prompt is answered (0 = once)
    notify_voice: str = ""                  # `say -v <voice>` ("" = system default)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            d = yaml.safe_load(f) or {}
        unknown = set(d) - {f.name for f in dataclasses.fields(cls)}
        if unknown:
            raise ValueError(f"unknown plan keys: {sorted(unknown)}")
        if d.get("focus_mode") == "fixed":
            d["focus_mode"] = "range"                  # old name
        p = cls(**d)
        if p.turntable_confirm not in ("mocap", "enter", "both"):
            raise ValueError("turntable_confirm must be mocap | enter | both")
        if p.focus_mode not in ("range", "pose", "table", "auto"):
            raise ValueError(f"focus_mode must be range | pose | table | auto, not {p.focus_mode!r}")
        if p.merge_fusion not in ("pyramid", "wavelet"):
            raise ValueError("merge_fusion must be pyramid | wavelet")
        if p.merge_format not in ("tif", "tif-linear", "png", "jpg"):
            raise ValueError("merge_format must be tif | tif-linear | png | jpg")
        for key in ("session_dir", "optitrack_repo", "rig_transforms", "focus_table"):
            v = getattr(p, key)
            if v:
                setattr(p, key, os.path.expanduser(v))
        p.calibration_files = [os.path.expanduser(v) for v in p.calibration_files]
        p.robot_files = [os.path.expanduser(v) for v in p.robot_files]
        if p.focus_mode == "table" and not p.focus_table:
            raise ValueError("focus_mode table needs focus_table (tools/focus_from_dataset.py --yaml)")
        if len(p.lens_position) == 1 and p.poses > 1:
            p.lens_position = p.lens_position * p.poses
        if len(p.lens_position) != p.poses:
            raise ValueError(f"lens_position has {len(p.lens_position)} values, poses = {p.poses}")
        return p

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
