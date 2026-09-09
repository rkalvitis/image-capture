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
    lens_position: List[float] = dataclasses.field(default_factory=lambda: [0.45] * 9)  # per pose k=1..N
    frames_per_stop: int = 1                # 1 = single frame at lens_position[k]; >1 = focus stack
    focus_stack_start: float = 0.0          # stack: lens positions linspace(start, end, frames_per_stop)
    focus_stack_end: float = 0.5
    settle_ms: int = 300
    exposure_mode: str = "locked"           # auto | locked (meter at hello, then hold) | custom
    exposure_duration_s: float = 0.01       # custom only
    exposure_iso: int = 64                  # custom only
    wb: dict = dataclasses.field(default_factory=lambda: {"mode": "locked"})   # auto | locked | custom {temperature_k, tint}
    still_window_s: float = 1.0
    still_phone_mm: float = 0.1
    still_popillia_mm: float = 0.1
    still_timeout_s: float = 15.0
    max_retries: int = 2
    phone_host: str = "127.0.0.1"
    phone_port: int = 5601
    phone_bundle_id: str = "ch.idsia.capture"
    robot_host: str = "192.168.201.10"
    robot_port: int = 5600
    output_root: str = "datasets"

    @classmethod
    def load(cls, path):
        with open(path) as f:
            d = yaml.safe_load(f) or {}
        unknown = set(d) - {f.name for f in dataclasses.fields(cls)}
        if unknown:
            raise ValueError(f"unknown plan keys: {sorted(unknown)}")
        p = cls(**d)
        if len(p.lens_position) == 1 and p.poses > 1:
            p.lens_position = p.lens_position * p.poses
        if len(p.lens_position) != p.poses:
            raise ValueError(f"lens_position has {len(p.lens_position)} values, poses = {p.poses}")
        return p

    def lens_for_pose(self, k):
        """Lens positions for robot pose k (1-based): one value, or the focus stack."""
        if self.frames_per_stop <= 1:
            return [float(self.lens_position[k - 1])]
        n = int(self.frames_per_stop)
        a, b = float(self.focus_stack_start), float(self.focus_stack_end)
        return [round(a + (b - a) * i / (n - 1), 4) for i in range(n)]

    def to_dict(self):
        return dataclasses.asdict(self)
