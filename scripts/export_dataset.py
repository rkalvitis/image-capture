#!/usr/bin/env python3
"""Export a recorded dataset bag to image files + a per-frame pose CSV.

For every color frame in the bag, the camera rigid-body pose is interpolated
(position lerp, orientation slerp) at the frame's capture timestamp and
written as one CSV row. Depth frames are paired to color frames by nearest
timestamp. Camera intrinsics are saved once as camera_info.yaml.

Usage (from the repo root):
  pixi run export -- datasets/optitrack_20260609_153000 --rigid-body camera

Key options:
  --rigid-body NAME    Motive rigid-body name on the camera (topic
                       /optitrack/NAME). Auto-detected when the bag holds
                       exactly one /optitrack/* topic.
  --time-offset SEC    Added to every image timestamp before the pose lookup.
                       Use to cancel a residual constant lag measured in the
                       lab (positive = "the true capture happened later than
                       the image stamp says"). Default 0.
  --max-pose-gap SEC   Skip frames whose bracketing poses are further apart
                       than this (tracking dropout). Default 0.1.
  --out DIR            Output directory. Default <bag>_export.

Output layout:
  <out>/color/000000.jpg ...      camera-native JPEGs (PNGs for old raw bags)
  <out>/depth/000000.png ...      16-bit grayscale PNGs (millimetres)
  <out>/frames.csv                one row per color frame (see header)
  <out>/camera_info.yaml          color + depth intrinsics
"""

import argparse
import csv
import os
import sys
from bisect import bisect_left

import numpy as np
from PIL import Image as PILImage

from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
import rosbag2_py


def _stamp_ns(header) -> int:
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def _open_bag(path: str) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    return reader


def _slerp(q0: np.ndarray, q1: np.ndarray, u: float) -> np.ndarray:
    """Spherical interpolation between two xyzw quaternions, u in [0, 1]."""
    dot = float(np.dot(q0, q1))
    if dot < 0.0:           # take the short arc
        q1, dot = -q1, -dot
    if dot > 0.9995:        # nearly parallel — lerp is fine and stable
        q = q0 + u * (q1 - q0)
        return q / np.linalg.norm(q)
    theta = np.arccos(np.clip(dot, -1.0, 1.0))
    s = np.sin(theta)
    return (np.sin((1.0 - u) * theta) * q0 + np.sin(u * theta) * q1) / s


class PoseTrack:
    """Time-sorted pose samples with interpolated lookup."""

    def __init__(self):
        self.times: list[int] = []          # ns
        self.pos:   list[np.ndarray] = []
        self.quat:  list[np.ndarray] = []   # xyzw

    def add(self, msg: PoseStamped):
        self.times.append(_stamp_ns(msg.header))
        p, o = msg.pose.position, msg.pose.orientation
        self.pos.append(np.array([p.x, p.y, p.z]))
        self.quat.append(np.array([o.x, o.y, o.z, o.w]))

    def sort(self):
        order = np.argsort(self.times)
        self.times = [self.times[i] for i in order]
        self.pos = [self.pos[i] for i in order]
        self.quat = [self.quat[i] for i in order]

    def lookup(self, t_ns: int, max_gap_ns: int):
        """Interpolated (pos, quat, gap_ns) at t_ns, or None if not covered."""
        i = bisect_left(self.times, t_ns)
        if i == 0 or i == len(self.times):
            return None                      # outside the recorded pose span
        t0, t1 = self.times[i - 1], self.times[i]
        gap = t1 - t0
        if gap > max_gap_ns:
            return None                      # tracking dropout around t_ns
        u = (t_ns - t0) / gap if gap > 0 else 0.0
        pos = (1.0 - u) * self.pos[i - 1] + u * self.pos[i]
        quat = _slerp(self.quat[i - 1], self.quat[i], u)
        return pos, quat, gap


def _save_color(msg: Image, path: str):
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    if msg.encoding == 'bgr8':
        arr = arr[:, :, ::-1]
    elif msg.encoding != 'rgb8':
        raise ValueError(f'unsupported color encoding {msg.encoding!r}')
    PILImage.fromarray(arr, mode='RGB').save(path)


def _save_depth(msg: Image, path: str):
    if msg.encoding != '16UC1':
        raise ValueError(f'unsupported depth encoding {msg.encoding!r}')
    arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
    PILImage.fromarray(arr, mode='I;16').save(path)


def _camera_info_yaml(name: str, ci: CameraInfo) -> str:
    return (
        f'{name}:\n'
        f'  width: {ci.width}\n'
        f'  height: {ci.height}\n'
        f'  distortion_model: {ci.distortion_model}\n'
        f'  K: {[round(v, 6) for v in ci.k]}\n'
        f'  D: {[round(v, 6) for v in ci.d]}\n'
        f'  P: {[round(v, 6) for v in ci.p]}\n'
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', help='path to the rosbag2 directory (datasets/optitrack_...)')
    ap.add_argument('--rigid-body', default=None,
                    help='Motive rigid-body name on the camera (default: auto-detect)')
    ap.add_argument('--time-offset', type=float, default=0.0,
                    help='seconds added to image stamps before the pose lookup (default 0)')
    ap.add_argument('--max-pose-gap', type=float, default=0.1,
                    help='skip frames bracketed by poses further apart than this many seconds')
    ap.add_argument('--out', default=None, help='output directory (default <bag>_export)')
    ap.add_argument('--camera-ns', default='camera', help='camera topic namespace (default camera)')
    args = ap.parse_args()

    bag = args.bag.rstrip('/')
    out = args.out or bag + '_export'

    color_topic = f'/{args.camera_ns}/color/image_raw'
    depth_topic = f'/{args.camera_ns}/depth/image_raw'
    cinfo_topic = f'/{args.camera_ns}/color/camera_info'
    dinfo_topic = f'/{args.camera_ns}/depth/camera_info'

    reader = _open_bag(bag)
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}

    optitrack_topics = sorted(t for t in topics if t.startswith('/optitrack/'))
    if args.rigid_body:
        pose_topic = f'/optitrack/{args.rigid_body}'
        if pose_topic not in topics:
            sys.exit(f'ERROR: {pose_topic} not in bag. OptiTrack topics found: '
                     f'{optitrack_topics or "none"}')
    elif len(optitrack_topics) == 1:
        pose_topic = optitrack_topics[0]
        print(f'Auto-detected rigid body topic: {pose_topic}')
    else:
        sys.exit(f'ERROR: pass --rigid-body to choose among: {optitrack_topics or "none"}')

    # New-style bags carry color as the camera's native JPEG bitstream on
    # .../image_raw/compressed; old bags have raw rgb8 on .../image_raw.
    compressed = color_topic + '/compressed' in topics
    if compressed:
        color_topic += '/compressed'
    elif color_topic not in topics:
        sys.exit(f'ERROR: {color_topic} not in bag (topics: {sorted(topics)})')

    # ── pass 1: poses + depth index + intrinsics ────────────────────────────
    track = PoseTrack()
    depth_stamps: list[int] = []
    depth_msgs: dict[int, Image] = {}
    color_info = depth_info = None

    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic == pose_topic:
            track.add(deserialize_message(raw, PoseStamped))
        elif topic == depth_topic:
            m = deserialize_message(raw, Image)
            depth_msgs[_stamp_ns(m.header)] = m
        elif topic == cinfo_topic and color_info is None:
            color_info = deserialize_message(raw, CameraInfo)
        elif topic == dinfo_topic and depth_info is None:
            depth_info = deserialize_message(raw, CameraInfo)

    if not track.times:
        sys.exit(f'ERROR: no pose messages on {pose_topic} in this bag')
    track.sort()
    depth_stamps = sorted(depth_msgs)
    pose_span_s = (track.times[-1] - track.times[0]) / 1e9
    pose_hz = (len(track.times) - 1) / pose_span_s if pose_span_s > 0 else 0.0
    print(f'{len(track.times)} poses on {pose_topic}, '
          f'span {pose_span_s:.1f} s ({pose_hz:.1f} Hz); '
          f'{len(depth_stamps)} depth frames')

    os.makedirs(os.path.join(out, 'color'), exist_ok=True)
    os.makedirs(os.path.join(out, 'depth'), exist_ok=True)

    if color_info or depth_info:
        with open(os.path.join(out, 'camera_info.yaml'), 'w') as f:
            if color_info:
                f.write(_camera_info_yaml('color', color_info))
            if depth_info:
                f.write(_camera_info_yaml('depth', depth_info))

    # ── pass 2: color frames → PNG + CSV row ────────────────────────────────
    offset_ns = int(args.time_offset * 1e9)
    max_gap_ns = int(args.max_pose_gap * 1e9)
    half_depth_period = None
    if len(depth_stamps) > 1:
        half_depth_period = int(np.median(np.diff(depth_stamps)) / 2)

    reader = _open_bag(bag)
    n_total = n_kept = 0
    csv_path = os.path.join(out, 'frames.csv')
    with open(csv_path, 'w', newline='') as f:
        wr = csv.writer(f)
        wr.writerow(['index', 'color_file', 'depth_file', 't_capture_ns',
                     'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
                     'pose_gap_ms', 'depth_dt_ms'])
        while reader.has_next():
            topic, raw, _ = reader.read_next()
            if topic != color_topic:
                continue
            n_total += 1
            msg = deserialize_message(raw, CompressedImage if compressed else Image)
            t = _stamp_ns(msg.header) + offset_ns

            hit = track.lookup(t, max_gap_ns)
            if hit is None:
                continue
            pos, quat, gap = hit

            # nearest depth frame, if close enough to be the same instant
            depth_file, depth_dt = '', ''
            if depth_stamps:
                j = bisect_left(depth_stamps, t)
                cand = [k for k in (j - 1, j) if 0 <= k < len(depth_stamps)]
                k = min(cand, key=lambda k: abs(depth_stamps[k] - t))
                dt = depth_stamps[k] - t
                if half_depth_period is None or abs(dt) <= half_depth_period:
                    depth_file = f'depth/{n_kept:06d}.png'
                    depth_dt = f'{dt / 1e6:.2f}'
                    _save_depth(depth_msgs[depth_stamps[k]],
                                os.path.join(out, depth_file))

            if compressed:
                # Write the JPEG bitstream as-is: zero recompression loss
                color_file = f'color/{n_kept:06d}.jpg'
                with open(os.path.join(out, color_file), 'wb') as cf:
                    cf.write(bytes(msg.data))
            else:
                color_file = f'color/{n_kept:06d}.png'
                _save_color(msg, os.path.join(out, color_file))
            wr.writerow([n_kept, color_file, depth_file, t,
                         f'{pos[0]:.6f}', f'{pos[1]:.6f}', f'{pos[2]:.6f}',
                         f'{quat[0]:.6f}', f'{quat[1]:.6f}',
                         f'{quat[2]:.6f}', f'{quat[3]:.6f}',
                         f'{gap / 1e6:.2f}', depth_dt])
            n_kept += 1

    print(f'Exported {n_kept}/{n_total} color frames to {out}')
    if n_kept < n_total:
        print(f'({n_total - n_kept} frames dropped: capture time outside pose '
              f'coverage or pose gap > {args.max_pose_gap} s)')
    print(f'CSV: {csv_path}')


if __name__ == '__main__':
    main()
