"""Mocap sources for the stillness gate and the turntable check.

`Ros2MocapSource` subscribes to /optitrack/<body> (geometry_msgs/PoseStamped)
in a background rclpy thread — run it with the optitrack pixi env python.
`SyntheticMocapSource` generates samples for dry runs and tests.

Sample = (t_seconds, pos[3] metres, quat_xyzw[4]). Stillness = the same
jitter definition as scripts/build_ring_poses.py in the optitrack repo:
norm of the per-axis position std over the window, in mm.
"""
import collections
import math
import threading
import time

import numpy as np


def yaw_deg(q_xyzw):
    """Rotation of the body's x-axis about world z, degrees (-180, 180]."""
    x, y, z, w = q_xyzw
    # body x-axis in world coordinates
    ax = 1 - 2 * (y * y + z * z)
    ay = 2 * (x * y + z * w)
    return math.degrees(math.atan2(ay, ax))


def yaw_diff_deg(a, b):
    d = (a - b + 180.0) % 360.0 - 180.0
    return d


class MocapSource:
    BODIES = ("phone", "popillia")
    enabled = True          # False = no mocap at all (NoMocapSource): no stillness gate, no poses

    def __init__(self, history_s=30.0):
        self.history_s = history_s
        self._hist = {b: collections.deque() for b in self.BODIES}
        self.counts = {b: 0 for b in self.BODIES}          # samples received per body (for the start-up check)
        self._lock = threading.Lock()

    # -- feed -----------------------------------------------------------
    def push(self, body, t, pos, quat):
        with self._lock:
            h = self._hist[body]
            self.counts[body] = self.counts.get(body, 0) + 1
            h.append((float(t), np.asarray(pos, float), np.asarray(quat, float)))
            cutoff = t - self.history_s
            while h and h[0][0] < cutoff:
                h.popleft()

    # -- queries --------------------------------------------------------
    def window(self, body, t0, t1):
        with self._lock:
            return [s for s in self._hist[body] if t0 <= s[0] <= t1]

    def latest(self, body):
        with self._lock:
            h = self._hist[body]
            return h[-1] if h else None

    def jitter_mm(self, body, t0, t1, min_samples=10):
        """(jitter_mm, n_samples) over [t0, t1]; jitter None if too few."""
        w = self.window(body, t0, t1)
        if len(w) < min_samples:
            return None, len(w)
        P = np.array([s[1] for s in w])
        return float(np.linalg.norm(P.std(axis=0)) * 1000.0), len(w)

    def mean_pose(self, body, t0, t1):
        w = self.window(body, t0, t1)
        if not w:
            return None, None
        P = np.array([s[1] for s in w]).mean(axis=0)
        Q = np.array([s[2] for s in w])
        Q[np.dot(Q, Q[0]) < 0] *= -1        # hemisphere-consistent average
        q = Q.mean(axis=0); q /= np.linalg.norm(q)
        return P, q

    def is_still(self, thresholds_mm, window_s, now=None, max_age_s=0.5):
        """thresholds_mm: {body: mm}. Returns (still, {body: jitter_mm})."""
        now = time.time() if now is None else now
        result, still = {}, True
        for body, thr in thresholds_mm.items():
            last = self.latest(body)
            if last is None or now - last[0] > max_age_s:
                result[body] = None; still = False; continue
            j, n = self.jitter_mm(body, now - window_s, now)
            result[body] = j
            if j is None or j > thr:
                still = False
        return still, result

    def wait_still(self, thresholds_mm, window_s, timeout_s, poll_s=0.1, on_poll=None):
        """Block until still; returns (ok, jitters, waited_s)."""
        t_start = time.time()
        while True:
            still, j = self.is_still(thresholds_mm, window_s)
            if on_poll:
                on_poll(still, j)
            if still:
                return True, j, time.time() - t_start
            if time.time() - t_start > timeout_s:
                return False, j, time.time() - t_start
            time.sleep(poll_s)

    def close(self):
        pass


class NoMocapSource(MocapSource):
    """No OptiTrack (robot/phone communication tests): every query says
    "still" with no data; the coordinator then waits a fixed settle time
    instead of the stillness gate and records no poses."""
    enabled = False

    def is_still(self, thresholds_mm, window_s, now=None, max_age_s=0.5):
        return True, {b: None for b in thresholds_mm}

    def wait_still(self, thresholds_mm, window_s, timeout_s, poll_s=0.1, on_poll=None):
        time.sleep(window_s)
        return True, {b: None for b in thresholds_mm}, window_s

    def jitter_mm(self, body, t0, t1, min_samples=10):
        return None, 0

    def mean_pose(self, body, t0, t1):
        return None, None


class SyntheticMocapSource(MocapSource):
    """Fake stream at `rate` Hz: phone fixed, popillia yaw follows
    `target_yaw` after `turn_delay_s`; `disturb(seconds)` adds motion."""

    def __init__(self, rate=120.0, noise_mm=0.03, turn_delay_s=1.0, phone_pos=(-1.07, 0.05, 1.62)):
        super().__init__()
        self.rate, self.noise_mm = rate, noise_mm
        self.turn_delay_s = turn_delay_s
        self.phone_pos = np.array(phone_pos)
        self.pop_pos = np.array([-1.08, 0.06, 1.57])
        self.yaw = 0.0
        self._target_yaw, self._target_since = 0.0, None
        self._disturb_until = 0.0
        self._stop = threading.Event()
        self._thr = threading.Thread(target=self._run, daemon=True)
        self._thr.start()

    def set_target_yaw(self, deg):
        self._target_yaw, self._target_since = deg, time.time()

    def disturb(self, seconds):
        self._disturb_until = time.time() + seconds

    def _run(self):
        rng = np.random.default_rng(0)
        dt = 1.0 / self.rate
        while not self._stop.is_set():
            t = time.time()
            if self._target_since is not None and t - self._target_since > self.turn_delay_s:
                self.yaw = self._target_yaw
                self._target_since = None
            turning = self._target_since is not None
            amp = self.noise_mm / 1000.0
            if t < self._disturb_until or turning:
                amp = 2.0 / 1000.0
            self.push("phone", t, self.phone_pos + rng.normal(0, amp, 3), [0, 0, 0, 1])
            yaw = self.yaw + (rng.normal(0, 3.0) if turning else 0.0)
            h = math.radians(yaw) / 2
            self.push("popillia", t, self.pop_pos + rng.normal(0, amp, 3), [0, 0, math.sin(h), math.cos(h)])
            time.sleep(dt)

    def close(self):
        self._stop.set()


class Ros2MocapSource(MocapSource):
    """rclpy subscriber on /optitrack/phone and /optitrack/popillia."""

    def __init__(self, bodies=("phone", "popillia"), topic_prefix="/optitrack/"):
        super().__init__()
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()
        self._node = Node("capture_coordinator")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        for b in bodies:
            self._node.create_subscription(PoseStamped, topic_prefix + b, self._cb(b), qos)
        self._stop = threading.Event()
        self._thr = threading.Thread(target=self._spin, daemon=True)
        self._thr.start()

    def _cb(self, body):
        def cb(msg):
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            p, o = msg.pose.position, msg.pose.orientation
            self.push(body, t, [p.x, p.y, p.z], [o.x, o.y, o.z, o.w])
        return cb

    def _spin(self):
        while not self._stop.is_set() and self._rclpy.ok():
            self._rclpy.spin_once(self._node, timeout_sec=0.1)

    def close(self):
        self._stop.set()
        self._thr.join(timeout=1.0)
        try:
            self._node.destroy_node()
        except Exception:
            pass
