#!/usr/bin/env python3
"""Link the robot-frame phone poses to the mocap frame for a turntable session.

Inputs (all measured, nothing taken from earlier calibrations):
  * a rosbag with /optitrack/phone and /optitrack/popillia (PoseStamped),
    recorded while the Panda replayed the manual poses (>= 1 full block);
  * a second "spin" bag where the turntable was rotated through a full turn
    with the popillia markers on the rotating stage (phone parked);
  * the per-block manual_poses_*.csv logs written by panda_manual_poses.py
    (flange pose in the robot base frame per waypoint, waypoint 0 = init).

Method:
  1. Still windows of /optitrack/phone are matched to CSV waypoints by
     rotation-angle invariants (relative rotation angles survive any fixed
     frame change, so no prior hand-eye is needed; visiting order is NOT
     assumed - the robot revisits poses in reverse on the way home).
  2. Joint alternating solve of  p_mocap -> R p + t  (robot<-mocap) and the
     marker-body origin r in the flange frame:
         R p_moc(k) + t = p_flange(k) + R_flange(k) r
     with one 3-sigma outlier-rejection pass.
  3. The turntable axis comes from an SVD plane + algebraic circle fit of the
     popillia centroid in the spin bag, then is mapped into the robot frame.

Output: <dataset>/registration.yaml + a printed report.

Usage:
  pixi run python scripts/link_robot_mocap.py datasets/phone_20260826_113325 \
      --spin-bag datasets/phone_20260826_113004/phone_20260826_113004_0.db3 \
      --csv manual_poses_20260826_112820.csv manual_poses_20260826_113921.csv
"""

import argparse
import glob
import os
import sqlite3
import struct

import numpy as np


# ---------- small rotation helpers ----------

def quat_mat(q):
    x, y, z, w = np.asarray(q, float)
    n = np.linalg.norm([x, y, z, w])
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat_quat(R):
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    if w > 1e-6:
        q = np.array([(R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w),
                      (R[1, 0] - R[0, 1]) / (4 * w), w])
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(max(0.0, 1 + R[i, i] - R[j, j] - R[k, k])) * 2
        v = [0.0, 0.0, 0.0]
        v[i] = s / 4
        v[j] = (R[j, i] + R[i, j]) / s
        v[k] = (R[k, i] + R[i, k]) / s
        q = np.array([*v, (R[k, j] - R[j, k]) / s])
    return q / np.linalg.norm(q)


def rot_angle_deg(R):
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


# ---------- bag reading (manual CDR walk, no rosbag2_py needed) ----------

def _parse_pose_cdr(blob):
    """geometry_msgs/PoseStamped CDR -> (stamp_s, [x y z qx qy qz qw])."""
    off = 4  # encapsulation header; CDR alignment is relative to byte 4

    def align(n, o):
        return o + ((-(o - 4)) % n)

    off = align(4, off)
    sec = struct.unpack_from('<i', blob, off)[0]
    off += 4
    off = align(4, off)
    nsec = struct.unpack_from('<I', blob, off)[0]
    off += 4
    off = align(4, off)
    slen = struct.unpack_from('<I', blob, off)[0]
    off += 4 + slen
    off = align(8, off)
    return sec + nsec * 1e-9, np.array(struct.unpack_from('<7d', blob, off))


def load_bag_topic(db_path, topic):
    """-> (stamps[N], vals[N,7]) ordered by stamp (px py pz qx qy qz qw)."""
    con = sqlite3.connect(db_path)
    tids = {name: tid for tid, name in con.execute("SELECT id, name FROM topics")}
    rows = con.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp",
                       (tids[topic],)).fetchall()
    parsed = [_parse_pose_cdr(d) for (d,) in rows]
    stamps = np.array([p[0] for p in parsed])
    vals = np.array([p[1] for p in parsed])
    order = np.argsort(stamps)
    return stamps[order], vals[order]


# ---------- CSV waypoints ----------

def load_run_csv(path):
    """-> list of 4x4 flange poses (robot base frame), lens positions [N,3]."""
    import csv
    T, lens = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            R = quat_mat([float(row['orientation_x']), float(row['orientation_y']),
                          float(row['orientation_z']), float(row['orientation_w'])])
            M = np.eye(4)
            M[:3, :3] = R
            M[:3, 3] = [float(row['position_x']), float(row['position_y']),
                        float(row['position_z'])]
            T.append(M)
            lens.append([float(row['lens_x']), float(row['lens_y']), float(row['lens_z'])])
    return T, np.array(lens)


# ---------- still windows ----------

def still_windows(t, p, q, lag=1.0, thresh=0.0025, min_dur=2.0,
                  quality_gate=False, max_dropout=0.5, max_std_mm=2.0,
                  min_samples=100, report=None):
    """Windows where displacement over `lag` seconds stays under `thresh` (m).
    -> list of (t_start, median_pos, median_quat); adjacent windows whose
    medians agree within 2 mm are merged (one hold, detector split).

    quality_gate: drop holds where the mocap tracking was bad — the phone
    goes horizontal at the low waypoints and the markers face away from the
    cameras, giving up to 99% frame dropout and cm-level jitter/drift
    (measured by scripts/pose_stability.py). A registration that ingests
    those holds is wrong by construction."""
    ilag = np.searchsorted(t, t - lag)
    moving = np.linalg.norm(p - p[ilag], axis=1) > thresh
    period = np.median(np.diff(t))
    out = []
    i, n = 0, len(moving)
    while i < n:
        j = i
        while j < n and moving[j] == moving[i]:
            j += 1
        if not moving[i] and t[j - 1] - t[i] >= min_dur:
            if quality_gate:
                dur = t[j - 1] - t[i]
                dropout = max(0.0, 1 - (j - i) / (dur / period + 1))
                std_mm = np.linalg.norm(p[i:j].std(axis=0)) * 1000
                if dropout > max_dropout or std_mm > max_std_mm \
                        or (j - i) < min_samples:
                    if report:
                        report(f'[gate] dropped hold at t={t[i]:.1f}s: '
                               f'dropout {dropout*100:.0f}%, '
                               f'std {std_mm:.1f} mm, n={j-i}')
                    i = j
                    continue
            c = np.median(p[i:j], axis=0)
            qm = np.median(q[i:j], axis=0)
            qm /= np.linalg.norm(qm)
            if out and np.linalg.norm(c - out[-1][1]) < 0.002:
                out[-1] = (out[-1][0], (out[-1][1] + c) / 2, qm)
            else:
                out.append((t[i], c, qm))
        i = j
    return out


def match_stills_to_waypoints(T_wp, stills):
    """Correspondence via rotation-angle invariants. The first (long, at the
    block start) still is the init hold = waypoint 0; every other still is
    assigned to the waypoint whose rotation angle w.r.t. waypoint 0 matches
    the still's angle w.r.t. still 0 within 3 deg (unique winners only).
    Robust to reverse revisiting and to missed/short holds."""
    ang_wp = np.array([rot_angle_deg(T_wp[0][:3, :3].T @ T[:3, :3]) for T in T_wp])
    matches = [(0, 0)]
    R0 = quat_mat(stills[0][2])
    for k in range(1, len(stills)):
        ang = rot_angle_deg(R0.T @ quat_mat(stills[k][2]))
        w = int(np.argmin(np.abs(ang_wp - ang)))
        if abs(ang_wp[w] - ang) < 3.0 and w not in (m[1] for m in matches):
            matches.append((k, w))
    return matches


# ---------- solvers ----------

def kabsch(A, B):
    """R, t with  R A + t ~= B."""
    ca, cb = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, cb - R @ ca


def init_marker_offset(T_fl, T_body):
    """AX=XB hand-eye on relative motions -> initial marker-body offset r in
    the flange frame (the alternating refinement below is non-convex and
    needs a sane starting point). T_fl: flange poses (robot), T_body: phone
    body poses (mocap), same correspondence order."""

    def lm(q):
        x, y, z, w = q
        return np.array([[w, -z, y, x], [z, w, -x, y], [-y, x, w, z], [-x, -y, -z, w]])

    def rm(q):
        x, y, z, w = q
        return np.array([[w, z, -y, x], [-z, w, x, y], [y, -x, w, z], [-x, -y, -z, w]])

    rels = []
    for i in range(len(T_fl)):
        for j in range(i + 1, len(T_fl)):
            P = np.linalg.inv(T_fl[i]) @ T_fl[j]
            Q = np.linalg.inv(T_body[i]) @ T_body[j]
            if rot_angle_deg(P[:3, :3]) > 10:
                rels.append((P, Q))
    M = np.vstack([lm(mat_quat(P[:3, :3])) - rm(mat_quat(Q[:3, :3])) for P, Q in rels])
    _, _, Vt = np.linalg.svd(M)
    Rx = quat_mat(Vt[-1])
    A = np.vstack([P[:3, :3] - np.eye(3) for P, _ in rels])
    b = np.concatenate([Rx @ Q[:3, 3] - P[:3, 3] for P, Q in rels])
    tx, *_ = np.linalg.lstsq(A, b, rcond=None)
    return tx, Rx


def solve_registration(p_moc, p_fl, R_fl, r0=np.zeros(3)):
    """Alternate  R p_moc + t = p_fl + R_fl r  for (R, t) and r."""
    r = np.asarray(r0, float)
    for _ in range(200):
        R, t = kabsch(p_moc, p_fl + np.einsum('nij,j->ni', R_fl, r))
        rhs = ((R @ p_moc.T).T + t - p_fl).reshape(-1)
        r_new, *_ = np.linalg.lstsq(np.vstack(R_fl), rhs, rcond=None)
        if np.linalg.norm(r_new - r) < 1e-10:
            r = r_new
            break
        r = r_new
    res = (R @ p_moc.T).T + t - (p_fl + np.einsum('nij,j->ni', R_fl, r))
    return R, t, r, np.linalg.norm(res, axis=1)


def fit_turntable_axis(P):
    """SVD plane + algebraic circle fit -> (center[3], normal[3], radius,
    planarity_rms, radial_rms)."""
    cen = P.mean(0)
    _, _, Vt = np.linalg.svd(P - cen, full_matrices=False)
    normal = Vt[2] if Vt[2][2] > 0 else -Vt[2]
    e1, e2 = Vt[0], Vt[1]
    uv = np.c_[(P - cen) @ e1, (P - cen) @ e2]
    sol, *_ = np.linalg.lstsq(np.c_[2 * uv, np.ones(len(uv))], (uv ** 2).sum(1), rcond=None)
    cu, cv, c0 = sol
    radius = np.sqrt(c0 + cu * cu + cv * cv)
    center = cen + cu * e1 + cv * e2
    radial = np.abs(np.sqrt(((uv - [cu, cv]) ** 2).sum(1)) - radius)
    plan = (P - cen) @ normal
    return center, normal, radius, float(np.std(plan)), float(np.sqrt((radial ** 2).mean()))


# ---------- continuous-trajectory registration (--joints) ----------
#
# With a /joint_states recording (record_joint_states.py on the robot PC)
# the registration does not need still windows at all: FK turns the joint
# stream into a continuous flange trajectory, the robot<->Mac clock offset
# is solved by cross-correlating the two motion-speed profiles (both sides
# stamp at capture time, so there is no lag bias - only a constant offset),
# and every mocap sample becomes a (p_mocap, T_flange) pair. Per-frame
# weights (slow-motion preference, local dropout, jitter, Huber IRLS)
# replace the still/quality hard gates, so the low poses where tracking is
# poor still contribute - just weakly - instead of being discarded.

def load_joints_csv(paths):
    """joint_states_*.db / .csv file(s) from record_joint_states.py ->
    (stamps[N] robot clock, joints[N,7]). The recorder writes sqlite
    (table joint_states: stamp, joint1..joint7); plain CSV with the same
    columns is accepted for older recordings."""
    import csv as _csv
    ts, js = [], []
    for path in paths:
        with open(path, 'rb') as f:
            is_sqlite = f.read(16).startswith(b'SQLite format 3')
        if is_sqlite:
            con = sqlite3.connect(path)
            for row in con.execute(
                    'SELECT stamp, joint1, joint2, joint3, joint4, '
                    'joint5, joint6, joint7 FROM joint_states '
                    'ORDER BY stamp'):
                ts.append(row[0])
                js.append(row[1:])
            con.close()
        else:
            with open(path) as f:
                for row in _csv.DictReader(f):
                    ts.append(float(row['stamp']))
                    js.append([float(row['joint%d' % k])
                               for k in range(1, 8)])
    ts, js = np.array(ts), np.array(js, float)
    order = np.argsort(ts)
    return ts[order], js[order]


def _speed_profile(t, p, grid, lag=0.2):
    """|velocity| resampled onto a uniform grid. Displacement is taken
    over `lag` seconds, not adjacent samples - adjacent diffs of 120 Hz
    mocap amplify sub-mm sensor noise into a cm/s noise floor that buries
    the real motion signal."""
    ilag = np.searchsorted(t, t - lag)
    dt = t - t[ilag]
    ok = dt > lag / 2
    v = np.linalg.norm(p - p[ilag], axis=1)[ok] / dt[ok]
    v = np.minimum(v, 1.0)
    tm = (t - dt / 2)[ok]
    prof = np.interp(grid, tm, v, left=0.0, right=0.0)
    win = max(1, int(round(0.25 / (grid[1] - grid[0]))))
    return np.convolve(prof, np.ones(win) / win, mode='same')


def sync_clock_offset(t_moc, p_moc, t_rob, p_rob, dt=0.05, top=8):
    """Candidate constant offsets (mac_time - robot_time) via full
    cross-correlation of the two speed profiles. Stop-and-go sessions are
    quasi-periodic, so the correlation has several similar peaks - all the
    strong local maxima are returned (best first) and the caller picks the
    one whose geometric registration fits best. -> (offsets, peak_zscore)."""
    gm = np.arange(t_moc[0], t_moc[-1], dt)
    gr = np.arange(t_rob[0], t_rob[-1], dt)
    sm = _speed_profile(t_moc, p_moc, gm)
    sr = _speed_profile(t_rob, p_rob, gr)
    # correlate the BINARY move/hold pattern, not raw speeds: mocap jitter
    # bursts (bad marker visibility) fake speeds far above real motion and
    # would dominate an amplitude correlation; the on/off timing pattern is
    # what actually identifies the alignment
    sm = (sm > max(0.005, 3 * np.median(sm))).astype(float)
    sr = (sr > max(0.005, 3 * np.median(sr))).astype(float)
    sm = (sm - sm.mean()) / (sm.std() + 1e-12)
    sr = (sr - sr.mean()) / (sr.std() + 1e-12)
    c = np.correlate(sm, sr, 'full')
    # only consider lags where the two recordings overlap substantially -
    # small-overlap lags produce spurious peaks and degenerate fits
    n_over = np.minimum.reduce([
        np.arange(1, len(c) + 1),
        np.arange(len(c), 0, -1),
        np.full(len(c), min(len(sm), len(sr)))])
    c = np.where(n_over >= 0.5 * min(len(sm), len(sr)), c, -np.inf)
    peaks = [i for i in range(1, len(c) - 1)
             if np.isfinite(c[i])
             and c[i] >= c[i - 1] and c[i] >= c[i + 1]
             and c[i] > 0.5 * c.max()]
    peaks.sort(key=lambda i: -c[i])
    offsets = []
    for i in peaks[:top]:
        shift = i - (len(gr) - 1)
        d = (c[i - 1] - c[i + 1]) / (2 * (c[i - 1] - 2 * c[i] + c[i + 1]) + 1e-12)
        offsets.append((t_moc[0] - t_rob[0]) + (shift + float(d)) * dt)
    cf = c[np.isfinite(c)]
    z = (cf.max() - cf.mean()) / (cf.std() + 1e-12)
    return offsets, float(z)


def estimate_offset(t_moc, p_moc, t_j, joints, fk_batch, r0, dec=4):
    """Clock offset: correlation candidates disambiguated by the actual
    geometric fit - a wrong-period alignment cannot fit hops of different
    lengths. r0 (marker offset from the still-window solve) matters: the
    alternating solve is non-convex, so each candidate must be scored from
    a sane start. -> (offset, corr_z, n_candidates)."""
    rate = len(t_j) / max(t_j[-1] - t_j[0], 1e-9)
    d = max(1, int(round(rate / 50)))
    p_rob, _ = fk_batch(joints[::d])
    cands, z = sync_clock_offset(t_moc, p_moc, t_j[::d], p_rob)
    if not cands:
        raise SystemExit('[traj] no correlation peak between mocap and '
                         'joint speeds - do the recordings overlap?')

    def cost(off):
        pm, pf, Rf, w = trajectory_pairs(
            t_moc, p_moc, t_j, joints, off, dec * 4, fk_batch)
        if len(pm) < 100:
            return np.inf, 0
        _, _, _, res, wt = solve_registration_weighted(
            pm, pf, Rf, w, r0, iters=2)
        return float(np.sqrt((wt * res ** 2).sum() / wt.sum())), len(pm)

    evals = [cost(o) for o in cands]
    n_max = max(n for _, n in evals)
    best = min(range(len(cands)),
               key=lambda i: evals[i][0] if evals[i][1] >= 0.6 * n_max
               else np.inf)
    return cands[best], z, len(cands)


def trajectory_pairs(t_moc, p_moc, t_j, joints, offset, stride, fk_batch):
    """Every stride-th mocap sample whose robot time is covered by the
    joint recording -> (p_moc[M], p_fl[M], R_fl[M], w_prior[M]).

    w_prior combines: slow-motion preference (sync/interp errors scale
    with speed), local mocap sample gap (dropout), and local jitter
    (high-frequency deviation from a 0.25 s moving mean)."""
    tr = t_moc - offset
    inside = (tr > t_j[0]) & (tr < t_j[-1])
    idx = np.where(inside)[0][::stride]

    # local speed (capped) on the mocap stream
    v = np.zeros(len(t_moc))
    dt = np.diff(t_moc)
    ok = dt > 1e-6
    v[1:][ok] = np.minimum(
        np.linalg.norm(np.diff(p_moc, axis=0), axis=1)[ok] / dt[ok], 1.0)
    # local gap to neighbours (dropout indicator)
    gap = np.zeros(len(t_moc))
    gap[1:-1] = np.maximum(dt[:-1], dt[1:])
    # jitter: deviation from a 0.25 s boxcar mean
    period = np.median(dt[ok]) if ok.any() else 1 / 120.0
    win = max(3, int(round(0.25 / period)) | 1)
    kern = np.ones(win) / win
    smooth = np.column_stack([
        np.convolve(p_moc[:, k], kern, mode='same') for k in range(3)])
    jit = np.linalg.norm(p_moc - smooth, axis=1)

    w = (1.0 / (1 + (v[idx] / 0.03) ** 2)
         * 1.0 / (1 + (gap[idx] / 0.05) ** 2)
         * 1.0 / (1 + (jit[idx] / 0.002) ** 2))

    ji = np.column_stack([
        np.interp(tr[idx], t_j, joints[:, k]) for k in range(7)])
    p_fl, R_fl = fk_batch(ji)
    return p_moc[idx], p_fl, R_fl, w


def solve_registration_weighted(p_moc, p_fl, R_fl, w, r0,
                                huber_mm=3.0, iters=4):
    """Weighted variant of solve_registration with Huber IRLS instead of
    a hard outlier gate. `iters` counts the outer Huber reweightings; each
    runs the (R, t) <-> r alternation to convergence like the original
    solver - a fixed handful of alternations leaves the non-convex scheme
    visibly short of the optimum. -> (R, t, r, residuals, w_total)."""
    r = np.asarray(r0, float)
    w = np.asarray(w, float)
    w_total = w.copy()
    R, t = np.eye(3), np.zeros(3)
    for _ in range(iters):
        for _ in range(200):
            tgt = p_fl + np.einsum('nij,j->ni', R_fl, r)
            ww = w_total / w_total.sum()
            ca = (ww[:, None] * p_moc).sum(0)
            cb = (ww[:, None] * tgt).sum(0)
            H = ((p_moc - ca) * ww[:, None]).T @ (tgt - cb)
            U, _, Vt = np.linalg.svd(H)
            D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
            R = Vt.T @ D @ U.T
            t = cb - R @ ca
            rhs = ((R @ p_moc.T).T + t - p_fl)
            A = np.vstack(R_fl * np.sqrt(w_total)[:, None, None])
            b = (rhs * np.sqrt(w_total)[:, None]).reshape(-1)
            r_new, *_ = np.linalg.lstsq(A, b, rcond=None)
            done = np.linalg.norm(r_new - r) < 1e-10
            r = r_new
            if done:
                break
        res = np.linalg.norm(
            (R @ p_moc.T).T + t
            - (p_fl + np.einsum('nij,j->ni', R_fl, r)), axis=1)
        w_total = w * np.minimum(1.0, (huber_mm / 1000.0) / (res + 1e-9))
    return R, t, r, res, w_total


def refine_offset(t_moc, p_moc, t_j, joints, offset0, r0, fk_batch,
                  span=0.15, steps=13, stride_mult=4):
    """1-D refinement of the clock offset by minimizing the weighted
    registration rms on a decimated pair set."""

    def cost(off):
        pm, pf, Rf, w = trajectory_pairs(
            t_moc, p_moc, t_j, joints, off, stride_mult, fk_batch)
        if len(pm) < 100:
            return np.inf
        _, _, _, res, wt = solve_registration_weighted(
            pm, pf, Rf, w, r0, iters=2)
        return float(np.sqrt((wt * res ** 2).sum() / wt.sum()))

    offs = offset0 + np.linspace(-span, span, steps)
    costs = [cost(o) for o in offs]
    i = int(np.argmin(costs))
    lo = offs[max(0, i - 1)]
    hi = offs[min(len(offs) - 1, i + 1)]
    for _ in range(12):
        m1 = lo + (hi - lo) * 0.382
        m2 = lo + (hi - lo) * 0.618
        if cost(m1) < cost(m2):
            hi = m2
        else:
            lo = m1
    return float((lo + hi) / 2)


# ---------- main ----------

def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('dataset', help='session folder (contains the photo-session bag)')
    ap.add_argument('--spin-bag', required=True,
                    help='.db3 where the turntable was spun a full turn')
    ap.add_argument('--csv', nargs='+', required=True,
                    help='manual_poses_*.csv logs of the blocks the mocap saw, '
                         'in block order (names relative to <dataset>/phone_poses_robot)')
    ap.add_argument('--block-gap', type=float, default=20.0,
                    help='phone must hold the init pose this long (s) to split blocks')
    ap.add_argument('--object-below-lens', type=float, default=0.03,
                    help='object distance straight below the init-pose lens (m), '
                         'as configured in panda_manual_poses.py')
    ap.add_argument('--joints', nargs='+', default=None,
                    help='joint_states_*.db (or .csv) from record_joint_states.py (robot '
                         'PC). Switches registration to the continuous-trajectory '
                         'solve: thousands of time-synced FK<->mocap pairs with '
                         'per-frame weights instead of still windows + hard gates')
    ap.add_argument('--traj-stride', type=int, default=0,
                    help='use every k-th mocap sample for the trajectory solve '
                         '(0 = auto, targets ~20k pairs)')
    args = ap.parse_args()

    ds = args.dataset.rstrip('/')
    bag = sorted(glob.glob(os.path.join(ds, '*.db3')))
    if not bag:
        raise SystemExit(f'no .db3 in {ds}')
    report = []

    def say(msg):
        print(msg)
        report.append(msg)

    # 1. turntable axis in the mocap frame
    _, spin = load_bag_topic(args.spin_bag, '/optitrack/popillia')
    center, normal, radius, plan_rms, rad_rms = fit_turntable_axis(spin[:, :3])
    say(f'[axis] mocap centroid circle: center {np.round(center, 4)}, '
        f'normal {np.round(normal, 4)}, radius {radius*1000:.1f} mm, '
        f'planarity rms {plan_rms*1000:.2f} mm, radial rms {rad_rms*1000:.2f} mm')

    # 2. phone stills per block
    t_ph_abs, ph = load_bag_topic(bag[0], '/optitrack/phone')
    t_ph = t_ph_abs - t_ph_abs[0]
    stills = still_windows(t_ph, ph[:, :3], ph[:, 3:7],
                           quality_gate=True, max_dropout=0.8,
                           max_std_mm=1.5, min_samples=60, report=say)
    # split into blocks: a new block starts at each return to the init pose
    # (same position as still 0) after having left it
    init_pos = stills[0][1]
    blocks = [[stills[0]]]
    for s in stills[1:]:
        if np.linalg.norm(s[1] - init_pos) < 0.005:
            blocks.append([s])
        else:
            blocks[-1].append(s)
    say(f'[stills] {len(stills)} still windows -> {len(blocks)} block(s): '
        f'{[len(b) for b in blocks]} holds')
    if len(blocks) < len(args.csv):
        say(f'[warn] fewer mocap blocks than CSVs; using first {len(blocks)} CSVs')

    # 3. correspondences
    p_moc, p_fl, R_fl, labels, T_fl_all, T_body_all = [], [], [], [], [], []
    for bi, csv_name in enumerate(args.csv[:len(blocks)]):
        T_wp, lens = load_run_csv(os.path.join(ds, 'phone_poses_robot', csv_name))
        m = match_stills_to_waypoints(T_wp, blocks[bi])
        say(f'[match] block {bi+1} ({csv_name}): still->waypoint {m}')
        for k, w in m:
            _, c, qm = blocks[bi][k]
            Tb = np.eye(4)
            Tb[:3, :3] = quat_mat(qm)
            Tb[:3, 3] = c
            p_moc.append(c)
            p_fl.append(T_wp[w][:3, 3])
            R_fl.append(T_wp[w][:3, :3])
            T_fl_all.append(T_wp[w])
            T_body_all.append(Tb)
            labels.append((bi + 1, w))
        if bi == 0:
            lens_init = lens[0]
    p_moc, p_fl, R_fl = np.array(p_moc), np.array(p_fl), np.array(R_fl)

    # 4. registration: AX=XB init, then alternating refinement + outlier pass
    r0, Rx = init_marker_offset(T_fl_all, T_body_all)
    say(f'[init] marker-body offset from AX=XB: {np.round(r0, 5)}')
    R, t, r, resid = solve_registration(p_moc, p_fl, R_fl, r0)
    keep = resid < 3 * np.median(resid)
    if keep.sum() < len(keep):
        dropped = [labels[i] for i in np.where(~keep)[0]]
        say(f'[solve] dropping outlier pair(s) {dropped} '
            f'({np.round(resid[~keep]*1000,1)} mm)')
        R, t, r, resid = solve_registration(p_moc[keep], p_fl[keep], R_fl[keep], r)
    rms = float(np.sqrt((resid ** 2).mean()))
    say(f'[solve] robot<-mocap over {len(resid)} pairs: rms {rms*1000:.2f} mm, '
        f'max {resid.max()*1000:.2f} mm')
    say(f'[solve] marker-body origin in flange frame: {np.round(r, 5)}')
    zt = R @ [0, 0, 1]
    say(f'[solve] mocap +z in robot frame: {np.round(zt, 4)} '
        f'(tilt {np.degrees(np.arccos(zt[2])):.2f} deg - both frames should be ~gravity-aligned)')

    # 4b. continuous-trajectory refinement (--joints): replace the
    # still-window estimate with the weighted whole-trajectory solve.
    method = 'still-windows'
    traj_info = None
    if args.joints:
        import panda_fk
        j_paths = [j if os.path.exists(j)
                   else os.path.join(ds, 'phone_poses_robot', j)
                   for j in args.joints]
        fkmm, fkdeg = panda_fk.validate_against_csv(
            os.path.join(ds, 'phone_poses_robot', args.csv[0]))
        say(f'[traj] FK validated against {args.csv[0]}: '
            f'worst {fkmm:.3f} mm / {fkdeg:.4f} deg')
        t_j, joints = load_joints_csv(j_paths)
        say(f'[traj] {len(t_j)} joint samples spanning '
            f'{t_j[-1] - t_j[0]:.0f} s from {len(j_paths)} file(s)')
        offset, zpeak, ncand = estimate_offset(
            t_ph_abs, ph[:, :3], t_j, joints,
            panda_fk.fk_flange_batch, r)
        say(f'[traj] clock offset (mac - robot) = {offset:.3f} s '
            f'(picked geometrically from {ncand} correlation '
            f'candidate(s), corr z = {zpeak:.1f})')
        offset = refine_offset(t_ph_abs, ph[:, :3], t_j, joints,
                               offset, r, panda_fk.fk_flange_batch)
        say(f'[traj] refined offset: {offset:.4f} s')
        stride = args.traj_stride or max(1, len(t_ph_abs) // 20000)
        pm, pf, Rf, w_prior = trajectory_pairs(
            t_ph_abs, ph[:, :3], t_j, joints, offset, stride,
            panda_fk.fk_flange_batch)
        say(f'[traj] {len(pm)} time-synced pairs (stride {stride})')
        R2, t2, r2, res2, wt = solve_registration_weighted(
            pm, pf, Rf, w_prior, r)
        wrms = float(np.sqrt((wt * res2 ** 2).sum() / wt.sum()))
        neff = float(wt.sum() / wt.max()) if len(wt) else 0.0
        say(f'[traj] weighted continuous solve: rms {wrms*1000:.2f} mm '
            f'over {len(res2)} pairs (effective {neff:.0f}); '
            f'still-window solve: {rms*1000:.2f} mm over {len(resid)}')
        say(f'[traj] marker-body origin in flange frame: '
            f'{np.round(r2, 5)} (still: {np.round(r, 5)}, '
            f'diff {np.linalg.norm(r2 - r)*1000:.1f} mm)')
        say(f'[traj] rotation vs still solve: '
            f'{rot_angle_deg(R.T @ R2):.3f} deg; '
            f'translation diff {np.linalg.norm(t2 - t)*1000:.1f} mm')
        R, t, r, resid, rms = R2, t2, r2, res2, wrms
        method = 'continuous-trajectory'
        traj_info = {
            'clock_offset_mac_minus_robot_s': float(offset),
            'corr_peak_z': float(zpeak),
            'n_pairs': int(len(res2)),
            'effective_pairs': neff,
            'stride': int(stride),
            'joints_files': [os.path.basename(j) for j in j_paths],
            'fk_validation_worst_mm': float(fkmm),
        }

    # 5. axis + object center in the robot frame
    axis_pt = R @ center + t
    axis_dir = R @ normal
    say(f'[axis] in ROBOT frame: point {np.round(axis_pt, 4)}, dir {np.round(axis_dir, 4)} '
        f'(tilt from robot +z {np.degrees(np.arccos(axis_dir[2])):.2f} deg)')
    obj_guess = lens_init - np.array([0, 0, args.object_below_lens])
    h = float((obj_guess - axis_pt) @ axis_dir)
    d_ax = float(np.linalg.norm(np.cross(obj_guess - axis_pt, axis_dir)))
    obj_on_axis = axis_pt + h * axis_dir
    say(f'[object] {args.object_below_lens*100:.0f} cm below init lens (robot): '
        f'{np.round(obj_guess, 4)}')
    say(f'[object] off-axis {d_ax*1000:.1f} mm; height above marker circle plane '
        f'{h*1000:.1f} mm (rod length cross-check)')
    say(f'[object] point ON the axis at that height: {np.round(obj_on_axis, 4)}')

    # 5b. lens offset + lens in the phone-body frame. The mount-model (mesh)
    # lens offset embedded in the CSV lens columns is TRUSTED - it is a direct
    # physical measurement, independently confirmed by the beetle-pixel solve
    # (solve_camera_axis.py: fixing the lens at the mesh ring recovers the
    # July-calibrated focal within 1.5%). The on-axis anchor below is reported
    # as a DIAGNOSTIC only: the axis direction extrapolated ~530 mm to rod-top
    # height carries +/-10-20 mm lateral uncertainty (plane-fit + registration
    # + rod lean), so a mismatch here does NOT indicate a lens-offset error.
    # (2026-08-28: an earlier version applied this delta as a "correction",
    # which pushed the lens 37 mm off the phone body - reverted.)
    lens_true_init = obj_on_axis + np.array([0, 0, args.object_below_lens])
    T_wp0, _ = load_run_csv(os.path.join(ds, 'phone_poses_robot', args.csv[0]))
    lens_fl_csv = T_wp0[0][:3, :3].T @ (lens_init - T_wp0[0][:3, 3])
    delta_fl = T_wp0[0][:3, :3].T @ (lens_true_init - lens_init)
    say(f'[lens] mesh lens offset in flange frame (trusted): {np.round(lens_fl_csv, 5)}')
    say(f'[lens] on-axis-anchor residual {np.round(delta_fl*1000, 1)} mm '
        f'(diagnostic only - dominated by axis-extrapolation uncertainty)')
    delta_fl = np.zeros(3)
    lens_moc = R.T @ (lens_init - t)
    lens_body_est = []
    for bi in range(len(blocks[:len(args.csv)])):
        _, c, qm = blocks[bi][0]  # first still of a block = init hold (wp 0)
        lens_body_est.append(quat_mat(qm).T @ (lens_moc - c))
    lens_body_est = np.array(lens_body_est)
    lens_body = lens_body_est.mean(0)
    say(f'[lens] in PHONE-BODY frame: {np.round(lens_body, 5)} '
        f'(per-block estimates agree within '
        f'{np.linalg.norm(lens_body_est - lens_body, axis=1).max()*1000:.1f} mm)')

    # 5c. shared ground plane: the Panda base and the rotation stage stand on
    # the same table (robot z=0). The marker-circle plane height above robot
    # z=0 should equal the physical marker height above the table (ruler-
    # checkable), and the frames' verticals should agree (reported above).
    say(f'[ground] popillia marker-circle plane sits {axis_pt[2]*1000:.1f} mm '
        f'above robot base z=0 (stage-base marker height - verify with a ruler)')

    # 5d. robot base in the POPILLIA BODY frame. The body rotates with the
    # stage, so this transform (anchored at block 1) plus a +60 deg/block
    # rotation about the axis maps any robot pose into an object-fixed frame -
    # reusable later without redoing the registration, as long as neither the
    # robot base nor the stage base moves on the table.
    t_po_all, po_all = load_bag_topic(bag[0], '/optitrack/popillia')
    t_po_all = t_po_all - t_po_all[0]

    def pop_body_pose(lo, hi):
        m = (t_po_all > lo) & (t_po_all < hi)
        q = np.median(po_all[m, 3:7], axis=0)
        T = np.eye(4)
        T[:3, :3] = quat_mat(q / np.linalg.norm(q))
        T[:3, 3] = np.median(po_all[m, :3], axis=0)
        return T

    # quiet middle of blocks 1 and 2 (block 2 exists whenever >= 2 CSVs matched)
    T_moc_pop1 = pop_body_pose(60, 300)
    T_moc_rob = np.eye(4)
    T_moc_rob[:3, :3] = R.T
    T_moc_rob[:3, 3] = -R.T @ t
    T_pop_rob = np.linalg.inv(T_moc_pop1) @ T_moc_rob
    beetle_moc = R.T @ (obj_on_axis - t)
    beetle_body = (np.linalg.inv(T_moc_pop1) @ np.array([*beetle_moc, 1]))[:3]
    axis_body = np.linalg.inv(T_moc_pop1)[:3, :3] @ (R.T @ axis_dir)
    say(f'[body] T_popillia<-robot (block 1): t {np.round(T_pop_rob[:3, 3], 5)}, '
        f'quat_xyzw {np.round(mat_quat(T_pop_rob[:3, :3]), 6)}')
    say(f'[body] beetle in popillia-body frame: {np.round(beetle_body, 4)} '
        f'(block-invariant); axis in body frame: {np.round(axis_body, 4)}')
    if len(blocks) >= 2:
        T_moc_pop2 = pop_body_pose(380, 600)
        bb2 = (np.linalg.inv(T_moc_pop2) @ np.array([*beetle_moc, 1]))[:3]
        say(f'[body] block1 vs block2 beetle-in-body agreement: '
            f'{np.round((beetle_body - bb2) * 1000, 2)} mm')
    say('[body] recipe: p_body(block b) = Rot(axis through beetle_in_body '
        'along axis_in_body, +(b-1)*60 deg) @ (T_popillia<-robot @ p_robot)')

    # 6. measured step angle between consecutive observed blocks (from the
    # photo-session bag popillia yaw plateaus)
    t_po, po = load_bag_topic(bag[0], '/optitrack/popillia')
    t_po = t_po - t_po[0]
    po_st = still_windows(t_po, po[:, :3], po[:, 3:7], thresh=0.002, min_dur=20.0)
    if len(po_st) >= 2:
        angs = [rot_angle_deg(quat_mat(po_st[0][2]).T @ quat_mat(s[2])) for s in po_st]
        say(f'[steps] observed popillia block orientations vs block 1: '
            f'{np.round(angs, 2)} deg (nominal 60 per step)')

    # 7. write YAML (no dependency on pyyaml formatting quirks - plain dump)
    import yaml
    out = {
        'date': __import__('datetime').datetime.now().isoformat(timespec='seconds'),
        'script': 'scripts/link_robot_mocap.py',
        'dataset': os.path.basename(ds),
        'spin_bag': args.spin_bag,
        'csvs_used': list(args.csv[:len(blocks)]),
        'T_robot_from_mocap': {
            'rotation_matrix': R.tolist(),
            'quaternion_xyzw': mat_quat(R).tolist(),
            'translation_m': t.tolist(),
        },
        'marker_body_origin_in_flange_m': r.tolist(),
        'R_flange_from_body': Rx.tolist(),
        'registration_method': method,
        'registration_rms_m': rms,
        'n_pairs': int(len(resid)),
        'trajectory': traj_info,
        'turntable_axis_mocap': {'point': center.tolist(), 'dir': normal.tolist(),
                                 'circle_radius_m': float(radius)},
        'turntable_axis_robot': {'point': axis_pt.tolist(), 'dir': axis_dir.tolist()},
        'object_center_robot_on_axis': obj_on_axis.tolist(),
        'object_center_robot_below_lens_guess': obj_guess.tolist(),
        'object_height_above_marker_plane_m': h,
        'object_guess_off_axis_m': d_ax,
        'lens_offset_in_flange_csv_m': lens_fl_csv.tolist(),
        'lens_offset_in_flange_corrected_m': (lens_fl_csv + delta_fl).tolist(),
        'lens_in_phone_body_m': lens_body.tolist(),
        'lens_note': 'lens solved from beetle-on-axis anchor at the init pose; '
                     'x/y from the turntable axis fit (~2 mm), z inherits the '
                     f'{args.object_below_lens*100:.0f} cm working-distance estimate; '
                     'valid only for the Motive phone body as defined this session',
        'marker_plane_above_robot_z0_m': float(axis_pt[2]),
        'T_popillia_from_robot_block1': {
            'rotation_matrix': T_pop_rob[:3, :3].tolist(),
            'quaternion_xyzw': mat_quat(T_pop_rob[:3, :3]).tolist(),
            'translation_m': T_pop_rob[:3, 3].tolist(),
        },
        'object_center_in_popillia_body_m': beetle_body.tolist(),
        'axis_dir_in_popillia_body': axis_body.tolist(),
        'block_rotation': '+60 deg per block about the axis through '
                          'object_center_in_popillia_body along '
                          'axis_dir_in_popillia_body (block 1 = 0 deg)',
    }
    out_path = os.path.join(ds, 'registration.yaml')
    with open(out_path, 'w') as f:
        yaml.safe_dump(out, f, sort_keys=False)
    with open(os.path.join(ds, 'registration_report.txt'), 'w') as f:
        f.write('\n'.join(report) + '\n')
    print(f'\nwrote {out_path} and registration_report.txt')


if __name__ == '__main__':
    main()
