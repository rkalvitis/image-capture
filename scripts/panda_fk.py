#!/usr/bin/env python3
"""Forward kinematics of the Franka Panda flange (panda_link8), ROS-free.

Used by link_robot_mocap.py to turn a recorded /joint_states trajectory
into flange poses, so registration can use the entire continuous motion
instead of still windows.

Convention: Craig modified DH as published by Franka Emika; the flange
frame is panda_link8 (the same frame panda_manual_poses.py logs). Because
DH conventions are easy to get subtly wrong, `validate_against_csv()`
checks the FK against a manual_poses_*.csv log (which contains joints AND
the flange pose captured at the same instant) and refuses to proceed on
disagreement.

Run directly for a standalone check:
    python scripts/panda_fk.py datasets/<ds>/phone_poses_robot/manual_poses_*.csv
"""

import numpy as np

# (a_{i-1}, d_i, alpha_{i-1}) per joint, then the fixed flange row.
_DH = [
    (0.0, 0.333, 0.0),
    (0.0, 0.0, -np.pi / 2),
    (0.0, 0.316, np.pi / 2),
    (0.0825, 0.0, np.pi / 2),
    (-0.0825, 0.384, -np.pi / 2),
    (0.0, 0.0, np.pi / 2),
    (0.088, 0.0, np.pi / 2),
]
_FLANGE_D = 0.107


def fk_flange(joints):
    """7 joint angles -> 4x4 pose of panda_link8 in the base frame."""
    T = np.eye(4)
    for (a, d, alpha), theta in zip(_DH, joints):
        ca, sa = np.cos(alpha), np.sin(alpha)
        ct, st = np.cos(theta), np.sin(theta)
        T = T @ np.array([
            [ct, -st, 0.0, a],
            [st * ca, ct * ca, -sa, -d * sa],
            [st * sa, ct * sa, ca, d * ca],
            [0.0, 0.0, 0.0, 1.0]])
    T = T @ np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, _FLANGE_D],
        [0.0, 0.0, 0.0, 1.0]])
    return T


def fk_flange_batch(joints_nx7):
    """[N,7] joint angles -> positions [N,3], rotations [N,3,3]."""
    P = np.empty((len(joints_nx7), 3))
    R = np.empty((len(joints_nx7), 3, 3))
    for i, j in enumerate(np.asarray(joints_nx7, float)):
        T = fk_flange(j)
        P[i] = T[:3, 3]
        R[i] = T[:3, :3]
    return P, R


def validate_against_csv(csv_path, tol_mm=1.0, tol_deg=0.1):
    """Compares FK(joints) with the logged flange pose for every row of a
    manual_poses_*.csv. Returns (worst_mm, worst_deg); raises on failure."""
    import csv as _csv
    worst_mm = worst_deg = 0.0
    with open(csv_path) as f:
        for row in _csv.DictReader(f):
            joints = [float(row['joint%d' % k]) for k in range(1, 8)]
            T = fk_flange(joints)
            p_log = np.array([float(row['position_x']),
                              float(row['position_y']),
                              float(row['position_z'])])
            q = np.array([float(row['orientation_x']),
                          float(row['orientation_y']),
                          float(row['orientation_z']),
                          float(row['orientation_w'])])
            q /= np.linalg.norm(q)
            x, y, z, w = q
            R_log = np.array([
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
            worst_mm = max(worst_mm,
                           float(np.linalg.norm(T[:3, 3] - p_log)) * 1000)
            ang = np.degrees(np.arccos(np.clip(
                (np.trace(R_log.T @ T[:3, :3]) - 1) / 2, -1, 1)))
            worst_deg = max(worst_deg, float(ang))
    if worst_mm > tol_mm or worst_deg > tol_deg:
        raise RuntimeError(
            'Panda FK does not reproduce the CSV flange poses '
            '(worst %.2f mm / %.3f deg vs tolerance %.1f mm / %.2f deg) - '
            'DH convention mismatch, do not trust the joint trajectory.'
            % (worst_mm, worst_deg, tol_mm, tol_deg))
    return worst_mm, worst_deg


if __name__ == '__main__':
    import sys
    for path in sys.argv[1:]:
        mm, deg = validate_against_csv(path)
        print('%s: FK matches logged flange poses (worst %.3f mm / %.4f deg)'
              % (path, mm, deg))
