"""Rigid-transform helpers shared by the coordinator and the exporters.

Conventions (same as scripts/link_robot_mocap.py / prepare_ring_gs.py in the
optitrack repo): quaternions are xyzw unless the name says otherwise; a pose
is a 4x4 T_world<-body; the camera frame is OpenCV (x right, y down in the
sensor-native 4032x3024 image, z = optical axis).

Camera pose chain (mocap route, no robot needed):
    T_world<-cam = T_world<-phone_body @ T_body<-cam        (T_body<-cam = hand-eye, rig_transforms.yaml)
    T_insect<-cam = inv(T_world<-popillia) @ T_world<-cam    (what camera_poses.csv / COLMAP use)
"""
import numpy as np


def quat_mat(q_xyzw):
    x, y, z, w = np.asarray(q_xyzw, float)
    n = np.linalg.norm([x, y, z, w])
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat_quat(R):
    """Rotation matrix -> xyzw."""
    R = np.asarray(R, float)
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


def mat_to_qwxyz(R):
    """Rotation matrix -> (w, x, y, z), COLMAP order."""
    q = mat_quat(R)
    return np.array([q[3], q[0], q[1], q[2]])


def transform(pos, q_xyzw):
    T = np.eye(4)
    T[:3, :3] = quat_mat(q_xyzw)
    T[:3, 3] = np.asarray(pos, float)
    return T


def load_rig(path):
    """T_body<-cam (4x4) from rig_transforms.yaml (key T_body_from_cam with
    rotation_matrix or quaternion_xyzw, and translation_m)."""
    import yaml
    with open(path) as f:
        d = yaml.safe_load(f)
    tbc = d["T_body_from_cam"]
    T = np.eye(4)
    if "rotation_matrix" in tbc:
        T[:3, :3] = np.asarray(tbc["rotation_matrix"], float)
    else:
        T[:3, :3] = quat_mat(tbc["quaternion_xyzw"])
    T[:3, 3] = np.asarray(tbc["translation_m"], float)
    return T


def insect_from_cam(phone_pos, phone_q, pop_pos, pop_q, T_body_cam):
    """T_insect<-cam from the two body poses (world) and the hand-eye."""
    T_wb = transform(phone_pos, phone_q)
    T_wp = transform(pop_pos, pop_q)
    T_wc = T_wb @ T_body_cam
    return np.linalg.inv(T_wp) @ T_wc


def fmt_vec(v, nd=6):
    return " ".join(f"{float(x):.{nd}f}" for x in v)


def parse_vec(s):
    return np.array([float(x) for x in str(s).split()], float)


def colmap_pose(T_ic, scale=1.0):
    """T_insect<-cam -> (qw qx qy qz, tx ty tz) of the world->cam transform COLMAP wants."""
    R = T_ic[:3, :3].T
    t = -R @ (T_ic[:3, 3] * scale)
    return mat_to_qwxyz(R), t


def rays_closest_point(centres, dirs):
    """Least-squares point closest to a set of lines (camera centres + optical axes)."""
    A = np.zeros((3, 3)); b = np.zeros(3)
    for c, d in zip(centres, dirs):
        d = np.asarray(d, float); d = d / np.linalg.norm(d)
        P = np.eye(3) - np.outer(d, d)
        A += P; b += P @ np.asarray(c, float)
    return np.linalg.lstsq(A, b, rcond=None)[0]
