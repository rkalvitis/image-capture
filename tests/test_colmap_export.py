"""Pose helpers + COLMAP export: synthetic ring of cameras looking at the
insect, poses round-trip through cameras.txt/images.txt, one camera per
image with its own intrinsics; the text model is validated with the colmap
binary when one is installed.   python tests/test_colmap_export.py"""
import csv
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "coordinator"))

from capture_coordinator import poses as P                      # noqa: E402
from capture_coordinator.colmap_export import export             # noqa: E402
from capture_coordinator.stack_merge import MERGED_COLUMNS       # noqa: E402


def look_at(centre, target, up=(0, 0, 1)):
    """T_world<-cam (OpenCV camera: z forward, y down) at `centre` looking at `target`."""
    z = np.asarray(target, float) - centre; z /= np.linalg.norm(z)
    x = np.cross(z, up); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, y, z, centre
    return T


class Poses(unittest.TestCase):
    def test_quat_roundtrip(self):
        rng = np.random.default_rng(1)
        for _ in range(20):
            q = rng.normal(size=4); q /= np.linalg.norm(q)
            R = P.quat_mat(q)
            q2 = P.mat_quat(R)
            self.assertTrue(np.allclose(q, q2, atol=1e-9) or np.allclose(q, -q2, atol=1e-9))
            self.assertTrue(np.allclose(P.mat_to_qwxyz(R)[1:], q2[:3]))

    def test_insect_from_cam(self):
        T_bc = P.transform([0.05, 0, -0.03], [0.35535, 0.849822, 0.357605, 0.153776])
        T_wb = P.transform([-1.0, 0.1, 1.6], [0, 0, 0.3826834, 0.9238795])
        T_wp = P.transform([-1.05, 0.05, 1.5], [0, 0, 0.7071068, 0.7071068])
        T_ic = P.insect_from_cam(T_wb[:3, 3], P.mat_quat(T_wb[:3, :3]), T_wp[:3, 3], P.mat_quat(T_wp[:3, :3]), T_bc)
        expect = np.linalg.inv(T_wp) @ T_wb @ T_bc
        self.assertTrue(np.allclose(T_ic, expect, atol=1e-9))
        q, t = P.colmap_pose(T_ic)
        R = P.quat_mat([q[1], q[2], q[3], q[0]])
        self.assertTrue(np.allclose(-R.T @ t, T_ic[:3, 3], atol=1e-9))      # centre recovered

    def test_rays_closest_point(self):
        target = np.array([0.01, -0.02, 0.5])
        rng = np.random.default_rng(2)
        cs = [target + rng.normal(size=3) * 0.1 for _ in range(8)]
        ds = [target - c for c in cs]
        self.assertTrue(np.allclose(P.rays_closest_point(cs, ds), target, atol=1e-9))


class ColmapExport(unittest.TestCase):
    def make_session(self, with_cam_cols):
        tmp = tempfile.mkdtemp(prefix="export_")
        os.makedirs(os.path.join(tmp, "merged"))
        T_bc = P.transform([0.065, -0.019, -0.048], [0.35535, 0.849822, 0.357605, 0.153776])
        T_wp = P.transform([-1.08, 0.06, 1.57], [0, 0, 0.1, 0.9949874])
        target_w = T_wp[:3, 3] + np.array([0.0, 0.0, 0.05])
        rows, truth = [], []
        for i in range(6):
            ang = np.radians(60 * i)
            c = target_w + 0.06 * np.array([np.cos(ang), np.sin(ang), 0.5])
            T_wc = look_at(c, target_w)
            T_wb = T_wc @ np.linalg.inv(T_bc)
            T_ic = np.linalg.inv(T_wp) @ T_wc
            truth.append(T_ic)
            name = f"MERGED_{i:04d}.jpg"
            with open(os.path.join(tmp, "merged", name), "wb") as f:
                f.write(b"\xff\xd8fake\xff\xd9")
            r = {"file": name, "stop": i, "step": i, "pose": 1, "ref_file": f"IMG_x_{i:04d}_20.dng",
                 "ref_focus_index": 20, "ref_lens_position": 0.26, "n_frames": 40,
                 "fx": 1760.0 + i, "fy": 1761.0 + i, "cx": 2008.3, "cy": 1513.7, "w": 4032, "h": 3024,
                 "capture_time": 1000 + i, "phone_jitter_mm": 0.05, "pop_jitter_mm": 0.04,
                 "phone_x": T_wb[0, 3], "phone_y": T_wb[1, 3], "phone_z": T_wb[2, 3],
                 "phone_quat_xyzw": P.fmt_vec(P.mat_quat(T_wb[:3, :3])),
                 "pop_x": T_wp[0, 3], "pop_y": T_wp[1, 3], "pop_z": T_wp[2, 3],
                 "pop_quat_xyzw": P.fmt_vec(P.mat_quat(T_wp[:3, :3])), "popillia_yaw_deg": 11.5,
                 "phone_world": P.fmt_vec(T_wb[:3, 3], 4)}
            if with_cam_cols:
                r.update({"cam_x": T_ic[0, 3], "cam_y": T_ic[1, 3], "cam_z": T_ic[2, 3],
                          "quat_xyzw": P.fmt_vec(P.mat_quat(T_ic[:3, :3]))})
            rows.append(r)
        with open(os.path.join(tmp, "merged", "merged_frames.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=MERGED_COLUMNS); w.writeheader()
            for r in rows:
                w.writerow({c: r.get(c, "") for c in MERGED_COLUMNS})
        rig = os.path.join(tmp, "rig.yaml")
        yaml.safe_dump({"T_body_from_cam": {"translation_m": T_bc[:3, 3].tolist(),
                                            "rotation_matrix": T_bc[:3, :3].tolist()}}, open(rig, "w"))
        return tmp, rig, truth, T_wp

    def check(self, tmp, truth, T_wp):
        sdir = os.path.join(tmp, "gs_upload", "sparse", "0")
        cams = [l.split() for l in open(os.path.join(sdir, "cameras.txt")) if not l.startswith("#")]
        self.assertEqual(len(cams), 6)
        self.assertEqual(cams[2][1], "PINHOLE")
        self.assertAlmostEqual(float(cams[2][4]), 1762.0)                       # per-image fx
        self.assertAlmostEqual(float(cams[2][6]), 2008.3)                       # true cx, no padding
        imgs = [l.split() for l in open(os.path.join(sdir, "images.txt")) if len(l.split()) == 10 and not l.startswith("#")]
        self.assertEqual(len(imgs), 6)
        for t, T_ic in zip(imgs, truth):
            self.assertEqual(int(t[0]), int(t[8]))                               # camera id = image id
            q = [float(v) for v in t[1:5]]; tr = np.array([float(v) for v in t[5:8]])
            R = P.quat_mat([q[1], q[2], q[3], q[0]])
            self.assertTrue(np.allclose(-R.T @ tr, T_ic[:3, 3], atol=1e-6))
            self.assertTrue(np.allclose(R.T, T_ic[:3, :3], atol=1e-5))   # quats are written with 6 decimals
        pts = np.array([[float(v) for v in l.split()[1:4]] for l in open(os.path.join(sdir, "points3D.txt")) if not l.startswith("#")])
        target_i = (np.linalg.inv(T_wp) @ np.append(T_wp[:3, 3] + [0, 0, 0.05], 1))[:3]
        self.assertLess(np.linalg.norm(pts.mean(axis=0) - target_i), 0.002)      # cloud centred on the beetle
        self.assertTrue(os.path.exists(os.path.join(tmp, "gs_upload", "images", "MERGED_0003.jpg")))
        cp = list(csv.DictReader(open(os.path.join(tmp, "gs_upload", "camera_poses.csv"))))
        self.assertEqual(cp[0]["image"], "MERGED_0000")
        self.assertEqual(cp[0]["popillia_yaw_deg"], "11.5")
        colmap = shutil.which("colmap")
        if colmap:
            out = tempfile.mkdtemp(prefix="colmap_bin_")
            res = subprocess.run([colmap, "model_converter", "--input_path", sdir, "--output_path", out,
                                  "--output_type", "BIN"], capture_output=True, text=True)
            self.assertEqual(res.returncode, 0, res.stderr[-500:])
            self.assertTrue(os.path.exists(os.path.join(out, "images.bin")))

    def test_export_from_cam_columns(self):
        tmp, rig, truth, T_wp = self.make_session(with_cam_cols=True)
        export(tmp, log=lambda *a: None)
        self.check(tmp, truth, T_wp)

    def test_export_renders_tiff_to_png(self):
        try:
            import cv2
        except ImportError:
            self.skipTest("opencv not installed")
        tmp, rig, truth, T_wp = self.make_session(with_cam_cols=True)
        # replace the fake jpgs by 16-bit linear tiffs (what the merge writes by default)
        rows = list(csv.DictReader(open(os.path.join(tmp, "merged", "merged_frames.csv"))))
        for r in rows:
            os.remove(os.path.join(tmp, "merged", r["file"]))
            r["file"] = r["file"][:-4] + ".tif"
            im = np.full((30, 40, 3), 0.2 * 65535, np.uint16)
            cv2.imwrite(os.path.join(tmp, "merged", r["file"]), im)
        with open(os.path.join(tmp, "merged", "merged_frames.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=MERGED_COLUMNS); w.writeheader(); w.writerows(rows)
        export(tmp, log=lambda *a: None)
        png = cv2.imread(os.path.join(tmp, "gs_upload", "images", "MERGED_0001.png"))
        self.assertIsNotNone(png)
        self.assertAlmostEqual(int(png[0, 0, 0]), 124, delta=2)            # linear 0.2 -> sRGB ~0.485
        imgs = [l.split() for l in open(os.path.join(tmp, "gs_upload", "sparse", "0", "images.txt")) if len(l.split()) == 10]
        self.assertEqual(imgs[1][9], "MERGED_0001.png")

    def test_export_from_body_poses_with_rig(self):
        tmp, rig, truth, T_wp = self.make_session(with_cam_cols=False)
        with self.assertRaises(RuntimeError):
            export(tmp, log=lambda *a: None)                # no cam columns and no rig
        export(tmp, rig=rig, log=lambda *a: None)
        self.check(tmp, truth, T_wp)


if __name__ == "__main__":
    unittest.main(verbosity=2)
