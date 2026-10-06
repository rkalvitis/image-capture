"""Focus-stack merge on a synthetic stack with known breathing (needs cv2):
the chained-ECC alignment must recover the scale/shift, the merged image
must be sharper than every input frame, --delete-raw must only touch the
Mac copies.   python tests/test_stack_merge.py"""
import csv
import json
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "coordinator"))

try:
    import cv2                                   # noqa: F401
    import numpy as np
    HAVE_CV2 = True
except ImportError:                              # pragma: no cover
    HAVE_CV2 = False

from capture_coordinator.session import FRAME_COLUMNS   # noqa: E402


def make_session(tmp, n=7, lens_max=0.5, ref_lens=0.25, stops=(0,)):
    from capture_coordinator.synth import SyntheticStack
    synth = SyntheticStack()
    os.makedirs(os.path.join(tmp, "images")); os.makedirs(os.path.join(tmp, "sidecars"))
    rows = []
    for stop in stops:
        for k in range(n):
            lens = lens_max * k / (n - 1)
            name = f"IMG_t_{stop:04d}_{k:02d}"
            with open(os.path.join(tmp, "images", name + ".png"), "wb") as f:
                f.write(synth.frame_png(lens, ref_lens))
            intr = synth.intrinsics(lens)
            json.dump({"file": name + ".png", "intrinsics": intr, "lens_position": lens},
                      open(os.path.join(tmp, "sidecars", name + ".json"), "w"))
            rows.append({"file": name + ".png", "stop": stop, "step": 0, "pose": 1, "focus_index": k,
                         "lens_position": lens, **{c: intr[c] for c in ("fx", "fy", "cx", "cy")},
                         "capture_time": 1000 + k, "phone_x": 0.1, "phone_y": 0.2, "phone_z": 1.5,
                         "phone_quat_xyzw": "0 0 0 1", "pop_x": 0, "pop_y": 0, "pop_z": 1.4,
                         "pop_quat_xyzw": "0 0 0 1"})
    with open(os.path.join(tmp, "frames.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FRAME_COLUMNS); w.writeheader()
        for r in rows:
            w.writerow({c: r.get(c, "") for c in FRAME_COLUMNS})
    return synth, rows


@unittest.skipUnless(HAVE_CV2, "opencv not installed (pip install -r coordinator/requirements-merge.txt)")
class StackMerge(unittest.TestCase):
    def test_alignment_and_sharpness(self):
        from capture_coordinator import stack_merge as sm
        tmp = tempfile.mkdtemp(prefix="merge_")
        synth, rows = make_session(tmp, n=7)
        ref = 3                                       # lens 0.25 = the synthetic reference
        sm.merge_session(tmp, ref, fmt="png", log=lambda *a: None)
        merged = cv2.imread(os.path.join(tmp, "merged", "MERGED_0000.png"))
        self.assertIsNotNone(merged)
        self.assertEqual(merged.shape[:2], (480, 640))
        # sharper than every input frame
        best = max(synth.sharpness(cv2.imread(os.path.join(tmp, "images", r["file"]))) for r in rows)
        self.assertGreater(synth.sharpness(merged), best * 1.05)
        # alignment recovered the true breathing (scale and shift) to sub-pixel accuracy
        al = list(csv.DictReader(open(os.path.join(tmp, "merged", "alignment.csv"))))
        self.assertEqual(len(al), 7)
        for a in al:
            k = int(a["focus_index"]); lens = 0.5 * k / 6
            true_scale = synth.intrinsics(0.25)["fx"] / synth.intrinsics(lens)["fx"]
            self.assertAlmostEqual(float(a["scale_meas"]), true_scale, delta=0.003, msg=a)
            true_shift = -(lens - 0.25) * synth.shift              # the frame was shifted +, so map back -
            self.assertAlmostEqual(float(a["shift_px_meas_x"]), true_shift, delta=1.0, msg=a)
            if k != ref:
                self.assertEqual(a["method"], "ecc")
                self.assertGreater(float(a["cc"]), 0.9)
        m = list(csv.DictReader(open(os.path.join(tmp, "merged", "merged_frames.csv"))))
        self.assertEqual(len(m), 1)
        self.assertEqual(m[0]["ref_focus_index"], "3")
        self.assertAlmostEqual(float(m[0]["fx"]), synth.intrinsics(0.25)["fx"], places=3)
        self.assertEqual(m[0]["phone_quat_xyzw"], "0 0 0 1")      # pose columns inherited from the reference row
        # rerun is a no-op (already merged)
        sm.merge_session(tmp, ref, fmt="png", log=lambda *a: None)

    def test_sam3_mask_falls_back_to_threshold(self):
        from capture_coordinator import stack_merge as sm
        tmp = tempfile.mkdtemp(prefix="merge_")
        make_session(tmp, n=5, stops=(0,))
        sm.merge_session(tmp, -1, fmt="jpg", align="intrinsics", mask="sam3", sam3_python="/nonexistent/python",
                         log=lambda *a: None)
        m = list(csv.DictReader(open(os.path.join(tmp, "merged", "merged_frames.csv"))))
        self.assertEqual(m[0]["mask_method"], "sam3(failed->threshold)")      # merged anyway, with the fast mask

    def test_depth_guard(self):
        from capture_coordinator import stack_merge as sm
        # next to an edge the DEFOCUSED photo has the most energy; the depth map must still say "the sharp photo"
        sharp = np.zeros((80, 120), np.float32); sharp[:, 60:] = 1.0
        grays = {0: sharp, 1: cv2.GaussianBlur(sharp, (0, 0), 5.0)}
        e = {k: np.abs(cv2.Laplacian(g, cv2.CV_32F, ksize=3)) for k, g in grays.items()}
        self.assertGreater(e[1][40, 66], e[0][40, 66])                        # the halo the guard exists for
        depth, known = sm.focus_depth_map(grays, {0: np.eye(3), 1: np.eye(3)}, 0, 1.0)
        self.assertTrue(known[40, 66])
        self.assertLess(float(depth[40, 54:67].max()), 0.5)
        # a guarded merge still beats every input frame
        tmp = tempfile.mkdtemp(prefix="merge_")
        synth, rows = make_session(tmp, n=7)
        best = max(synth.sharpness(cv2.imread(os.path.join(tmp, "images", r["file"]))) for r in rows)
        for out, space in (("guard_linear", "linear"), ("guard_srgb", "srgb")):      # same for both fusion spaces
            sm.merge_session(tmp, 3, out_name=out, fmt="png", depth_guard=1, fusion_space=space, log=lambda *a: None)
            merged = cv2.imread(os.path.join(tmp, out, "MERGED_0000.png"))
            self.assertGreater(synth.sharpness(merged), best * 1.05, space)

    def test_intrinsics_only_and_delete_raw(self):
        from capture_coordinator import stack_merge as sm
        tmp = tempfile.mkdtemp(prefix="merge_")
        make_session(tmp, n=5, stops=(0, 1))
        sm.merge_session(tmp, 2, fmt="jpg", align="intrinsics", stops=[1], delete_raw=True, log=lambda *a: None)
        self.assertTrue(os.path.exists(os.path.join(tmp, "merged", "MERGED_0001.jpg")))
        self.assertFalse(os.path.exists(os.path.join(tmp, "merged", "MERGED_0000.jpg")))
        left = sorted(os.listdir(os.path.join(tmp, "images")))
        self.assertEqual(len(left), 5)                                  # stop 0 untouched, stop 1 deleted
        self.assertTrue(all("_0000_" in n for n in left))
        self.assertEqual(len(os.listdir(os.path.join(tmp, "sidecars"))), 10)   # sidecars never deleted
        m = list(csv.DictReader(open(os.path.join(tmp, "merged", "merged_frames.csv"))))
        self.assertEqual(m[0]["raw_deleted_on_mac"], "1")
        self.assertEqual(m[0]["align"], "intrinsics")

    def test_reference_from_focus_centre(self):
        from capture_coordinator.stack_merge import choose_reference
        frames = [{"focus_index": i, "lens_position": 0.1 * i, "focus_centre": "0.3"} for i in range(7)]
        self.assertEqual(choose_reference(frames), 3)                       # the init-focus frame
        self.assertEqual(choose_reference(frames, 5), 5)                    # explicit index wins
        for f in frames: f["focus_centre"] = ""
        self.assertEqual(choose_reference(frames), 3)                       # no centre -> middle

    def test_similarity_projection_keeps_centre(self):
        from capture_coordinator.stack_merge import _similarity_from_affine
        c = (500.0, 300.0)
        th = np.radians(0.3); R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]]) * 1.01
        W = np.eye(3); W[:2, :2] = R; W[:2, 2] = np.array(c) - R @ np.array(c)     # rotation+scale about c
        Sm = _similarity_from_affine(W, c)
        self.assertAlmostEqual(Sm[0, 0], 1.01, places=6)
        self.assertTrue(np.allclose(Sm @ [c[0], c[1], 1], [c[0], c[1], 1], atol=1e-9))   # centre fixed, no spurious shift

    def test_parallel_jobs(self):
        from capture_coordinator import stack_merge as sm
        tmp = tempfile.mkdtemp(prefix="merge_")
        make_session(tmp, n=3, stops=(0, 1, 2))
        sm.merge_session(tmp, 1, fmt="png", jobs=2, log=lambda *a: None)
        m = list(csv.DictReader(open(os.path.join(tmp, "merged", "merged_frames.csv"))))
        self.assertEqual([r["stop"] for r in m], ["0", "1", "2"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
