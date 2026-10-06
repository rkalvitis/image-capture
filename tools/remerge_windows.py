#!/usr/bin/env python3
"""Re-merge EVERY focus stack of a dataset from its cloud `raw/` folder with the 2026-10-06 settings:
alignment fitted on the beetle, fusion in gamma (sRGB) space, depth guard 4. Written for a Windows PC
with the OneDrive client (works on macOS/Linux too).

    py tools\\remerge_windows.py --raw "C:\\Users\\<you>\\OneDrive - USI\\SUPSI\\dataset_popillia_2026-09-30\\raw" --out "D:\\popillia_gamma"

Without --raw / --out the script asks for them. Try 2 stops first:  ... --limit 2

Input  (never modified, never deleted): <raw>/IMG_<session>_<stop>_<frame>.dng + .json, flat, as uploaded by
        the coordinator. Nothing else of the dataset is needed: step, pose, lens position and intrinsics come
        from the sidecars.
Output (the folder YOU choose, same layout as the dataset's cloud folder):
        <out>/stacked/<image>.tif       16-bit sRGB master, image = s<step>_p<pose>
        <out>/previews/<image>.jpg
        <out>/stack_meta/<image>.json   merge report (reference photo, alignment, sharpness per photo, options)
        <out>/masks/<image>_*.png       beetle mask and depth map used by the merge (to check)
        <out>/stacks.csv, alignment.csv, remerge_log.txt, README_remerge.md

OneDrive: cloud-only files are downloaded by OneDrive when the script reads them, one stop at a time
(about 0.75 GB per stop). Files that were cloud-only before are made cloud-only again after their stop was
merged and verified (`attrib +U -P`, Windows only), so the disk never holds more than the stops in work.
--keep-local leaves them downloaded (the whole raw/ folder is 351 GB). Files that were already on the disk
are left as they are.

Size and time for dataset_popillia_2026-09-30: 468 stops x 50 photos -> about 31 GB of masters; on the Mac one
stop takes ~110 s, three in parallel ~55 s per stop = about 7 h, plus the download.

Resume: run the same command again; stops whose master already exists in <out> are skipped.

Setup on the PC (once): Python 3.10+ from python.org, then
    py -m pip install numpy opencv-python rawpy
and copy the repo folders `coordinator/` and `tools/` next to each other (this file imports
coordinator/capture_coordinator/stack_merge.py).
"""
import argparse
import csv
import json
import multiprocessing as mp
import os
import re
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "coordinator"))

DATASET = "dataset_popillia_2026-09-30"
FRAME_RE = re.compile(r"^IMG_(\d{8}_\d{6})_(\d{4})_(\d{2})\.dng$", re.IGNORECASE)
ONLINE_ONLY = 0x00400000 | 0x00040000 | 0x00001000      # RECALL_ON_DATA_ACCESS | RECALL_ON_OPEN | OFFLINE
MASTER_MB = 67                                          # one 4032x3024 16-bit TIFF


def default_raw():
    """The dataset's raw/ folder inside the OneDrive folder of this computer, or ""."""
    roots = [os.environ.get(v) for v in ("OneDriveCommercial", "OneDrive")]
    roots.append(os.path.expanduser("~/Library/CloudStorage/OneDrive-USI"))
    for r in roots:
        p = os.path.join(r, "SUPSI", DATASET, "raw") if r else ""
        if p and os.path.isdir(p):
            return p
    return ""


def ask(prompt, default=""):
    if not sys.stdin.isatty():
        return default
    v = input(f"{prompt}{f' [{default}]' if default else ''}: ").strip().strip('"')
    return v or default


def scan(raw):
    """{stop: [(frame index, dng name)]} from the file NAMES only (no file is opened, nothing downloads),
    plus the total size of the DNGs and the names without a sidecar."""
    stops, names, size = {}, set(), 0
    with os.scandir(raw) as it:
        for e in it:
            names.add(e.name)
            m = FRAME_RE.match(e.name)
            if m:
                stops.setdefault(int(m.group(2)), []).append((int(m.group(3)), e.name))
                try:
                    size += e.stat().st_size
                except OSError:
                    pass
    orphans = [n for fr in stops.values() for _, n in fr if os.path.splitext(n)[0] + ".json" not in names]
    for fr in stops.values():
        fr.sort()
    return stops, size, orphans


def online_only(path):
    try:
        return bool(os.stat(path).st_file_attributes & ONLINE_ONLY)
    except (AttributeError, OSError):                    # not Windows / not there
        return False


def fetch(path, tries=6):
    """Read the whole file once: OneDrive downloads a cloud-only file here, with retries, instead of failing
    somewhere inside the raw decoder."""
    err = None
    for i in range(tries):
        try:
            with open(path, "rb") as f:
                while f.read(8 << 20):
                    pass
            return
        except OSError as e:
            err = e
            time.sleep(5 * (i + 1))
    raise OSError(f"cannot read {path} (OneDrive not signed in / offline?): {err}")


def free_local_copy(paths):
    """Make these files cloud-only again (their content stays in OneDrive). Windows only."""
    if os.name != "nt":
        return
    for p in paths:
        subprocess.run(["attrib", "+U", "-P", p], capture_output=True)


def out_paths(out, image):
    return {"tif": os.path.join(out, "stacked", image + ".tif"),
            "jpg": os.path.join(out, "previews", image + ".jpg"),
            "meta": os.path.join(out, "stack_meta", image + ".json")}


def work(job):
    """One stop, in a worker process. Returns a dict; never raises (one bad stop must not end the run)."""
    t0 = time.time()
    stop, raw, out = job["stop"], job["raw"], job["out"]
    res = {"stop": stop, "image": "", "row": None, "align": [], "error": "", "skipped": False, "seconds": 0.0}
    try:
        from capture_coordinator import stack_merge as sm
        from capture_coordinator.session import stack_name
        dngs = [os.path.join(raw, n) for _, n in job["frames"]]
        jsons = [os.path.splitext(p)[0] + ".json" for p in dngs]
        was_cloud = [p for p in dngs + jsons if online_only(p)]
        rows, dst = [], None
        for (k, name), jp in zip(job["frames"], jsons):
            fetch(jp)
            with open(jp) as f:
                sc = json.load(f)
            intr = sc["intrinsics"]
            rows.append({"file": name, "raw_dir": raw, "stop": stop, "step": sc["step"], "pose": sc["pose"],
                         "image": stack_name(sc["step"], sc["pose"]), "focus_index": sc.get("focus_index", k),
                         "lens_position": sc["lens_position"], "capture_time": sc.get("capture_time", ""),
                         **{c: intr[c] for c in ("fx", "fy", "cx", "cy")}})
            if dst is None:                              # the first sidecar names the image: done already?
                image = res["image"] = rows[0]["image"]
                dst = out_paths(out, image)
                if not job["redo"] and os.path.exists(dst["tif"]) and os.path.exists(dst["meta"]):
                    res["skipped"] = True
                    return res
        if len({r["image"] for r in rows}) != 1:
            raise ValueError(f"the sidecars of stop {stop} name several images: {sorted({r['image'] for r in rows})}")
        for p in dngs:
            fetch(p)
        wdir = os.path.join(out, "_work")                # _merge_one writes <wdir>/images/<image>/...
        idir = os.path.join(wdir, "images", image)
        os.makedirs(os.path.join(idir, "stack"), exist_ok=True)
        mjob = {"session_dir": wdir, "stop": stop, "rows": rows, "fmt": "tif", "out_dir": wdir, "ref_index": None,
                "align": "ecc", "levels": 6, "half": False, "roi": 1.0, "fusion": "pyramid", "consistency": 1,
                "index_maps": False, "preview": True,
                "delete_raw": False,                     # NEVER: raw_dir is the OneDrive folder
                "mask": job["mask"], "sam3_python": job["sam3_python"], "sam3_prompt": "insect",
                "align_on": "beetle", "min_rel_sharpness": 0.0,
                "depth_guard": job["depth_guard"], "fusion_space": job["fusion_space"]}
        _, row, extra = sm._merge_one(mjob)
        if row is None:
            raise RuntimeError(str(extra))
        with open(os.path.join(idir, "stack", "stack.json")) as f:
            meta = json.load(f)
        if not meta.get("verified"):
            raise RuntimeError("the master was written but could not be read back")
        for d in ("stacked", "previews", "stack_meta", "masks"):
            os.makedirs(os.path.join(out, d), exist_ok=True)
        for n in os.listdir(os.path.join(idir, "stack")):
            if n.endswith(".png"):
                os.replace(os.path.join(idir, "stack", n), os.path.join(out, "masks", n))
        os.replace(os.path.join(idir, image + ".jpg"), dst["jpg"])
        os.replace(os.path.join(idir, "stack", "stack.json"), dst["meta"])
        os.replace(os.path.join(idir, image + ".tif"), dst["tif"])       # last: its presence = "this stop is done"
        shutil.rmtree(idir, ignore_errors=True)
        if job["free"]:
            free_local_copy(was_cloud)
        res["row"], res["align"] = row, extra
    except Exception as e:                               # noqa: BLE001
        res["error"] = f"{type(e).__name__}: {e}"
    res["seconds"] = time.time() - t0
    return res


def read_csv(path, key):
    if not os.path.exists(path):
        return {}
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    out = {}
    for r in rows:
        out.setdefault(int(r[key]), []).append(r)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--raw", default="", help="the dataset's raw/ folder inside OneDrive (asked if omitted)")
    ap.add_argument("--out", default="", help="output folder of your choice (asked if omitted)")
    ap.add_argument("--jobs", type=int, default=3, help="stops merged in parallel, each ~3 GB RAM (default 3)")
    ap.add_argument("--stops", type=int, nargs="*", help="only these stop numbers (the 4 digits in the file names)")
    ap.add_argument("--limit", type=int, default=0, help="only the first N stops (a test run)")
    ap.add_argument("--fusion-space", choices=["srgb", "linear"], default="srgb", help="srgb = gamma space (default)")
    ap.add_argument("--depth-guard", type=float, default=4.0,
                    help="a pixel takes detail only from photos within this many focus steps of its sharpest photo "
                         "(default 4; 0 = off)")
    ap.add_argument("--mask", choices=["threshold", "sam3"], default="threshold",
                    help="beetle mask for the anchor photo and the alignment: threshold = built in (default), "
                         "sam3 = SAM 3 (needs --sam3-python)")
    ap.add_argument("--sam3-python", default=None, help="python.exe of an environment with torch + transformers + SAM 3")
    ap.add_argument("--min-frames", type=int, default=10, help="skip stops with fewer photos (exposure-check shots)")
    ap.add_argument("--keep-local", action="store_true", help="do not make the raw files cloud-only again")
    ap.add_argument("--redo", action="store_true", help="merge again even if the master exists in --out")
    ap.add_argument("--list", action="store_true", help="only show what would be merged; opens no file")
    a = ap.parse_args(argv)

    raw = a.raw or ask("raw folder of the dataset in OneDrive", default_raw())
    if not raw:
        sys.exit("no raw folder given (--raw)")
    raw = os.path.abspath(raw)
    if not os.path.isdir(raw):
        sys.exit(f"raw folder not found: {raw}")
    stops, raw_bytes, orphans = scan(raw)
    small = sorted(s for s, fr in stops.items() if len(fr) < a.min_frames)
    todo = sorted(s for s in stops if s not in small and (not a.stops or s in set(a.stops)))
    if a.limit:
        todo = todo[:a.limit]
    sizes = sorted({len(stops[s]) for s in todo})
    print(f"raw: {raw}\n  {sum(len(v) for v in stops.values())} photos, {raw_bytes / 1e9:.0f} GB, {len(stops)} stops; "
          f"{len(todo)} to merge ({'/'.join(map(str, sizes))} photos each)"
          + (f"; skipped (fewer than {a.min_frames} photos): {small}" if small else ""))
    if orphans:
        sys.exit(f"{len(orphans)} DNG without a sidecar JSON, e.g. {orphans[:3]}: is OneDrive still syncing?")
    if a.list:
        for s in todo:
            print(f"  stop {s:04d}: {len(stops[s])} photos  {stops[s][0][1]} .. {stops[s][-1][1]}")
        return 0
    if not todo:
        sys.exit("nothing to merge")

    out = a.out or ask("output folder (your choice)")
    if not out:
        sys.exit("no output folder given (--out)")
    out = os.path.abspath(out)
    if os.name == "nt" and not out.isascii():
        sys.exit(f"the output path has non-ASCII characters, OpenCV cannot write there on Windows: {out}")
    if (out + os.sep).startswith(raw + os.sep):
        sys.exit("the output folder must not be inside raw/")
    os.makedirs(out, exist_ok=True)
    if os.name == "nt" and not raw.isascii():
        print("WARNING: the raw path has non-ASCII characters; if every stop fails to decode, map it to a drive letter")

    import cv2                                           # noqa: F401  fail here, not in every worker
    import rawpy                                         # noqa: F401
    from capture_coordinator import stack_merge as sm
    if a.mask == "sam3" and not (a.sam3_python and os.path.exists(a.sam3_python)):
        sys.exit("--mask sam3 needs --sam3-python <python.exe with SAM 3>")

    free = not a.keep_local and os.name == "nt"
    need = len(todo) * MASTER_MB / 1e3 + 2.0
    have = shutil.disk_usage(out).free / 1e9
    print(f"out: {out}\n  about {len(todo) * MASTER_MB / 1e3:.0f} GB of masters, {have:.0f} GB free there")
    if have < need:
        sys.exit(f"not enough space in the output folder's drive: {need:.0f} GB needed")
    if os.name == "nt" and not free:
        rfree = shutil.disk_usage(raw).free / 1e9
        print(f"  --keep-local: OneDrive will keep up to {raw_bytes / 1e9:.0f} GB of raw on its drive ({rfree:.0f} GB free)")

    csv_path, align_path = os.path.join(out, "stacks.csv"), os.path.join(out, "alignment.csv")
    done, aligned = read_csv(csv_path, "stop"), read_csv(align_path, "stop")
    opts = {"fusion_space": a.fusion_space, "depth_guard": a.depth_guard, "mask": a.mask, "align_on": "beetle",
            "fusion": "pyramid", "format": "tif (16-bit, sRGB-encoded)"}
    with open(os.path.join(out, "README_remerge.md"), "w") as f:
        f.write(f"# Re-merged focus stacks\n\nMade by `tools/remerge_windows.py` (image-capture repo), started "
                f"{time.strftime('%Y-%m-%d %H:%M')}.\n\n- source (not modified): `{raw}`\n- options: `{json.dumps(opts)}`\n"
                f"- layout: `stacked/<image>.tif`, `previews/<image>.jpg`, `stack_meta/<image>.json`, "
                f"`masks/<image>_*.png`, `stacks.csv`, `alignment.csv`, `remerge_log.txt`\n"
                f"- image = `s<step>_p<pose>`; the pose columns of `stacks.csv` are empty (the sidecars carry no "
                f"mocap): take them from the dataset's own frames.csv / poses.\n")
    logf = open(os.path.join(out, "remerge_log.txt"), "a")

    def log(msg):
        line = f"{time.strftime('%H:%M:%S')} {msg}"
        print(line, flush=True)
        logf.write(line + "\n")
        logf.flush()

    jobs = [{"stop": s, "frames": stops[s], "raw": raw, "out": out, "redo": a.redo, "free": free, "mask": a.mask,
             "sam3_python": a.sam3_python, "depth_guard": a.depth_guard, "fusion_space": a.fusion_space} for s in todo]
    log(f"start: {len(jobs)} stops, {a.jobs} in parallel, options {opts}"
        + (", raw files go back to cloud-only after each stop" if free else ""))
    t0, n_ok, n_skip, failed = time.time(), 0, 0, []

    def results():
        if a.jobs > 1 and len(jobs) > 1:
            with mp.get_context("spawn").Pool(a.jobs) as pool:
                yield from pool.imap_unordered(work, jobs)
        else:
            yield from map(work, jobs)

    try:
        for i, r in enumerate(results(), 1):
            if r["skipped"]:
                n_skip += 1
                continue
            if r["error"]:
                failed.append(r["stop"])
                log(f"[{i}/{len(jobs)}] stop {r['stop']:04d} {r['image']} FAILED: {r['error']}")
                continue
            n_ok += 1
            done[r["stop"]], aligned[r["stop"]] = [r["row"]], r["align"]
            sm._write_csv(csv_path, sm.MERGED_COLUMNS, [x for s in sorted(done) for x in done[s]])
            sm._write_csv(align_path, sm.ALIGN_COLUMNS, [x for s in sorted(aligned) for x in aligned[s]])
            left = (len(jobs) - i) * (time.time() - t0) / max(1, n_ok)
            log(f"[{i}/{len(jobs)}] stop {r['stop']:04d} -> {r['image']}.tif in {r['seconds']:.0f} s "
                f"(anchor photo {r['row']['ref_focus_index']}, min cc {r['row']['align_min_cc']}, "
                f"mask {r['row']['mask_method']}); about {left / 3600:.1f} h left")
    except KeyboardInterrupt:
        log("interrupted; run the same command again to continue")
    shutil.rmtree(os.path.join(out, "_work"), ignore_errors=True)
    log(f"done: {n_ok} merged, {n_skip} already there, {len(failed)} failed"
        + (f" (stops {failed}; run again to retry them)" if failed else "")
        + f", {(time.time() - t0) / 3600:.2f} h")
    return 1 if failed else 0


if __name__ == "__main__":
    mp.freeze_support()
    sys.exit(main())
