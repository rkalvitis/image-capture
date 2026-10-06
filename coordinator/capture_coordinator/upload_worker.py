"""Background upload of every finished stop to OneDrive (or any rclone remote) during the
session, with verification and, on request, deletion of the local and the phone copies.

    plan: upload_remote: "onedrive:SUPSI/<dataset>"   upload_what: masters | raw | all
          upload_delete_local / upload_delete_phone   (docs/ONEDRIVE.md)

Cloud layout = the user's dataset layout (OneDrive-USI/SUPSI/dataset_popillia_15_09_2026_FS, 2026-09-29):
    <remote>/raw/<frame>.dng + <frame>.json     every raw frame of every stop, flat, phone file names
    <remote>/stacked/<image>.tif                one master per stop (s<step>_p<pose>)
    <remote>/previews/<image>.jpg               the JPEG previews
    <remote>/stack_meta/<image>.json            the per-stop merge report (stack.json)
    <remote>/poses/ robot/ calibrations/ exposure_check/ + the session files (plan, csv, jsonl, summary)
(locally the session keeps images/<image>/stack/ as before.)

One worker thread; per stop (images/<image>/), in TWO phases when the merge runs inline:
  phase "raw"  right after the pull, in parallel with the merge: the stack/ (DNGs + sidecars)
               goes up and is verified; the phone copy may be deleted then (delete_phone);
               nothing local is deleted (the merge still reads the raw)
  phase "final" after the merge: copy + check of the full `upload_what` set (the master joins;
               the raw is re-checked, already there) -> local deletion as configured
Without an inline merge there is one phase ("final") right after the pull.
  1. `rclone copy`  images/<image>  <remote>/images/<image>   (filters by upload_what)
  2. `rclone check --one-way` of the same set: every local file must exist remotely with the
     same hash (OneDrive: QuickXorHash, computed locally by rclone) -> exit 0 = verified
  3. verified + delete_local: with upload_what all the whole local images/<image>/ folder is
     removed (frames, sidecars, master, preview, stack.json, mocap.csv — all in the cloud, the
     mocap samples also in poses/); masters|raw: only those files. --resume needs frames.csv only
  4. verified + delete_phone: the stop's DNG + sidecar files are removed from the phone
     (AFC over house arrest, the same connection the pull uses); only when the raw frames
     were part of the upload (upload_what raw | all — plan.py refuses the other case)
  5. events: stop_uploaded / stop_upload_failed (+ retries) -> summary.json counts
A failed copy or check is retried (3 attempts); after that the stop stays local and on the
phone, the event is logged and the session goes on. The session-level files (plan, csv,
jsonl, summary, calibrations/, robot/, exposure_check/) are synced every few minutes and
at the end together with mocap/ (-> <remote>/poses/mocap_bag/). <session>/poses/ (stops.csv,
turntable.csv, robot_poses.csv, mocap/<image>.csv, robot_runs/) is part of that sync (poses_collect.py).

If the merge of a stop failed, its files are still uploaded but NOTHING is deleted (the raw
frames are needed to merge it again).
"""
import os
import queue
import shutil
import subprocess
import sys
import threading
import time

RAW_EXT = (".dng",)
MASTER_EXT = (".tif", ".tiff", ".png", ".jpg", ".jpeg")
IMAGE_EXT = RAW_EXT + MASTER_EXT
ALWAYS_EXCLUDE = ["*.part", "*.tmp", "*.part*"]


def _match(rel, pattern):
    """rclone filter semantics for the patterns used here: a leading / anchors at the transfer
    root, otherwise the pattern matches the file name at any depth; * does not cross /."""
    import fnmatch
    if pattern.startswith("/"):
        return os.sep not in rel and fnmatch.fnmatch(rel, pattern[1:])
    return fnmatch.fnmatch(os.path.basename(rel), pattern)


def apply_rules(rel, rules):
    """First matching (+|-, pattern) rule wins; no match = included (rclone default)."""
    for sign, pat in rules:
        if _match(rel, pat):
            return sign == "+"
    return True


def filter_args(rules):
    args = []
    for sign, pat in rules:
        args += ["--filter", f"{sign} {pat}"]
    return args


def stop_specs(image, what):
    """Transfers of one stop: (local subdir, remote subdir, rules). raw = the stack/ folder ->
    raw/ (DNG + sidecar, flat); masters = the merged tif -> stacked/, jpg -> previews/."""
    specs = []
    if what in ("raw", "all"):
        specs.append(("stack", "raw", [("-", "stack.json"), ("-", "mocap.csv"), ("-", "*.part*"), ("-", "*.tmp"),
                                       ("-", "*_index.png"),                     # merge_index_maps output, not a frame
                                       ("+", "*.dng"), ("+", "*.png"), ("+", "*.json"), ("-", "*")]))   # png = dry-run frames
    if what in ("masters", "all"):
        specs.append(("", "stacked", [("-", "*.part*"), ("+", "/*.tif"), ("+", "/*.tiff"), ("-", "*")]))
        specs.append(("", "previews", [("-", "*.part*"), ("+", "/*.jpg"), ("+", "/*.jpeg"), ("-", "*")]))
    return specs


def deletable_ext(what):
    if what == "masters":
        return MASTER_EXT
    if what == "raw":
        return RAW_EXT
    return IMAGE_EXT


class UploadWorker:
    def __init__(self, session_dir, remote, what="all", delete_local=False, delete_phone=False,
                 puller=None, phone_session=None, rclone="rclone", transfers=4, bwlimit="", timeout_s=7200,
                 log=print, event=None, retries=3, retry_wait_s=20.0, session_sync_s=180.0):
        if what not in ("masters", "raw", "all"):
            raise ValueError("upload_what must be masters | raw | all")
        self.session_dir, self.remote, self.what = session_dir, remote.rstrip("/"), what
        self.delete_local = "raw" if delete_local == "raw" else bool(delete_local)
        self.delete_phone = bool(delete_phone)
        self.puller, self.phone_session = puller, phone_session
        self.rclone = rclone or "rclone"
        self.transfers, self.bwlimit, self.timeout_s = int(transfers), bwlimit or "", float(timeout_s)
        self.log = log
        self.event = event or (lambda kind, **d: None)
        self.retries, self.retry_wait_s, self.session_sync_s = int(retries), float(retry_wait_s), float(session_sync_s)
        self.q = queue.Queue()
        self.results = {}                     # stop -> dict(ok, seconds, bytes, ...)   (the final phase)
        self.raw_ok = set()                   # stops whose raw phase was verified
        self._pending_stops = {}              # stop -> number of queued/running jobs
        self.pending = 0                      # distinct stops with unfinished jobs (disk buffer accounting)
        self.bytes_uploaded = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._last_sync = 0.0
        self._thr = threading.Thread(target=self._run, daemon=True, name="upload")
        self._thr.start()

    # -- rclone ------------------------------------------------------------
    def _base_args(self):
        args = ["--retries", "3", "--low-level-retries", "20", "--stats", "0", "-q"]
        if self.bwlimit:
            args += ["--bwlimit", self.bwlimit]
        return args

    def _rc(self, *args, timeout=None):
        cmd = [self.rclone, *args]
        if self.rclone.endswith(".py"):                  # tests/fake_rclone.py
            cmd = [sys.executable, *cmd]
        try:
            res = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout or self.timeout_s)
        except FileNotFoundError:
            return 127, f"{self.rclone}: not found (install rclone, docs/ONEDRIVE.md)"
        except subprocess.TimeoutExpired:
            return 124, f"rclone timed out after {timeout or self.timeout_s:.0f} s"
        tail = (res.stderr.strip() or res.stdout.strip()).splitlines()[-3:]
        return res.returncode, " | ".join(tail)

    def check_remote(self):
        """Pre-flight at session start: the remote must answer and the folder gets created.
        Raises RuntimeError with rclone's message (token expired, tenant blocked, no rclone)."""
        if not shutil.which(self.rclone) and not os.path.exists(self.rclone):
            raise RuntimeError(f"rclone not found ({self.rclone}); install it and configure the remote (docs/ONEDRIVE.md)")
        rc, msg = self._rc("mkdir", self.remote, "--retries", "1", "--low-level-retries", "3", timeout=120)
        if rc != 0:
            raise RuntimeError(f"cannot reach {self.remote}: {msg}")
        rc, msg = self._rc("lsd", self.remote, "--retries", "1", timeout=120)
        if rc != 0:
            raise RuntimeError(f"cannot list {self.remote}: {msg}")
        return True

    def about(self):
        """Free space on the remote in bytes (None if the backend does not say)."""
        import json
        remote_root = self.remote.split(":", 1)[0] + ":" if ":" in self.remote else self.remote
        try:
            res = subprocess.run([self.rclone, "about", "--json", remote_root], capture_output=True, text=True, timeout=60)
            if res.returncode == 0:
                d = json.loads(res.stdout)
                return d.get("free"), d.get("total"), d.get("used")
        except (OSError, ValueError, subprocess.TimeoutExpired):
            pass
        return None, None, None

    # -- queue ---------------------------------------------------------------
    def submit(self, stop, image, phone_files=None, merged_ok=None, phase="final"):
        """phone_files: the stop's file names on the phone (DNG + sidecar); merged_ok: None = no
        inline merge, True/False = the merge result (False -> upload but never delete);
        phase: "raw" = right after the pull (stack/ only, no local deletion), "final" = the whole set."""
        with self._lock:
            self._pending_stops[int(stop)] = self._pending_stops.get(int(stop), 0) + 1
            self.pending = len(self._pending_stops)
        self.q.put({"stop": int(stop), "image": image, "phone_files": list(phone_files or []), "merged_ok": merged_ok,
                    "phase": phase})

    def _run(self):
        while not self._stop.is_set():
            try:
                job = self.q.get(timeout=0.2)
            except queue.Empty:
                if self._last_sync and time.time() - self._last_sync > self.session_sync_s:
                    self.upload_session_files()
                continue
            try:
                self._upload_stop(job)
            except Exception as err:                       # noqa: BLE001
                self.log(f"[upload] stop {job['stop']} ({job['phase']}): FAILED — {err}")
                self.event("stop_upload_failed", stop=job["stop"], image=job["image"], phase=job["phase"], reason=str(err))
                if job["phase"] == "final":
                    with self._lock:
                        self.results[job["stop"]] = {"ok": False, "reason": str(err)}
            finally:
                with self._lock:
                    n = self._pending_stops.get(job["stop"], 1) - 1
                    if n <= 0:
                        self._pending_stops.pop(job["stop"], None)
                    else:
                        self._pending_stops[job["stop"]] = n
                    self.pending = len(self._pending_stops)
            if time.time() - self._last_sync > self.session_sync_s:
                self.upload_session_files()

    @staticmethod
    def _spec_files(src_dir, rules):
        """(absolute path, size) of the files a spec covers (same rules as rclone)."""
        out = []
        if not os.path.isdir(src_dir):
            return out
        for root, _dirs, files in os.walk(src_dir):
            for fn in files:
                rel = os.path.relpath(os.path.join(root, fn), src_dir)
                if apply_rules(rel, rules):
                    try:
                        out.append((os.path.join(root, fn), os.path.getsize(os.path.join(root, fn))))
                    except OSError:
                        continue
        return out

    def _upload_stop(self, job):
        stop, image, phase = job["stop"], job["image"], job.get("phase", "final")
        src = os.path.join(self.session_dir, "images", image)
        dst = f"{self.remote}/images/{image}"
        if not os.path.isdir(src):
            raise RuntimeError(f"no folder {src}")
        what = "raw" if phase == "raw" else self.what
        specs = stop_specs(image, what)
        files = []
        for sub, _remote_sub, rules in specs:
            files += self._spec_files(os.path.join(src, sub) if sub else src, rules)
        n_bytes = sum(sz for _, sz in files)
        t0 = time.time()
        last = ""
        for attempt in range(1, self.retries + 1):
            ok = True
            for sub, remote_sub, rules in specs:
                src_dir = os.path.join(src, sub) if sub else src
                if not os.path.isdir(src_dir):
                    continue
                dst_dir = f"{self.remote}/{remote_sub}"
                rc, msg = self._rc("copy", src_dir, dst_dir, "--transfers", str(self.transfers), "--checkers", "8",
                                   *filter_args(rules), *self._base_args())
                if rc != 0:
                    ok, last = False, f"copy {remote_sub}/ failed: {msg}"
                    break
                rc, msg = self._rc("check", src_dir, dst_dir, "--one-way", *filter_args(rules), "--retries", "1", "-q")
                if rc != 0:
                    ok, last = False, f"check {remote_sub}/ failed: {msg}"
                    break
            if ok and phase != "raw" and what in ("masters", "all"):
                meta = os.path.join(src, "stack", "stack.json")
                if os.path.exists(meta):
                    rc, msg = self._rc("copyto", meta, f"{self.remote}/stack_meta/{image}.json", *self._base_args())
                    if rc != 0:
                        ok, last = False, f"copyto stack_meta failed: {msg}"
            if ok:
                break
            self.log(f"[upload] stop {stop} {image}: attempt {attempt} {last}")
            self.event("stop_upload_retry", stop=stop, image=image, attempt=attempt, reason=last)
            if attempt < self.retries:
                time.sleep(self.retry_wait_s)
        else:
            raise RuntimeError(last)
        dst = self.remote
        seconds = time.time() - t0
        rate = n_bytes / max(seconds, 1e-6) / 1e6
        if phase == "raw":
            # the merge still needs the local raw: nothing deleted here except (delete_phone) on the phone
            deleted_phone = 0
            if self.delete_phone and job["phone_files"] and self.puller is not None and self.what in ("raw", "all"):
                deleted_phone = self._delete_on_phone(stop, image, job["phone_files"])
            with self._lock:
                self.raw_ok.add(stop)
                self.bytes_uploaded += n_bytes
            self.log(f"[upload] stop {stop} {image}: raw {len(files)} files, {n_bytes / 1e6:.0f} MB in {seconds:.0f} s "
                     f"({rate:.1f} MB/s) -> {dst}, verified (master follows after the merge)"
                     + (f"; deleted {deleted_phone} on the phone" if deleted_phone else ""))
            self.event("stop_raw_uploaded", stop=stop, image=image, remote=dst, files=len(files), bytes=n_bytes,
                       seconds=round(seconds, 1), deleted_phone=deleted_phone)
            return
        deleted_local, deleted_phone = 0, 0
        keep_reason = None
        if job["merged_ok"] is False:
            keep_reason = "merge failed: raw frames kept locally and on the phone for a re-merge"
        elif self.delete_local == "raw":
            # Mac: the dataset folder stays (master, preview, sidecars, stack.json, mocap.csv); only the raw
            # frames go, after the merge used them and the cloud copy was verified
            for p, _ in files:
                if p.lower().endswith(RAW_EXT + (".png",)) and os.sep + "stack" + os.sep in p and not p.endswith("_index.png"):
                    try:
                        os.remove(p); deleted_local += 1
                    except OSError as e:
                        self.log(f"[upload] cannot delete {p}: {e}")
        elif self.delete_local and self.what == "all":
            # everything of this stop is in the cloud (raw + sidecars in raw/, master, preview, stack_meta) and the
            # mocap samples sit in <session>/poses/mocap/ -> the whole local images/<image>/ folder goes (user, 2026-09-29)
            deleted_local = sum(len(fs) for _r, _d, fs in os.walk(src))
            try:
                shutil.rmtree(src)
            except OSError as e:
                self.log(f"[upload] cannot delete {src}: {e}")
        elif self.delete_local:
            ext = deletable_ext(self.what) + ((".json",) if self.what == "raw" else ())    # raw: sidecars too
            for p, _ in files:
                if p.lower().endswith(ext) and os.path.basename(p) != "stack.json":
                    try:
                        os.remove(p); deleted_local += 1
                    except OSError as e:
                        self.log(f"[upload] cannot delete {p}: {e}")
        if keep_reason is None and self.delete_phone and job["phone_files"] and self.puller is not None \
                and stop not in self.raw_ok:                      # not already done by the raw phase
            if self.what in ("raw", "all"):
                deleted_phone = self._delete_on_phone(stop, image, job["phone_files"])
        with self._lock:
            new_bytes = n_bytes if stop not in self.raw_ok else sum(sz for pth, sz in files if pth.lower().endswith(MASTER_EXT))
            self.results[stop] = {"ok": True, "seconds": seconds, "bytes": n_bytes, "files": len(files),
                                  "deleted_local": deleted_local, "deleted_phone": deleted_phone}
            self.bytes_uploaded += new_bytes
        self.log(f"[upload] stop {stop} {image}: {len(files)} files, {n_bytes / 1e6:.0f} MB checked in {seconds:.0f} s "
                 f"({rate:.1f} MB/s) -> {dst}, verified"
                 + (f"; deleted {deleted_local} local" if deleted_local else "")
                 + (f", {deleted_phone} on the phone" if deleted_phone else "")
                 + (f"; {keep_reason}" if keep_reason else ""))
        self.event("stop_uploaded", stop=stop, image=image, remote=dst, what=self.what, files=len(files), bytes=n_bytes,
                   seconds=round(seconds, 1), deleted_local=deleted_local, deleted_phone=deleted_phone,
                   kept_reason=keep_reason)

    def _delete_on_phone(self, stop, image, names):
        try:
            res = self.puller.delete(self.phone_session, names)
        except Exception as err:                        # noqa: BLE001
            self.log(f"[upload] stop {stop} {image}: phone delete failed (files stay on the phone): {err}")
            self.event("phone_delete_failed", stop=stop, image=image, reason=str(err))
            return 0
        n = len(res.get("deleted", res) if isinstance(res, dict) else res)
        missing = res.get("missing", []) if isinstance(res, dict) else []
        if missing:
            self.log(f"[upload] stop {stop} {image}: {len(missing)} file(s) were not on the phone: {missing[:3]}")
        self.event("phone_deleted", stop=stop, image=image, n=n, missing=len(missing))
        return n

    # -- session-level files ----------------------------------------------------
    def upload_session_files(self, final=False, check=False):
        """plan / csv / jsonl / summary at the session root + calibrations, robot, exposure_check
        (and mocap/ at the end). rclone copies only what changed. check=True: every copied set is
        verified with `rclone check --one-way` (the condition for deleting the session folder)."""
        self._last_sync = time.time()
        excl = []
        for e in ALWAYS_EXCLUDE + ["PAUSE"]:
            excl += ["--exclude", e]
        ok = True
        rc, msg = self._rc("copy", self.session_dir, self.remote, "--max-depth", "1", *excl, *self._base_args(), timeout=600)
        if rc == 0 and check:
            rc, msg = self._rc("check", self.session_dir, self.remote, "--one-way", "--max-depth", "1", *excl, "-q", timeout=600)
        if rc != 0:
            ok = False
            self.log(f"[upload] session files: {msg}")
        subdirs = ["poses", "calibrations", "robot", "exposure_check"] + (["mocap"] if final else [])
        remote_name = {"mocap": "poses/mocap_bag"}            # the rosbag joins the poses folder in the cloud
        for sub in subdirs:
            d = os.path.join(self.session_dir, sub)
            if not os.path.isdir(d) or not any(os.scandir(d)):
                continue
            rc, msg = self._rc("copy", d, f"{self.remote}/{remote_name.get(sub, sub)}", *excl, *self._base_args(),
                               timeout=self.timeout_s)
            if rc == 0 and check:
                rc, msg = self._rc("check", d, f"{self.remote}/{remote_name.get(sub, sub)}", "--one-way", *excl, "-q",
                                   timeout=self.timeout_s)
            if rc != 0:
                ok = False
                self.log(f"[upload] {sub}/: {msg}")
        self.event("session_files_uploaded", ok=ok, final=final, subdirs=subdirs)
        return ok

    # -- flow control -------------------------------------------------------------
    def wait_for_room(self, max_pending, extra_pending=lambda: 0, log=None, log_every=30.0):
        """Block while `max_pending` or more stops are still on this disk waiting for the
        upload (+ extra_pending(), e.g. the merge queue). Returns the seconds waited."""
        t0 = time.time(); last = t0
        while True:
            with self._lock:
                n = self.pending
            n += int(extra_pending() or 0)
            if n < max_pending:
                return time.time() - t0
            if log and time.time() - last > log_every:
                log(f"[upload] buffer full: {n} stop(s) still merging/uploading — the next pull waits")
                last = time.time()
            time.sleep(0.5)

    def drain(self, timeout=None, log_every=30.0):
        t0 = time.time(); last = t0
        while True:
            with self._lock:
                n = self.pending
            if n == 0:
                return True
            if timeout is not None and time.time() - t0 > timeout:
                return False
            if time.time() - last > log_every:
                self.log(f"[upload] waiting for {n} stop(s) still uploading...")
                last = time.time()
            time.sleep(0.5)

    def close(self):
        self._stop.set()
        self._thr.join(timeout=2.0)

    def summary(self):
        with self._lock:
            ok = [v for v in self.results.values() if v.get("ok")]
            return {"uploaded_ok": len(ok), "uploaded_failed": len(self.results) - len(ok),
                    "uploaded_gb": round(sum(v.get("bytes", 0) for v in ok) / 1e9, 3),
                    "deleted_local": sum(v.get("deleted_local", 0) for v in ok),
                    "deleted_phone": sum(v.get("deleted_phone", 0) for v in ok)}


def main(argv=None):
    """Re-run the final upload phase of chosen stops (e.g. after a merge that failed inline and was
    redone by hand): python -m capture_coordinator.upload_worker <session_dir> --stops 0 1 2
    Remote, upload_what and the deletion rules come from the session's plan.json."""
    import argparse
    import json
    from .session import read_frames_csv
    ap = argparse.ArgumentParser()
    ap.add_argument("session_dir")
    ap.add_argument("--stops", type=int, nargs="+", required=True)
    ap.add_argument("--remote", help="override the plan's upload_remote")
    ap.add_argument("--no-delete", action="store_true", help="upload + check only, keep every local file")
    a = ap.parse_args(argv)
    with open(os.path.join(a.session_dir, "plan.json")) as f:
        plan = json.load(f)
    remote = a.remote or plan.get("upload_remote")
    if not remote:
        raise SystemExit("no upload_remote in plan.json (pass --remote)")
    images = {}
    for r in read_frames_csv(os.path.join(a.session_dir, "frames.csv")):
        try:
            images[int(r["stop"])] = r["image"]
        except (KeyError, ValueError):
            continue
    w = UploadWorker(a.session_dir, remote, what=plan.get("upload_what", "all"),
                     delete_local=False if a.no_delete else plan.get("upload_delete_local", False),
                     delete_phone=False, rclone=plan.get("upload_rclone", "rclone"),
                     transfers=int(plan.get("upload_transfers", 4)), bwlimit=plan.get("upload_bwlimit", ""),
                     timeout_s=float(plan.get("upload_timeout_s", 7200)), log=print)
    for stop in a.stops:
        if stop not in images:
            print(f"stop {stop}: not in frames.csv — skipped"); continue
        w.submit(stop, images[stop], [], merged_ok=True, phase="final")
    w.drain()
    w.close()
    print(w.summary())


if __name__ == "__main__":
    main()
