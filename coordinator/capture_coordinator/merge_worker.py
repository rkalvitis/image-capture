"""Background focus-stack merge during a session (plan: merge_inline).

The coordinator submits every verified stop; `jobs` worker threads merge
the stops in parallel, each in its own subprocess (stack_merge.py,
optionally with a different interpreter that has rawpy/dtcwt), so the
capture loop is never blocked. With merge_delete_raw the merge removes
that stop's DNGs from the Mac session folder after the merged image was
written and read back — the phone keeps everything (it never deletes),
sidecars stay on the Mac too.

Measured 2026-09-15 on the 9 Sep rig stacks (50 frames, full res): pyramid
fusion ~90 s per stop, wavelet fusion ~4-5 min; the capture cycle is ~85 s
per stop, so the wavelet needs 3-4 parallel jobs to keep up.
"""
import os
import queue
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))


class MergeWorker:
    def __init__(self, session_dir, ref_index, python=None, delete_raw=False, log=print, extra_args=(), jobs=1):
        self.session_dir, self.ref_index = session_dir, int(ref_index)
        self.python = python or sys.executable
        self.delete_raw, self.log, self.extra = delete_raw, log, list(extra_args)
        self.q = queue.Queue()
        self.results = {}          # stop -> (ok, seconds, message)
        self.pending = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._threads = [threading.Thread(target=self._run, daemon=True, name=f"merge-{i}") for i in range(max(1, int(jobs)))]
        for t in self._threads:
            t.start()

    def submit(self, stop):
        with self._lock:
            self.pending += 1
        self.q.put(int(stop))

    def _cmd(self, stop):
        cmd = [self.python, "-m", "capture_coordinator.stack_merge", self.session_dir, "--stops", str(stop)] + self.extra
        if self.ref_index >= 0:
            cmd += ["--ref-index", str(self.ref_index)]
        if self.delete_raw:
            cmd.append("--delete-raw")
        return cmd

    def _run(self):
        env = dict(os.environ)
        env["PYTHONPATH"] = os.path.dirname(HERE) + os.pathsep + env.get("PYTHONPATH", "")
        env.setdefault("OMP_NUM_THREADS", "2")
        env.setdefault("OPENBLAS_NUM_THREADS", "2")
        while not self._stop.is_set():
            try:
                stop = self.q.get(timeout=0.2)
            except queue.Empty:
                continue
            t0 = time.time()
            try:
                res = subprocess.run(self._cmd(stop), capture_output=True, text=True, env=env, timeout=3600)
                ok = res.returncode == 0
                msg = (res.stdout.strip().splitlines() or [""])[-1] if ok else \
                    (res.stderr.strip().splitlines() or [f"exit {res.returncode}"])[-1]
            except Exception as err:                # noqa: BLE001
                ok, msg = False, str(err)
            with self._lock:
                self.results[stop] = (ok, time.time() - t0, msg)
                self.pending -= 1
            self.log(f"[merge] stop {stop}: {'ok' if ok else 'FAILED'} in {time.time() - t0:.0f}s — {msg}")

    def wait_for_room(self, max_pending, log=None, log_every=30.0):
        """Block while `max_pending` or more stops are still unmerged (the raw
        frames of those are on the Mac). Returns the seconds waited."""
        t0 = time.time(); last = t0
        while True:
            with self._lock:
                n = self.pending
            if n < max_pending:
                return time.time() - t0
            if log and time.time() - last > log_every:
                log(f"[merge] buffer full: {n} stacks on the Mac still merging — the next pull waits")
                last = time.time()
            time.sleep(0.5)

    def drain(self, timeout=None, log_every=30.0):
        """Block until every submitted stop is merged (or timeout)."""
        t0 = time.time(); last = t0
        while True:
            with self._lock:
                n = self.pending
            if n == 0:
                return True
            if timeout is not None and time.time() - t0 > timeout:
                return False
            if time.time() - last > log_every:
                self.log(f"[merge] waiting for {n} stop(s) still merging...")
                last = time.time()
            time.sleep(0.5)

    def close(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=2.0)

    def summary(self):
        with self._lock:
            ok = sum(1 for v in self.results.values() if v[0])
            return {"merged_ok": ok, "merged_failed": len(self.results) - ok}
