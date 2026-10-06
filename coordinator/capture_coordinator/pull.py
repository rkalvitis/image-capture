"""Pull DNG + sidecar files from the phone after each stop.

`UsbPuller` uses pymobiledevice3's house-arrest (app container) AFC — the
app must have file sharing enabled. `LocalDirPuller` copies from a local
folder (fake phone / dry run). `delete()` removes a stop's files from the phone —
called ONLY by the upload worker after `rclone check` verified the cloud copy
(plan `upload_delete_phone`); nothing else ever deletes on the phone.
"""
import json
import os
import shutil
import subprocess
import sys
import threading


class LocalDirPuller:
    def __init__(self, phone_docs_dir):
        self.root = phone_docs_dir

    def pull(self, session, remote_names, images_dir, sidecar_dir):
        out = []
        for name in remote_names:
            src = os.path.join(self.root, session, name)
            dst_dir = sidecar_dir if name.endswith(".json") else images_dir
            dst = os.path.join(dst_dir, name)
            shutil.copy2(src, dst + ".part")
            os.replace(dst + ".part", dst)
            out.append(dst)
        return out

    def delete(self, session, remote_names):
        deleted, missing = [], []
        for name in remote_names:
            p = os.path.join(self.root, session, name)
            if os.path.exists(p):
                os.remove(p); deleted.append(name)
            else:
                missing.append(name)
        return {"deleted": deleted, "missing": missing}


class UsbPuller:
    """Pulls through pymobiledevice3's house-arrest AFC. The library may live
    in a different interpreter (pipx venv), so the work runs in a subprocess
    with the interpreter behind the `pymobiledevice3` executable (override
    with PYMOBILEDEVICE3_PYTHON)."""

    def __init__(self, bundle_id, udid=None):
        self.bundle_id = bundle_id
        self.python = UsbPuller.find_python()
        self.worker = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pull_worker.py")
        self._lock = threading.Lock()          # the finalize thread pulls, the upload thread deletes: one at a time

    @staticmethod
    def find_python():
        env = os.environ.get("PYMOBILEDEVICE3_PYTHON")
        if env:
            return env
        try:
            import pymobiledevice3  # noqa: F401
            return sys.executable
        except ImportError:
            pass
        exe = shutil.which("pymobiledevice3")
        if exe:
            with open(exe, "rb") as f:
                first = f.readline().decode(errors="ignore").strip()
            if first.startswith("#!") and os.path.exists(first[2:].strip()):
                return first[2:].strip()
        raise RuntimeError("pymobiledevice3 not found: pipx install pymobiledevice3 (or set PYMOBILEDEVICE3_PYTHON)")

    def _run(self, *args):
        with self._lock:
            res = subprocess.run([self.python, self.worker, *args], capture_output=True, text=True, timeout=600)
        if res.returncode != 0:
            raise RuntimeError(f"pull worker failed: {res.stderr.strip().splitlines()[-1] if res.stderr.strip() else res.returncode}")
        return json.loads(res.stdout.strip().splitlines()[-1])

    def listdir(self, session):
        return self._run("list", self.bundle_id, session)["files"]

    def pull(self, session, remote_names, images_dir, sidecar_dir):
        return self._run("pull", self.bundle_id, session, images_dir, sidecar_dir, *remote_names)["pulled"]

    def delete(self, session, remote_names):
        """{"deleted": [...], "missing": [...]} — upload worker only, after the verified cloud copy."""
        return self._run("delete", self.bundle_id, session, *remote_names)
