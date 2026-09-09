#!/usr/bin/env python3
"""Fake iPhone app: speaks docs/PROTOCOL.md over TCP and writes dummy
DNG + sidecar files into <docs_dir>/<session>/ (like the app's Documents).

    python -m capture_coordinator.fake_phone --port 5601 --docs-dir /tmp/fakephone [--fail-every 5]
"""
import argparse
import json
import os
import socket
import threading
import time


class FakePhone:
    def __init__(self, docs_dir, port=5601, fail_every=0, capture_s=0.15):
        self.docs_dir, self.port, self.fail_every, self.capture_s = docs_dir, port, fail_every, capture_s
        self.session, self.lens, self.settle_ms = None, 0.5, 300
        self.n_stacks = 0
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind(("127.0.0.1", port)); self._srv.listen(2)
        self._stop = threading.Event()
        self._thr = threading.Thread(target=self._serve, daemon=True); self._thr.start()

    def _serve(self):
        self._srv.settimeout(0.5)
        while not self._stop.is_set():
            try:
                conn, _ = self._srv.accept()
            except socket.timeout:
                continue
            threading.Thread(target=self._client, args=(conn,), daemon=True).start()

    def _client(self, conn):
        buf = b""
        def send(o): conn.sendall((json.dumps(o) + "\n").encode())
        try:
            while True:
                while b"\n" not in buf:
                    chunk = conn.recv(65536)
                    if not chunk:
                        return
                    buf += chunk
                line, buf = buf.split(b"\n", 1)
                if not line.strip():
                    continue
                m = json.loads(line)
                t = m.get("type")
                if t == "hello":
                    self.session, self.settle_ms = m["session"], m.get("settle_ms", 300)
                    self.lens = float(m["lens_positions"][0])
                    os.makedirs(os.path.join(self.docs_dir, self.session), exist_ok=True)
                    send({"type": "ready", "t": time.time(), "device": "FakePhone", "lens": "ultra-wide",
                          "format": "ProRAW-fake", "free_bytes": 50 << 30, "calib_available": False})
                elif t == "ping":
                    send({"type": "pong", "t": time.time()})
                elif t == "focus":
                    self.lens = float(m["lens_position"]); time.sleep(0.05)
                    send({"type": "focus", "t": time.time(), "lens_position": self.lens})
                elif t == "status":
                    send({"type": "status", "t": time.time(), "thermal": "nominal", "free_bytes": 50 << 30,
                          "battery": 0.9, "charging": True})
                elif t == "stack":
                    self.n_stacks += 1
                    if self.fail_every and self.n_stacks % self.fail_every == 0:
                        send({"type": "error", "id": m["id"], "reason": "simulated capture failure"}); continue
                    frames = []
                    for k, lp in enumerate(m.get("lens_positions", [self.lens])):
                        self.lens = float(lp); time.sleep(self.settle_ms / 1000.0)
                        time.sleep(self.capture_s)
                        ct = time.time()
                        name = f"IMG_{self.session}_{int(m['id']):04d}_{k:02d}"
                        d = os.path.join(self.docs_dir, self.session)
                        with open(os.path.join(d, name + ".dng"), "wb") as f:
                            f.write(b"II*\x00" + os.urandom(4096))
                        side = {"file": name + ".dng", "stop": m["id"], "pose": m.get("pose"), "step": m.get("step"),
                                "focus_index": k, "lens_position": self.lens, "capture_time": ct,
                                "exposure": {"duration_s": 0.01, "iso": 64},
                                "intrinsics": {"fx": 1976.0 - 300 * self.lens, "fy": 1971.0 - 300 * self.lens,
                                               "cx": 2036.7, "cy": 1421.0, "w": 4032, "h": 3024},
                                "calibration": None, "gdc_enabled": True, "orientation": 6,
                                "device": "FakePhone", "lens": "ultra-wide", "thermal": "nominal"}
                        with open(os.path.join(d, name + ".json"), "w") as f:
                            json.dump(side, f)
                        fr = {"k": k, "file": name + ".dng", "capture_time": ct, "lens_position": self.lens,
                              "intrinsics": side["intrinsics"], "exposure": side["exposure"], "thermal": "nominal"}
                        send({"type": "frame", "id": m["id"], **fr})
                        frames.append(fr)
                    send({"type": "done", "id": m["id"], "t": time.time(), "frames": frames})
                elif t == "bye":
                    send({"type": "status", "t": time.time(), "thermal": "nominal"}); return
        except (ConnectionError, OSError):
            return
        finally:
            conn.close()

    def close(self):
        self._stop.set(); self._thr.join(timeout=2); self._srv.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", type=int, default=5601)
    ap.add_argument("--docs-dir", required=True)
    ap.add_argument("--fail-every", type=int, default=0)
    a = ap.parse_args()
    fp = FakePhone(a.docs_dir, a.port, a.fail_every)
    print(f"[fake_phone] listening on 127.0.0.1:{a.port}, files -> {a.docs_dir}")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        fp.close()


if __name__ == "__main__":
    main()
