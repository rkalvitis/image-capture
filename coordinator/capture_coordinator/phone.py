"""Client for the iPhone capture app (docs/PROTOCOL.md)."""
import time

from .jsonline import JsonLineClient


class PhoneClient:
    def __init__(self, host="127.0.0.1", port=5601, log=print):
        self.conn = JsonLineClient(host, port, name="phone")
        self.log = log
        self.ready = None          # last `ready` reply
        self.clock_offset_s = 0.0  # phone_time - mac_time
        self._hello = None

    def connect(self, retry_for=30.0):
        if not self.conn.connect(retry_for=retry_for, log=self.log):
            raise ConnectionError("phone app not reachable (is the app open and the USB tunnel up?)")

    def close(self):
        try:
            if self.conn.connected:
                self.conn.send({"type": "bye", "t": time.time()})
        except Exception:
            pass
        self.conn.close()

    def _reconnect(self):
        self.log("[phone] reconnecting...")
        self.conn.close()
        self.connect(retry_for=60.0)
        if self._hello:
            self.hello(**self._hello)

    def hello(self, session, lens_positions, settle_ms, exposure, wb):
        self._hello = dict(session=session, lens_positions=lens_positions,
                           settle_ms=settle_ms, exposure=exposure, wb=wb)
        msg = {"type": "hello", "t": time.time(), "session": session,
               "lens_positions": list(lens_positions), "settle_ms": int(settle_ms),
               "exposure": exposure, "wb": wb}
        reply = self.conn.request(msg, "ready", timeout=30.0)
        if reply.get("type") != "ready":
            raise RuntimeError(f"phone hello failed: {reply}")
        self.ready = reply
        return reply

    def ping(self, n=5):
        """Measure phone-vs-Mac clock offset by round-trip midpoint."""
        offs = []
        for _ in range(n):
            t0 = time.time()
            pong = self.conn.request({"type": "ping", "t": t0}, "pong", timeout=5.0)
            t1 = time.time()
            offs.append(float(pong["t"]) - (t0 + t1) / 2.0)
        offs.sort()
        self.clock_offset_s = offs[len(offs) // 2]
        return self.clock_offset_s

    def focus(self, lens_position, timeout=10.0):
        reply = self.conn.request({"type": "focus", "t": time.time(),
                                   "lens_position": float(lens_position)},
                                  "focus", timeout=timeout)
        if reply.get("type") != "focus":
            raise RuntimeError(f"focus failed: {reply}")
        return float(reply["lens_position"])

    def stack(self, stop_id, pose, step, lens_positions, timeout_per_frame=12.0, on_frame=None):
        """Returns the `done` message (frames list) or raises."""
        msg = {"type": "stack", "t": time.time(), "id": int(stop_id), "pose": int(pose),
               "step": int(step), "lens_positions": [float(v) for v in lens_positions]}
        timeout = timeout_per_frame * max(1, len(lens_positions)) + 330.0   # + thermal pause allowance
        try:
            reply = self.conn.request(msg, "done", timeout=timeout,
                                      on_other=lambda m: on_frame(m) if (on_frame and m.get("type") == "frame") else None)
        except (ConnectionError, TimeoutError) as err:
            raise RuntimeError(f"stack {stop_id}: {err}")
        if reply.get("type") != "done":
            raise RuntimeError(f"stack {stop_id} failed on the phone: {reply.get('reason', reply)}")
        return reply

    def status(self):
        return self.conn.request({"type": "status", "t": time.time()}, "status", timeout=5.0)
