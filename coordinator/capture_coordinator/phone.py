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

    on_reconnect = None        # optional hook (e.g. restart the USB tunnel) called before reconnecting

    def _reconnect(self):
        """The USB cable was pulled (or the app restarted): reconnect through the
        tunnel, retrying for up to 10 minutes, and re-send the last hello so the
        app applies the same session settings (exposure, WB, focus, power)."""
        self.log("[phone] connection lost — reconnecting (USB cable in? app in the foreground?) ...")
        self.conn.close()
        deadline = time.time() + 600.0
        while True:
            if self.on_reconnect:
                try:
                    self.on_reconnect()
                except Exception as err:            # noqa: BLE001
                    self.log(f"[phone] on_reconnect failed: {err}")
            if self.conn.connect(retry_for=15.0, log=self.log):
                try:
                    if self._hello:
                        self.hello(**self._hello)
                    self.ping()
                    self.log("[phone] reconnected, session settings re-applied")
                    return
                except (OSError, RuntimeError, TimeoutError) as err:
                    self.log(f"[phone] reconnect handshake failed: {err}")
                    self.conn.close()
            if time.time() > deadline:
                raise ConnectionError("phone not reachable after 10 minutes of reconnect attempts")
            time.sleep(2.0)

    def _request(self, msg, expect, timeout, on_other=None):
        """request with ONE automatic reconnect on a broken connection."""
        for attempt in (1, 2):
            try:
                return self.conn.request(msg, expect, timeout=timeout, on_other=on_other)
            except (ConnectionError, OSError) as err:
                if attempt == 2 or isinstance(err, TimeoutError):
                    raise
                self.log(f"[phone] {err}")
                self._reconnect()

    def hello(self, session, lens_positions, settle_ms, exposure, wb, power=None):
        """power: optional {raw, quality, frame_rate, screen_brightness, preview, blackout} (heat reduction)."""
        self._hello = dict(session=session, lens_positions=lens_positions,
                           settle_ms=settle_ms, exposure=exposure, wb=wb, power=power)
        msg = {"type": "hello", "t": time.time(), "session": session,
               "lens_positions": list(lens_positions), "settle_ms": int(settle_ms),
               "exposure": exposure, "wb": wb}
        msg.update({k: v for k, v in (power or {}).items() if v is not None})
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
        reply = self._request({"type": "focus", "t": time.time(),
                               "lens_position": float(lens_position)},
                              "focus", timeout)
        if reply.get("type") != "focus":
            raise RuntimeError(f"focus failed: {reply}")
        return float(reply["lens_position"])

    def focus_auto(self, timeout=15.0):
        """One-shot autofocus on the frame centre; returns the lens position the app locked at."""
        reply = self._request({"type": "focus", "t": time.time(), "lens_position": "auto"}, "focus", timeout)
        if reply.get("type") != "focus":
            raise RuntimeError(f"autofocus failed: {reply}")
        return float(reply["lens_position"])

    def stack(self, stop_id, pose, step, lens_positions, timeout_per_frame=12.0, on_frame=None):
        """Returns the `done` message (frames list) or raises."""
        msg = {"type": "stack", "t": time.time(), "id": int(stop_id), "pose": int(pose),
               "step": int(step), "lens_positions": [float(v) for v in lens_positions]}
        timeout = timeout_per_frame * max(1, len(lens_positions)) + 330.0   # + thermal pause allowance
        try:
            reply = self._request(msg, "done", timeout,
                                  on_other=lambda m: on_frame(m) if (on_frame and m.get("type") == "frame") else None)
        except (ConnectionError, TimeoutError, OSError) as err:
            raise RuntimeError(f"stack {stop_id}: {err}")
        if reply.get("type") != "done":
            raise RuntimeError(f"stack {stop_id} failed on the phone: {reply.get('reason', reply)}")
        return reply

    def status(self):
        return self._request({"type": "status", "t": time.time()}, "status", 5.0)
