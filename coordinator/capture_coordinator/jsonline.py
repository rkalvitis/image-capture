"""Newline-delimited JSON over TCP, client side, with reconnect."""
import json
import socket
import threading
import time


class JsonLineClient:
    def __init__(self, host, port, name="peer", connect_timeout=5.0):
        self.host, self.port, self.name = host, int(port), name
        self.connect_timeout = connect_timeout
        self._sock = None
        self._buf = b""
        self._lock = threading.Lock()

    # -- connection -----------------------------------------------------
    @property
    def connected(self):
        return self._sock is not None

    def connect(self, retry_for=0.0, backoff=0.5, log=print):
        """Connect; keep retrying for `retry_for` seconds (0 = one try)."""
        deadline = time.time() + retry_for
        while True:
            try:
                s = socket.create_connection((self.host, self.port), timeout=self.connect_timeout)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                s.settimeout(None)
                self._sock, self._buf = s, b""
                return True
            except OSError as err:
                if time.time() >= deadline:
                    log(f"[{self.name}] connect {self.host}:{self.port} failed: {err}")
                    return False
                time.sleep(backoff)

    def close(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock, self._buf = None, b""

    # -- io -------------------------------------------------------------
    def send(self, obj):
        if self._sock is None:
            raise ConnectionError(f"{self.name}: not connected")
        data = (json.dumps(obj, separators=(",", ":")) + "\n").encode("utf-8")
        with self._lock:
            self._sock.sendall(data)

    def recv(self, timeout=None):
        """One JSON object; raises TimeoutError / ConnectionError."""
        if self._sock is None:
            raise ConnectionError(f"{self.name}: not connected")
        self._sock.settimeout(timeout)
        try:
            while b"\n" not in self._buf:
                chunk = self._sock.recv(65536)
                if not chunk:
                    self.close()
                    raise ConnectionError(f"{self.name}: connection closed")
                self._buf += chunk
        except socket.timeout:
            raise TimeoutError(f"{self.name}: no message within {timeout}s")
        finally:
            if self._sock is not None:
                self._sock.settimeout(None)
        line, self._buf = self._buf.split(b"\n", 1)
        if not line.strip():
            return self.recv(timeout)
        return json.loads(line.decode("utf-8"))

    def request(self, obj, expect, timeout=10.0, on_other=None):
        """send obj, return the first message whose type is in `expect`
        (str or set); other messages go to on_other(msg) if given."""
        expect = {expect} if isinstance(expect, str) else set(expect)
        self.send(obj)
        deadline = time.time() + timeout
        while True:
            left = max(0.01, deadline - time.time())
            msg = self.recv(timeout=left)
            if msg.get("type") in expect:
                return msg
            if msg.get("type") == "error":
                return msg
            if on_other:
                on_other(msg)
