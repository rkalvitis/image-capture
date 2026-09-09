#!/usr/bin/env python3
"""Bench test client for the phone app (stdlib only).

    python tools/mac_client.py hello --session bench1 --lens 0.45
    python tools/mac_client.py focus 0.5
    python tools/mac_client.py stack --stop 1 --lens 0.45                 (single frame)
    python tools/mac_client.py stack --stop 1 --stack-start 0 --stack-end 0.5 --stack-frames 40
    python tools/mac_client.py status | ping | bye

Default host 127.0.0.1:5601 = through the USB tunnel (tools/usb_tunnel.sh).
"""
import argparse
import json
import socket
import sys
import time


def talk(host, port, msg, expect, timeout=30.0):
    s = socket.create_connection((host, port), timeout=5)
    s.sendall((json.dumps(msg) + "\n").encode())
    s.settimeout(timeout)
    buf = b""
    t_end = time.time() + timeout
    while time.time() < t_end:
        chunk = s.recv(65536)
        if not chunk:
            break
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            if not line.strip():
                continue
            m = json.loads(line)
            print(json.dumps(m, indent=1))
            if m.get("type") in (expect, "error"):
                s.close()
                return m
    s.close()
    print("timeout", file=sys.stderr)
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["hello", "focus", "stack", "status", "ping", "bye", "ui"])
    ap.add_argument("--horizontal", choices=["on", "off"]); ap.add_argument("--clips", choices=["on", "off"]); ap.add_argument("--fill", choices=["on", "off"])
    ap.add_argument("value", nargs="?", help="focus: lens position 0..1")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5601)
    ap.add_argument("--session", default="bench")
    ap.add_argument("--lens", type=float, default=0.45)
    ap.add_argument("--stop", type=int, default=0)
    ap.add_argument("--stack-start", type=float, help="focus stack: first lens position (with --stack-end/--stack-frames)")
    ap.add_argument("--stack-end", type=float, default=0.5)
    ap.add_argument("--stack-frames", type=int, default=40)
    ap.add_argument("--iso", type=int, help="with --duration: custom locked exposure")
    ap.add_argument("--duration", type=float, help="seconds, e.g. 0.01")
    ap.add_argument("--lock", action="store_true", help="meter once at hello, then lock exposure + WB")
    ap.add_argument("--wb-k", type=float, help="custom white balance temperature (K)")
    a = ap.parse_args()
    t = time.time()
    if a.cmd == "hello":
        if a.iso is not None and a.duration is not None:
            exposure = {"mode": "custom", "duration_s": a.duration, "iso": a.iso}
        else:
            exposure = {"mode": "locked" if a.lock else "auto"}
        wb = {"mode": "custom", "temperature_k": a.wb_k, "tint": 0} if a.wb_k else {"mode": "locked" if a.lock else "auto"}
        talk(a.host, a.port, {"type": "hello", "t": t, "session": a.session, "lens_positions": [a.lens],
                              "settle_ms": 300, "exposure": exposure, "wb": wb}, "ready")
    elif a.cmd == "focus":
        v = float(a.value if a.value else a.lens)
        talk(a.host, a.port, {"type": "focus", "t": t, "lens_position": v}, "focus")
    elif a.cmd == "stack":
        if a.stack_start is not None:
            n = max(1, a.stack_frames)
            lens = [round(a.stack_start + (a.stack_end - a.stack_start) * i / max(1, n - 1), 4) for i in range(n)]
        else:
            lens = [a.lens]
        talk(a.host, a.port, {"type": "stack", "t": t, "id": a.stop, "pose": 0, "step": 0, "lens_positions": lens}, "done", timeout=12 * len(lens) + 330)
    elif a.cmd == "ui":
        m = {"type": "ui", "t": t}
        for k in ("horizontal", "clips", "fill"):
            v = getattr(a, k)
            if v: m[k] = (v == "on")
        talk(a.host, a.port, m, "ui")
    elif a.cmd == "ping":
        t0 = time.time(); m = talk(a.host, a.port, {"type": "ping", "t": t0}, "pong"); t1 = time.time()
        if m: print(f"round trip {1000*(t1-t0):.1f} ms, clock offset {1000*(m['t']-(t0+t1)/2):+.1f} ms")
    else:
        talk(a.host, a.port, {"type": a.cmd, "t": t}, "status")


if __name__ == "__main__":
    main()
