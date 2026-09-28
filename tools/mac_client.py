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


def power_fields(a):
    f = {}
    if a.raw: f["raw"] = a.raw
    if a.quality: f["quality"] = a.quality
    if a.fps: f["frame_rate"] = a.fps
    if a.brightness is not None: f["screen_brightness"] = a.brightness
    if a.preview: f["preview"] = a.preview == "on"
    return f


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
    ap.add_argument("value", nargs="?", help="focus: lens position 0..1, or 'auto' (one-shot AF on the frame centre)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5601)
    ap.add_argument("--session", default="bench")
    ap.add_argument("--lens", type=float, nargs="+", default=[0.45],
                    help="lens position(s) 0..1: hello uses the first, stack shoots one frame per value")
    ap.add_argument("--stop", type=int, default=0)
    ap.add_argument("--stack-start", type=float, help="focus stack: first lens position (with --stack-end/--stack-frames)")
    ap.add_argument("--stack-end", type=float, default=0.5)
    ap.add_argument("--stack-frames", type=int, default=40)
    ap.add_argument("--iso", type=int, help="with --duration: custom locked exposure")
    ap.add_argument("--duration", type=float, help="seconds, e.g. 0.01")
    ap.add_argument("--lock", action="store_true", help="meter once at hello, then lock exposure + WB")
    ap.add_argument("--wb-k", type=float, help="custom white balance temperature (K)")
    ap.add_argument("--raw", choices=["proraw", "bayer"], help="hello/ui: RAW kind (bayer = no on-phone processing)")
    ap.add_argument("--quality", choices=["quality", "balanced", "speed"], help="hello/ui: ProRAW processing effort")
    ap.add_argument("--fps", type=int, help="hello/ui: cap the preview/intrinsics frame rate")
    ap.add_argument("--brightness", type=float, help="hello/ui: screen brightness 0..1")
    ap.add_argument("--preview", choices=["on", "off"], help="hello/ui: live preview on the phone")
    a = ap.parse_args()
    t = time.time()
    if a.cmd == "hello":
        if a.iso is not None and a.duration is not None:
            exposure = {"mode": "custom", "duration_s": a.duration, "iso": a.iso}
        else:
            exposure = {"mode": "locked" if a.lock else "auto"}
        wb = {"mode": "custom", "temperature_k": a.wb_k, "tint": 0} if a.wb_k else {"mode": "locked" if a.lock else "auto"}
        msg = {"type": "hello", "t": t, "session": a.session, "lens_positions": [a.lens[0]],
               "settle_ms": 300, "exposure": exposure, "wb": wb}
        msg.update(power_fields(a))
        talk(a.host, a.port, msg, "ready")
    elif a.cmd == "focus":
        v = "auto" if a.value == "auto" else float(a.value if a.value else a.lens[0])   # auto = AF on the frame centre
        talk(a.host, a.port, {"type": "focus", "t": t, "lens_position": v}, "focus", timeout=10)
    elif a.cmd == "stack":
        if a.stack_start is not None:
            n = max(1, a.stack_frames)
            lens = [round(a.stack_start + (a.stack_end - a.stack_start) * i / max(1, n - 1), 4) for i in range(n)]
        else:
            lens = list(a.lens)
        talk(a.host, a.port, {"type": "stack", "t": t, "id": a.stop, "pose": 0, "step": 0, "lens_positions": lens}, "done", timeout=12 * len(lens) + 330)
    elif a.cmd == "ui":
        m = {"type": "ui", "t": t}
        for k in ("horizontal", "clips", "fill"):
            v = getattr(a, k)
            if v: m[k] = (v == "on")
        m.update(power_fields(a))
        talk(a.host, a.port, m, "ui")
    elif a.cmd == "ping":
        t0 = time.time(); m = talk(a.host, a.port, {"type": "ping", "t": t0}, "pong"); t1 = time.time()
        if m: print(f"round trip {1000*(t1-t0):.1f} ms, clock offset {1000*(m['t']-(t0+t1)/2):+.1f} ms")
    else:
        talk(a.host, a.port, {"type": a.cmd, "t": t}, "status")


if __name__ == "__main__":
    main()
