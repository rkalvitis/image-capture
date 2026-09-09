#!/usr/bin/env python3
"""Pull a session's DNG + sidecars from the app's Documents over USB
(house-arrest AFC, file sharing must be enabled in the app). Never deletes.

    python tools/pull_frames.py <session_id> <dest_dir> [--bundle ch.idsia.capture] [--list]
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "coordinator"))
from capture_coordinator.pull import UsbPuller  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session"); ap.add_argument("dest")
    ap.add_argument("--bundle", default="ch.idsia.capture")
    ap.add_argument("--list", action="store_true", help="only list the files")
    a = ap.parse_args()
    p = UsbPuller(a.bundle)
    names = sorted(p.listdir(a.session))
    print(f"{len(names)} files in Documents/{a.session}")
    if a.list:
        print("\n".join(names)); return
    img = os.path.join(a.dest, "images"); side = os.path.join(a.dest, "sidecars")
    os.makedirs(img, exist_ok=True); os.makedirs(side, exist_ok=True)
    out = p.pull(a.session, names, img, side)
    print(f"pulled {len(out)} files to {a.dest}")


if __name__ == "__main__":
    main()
