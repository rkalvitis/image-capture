#!/usr/bin/env python3
"""Stand-in for the rclone binary in the tests (plan upload_rclone: tests/fake_rclone.py).
The "remote" is a local folder: `fakeremote:<path>` or a plain path.

    copy SRC DST [--exclude PAT | --filter "+ PAT" | --filter "- PAT" ...] [--max-depth N] [other flags ignored]
    check SRC DST --one-way [same filters]          exit 1 if a source file is missing/different
    copyto SRC_FILE DST_FILE | mkdir DST | lsd DST | about --json REMOTE: | version
Filter rules are applied in command-line order, first match wins, no match = included (rclone).

Faults for the tests (environment):
    FAKE_RCLONE_DROP=<basename>      copy skips that file (so `check` fails)
    FAKE_RCLONE_FAIL_COPY_ONCE=<dir> the first copy into that folder fails (exit 1), later ones work
    FAKE_RCLONE_LOG=<file>           every invocation appended as one line
"""
import fnmatch
import hashlib
import json
import os
import shutil
import sys


def _path(remote):
    return remote.split(":", 1)[1] if ":" in remote and not os.path.isabs(remote) else remote


def _parse(argv):
    pos, excludes, max_depth = [], [], None          # excludes = ordered (sign, pattern) rules
    it = iter(argv)
    for a in it:
        if a == "--exclude":
            excludes.append(("-", next(it)))
        elif a == "--include":
            excludes.append(("+", next(it)))
        elif a == "--filter":
            rule = next(it)
            excludes.append((rule[0], rule[1:].strip()))
        elif a == "--max-depth":
            max_depth = int(next(it))
        elif a in ("--transfers", "--checkers", "--retries", "--low-level-retries", "--stats", "--bwlimit"):
            next(it)
        elif a.startswith("-"):
            continue
        else:
            pos.append(a)
    return pos, excludes, max_depth


def _excluded(rel, rules):
    base = os.path.basename(rel)
    for sign, pat in rules:
        if pat.startswith("/"):
            hit = os.sep not in rel and fnmatch.fnmatch(rel, pat[1:])
        else:
            hit = fnmatch.fnmatch(base, pat)
        if hit:
            return sign == "-"
    return False


def _files(root, excludes, max_depth=None):
    out = []
    for dirpath, dirs, files in os.walk(root):
        rel_dir = os.path.relpath(dirpath, root)
        depth = 0 if rel_dir == "." else rel_dir.count(os.sep) + 1
        if max_depth is not None and depth >= max_depth:
            dirs[:] = []
        for fn in files:
            rel = fn if rel_dir == "." else os.path.join(rel_dir, fn)
            if not _excluded(rel, excludes):
                out.append(rel)
    return sorted(out)


def _md5(p):
    h = hashlib.md5()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def main():
    argv = sys.argv[1:]
    log = os.environ.get("FAKE_RCLONE_LOG")
    if log:
        with open(log, "a") as f:
            f.write(" ".join(argv) + "\n")
    if not argv:
        sys.exit("fake rclone: no command")
    cmd, rest = argv[0], argv[1:]
    pos, excludes, max_depth = _parse(rest)
    if cmd == "version":
        print("rclone v0.0-fake"); return 0
    if cmd == "mkdir":
        os.makedirs(_path(pos[0]), exist_ok=True); return 0
    if cmd == "lsd":
        d = _path(pos[0])
        if not os.path.isdir(d):
            sys.stderr.write(f"directory not found: {d}\n"); return 3
        for n in sorted(os.listdir(d)):
            if os.path.isdir(os.path.join(d, n)):
                print(f"          -1 2026-01-01 00:00:00        -1 {n}")
        return 0
    if cmd == "about":
        print(json.dumps({"total": 1000 * 10**9, "used": 10**9, "free": 999 * 10**9})); return 0
    if cmd == "copy":
        src, dst = _path(pos[0]), _path(pos[1])
        once = os.environ.get("FAKE_RCLONE_FAIL_COPY_ONCE")
        if once and os.path.abspath(dst) == os.path.abspath(once):
            marker = dst + ".failed_once"
            if not os.path.exists(marker):
                os.makedirs(os.path.dirname(marker), exist_ok=True)
                open(marker, "w").close()
                sys.stderr.write("fake rclone: simulated network error\n"); return 1
        drop = os.environ.get("FAKE_RCLONE_DROP")
        if not os.path.isdir(src):
            sys.stderr.write(f"source not found: {src}\n"); return 3
        for rel in _files(src, excludes, max_depth):
            if drop and os.path.basename(rel) == drop:
                continue
            s, d = os.path.join(src, rel), os.path.join(dst, rel)
            if os.path.exists(d) and os.path.getsize(d) == os.path.getsize(s) and _md5(d) == _md5(s):
                continue
            os.makedirs(os.path.dirname(d), exist_ok=True)
            shutil.copy2(s, d + ".part")
            os.replace(d + ".part", d)
        return 0
    if cmd == "copyto":
        src, dst = _path(pos[0]), _path(pos[1])
        if not os.path.isfile(src):
            sys.stderr.write(f"source not found: {src}\n"); return 3
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copy2(src, dst + ".part"); os.replace(dst + ".part", dst)
        return 0
    if cmd == "check":
        src, dst = _path(pos[0]), _path(pos[1])
        bad = 0
        for rel in _files(src, excludes, max_depth):
            s, d = os.path.join(src, rel), os.path.join(dst, rel)
            if not os.path.exists(d):
                sys.stderr.write(f"ERROR : {rel}: file not in destination\n"); bad += 1
            elif os.path.getsize(d) != os.path.getsize(s) or _md5(d) != _md5(s):
                sys.stderr.write(f"ERROR : {rel}: hashes differ\n"); bad += 1
        if bad:
            sys.stderr.write(f"{bad} differences found\n"); return 1
        return 0
    sys.stderr.write(f"fake rclone: unknown command {cmd}\n")
    return 2


if __name__ == "__main__":
    sys.exit(main())
