#!/usr/bin/env python3
"""Organize a flat capture folder into per-stack subfolders.

The camera writes a repeating sequence: FOCUS_SWEEP_COUNT focus-bracketed
frames (raw) followed by one focus-stacked result (jpg). This script sorts
the flat folder

    images/IMG_0001.DNG ... IMG_0021.DNG IMG_0022.JPG IMG_0023.DNG ...

into

    images/1/IMG_0022.JPG
    images/1/focus_stack/IMG_0001.DNG ... IMG_0021.DNG
    images/2/...

Files are matched by sorted filename order, so the sequence must be complete.
Nothing is moved unless the whole folder validates against the expected
pattern first. Use --dry-run to preview.
"""

import argparse
import sys
from pathlib import Path

# Defaults matching the iPhone focus-bracketing sessions (see notes.txt):
FOCUS_SWEEP_COUNT = 21          # frames per focus sweep
STACKED_EXTENSIONS = {".jpg", ".jpeg"}   # the merged/stacked image
FRAME_EXTENSIONS = {".dng"}              # the individual focus frames
FOCUS_DIR_NAME = "focus_stack"


def collect_files(images_dir: Path, extensions: set[str]) -> list[Path]:
    files = [
        p for p in images_dir.iterdir()
        if p.is_file() and p.suffix.lower() in extensions
    ]
    return sorted(files, key=lambda p: p.name)


def validate(files: list[Path], sweep: int, stacked_exts: set[str],
             frame_exts: set[str]) -> list[str]:
    """Return a list of pattern violations (empty = valid)."""
    group = sweep + 1
    errors = []
    if not files:
        errors.append("no image files found")
        return errors
    if len(files) % group != 0:
        errors.append(
            f"{len(files)} files is not a multiple of {group} "
            f"({sweep} frames + 1 stacked image)")
    for i, f in enumerate(files):
        is_stack_slot = (i + 1) % group == 0
        ext = f.suffix.lower()
        if is_stack_slot and ext not in stacked_exts:
            errors.append(
                f"position {i + 1} ({f.name}): expected stacked image "
                f"({'/'.join(sorted(stacked_exts))}), got {ext}")
        elif not is_stack_slot and ext not in frame_exts:
            errors.append(
                f"position {i + 1} ({f.name}): expected focus frame "
                f"({'/'.join(sorted(frame_exts))}), got {ext}")
    return errors


def organize(images_dir: Path, sweep: int, stacked_exts: set[str],
             frame_exts: set[str], dry_run: bool) -> int:
    files = collect_files(images_dir, stacked_exts | frame_exts)
    errors = validate(files, sweep, stacked_exts, frame_exts)
    if errors:
        print("Folder does not match the expected pattern; nothing moved:",
              file=sys.stderr)
        for e in errors[:20]:
            print(f"  - {e}", file=sys.stderr)
        if len(errors) > 20:
            print(f"  ... and {len(errors) - 20} more", file=sys.stderr)
        return 1

    group = sweep + 1
    n_stacks = len(files) // group
    print(f"{len(files)} files -> {n_stacks} stacks "
          f"({sweep} frames + 1 stacked image each)")

    for s in range(n_stacks):
        chunk = files[s * group:(s + 1) * group]
        frames, stacked = chunk[:sweep], chunk[sweep]
        stack_dir = images_dir / str(s + 1)
        focus_dir = stack_dir / FOCUS_DIR_NAME
        print(f"  {stack_dir.name}/: {stacked.name}  "
              f"({FOCUS_DIR_NAME}/: {frames[0].name} .. {frames[-1].name})")
        if dry_run:
            continue
        focus_dir.mkdir(parents=True, exist_ok=True)
        stacked.rename(stack_dir / stacked.name)
        for f in frames:
            f.rename(focus_dir / f.name)

    print("dry run — no files moved" if dry_run else "done")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("images_dir", type=Path,
                    help="flat folder with the capture sequence "
                         "(e.g. datasets/phone_<ts>/images)")
    ap.add_argument("--sweep-count", type=int, default=FOCUS_SWEEP_COUNT,
                    help=f"focus frames per stack (default {FOCUS_SWEEP_COUNT})")
    ap.add_argument("--stacked-ext", nargs="+",
                    default=sorted(STACKED_EXTENSIONS),
                    help="extension(s) of the stacked image (default .jpg .jpeg)")
    ap.add_argument("--frame-ext", nargs="+", default=sorted(FRAME_EXTENSIONS),
                    help="extension(s) of the focus frames (default .dng)")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the plan without moving files")
    args = ap.parse_args()

    if not args.images_dir.is_dir():
        ap.error(f"{args.images_dir} is not a directory")
    norm = lambda exts: {e.lower() if e.startswith(".") else "." + e.lower()
                         for e in exts}
    return organize(args.images_dir, args.sweep_count,
                    norm(args.stacked_ext), norm(args.frame_ext),
                    args.dry_run)


if __name__ == "__main__":
    sys.exit(main())
