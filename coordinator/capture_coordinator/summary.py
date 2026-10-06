"""Session summary: <session>/summary.json + SUMMARY.md, built from the files the session
writes anyway (plan.json, session.jsonl, frames.csv, stacks.csv, robot/robot_poses.csv,
images/). The coordinator rewrites it after every stop and at the end; it can be
regenerated any time:

    python -m capture_coordinator.summary <session_dir>

Nothing here is needed to process the data; it is the human-readable record of HOW the
dataset was collected (loop order, turntable, exposure, focus, timing, counts, problems).
"""
import csv
import datetime
import glob
import json
import os
import statistics
import sys


def _load_events(path):
    ev = []
    if not os.path.exists(path):
        return ev
    with open(path) as f:
        for line in f:
            try:
                ev.append(json.loads(line))
            except ValueError:
                continue                      # a line still being written
    return ev


def _load_csv(path):
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


def _iso(t):
    return datetime.datetime.fromtimestamp(t).strftime("%Y-%m-%d %H:%M:%S") if t else None


def _hms(seconds):
    if seconds is None:
        return None
    s = int(round(seconds))
    return f"{s // 3600}:{(s % 3600) // 60:02d}:{s % 60:02d}"


def _stat(vals, digits=3):
    vals = [v for v in vals if v is not None]
    if not vals:
        return None
    return {"n": len(vals), "mean": round(statistics.fmean(vals), digits), "min": round(min(vals), digits),
            "max": round(max(vals), digits)}


def _float(v):
    try:
        return float(v) if v not in (None, "") else None
    except ValueError:
        return None


def _dir_size(path):
    total, dngs, tifs = 0, 0, 0
    for root, _dirs, files in os.walk(path):
        for fn in files:
            try:
                total += os.path.getsize(os.path.join(root, fn))
            except OSError:
                continue
            if fn.lower().endswith(".dng"):
                dngs += 1
            elif fn.lower().endswith((".tif", ".tiff")):
                tifs += 1
    return total, dngs, tifs


def build_summary(session_dir, extra=None):
    d = session_dir
    plan = (extra or {}).get("plan") or {}
    if not plan and os.path.exists(os.path.join(d, "plan.json")):
        with open(os.path.join(d, "plan.json")) as f:
            plan = json.load(f)
    ev = _load_events(os.path.join(d, "session.jsonl"))
    from .session import read_frames_csv
    frames = read_frames_csv(os.path.join(d, "frames.csv"))
    stacks = _load_csv(os.path.join(d, "stacks.csv"))
    robot_rows = _load_csv(os.path.join(d, "robot", "robot_poses.csv"))
    by = lambda kind: [e for e in ev if e.get("event") == kind]

    starts, ends, aborts = by("session_start"), by("session_end"), by("session_abort")
    first = starts[0] if starts else {}
    t_start = first.get("t")
    last_end = (ends + aborts)[-1] if (ends or aborts) else None
    t_end = last_end["t"] if last_end else (ev[-1]["t"] if ev else None)
    if last_end and ev and ev[-1]["t"] > last_end["t"] + 1 and any(e is last_end for e in ev[:-1]):
        # events after the last end = a resumed run that is still going
        t_end = ev[-1]["t"]
    status = "running"
    if last_end and last_end is ev[-1] or (last_end and (extra or {}).get("final")):
        status = "finished" if last_end.get("event") == "session_end" else "aborted"
    # runs = one per session_start (resumes); capture time = sum of run durations
    run_spans, run_ends = [], ends + aborts
    for i, s in enumerate(starts):
        nxt = starts[i + 1]["t"] if i + 1 < len(starts) else None
        e = [x["t"] for x in run_ends if x["t"] > s["t"] and (nxt is None or x["t"] < nxt)]
        run_spans.append((s["t"], e[0] if e else (nxt or (ev[-1]["t"] if ev else s["t"]))))
    capture_s = sum(b - a for a, b in run_spans)
    # pauses
    pause_s, pauses = 0.0, by("paused")
    resumed = by("resumed")
    for p_ in pauses:
        r = [x["t"] for x in resumed if x["t"] > p_["t"]]
        if r:
            pause_s += r[0] - p_["t"]

    n_poses = int(plan.get("poses", 0)) + (1 if plan.get("shoot_initial_pose") else 0)
    steps = int(plan.get("turntable_steps", 0))
    planned = steps * n_poses
    fps = int(plan.get("frames_per_stop", 1))
    ok = by("stop_ok")
    per_pose, per_step = {}, {}
    for e in ok:
        per_pose[e.get("pose")] = per_pose.get(e.get("pose"), 0) + 1
        per_step[e.get("step")] = per_step.get(e.get("step"), 0) + 1
    captured = by("captured")
    thermal = {}
    for e in captured:
        for k, v in (e.get("thermal") or {}).items():
            thermal[k] = thermal.get(k, 0) + int(v)
    failed = by("stop_failed")
    final_fail = [e for e in failed if e.get("attempt", 0) > int(plan.get("max_retries", 0))]
    turns = by("turntable")
    tt_ready = by("turntable_ready")
    yaw_err = [abs(((e["yaw"] - e["target_yaw"]) + 180) % 360 - 180) for e in turns
               if e.get("yaw") is not None and e.get("target_yaw") is not None]
    phone_ready = by("phone_ready")
    reply = (phone_ready[-1].get("reply") or {}) if phone_ready else {}
    exp_applied = by("exposure_applied")
    exp_acc = by("exposure_accepted")
    merge_done = by("merge_done")
    uploaded = by("stop_uploaded")
    raw_uploaded = by("stop_raw_uploaded")
    upload_failed = by("stop_upload_failed")
    upload_done = by("upload_done")
    poses_dir = os.path.join(d, "poses")
    poses_files = {sub: len(os.listdir(os.path.join(poses_dir, sub))) for sub in ("mocap", "robot_runs")
                   if os.path.isdir(os.path.join(poses_dir, sub))} if os.path.isdir(poses_dir) else {}
    images_dir = os.path.join(d, "images")
    size_b, dngs, tifs = _dir_size(images_dir) if os.path.isdir(images_dir) else (0, 0, 0)
    mocap_rows = sum(1 for r in frames if r.get("pop_x"))
    stats = (extra or {}).get("stats") or {}
    turntable_drive = (tt_ready[-1].get("drive") if tt_ready else None) or first.get("turntable_drive") or plan.get("turntable_drive", "manual")

    loop_order = plan.get("loop_order", "poses-per-rotation")
    if loop_order == "rotations-per-pose":
        loop_text = (f"robot pose outer loop: at each of the {n_poses} robot poses all {steps} insect rotations "
                     f"({plan.get('turntable_step_deg')} deg each), platter reversed back to step 0 before the next pose; one robot run")
    else:
        loop_text = (f"insect rotation outer loop: all {n_poses} robot poses at one insect rotation, then the insect is "
                     f"turned by {plan.get('turntable_step_deg')} deg; {steps} rotations = {steps} robot runs")

    summary = {
        "session": {
            "id": first.get("session"), "folder": os.path.abspath(d), "plan_name": plan.get("name"),
            "subject": plan.get("subject", ""), "notes": plan.get("notes", ""),
            "status": status, "started": _iso(t_start), "ended": _iso(t_end) if status != "running" else None,
            "last_event": _iso(ev[-1]["t"]) if ev else None,
            "wall_time": _hms(t_end - t_start) if t_start and t_end else None,
            "capture_time": _hms(capture_s), "paused": _hms(pause_s), "pauses": len(pauses),
            "runs": len(starts), "resumes": max(0, len(starts) - 1),
            "host": first.get("host"), "platform": first.get("platform"), "coordinator_commit": first.get("coordinator_commit"),
        },
        "collection": {
            "loop_order": loop_order, "loop_order_text": loop_text,
            "robot_poses": int(plan.get("poses", 0)), "initial_pose_shot": bool(plan.get("shoot_initial_pose")),
            "turntable_steps": steps, "turntable_step_deg": plan.get("turntable_step_deg"),
            "turntable_drive": turntable_drive, "turntable_confirm": plan.get("turntable_confirm"),
            "turntable_returns": len(by("turntable_return")),
            "frames_per_stop": fps, "focus_mode": plan.get("focus_mode"),
            "focus_stack": {"start": plan.get("focus_stack_start"), "end": plan.get("focus_stack_end"),
                            "step": plan.get("focus_stack_step"), "table": plan.get("focus_table") or None},
            "settle_after_move_s": plan.get("settle_after_move_s"), "settle_ms": plan.get("settle_ms"),
            "stillness_gate": plan.get("stillness_gate"),
            "exposure": {"mode": plan.get("exposure_mode"), "iso": plan.get("exposure_iso"),
                         "duration_s": plan.get("exposure_duration_s"), "check": plan.get("exposure_check"),
                         "ev_applied": exp_applied[-1].get("ev") if exp_applied else None,
                         "final": (exp_applied[-1].get("exposure") if exp_applied else
                                   (exp_acc[-1].get("exposure") if exp_acc else None))},
            "white_balance": plan.get("wb"),
            "phone": {"device": reply.get("device"), "lens": reply.get("lens"), "format": reply.get("format"),
                      "raw": reply.get("raw"), "quality": reply.get("quality"),
                      "clock_offset_ms": round(1000 * phone_ready[-1]["clock_offset_s"], 1) if phone_ready and phone_ready[-1].get("clock_offset_s") is not None else None},
            "mocap": {"recorded": plan.get("record_mocap"), "bodies": plan.get("mocap_bodies"),
                      "frames_with_poses": mocap_rows, "frames_total": len(frames)},
            "merge": {"inline": plan.get("merge_inline"), "fusion": plan.get("merge_fusion"), "format": plan.get("merge_format"),
                      "delete_raw": plan.get("merge_delete_raw")},
            "upload": {"remote": plan.get("upload_remote") or None, "what": plan.get("upload_what") if plan.get("upload_remote") else None,
                       "delete_local": plan.get("upload_delete_local"), "delete_phone": plan.get("upload_delete_phone"),
                       "poses_collect": plan.get("poses_collect") or None},
            "robot": {"host": plan.get("robot_host"), "files": plan.get("robot_files")},
        },
        "results": {
            "stops_planned": planned, "stops_ok": len(ok), "stops_skipped": len(by("stop_skipped")),
            "stops_failed_final": len(final_fail), "retries": len(failed) - len(final_fail),
            "stops_already_done_on_resume": len(by("stop_already_done")),
            "stops_per_robot_pose": {f"pose {k}": v for k, v in sorted(per_pose.items(), key=lambda kv: (kv[0] is None, kv[0]))},
            "rotations_complete": sum(1 for s_, n in per_step.items() if n >= n_poses),
            "frames_planned": planned * fps, "frames_captured": len(frames),
            "merged_masters": tifs, "merge": merge_done[-1] if merge_done else None,
            "stacks_csv_rows": len(stacks), "robot_pose_rows": len(robot_rows),
            "raw_dng_on_disk": dngs, "images_dir_gb": round(size_b / 1e9, 2),
            "stops_uploaded": len(uploaded) if plan.get("upload_remote") else None,
            "stops_raw_uploaded": len(raw_uploaded) if raw_uploaded else None,
            "stops_upload_failed": len(upload_failed) if plan.get("upload_remote") else None,
            "stops_still_local": (len(ok) - len(uploaded)) if plan.get("upload_remote") else None,
            "uploaded_gb": round(sum(e.get("bytes", 0) for e in uploaded) / 1e9, 2) if uploaded else None,
            "upload_mb_s": _stat([e["bytes"] / e["seconds"] / 1e6 for e in (raw_uploaded or uploaded) if e.get("seconds")], 1)
                           if (raw_uploaded or uploaded) else None,
            "files_deleted_local": sum(e.get("deleted_local", 0) for e in uploaded) if uploaded else None,
            "files_deleted_on_phone": sum(e.get("deleted_phone", 0) for e in uploaded + raw_uploaded) if (uploaded or raw_uploaded) else None,
            "upload": upload_done[-1] if upload_done else None,
            "poses_folder": {"stops_csv": os.path.exists(os.path.join(poses_dir, "stops.csv")), **{f"{k}_files": v for k, v in poses_files.items()}} if poses_files or os.path.isdir(poses_dir) else None,
            "coordinator_stats": stats or None,
        },
        "timing": {
            "stop_capture_s": _stat([e.get("duration_s") for e in captured]),
            "frame_s": _stat([e["duration_s"] / e["n"] for e in captured if e.get("duration_s") and e.get("n")]),
            "settle_wait_s": _stat([e.get("waited_s") for e in by("settled")]),
            "per_stop_wall_s": round(capture_s / len(ok), 1) if ok else None,
            "turntable_move_s": _stat([e.get("move_s") for e in turns + by("turntable_return")]),
        },
        "quality": {
            "phone_jitter_mm": _stat([_float(r.get("phone_jitter_mm")) for r in frames]),
            "popillia_jitter_mm": _stat([_float(r.get("pop_jitter_mm")) for r in frames]),
            "thermal_frames": thermal or None,
            "turntable_yaw_error_deg": _stat(yaw_err),
            "turntable_events": len(turns), "turntable_accepted_off_target": sum(1 for e in turns if e.get("accepted")),
            "turntable_errors": len(by("turntable_error")),
            "operator_prompts": len(pauses) + len(final_fail) + sum(1 for e in turns if e.get("accepted")) + len(by("turntable_error")),
            "focus_sources": dict(sorted(_count(r.get("focus_source", "").split(":")[0] for r in frames).items())),
        },
    }
    return summary


def _count(it):
    out = {}
    for k in it:
        out[k] = out.get(k, 0) + 1
    return out


def _md_kv(title, d):
    lines = [f"## {title}", "", "| | |", "|---|---|"]
    for k, v in d.items():
        if isinstance(v, dict):
            v = ", ".join(f"{kk}: {vv}" for kk, vv in v.items() if vv is not None)
        elif isinstance(v, list):
            v = ", ".join(str(x) for x in v)
        lines.append(f"| {k.replace('_', ' ')} | {'' if v is None else v} |")
    return "\n".join(lines) + "\n"


def render_markdown(s):
    se, co, re, ti, qu = s["session"], s["collection"], s["results"], s["timing"], s["quality"]
    out = [f"# Session {se.get('id') or ''} — {se.get('plan_name') or ''}", ""]
    if se.get("subject"):
        out.append(f"**Subject:** {se['subject']}  ")
    if se.get("notes"):
        out.append(f"**Notes:** {se['notes']}  ")
    out.append(f"**How:** {co['loop_order_text']}  ")
    out.append(f"**Status:** {se['status']}, started {se['started']}, "
               + (f"ended {se['ended']}, " if se.get("ended") else f"last event {se.get('last_event')}, ")
               + f"wall time {se['wall_time']}, capture time {se['capture_time']}"
               + (f", paused {se['paused']} ({se['pauses']}x)" if se.get("pauses") else "")
               + (f", resumed {se['resumes']}x" if se.get("resumes") else "") + "  ")
    out.append(f"**Result:** {re['stops_ok']} / {re['stops_planned']} stops, {re['frames_captured']} / {re['frames_planned']} frames, "
               f"{re['merged_masters']} merged masters, {re['rotations_complete']} complete rotations"
               + (f", {re['stops_uploaded']} uploaded to {s['collection']['upload']['remote']}" if re.get("stops_uploaded") is not None else "")
               + (f", {re['stops_failed_final']} failed" if re.get("stops_failed_final") else "")
               + (f", {re['stops_skipped']} skipped" if re.get("stops_skipped") else "") + "\n")
    out.append(_md_kv("Session", {k: v for k, v in se.items() if k not in ("subject", "notes")}))
    flat = {}
    for k, v in co.items():
        if k == "loop_order_text":
            continue
        if isinstance(v, dict):
            for kk, vv in v.items():
                flat[f"{k} {kk}"] = vv
        else:
            flat[k] = v
    out.append(_md_kv("Collection setup", flat))
    out.append(_md_kv("Results", re))
    out.append(_md_kv("Timing (seconds)", ti))
    out.append(_md_kv("Quality / problems", qu))
    out.append("Files: `frames.csv` (one row per raw frame, body poses), `session.jsonl` (every event), "
               "`robot/robot_poses.csv` (joints per stop), `stacks.csv` / `alignment.csv` (merge), "
               "`images/<s..._p.>/` (master TIFF + `stack/` sidecars, mocap.csv, stack.json), "
               "`poses/` (stops.csv, turntable.csv, robot_poses.csv, mocap/<image>.csv, robot_runs/). "
               "Regenerate this file: `python -m capture_coordinator.summary <session_dir>`.\n")
    return "\n".join(out)


def write_summary(session_dir, extra=None):
    s = build_summary(session_dir, extra)
    jp, mp = os.path.join(session_dir, "summary.json"), os.path.join(session_dir, "SUMMARY.md")
    tmp = jp + ".tmp"
    with open(tmp, "w") as f:
        json.dump(s, f, indent=2, default=str)
    os.replace(tmp, jp)
    tmp = mp + ".tmp"
    with open(tmp, "w") as f:
        f.write(render_markdown(s))
    os.replace(tmp, mp)
    return jp, mp


def main(argv=None):
    args = argv if argv is not None else sys.argv[1:]
    if not args:
        sys.exit("usage: python -m capture_coordinator.summary <session_dir> [--print]")
    jp, mp = write_summary(args[0], {"final": True})
    print(f"wrote {jp}\n      {mp}")
    if "--print" in args:
        print(open(mp).read())


if __name__ == "__main__":
    main()
