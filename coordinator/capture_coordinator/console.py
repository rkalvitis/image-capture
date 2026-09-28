"""Operator console: one stdin reader for the whole session.

The coordinator spends most of its time inside blocking calls (a 100 s
stack on the phone, waiting for the robot), so a plain `input()` cannot
listen for a pause request. This reader thread takes every line typed in
the terminal; while a prompt is open the line answers the prompt, at any
other time it is a command:

    p / pause      pause at the next checkpoint (after the running stack,
                   the arm holds its pose)
    r / resume     clear a pending pause request
    s / status     print the coordinator's one-line status

Two more ways to pause without the keyboard: Ctrl-C once (a second
Ctrl-C aborts), or `touch <session>/PAUSE` from any terminal (removed on
resume). The prompt parsing is the same as Coordinator._ask_operator.
"""
import os
import queue
import sys
import threading


class Console:
    def __init__(self, log=print):
        self.log = log
        self.pause_requested = False
        self.status_fn = None
        self._lines = queue.Queue()
        self._prompt_open = threading.Event()
        self.interactive = bool(sys.stdin and sys.stdin.isatty())
        if self.interactive:
            threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        while True:
            try:
                line = sys.stdin.readline()
            except (OSError, ValueError):
                return
            if line == "":
                return                                  # stdin closed
            if self._prompt_open.is_set():
                self._lines.put(line)
                continue
            cmd = line.strip().lower()
            if cmd in ("p", "pause"):
                self.pause_requested = True
                self.log("\n>>> pause requested — pausing at the next checkpoint (after the running stack)")
            elif cmd in ("r", "resume"):
                self.pause_requested = False
                self.log("\n>>> pause request cleared")
            elif cmd in ("s", "status") and self.status_fn:
                self.log("\n" + self.status_fn())
            elif cmd:
                self.log(f"\n>>> '{cmd}' ignored — commands: p = pause, r = resume, s = status")

    def ask(self, question, choices):
        """Operator prompt; same answers as Coordinator._ask_operator."""
        if not self.interactive:
            return input(f"{question} [{'/'.join(choices)}] ").strip().lower() or ("enter" if "enter" in choices else choices[0])
        while True:
            sys.stdout.write(f"{question} [{'/'.join(choices)}] ")
            sys.stdout.flush()
            self._prompt_open.set()
            try:
                line = self._lines.get()
            finally:
                self._prompt_open.clear()
            ans = line.strip().lower()
            if ans == "" and "enter" in choices:
                return "enter"
            if "lens" in choices or "ev" in choices:
                try:
                    v = float(ans)
                    if "ev" in choices or 0.0 <= v <= 1.0:
                        return v
                except ValueError:
                    pass
            for c in choices:
                if ans == c or (ans and c.startswith(ans)):
                    return c


def pause_file(session_dir):
    return os.path.join(session_dir, "PAUSE")
