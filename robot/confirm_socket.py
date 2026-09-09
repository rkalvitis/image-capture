#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Socket confirmation for panda_manual_poses.py (`_confirm_via:=socket`).

Replaces the three "press Enter" prompts of a run with a TCP exchange so
the Mac coordinator decides when the arm may leave a pose. The ROBOT is
the server: it listens on `_confirm_port` (default 5600) and the Mac
connects. Newline-delimited JSON, one object per line:

    robot -> mac   {"type":"hello","poses":N}                 on connect
    robot -> mac   {"type":"reached","id":k,"joints":[7]}     hold reached
    mac   -> robot {"type":"go","id":k}                        leave the pose
    mac   -> robot {"type":"abort"}                            finish the run now
    robot -> mac   {"type":"done"}                             back at the initial pose

id 0 = the initial pose ("place/check the object" = the turntable moment),
1..N = the recorded poses. The run itself is unchanged: after the last
pose the arm returns to the initial pose as always. Repeating runs (one
per turntable step) is a shell loop around rosrun - see run_cycles.sh.

Pure stdlib, Python 2/3, no ROS import: usable on the robot PC as-is and
testable on any machine (fake_robot.py).
"""

from __future__ import print_function

import json
import socket
import sys
import threading
import time


def _log(msg):
    try:
        import rospy
        rospy.loginfo(msg)
    except Exception:
        print("[confirm_socket] " + msg)
        sys.stdout.flush()


class ConfirmServer(object):
    """One client at a time; survives client disconnects (waits for the
    Mac to reconnect, re-sending the pending `reached`)."""

    def __init__(self, port=5600, host="0.0.0.0", poses=0):
        self.port = int(port)
        self.host = host
        self.poses = int(poses)
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((self.host, self.port))
        self._srv.listen(1)
        self._conn = None
        self._buf = b""
        self._lock = threading.Lock()
        _log("confirm socket listening on %s:%d" % (self.host, self.port))

    # -- connection -----------------------------------------------------
    def _ensure_client(self):
        if self._conn is not None:
            return
        _log("waiting for the coordinator to connect...")
        conn, addr = self._srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._conn = conn
        self._buf = b""
        _log("coordinator connected from %s:%d" % addr)
        self._send({"type": "hello", "poses": self.poses})

    def _drop_client(self):
        if self._conn is not None:
            try:
                self._conn.close()
            except Exception:
                pass
        self._conn = None
        self._buf = b""

    def _send(self, obj):
        data = (json.dumps(obj) + "\n").encode("utf-8")
        with self._lock:
            self._conn.sendall(data)

    def _recv(self):
        """Blocks until one JSON line arrives; None on disconnect."""
        while b"\n" not in self._buf:
            chunk = self._conn.recv(4096)
            if not chunk:
                return None
            self._buf += chunk
        line, self._buf = self._buf.split(b"\n", 1)
        line = line.strip()
        if not line:
            return self._recv()
        try:
            return json.loads(line.decode("utf-8"))
        except ValueError:
            _log("ignoring malformed line: %r" % line)
            return self._recv()

    # -- the one call the script makes --------------------------------
    def wait(self, pose_id, joints, message=""):
        """Announce `reached` for pose_id and block until `go`.
        Returns True on `go`, False on `abort`."""
        if message:
            _log(message)
        while True:
            try:
                self._ensure_client()
                self._send({"type": "reached", "id": int(pose_id),
                            "joints": [float(j) for j in joints],
                            "t": time.time()})
                while True:
                    msg = self._recv()
                    if msg is None:
                        raise socket.error("coordinator disconnected")
                    t = msg.get("type")
                    if t == "go" and int(msg.get("id", pose_id)) == int(pose_id):
                        return True
                    if t == "go":
                        _log("go for id %s while at %d - ignored"
                             % (msg.get("id"), pose_id))
                        continue
                    if t == "abort":
                        _log("abort received from the coordinator")
                        return False
                    if t == "ping":
                        self._send({"type": "pong", "t": time.time()})
                        continue
                    _log("unexpected message %r - ignored" % (msg,))
            except (socket.error, OSError) as err:
                _log("socket error (%s) - waiting for reconnect" % err)
                self._drop_client()
                time.sleep(0.5)

    def done(self):
        try:
            if self._conn is not None:
                self._send({"type": "done", "t": time.time()})
        except Exception:
            pass

    def close(self):
        self._drop_client()
        try:
            self._srv.close()
        except Exception:
            pass


class KeyboardConfirm(object):
    """Unchanged behaviour: press Enter (uses arc.wait_for_enter)."""

    def __init__(self, wait_for_enter):
        self._wait = wait_for_enter

    def wait(self, pose_id, joints, message=""):
        self._wait(message)
        return True

    def done(self):
        pass

    def close(self):
        pass
