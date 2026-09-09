"""Client for panda_manual_poses.py in `_confirm_via:=socket` mode
(robot/confirm_socket.py). The robot is the server; one connection per
run (= one turntable step)."""
import time

from .jsonline import JsonLineClient


class RobotClient:
    def __init__(self, host, port=5600, log=print):
        self.conn = JsonLineClient(host, port, name="robot")
        self.log = log
        self.poses = None

    def connect(self, retry_for=600.0):
        """Wait for the robot script to come up (it listens only while a
        run is active), then read its `hello`."""
        deadline = time.time() + retry_for
        while True:
            if not self.conn.connect(retry_for=max(0.0, deadline - time.time()), log=self.log):
                raise ConnectionError("robot confirm socket not reachable")
            try:
                msg = self.conn.recv(timeout=30.0)
            except (ConnectionError, TimeoutError, OSError) as err:
                # a run that is just ending/starting closes its socket under
                # us (old listen backlog) - simply try again
                self.conn.close()
                if time.time() > deadline:
                    raise ConnectionError(f"robot: no greeting ({err})")
                time.sleep(0.5)
                continue
            if msg.get("type") != "hello":
                raise RuntimeError(f"robot: expected hello, got {msg}")
            self.poses = int(msg.get("poses", 0))
            return msg

    def wait_reached(self, timeout=300.0):
        """Block until the robot reports a reached pose; returns the message."""
        while True:
            msg = self.conn.recv(timeout=timeout)
            t = msg.get("type")
            if t == "reached":
                return msg
            if t == "done":
                return msg
            if t == "error":
                raise RuntimeError(f"robot error: {msg.get('reason')}")
            self.log(f"[robot] ignoring {msg}")

    def go(self, pose_id):
        self.conn.send({"type": "go", "id": int(pose_id), "t": time.time()})

    def abort(self):
        try:
            self.conn.send({"type": "abort", "t": time.time()})
        except Exception:
            pass

    def wait_done(self, timeout=600.0):
        while True:
            msg = self.conn.recv(timeout=timeout)
            if msg.get("type") == "done":
                return msg

    def close(self):
        self.conn.close()
