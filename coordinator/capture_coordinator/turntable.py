"""Motorised insect turntable: NEMA 17 (17HS4401) + A4988 driver on a small USB-serial
board (Arduino class) running hardware/turntable/turntable_a4988.ino.

The PC cannot pulse STEP/DIR itself, so the board does the motion; the coordinator only
sends ABSOLUTE platter angles. The board keeps the position in microsteps and rounds the
target once per command, so a 10 deg step that is not an integer number of microsteps
(200 * 16 / 36 = 88.9) never accumulates an error (< half a microstep at any step).

Line protocol (115200 8N1, one command per line, one reply per command):
  host -> board                          board -> host
  CFG <steps_per_rev> <max_deg_s> <accel_deg_s2> <dir>   OK CFG
       steps_per_rev = MICROsteps per PLATTER revolution (float), dir = 1 | -1
  SETPOS <deg>        current position := deg           OK <pos_deg>
  GOTO <deg>          absolute move, reply when done    OK <pos_deg>
  MOVE <deg>          relative move                     OK <pos_deg>
  POS                                                   OK <pos_deg>
  EN 0|1              driver enable (1 = hold torque)   OK EN <0|1>
  PING                                                  OK PONG <version>
  anything else / bad value                             ERR <message>
The board prints "READY <version>" once after reset (opening the port resets an Arduino).

Plan keys (plan.py): turntable_drive manual | serial | fake, turntable_port ("auto" = first
USB serial port), turntable_steps_per_rev (motor full steps), turntable_microsteps (MS1..3),
turntable_gear_ratio (motor revolutions per platter revolution), turntable_direction,
turntable_speed_deg_s, turntable_accel_deg_s2, turntable_hold.
"""
import glob
import math
import sys
import threading
import time


class TurntableError(RuntimeError):
    pass


class Turntable:
    """Base: what the coordinator calls. Angles are platter degrees, absolute, growing with
    the step index (0, 10, 20, ...); the board's `dir` flips the physical sense."""
    drive = "manual"

    def connect(self):
        return {}

    def set_position(self, deg):
        return {"pos_deg": float(deg)}

    def goto(self, deg):
        return {"pos_deg": float(deg)}

    def position(self):
        return None

    def close(self):
        pass


# --------------------------------------------------------------------------------------
class SerialTurntable(Turntable):
    drive = "serial"
    VERSION_MIN = 1

    def __init__(self, port="auto", baud=115200, steps_per_rev=200, microsteps=16, gear_ratio=1.0,
                 direction=1, speed_deg_s=20.0, accel_deg_s2=40.0, hold=True, transport=None, log=print):
        self.port, self.baud = port, int(baud)
        self.microsteps_per_rev = float(steps_per_rev) * float(microsteps) * float(gear_ratio)
        self.direction = 1 if int(direction) >= 0 else -1
        self.speed, self.accel = float(speed_deg_s), float(accel_deg_s2)
        self.hold = bool(hold)
        self.log = log
        self._t = transport                       # object with readline() -> bytes, write(bytes), close()
        self._lock = threading.Lock()
        self.pos_deg = None
        self.version = None

    # -- port -----------------------------------------------------------------------
    @staticmethod
    def find_port():
        pats = ["/dev/ttyACM*", "/dev/ttyUSB*", "/dev/cu.usbmodem*", "/dev/cu.usbserial*", "/dev/cu.wchusbserial*"]
        found = [p for pat in pats for p in sorted(glob.glob(pat))]
        return found[0] if found else None

    def connect(self):
        if self._t is None:
            try:
                import serial
            except ImportError:
                raise TurntableError("pyserial missing: `pixi add pyserial` in optitrack/ (done 2026-09-28) "
                                     "or pip install pyserial") from None
            port = self.port
            if not port or port == "auto":
                port = self.find_port()
                if not port:
                    raise TurntableError("no USB serial port found (turntable_port: auto); is the board plugged in?")
            self.log(f"[turntable] opening {port} @ {self.baud}")
            self._t = serial.Serial(port, self.baud, timeout=0.5)
            self.port = port
        # an Arduino resets when the port opens: wait for READY (or nothing, for a board without auto-reset)
        t0 = time.time()
        while time.time() - t0 < 3.0:
            line = self._readline()
            if line.startswith("READY"):
                break
        pong = self._cmd("PING", timeout=3.0)
        parts = pong.split()
        self.version = parts[-1] if len(parts) >= 3 else "?"
        self._cmd(f"CFG {self.microsteps_per_rev:.4f} {self.speed:.3f} {self.accel:.3f} {self.direction}")
        self._cmd(f"EN {1 if self.hold else 0}")
        self.pos_deg = self._pos_from(self._cmd("POS"))
        self.log(f"[turntable] board v{self.version} on {self.port}: {self.microsteps_per_rev:.1f} microsteps/rev "
                 f"({360.0 / self.microsteps_per_rev:.4f} deg each), {self.speed:g} deg/s, {self.accel:g} deg/s2, "
                 f"dir {self.direction:+d}, hold {self.hold}, pos {self.pos_deg:.3f} deg")
        return {"port": self.port, "version": self.version, "microsteps_per_rev": self.microsteps_per_rev,
                "pos_deg": self.pos_deg}

    # -- commands -------------------------------------------------------------------
    def set_position(self, deg):
        self.pos_deg = self._pos_from(self._cmd(f"SETPOS {float(deg):.4f}"))
        return {"pos_deg": self.pos_deg}

    def goto(self, deg):
        deg = float(deg)
        delta = abs(deg - (self.pos_deg or 0.0))
        t = delta / max(self.speed, 1e-3) + 2 * self.speed / max(self.accel, 1e-3)
        t0 = time.time()
        self.pos_deg = self._pos_from(self._cmd(f"GOTO {deg:.4f}", timeout=t + 10.0))
        err = self.pos_deg - deg
        if abs(err) > 360.0 / self.microsteps_per_rev:
            raise TurntableError(f"board reports {self.pos_deg:.4f} deg after GOTO {deg:.4f} (err {err:+.4f})")
        return {"pos_deg": self.pos_deg, "move_s": round(time.time() - t0, 3)}

    def position(self):
        self.pos_deg = self._pos_from(self._cmd("POS"))
        return self.pos_deg

    def enable(self, on):
        self._cmd(f"EN {1 if on else 0}")

    def close(self):
        if self._t is None:
            return
        try:
            if not self.hold:
                self._cmd("EN 0", timeout=1.0)
        except Exception:
            pass
        try:
            self._t.close()
        except Exception:
            pass
        self._t = None

    # -- transport ------------------------------------------------------------------
    def _readline(self):
        try:
            raw = self._t.readline()
        except Exception as e:
            raise TurntableError(f"serial read failed: {e}") from None
        return raw.decode("ascii", "replace").strip() if raw else ""

    def _cmd(self, line, timeout=5.0):
        with self._lock:
            if self._t is None:
                raise TurntableError("turntable not connected")
            try:
                self._t.write((line + "\n").encode("ascii"))
            except Exception as e:
                raise TurntableError(f"serial write failed: {e}") from None
            t0 = time.time()
            while time.time() - t0 < timeout:
                reply = self._readline()
                if not reply or reply.startswith("READY"):
                    continue
                if reply.startswith("OK"):
                    return reply
                if reply.startswith("ERR"):
                    raise TurntableError(f"board: {reply} (command {line!r})")
                self.log(f"[turntable] ignoring {reply!r}")
            raise TurntableError(f"no reply to {line!r} within {timeout:.0f} s")

    @staticmethod
    def _pos_from(reply):
        parts = reply.split()
        try:
            return float(parts[1])
        except (IndexError, ValueError):
            raise TurntableError(f"bad position reply {reply!r}") from None


# --------------------------------------------------------------------------------------
class PortentaTurntable(Turntable):
    """The user's stepper controller (repo phd_roberts_thesis_stepper_motor_control, 2026-09-30):
    Arduino Portenta H7 + A4988 + NEMA 17 17HS4401, plain text over USB serial, 115200 8N1.
    Protocol (that repo's README §5): PING -> OK PONG; INFO -> OK fw=.. steps_per_rev=200
    microsteps=16 max_speed=.. accel=..; STATUS -> OK pos=.. target=.. speed=.. moving=0|1
    enabled=0|1; POS -> OK <pos>; MOVETO <p> / MOVE <n> -> OK, later DONE <pos> (asynchronous,
    one per finished motion); ZERO [<p>]; SPEED [<v>]; ACCEL [<a>]; ENABLE 0|1; errors ERR <reason>
    (BAD_ARG, OUT_OF_RANGE, BUSY, ...). The board prints READY once after reset. Positions are
    MICROSTEPS in the board's RAM (0 after a reset: no homing — the operator places the insect,
    the coordinator sends ZERO). This class keeps the coordinator's contract: absolute platter
    degrees, `set_position` = ZERO, `goto` = MOVETO + wait for DONE + STATUS check, never a
    relative move, so the 360/N rounding never accumulates. The plan's steps_per_rev /
    microsteps are checked against INFO (the board's values win, a mismatch is logged);
    gear_ratio and direction come from the plan (the board knows only the motor shaft)."""
    drive = "portenta"
    ARDUINO_VID = 0x2341

    def __init__(self, port="auto", baud=115200, steps_per_rev=200, microsteps=16, gear_ratio=1.0,
                 direction=1, speed_deg_s=20.0, accel_deg_s2=40.0, hold=True, transport=None, log=print):
        self.port, self.baud = port, int(baud)
        self.plan_spr, self.plan_ms, self.gear = int(steps_per_rev), int(microsteps), float(gear_ratio)
        self.direction = 1 if int(direction) >= 0 else -1
        self.speed, self.accel, self.hold = float(speed_deg_s), float(accel_deg_s2), bool(hold)
        self._t, self.log = transport, log
        self.microsteps_per_rev = float(self.plan_spr * self.plan_ms) * self.gear      # per PLATTER revolution
        self.info = {}
        self.pos_deg = None
        self._lock = threading.Lock()
        self._buf = b""

    # -- port -------------------------------------------------------------------
    @classmethod
    def find_port(cls):
        try:
            from serial.tools import list_ports
            ports = [p.device for p in list_ports.comports() if p.vid == cls.ARDUINO_VID]
            if len(ports) == 1:
                return ports[0]
            if len(ports) > 1:
                raise TurntableError(f"several Arduino boards: {ports} — set turntable_port")
        except ImportError:
            pass
        cands = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
        if not cands:
            raise TurntableError("no Portenta found (no Arduino USB device, no /dev/cu.usbmodem* or /dev/ttyACM*)")
        return cands[0]

    def connect(self):
        if self._t is None:
            try:
                import serial
            except ImportError as e:
                raise TurntableError("pyserial is not installed (pip install pyserial)") from e
            port = self.find_port() if self.port in ("", "auto", None) else self.port
            try:
                self._t = serial.Serial(port, self.baud, timeout=0.1)
            except (OSError, serial.SerialException) as e:
                raise TurntableError(f"cannot open {port}: {e}") from e
            self.port = port
            time.sleep(0.3)                                  # READY banner / half line after the port reset
            try:
                self._t.reset_input_buffer()
            except Exception:                                # noqa: BLE001
                pass
            self._t.write(b"\n")
        self._cmd("PING")
        self.info = self._fields(self._cmd("INFO"))
        spr, ms = int(self.info.get("steps_per_rev", self.plan_spr)), int(self.info.get("microsteps", self.plan_ms))
        if (spr, ms) != (self.plan_spr, self.plan_ms):
            self.log(f"[turntable] board reports steps_per_rev={spr} microsteps={ms}, plan says "
                     f"{self.plan_spr}/{self.plan_ms} -> using the board's values")
        self.microsteps_per_rev = float(spr * ms) * self.gear
        # speed / acceleration in microsteps of the MOTOR shaft per second
        self._cmd(f"SPEED {max(1, int(round(self.speed * spr * ms * self.gear / 360.0)))}")
        st = self._fields(self._cmd("STATUS"))
        if not int(st.get("moving", 0)):
            self._cmd(f"ACCEL {max(1, int(round(self.accel * spr * ms * self.gear / 360.0)))}")
        if self.hold:
            self._cmd("ENABLE 1")
        self.pos_deg = self._deg(int(st.get("pos", 0)))
        self.log(f"[turntable] portenta fw {self.info.get('fw', '?')} on {self.port}: {spr}x{ms} microsteps/rev x gear "
                 f"{self.gear:g} = {self.microsteps_per_rev:.0f} per platter rev ({360.0 / self.microsteps_per_rev:.4f} deg each), "
                 f"{self.speed:g} deg/s, {self.accel:g} deg/s2, dir {self.direction:+d}, hold {self.hold}, pos {self.pos_deg:.3f} deg")
        return {"port": self.port, "version": str(self.info.get("fw", "")), "microsteps_per_rev": self.microsteps_per_rev,
                "pos_deg": self.pos_deg}

    # -- coordinator contract -------------------------------------------------------
    def _steps(self, deg):
        return int(round(float(deg) * self.microsteps_per_rev / 360.0)) * self.direction

    def _deg(self, steps):
        return float(steps) * self.direction * 360.0 / self.microsteps_per_rev

    def set_position(self, deg):
        with self._lock:
            self._cmd(f"ZERO {self._steps(deg)}")
            self.pos_deg = self._deg(int(self._cmd("POS").strip()))
            return {"pos_deg": self.pos_deg}

    def goto(self, deg):
        deg = float(deg)
        target = self._steps(deg)
        with self._lock:
            delta = abs(deg - (self.pos_deg or 0.0))
            t0 = time.time()
            timeout = delta / max(self.speed, 0.1) + 2.0 * self.speed / max(self.accel, 0.1) + 10.0
            self._cmd(f"MOVETO {target}")
            pos = self._wait_done(timeout)
            st = self._fields(self._cmd("STATUS"))
            pos = int(st.get("pos", pos))
            if int(st.get("moving", 0)):
                raise TurntableError(f"board still moving after DONE (target {target}, pos {pos})")
            self.pos_deg = self._deg(pos)
            if pos != target:
                raise TurntableError(f"board at {pos} microsteps after MOVETO {target} ({self.pos_deg:.4f} deg, wanted {deg:.4f})")
            return {"pos_deg": self.pos_deg, "move_s": round(time.time() - t0, 3)}

    def position(self):
        with self._lock:
            self.pos_deg = self._deg(int(self._cmd("POS").strip()))
            return self.pos_deg

    def enable(self, on):
        with self._lock:
            self._cmd(f"ENABLE {1 if on else 0}")

    def close(self):
        if self._t is None:
            return
        try:
            if not self.hold:
                self.enable(False)
        except TurntableError:
            pass
        try:
            self._t.close()
        except Exception:                                    # noqa: BLE001
            pass
        self._t = None

    # -- wire -------------------------------------------------------------------------
    def _readline(self, deadline):
        while True:
            self._buf += self._t.readline()
            if self._buf.endswith(b"\n"):
                line = self._buf.decode("ascii", "replace").strip()
                self._buf = b""
                if line:
                    return line
            elif time.time() > deadline:
                return None

    def _cmd(self, line, timeout=3.0):
        """One command, one OK/ERR reply; DONE lines that arrive meanwhile are kept."""
        self._t.write((line + "\n").encode("ascii"))
        deadline = time.time() + timeout
        while True:
            reply = self._readline(deadline)
            if reply is None:
                raise TurntableError(f"no reply to {line!r} within {timeout:.0f} s (firmware flashed? port busy?)")
            if reply.startswith("DONE"):
                self._pending_done = int(reply.split()[1])
                continue
            if reply == "OK" or reply.startswith("OK "):
                return reply[3:]
            if reply.startswith("ERR"):
                raise TurntableError(f"{line!r} rejected: {reply[4:]}")
            # READY banner or noise: ignore

    def _wait_done(self, timeout):
        self._pending_done = None
        deadline = time.time() + timeout
        while True:
            line = self._readline(deadline)
            if line is None:
                try:
                    self._cmd("STOP")
                except TurntableError:
                    pass
                raise TurntableError(f"no DONE within {timeout:.0f} s (motor stalled? speed too high?)")
            if line.startswith("DONE"):
                return int(line.split()[1])

    @staticmethod
    def _fields(text):
        out = {}
        for item in text.split():
            k, _, v = item.partition("=")
            try:
                out[k] = int(v)
            except ValueError:
                out[k] = v
        return out


class FakePortentaTransport:
    """In-process emulation of the Portenta firmware's protocol (README §5) for the tests:
    the same readline/write surface as serial.Serial. `move_time` = seconds per move."""
    FW = "1.0"

    def __init__(self, steps_per_rev=200, microsteps=16, move_time=0.0, auto_reset=True):
        self.replies, self.log = [], []
        self.spr, self.ms = steps_per_rev, microsteps
        self.pos, self.speed, self.accel, self.enabled = 0, 3200, 6400, 1
        self.move_time = move_time
        if auto_reset:
            self.replies.append(f"READY stepper-controller {self.FW}")

    def readline(self):
        if not self.replies:
            time.sleep(0.005)
            return b""
        return (self.replies.pop(0) + "\r\n").encode()

    def reset_input_buffer(self):
        self.replies = []

    def write(self, data):
        line = data.decode().strip()
        if not line:
            return
        self.log.append(line)
        parts = line.split()
        cmd = parts[0].upper()
        try:
            if cmd == "PING":
                self.replies.append("OK PONG")
            elif cmd == "INFO":
                self.replies.append(f"OK fw={self.FW} steps_per_rev={self.spr} microsteps={self.ms} max_speed={self.speed} accel={self.accel}")
            elif cmd == "STATUS":
                self.replies.append(f"OK pos={self.pos} target={self.pos} speed=0 moving=0 enabled={self.enabled}")
            elif cmd == "POS":
                self.replies.append(f"OK {self.pos}")
            elif cmd in ("MOVE", "MOVETO"):
                target = int(parts[1]) if cmd == "MOVETO" else self.pos + int(parts[1])
                self.replies.append("OK")
                if self.move_time:
                    time.sleep(self.move_time)
                self.pos = target
                self.enabled = 1
                self.replies.append(f"DONE {self.pos}")
            elif cmd in ("STOP", "HALT"):
                self.replies.append("OK"); self.replies.append(f"DONE {self.pos}")
            elif cmd == "SPEED":
                if len(parts) > 1:
                    self.speed = int(parts[1])
                self.replies.append(f"OK {self.speed}")
            elif cmd == "ACCEL":
                if len(parts) > 1:
                    self.accel = int(parts[1])
                self.replies.append(f"OK {self.accel}")
            elif cmd == "ZERO":
                self.pos = int(parts[1]) if len(parts) > 1 else 0
                self.replies.append("OK")
            elif cmd == "ENABLE":
                self.enabled = int(parts[1])
                self.replies.append("OK")
            else:
                self.replies.append("ERR UNKNOWN_COMMAND")
        except (IndexError, ValueError):
            self.replies.append("ERR BAD_ARG")

    def close(self):
        pass


# --------------------------------------------------------------------------------------
class FakeTurntable(Turntable):
    """Dry run: no hardware, instant moves, `on_move(deg)` hook (e.g. synthetic mocap)."""
    drive = "fake"

    def __init__(self, on_move=None, move_s=0.0, log=print):
        self.on_move, self.move_s, self.log = on_move, float(move_s), log
        self.pos_deg = 0.0
        self.moves = []

    def connect(self):
        return {"pos_deg": self.pos_deg}

    def set_position(self, deg):
        self.pos_deg = float(deg)
        return {"pos_deg": self.pos_deg}

    def goto(self, deg):
        deg = float(deg)
        if self.move_s:
            time.sleep(self.move_s)
        self.pos_deg = deg
        self.moves.append(deg)
        if self.on_move:
            self.on_move(deg)
        return {"pos_deg": deg, "move_s": self.move_s}

    def position(self):
        return self.pos_deg


class FakeBoardTransport:
    """In-process emulation of the firmware's protocol (tests): the same readline/write
    surface as serial.Serial, without a port."""
    VERSION = "1"

    def __init__(self, auto_reset=True, move_time=0.0):
        self.replies = []
        self.log = []
        self.spr, self.vmax, self.acc, self.dir = 3200.0, 20.0, 40.0, 1
        self.pos_steps = 0
        self.enabled = False
        self.move_time = move_time
        if auto_reset:
            self.replies.append(f"READY {self.VERSION}")

    def readline(self):
        if not self.replies:
            time.sleep(0.01)
            return b""
        return (self.replies.pop(0) + "\r\n").encode()

    def _deg(self):
        return self.pos_steps * 360.0 / self.spr

    def write(self, data):
        line = data.decode().strip()
        self.log.append(line)
        parts = line.split()
        cmd = parts[0].upper() if parts else ""
        try:
            if cmd == "PING":
                self.replies.append(f"OK PONG {self.VERSION}")
            elif cmd == "CFG":
                self.spr, self.vmax, self.acc, self.dir = float(parts[1]), float(parts[2]), float(parts[3]), int(parts[4])
                self.replies.append("OK CFG")
            elif cmd == "EN":
                self.enabled = parts[1] == "1"
                self.replies.append(f"OK EN {1 if self.enabled else 0}")
            elif cmd == "POS":
                self.replies.append(f"OK {self._deg():.4f}")
            elif cmd == "SETPOS":
                self.pos_steps = int(round(float(parts[1]) * self.spr / 360.0))
                self.replies.append(f"OK {self._deg():.4f}")
            elif cmd in ("GOTO", "MOVE"):
                target = float(parts[1]) if cmd == "GOTO" else self._deg() + float(parts[1])
                if self.move_time:
                    time.sleep(self.move_time)
                self.pos_steps = int(round(target * self.spr / 360.0))
                self.replies.append(f"OK {self._deg():.4f}")
            else:
                self.replies.append(f"ERR unknown command {cmd}")
        except (IndexError, ValueError):
            self.replies.append(f"ERR bad arguments: {line}")

    def close(self):
        pass


def make_turntable(plan, on_move=None, log=print, transport=None):
    """None for the manual platter (Enter), else the driver named by plan.turntable_drive."""
    d = getattr(plan, "turntable_drive", "manual")
    if d == "manual":
        return None
    if d == "fake":
        return FakeTurntable(on_move=on_move, log=log)
    if d == "portenta":
        return PortentaTurntable(port=plan.turntable_port, baud=plan.turntable_baud,
                                 steps_per_rev=plan.turntable_steps_per_rev, microsteps=plan.turntable_microsteps,
                                 gear_ratio=plan.turntable_gear_ratio, direction=plan.turntable_direction,
                                 speed_deg_s=plan.turntable_speed_deg_s, accel_deg_s2=plan.turntable_accel_deg_s2,
                                 hold=plan.turntable_hold, transport=transport, log=log)
    if d == "serial":
        return SerialTurntable(port=plan.turntable_port, baud=plan.turntable_baud,
                               steps_per_rev=plan.turntable_steps_per_rev, microsteps=plan.turntable_microsteps,
                               gear_ratio=plan.turntable_gear_ratio, direction=plan.turntable_direction,
                               speed_deg_s=plan.turntable_speed_deg_s, accel_deg_s2=plan.turntable_accel_deg_s2,
                               hold=plan.turntable_hold, transport=transport, log=log)
    raise ValueError(f"turntable_drive must be manual | portenta | serial | fake, not {d!r}")


def main(argv=None):
    """Bench test: python -m capture_coordinator.turntable [--drive portenta] [--port auto] goto 10 | move 10 | pos | zero | en 0|1 | cycle 36 10"""
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--drive", choices=["portenta", "serial"], default="portenta",
                    help="portenta = the Portenta H7 stepper controller (default); serial = the old Arduino sketch")
    ap.add_argument("--port", default="auto")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--steps-per-rev", type=int, default=200)
    ap.add_argument("--microsteps", type=int, default=16)
    ap.add_argument("--gear-ratio", type=float, default=1.0)
    ap.add_argument("--direction", type=int, default=1)
    ap.add_argument("--speed", type=float, default=20.0, help="deg/s")
    ap.add_argument("--accel", type=float, default=40.0, help="deg/s^2")
    ap.add_argument("--no-hold", action="store_true")
    ap.add_argument("cmd", nargs="+")
    a = ap.parse_args(argv)
    cls = PortentaTurntable if a.drive == "portenta" else SerialTurntable
    tt = cls(a.port, a.baud, a.steps_per_rev, a.microsteps, a.gear_ratio, a.direction, a.speed, a.accel,
             hold=not a.no_hold)
    print(tt.connect())
    c, args = a.cmd[0], a.cmd[1:]
    try:
        if c == "goto":
            print(tt.goto(float(args[0])))
        elif c == "move":
            print(tt.goto(tt.position() + float(args[0])))
        elif c == "pos":
            print(tt.position())
        elif c == "zero":
            print(tt.set_position(0.0))
        elif c == "en":
            tt.enable(args[0] == "1")
        elif c == "cycle":                       # cycle N STEP_DEG [pause_s]: N absolute steps like a session
            n, step = int(args[0]), float(args[1])
            pause = float(args[2]) if len(args) > 2 else 1.0
            tt.set_position(0.0)
            for i in range(1, n + 1):
                print(i, tt.goto(i * step))
                time.sleep(pause)
        else:
            sys.exit(f"unknown command {c}")
    finally:
        tt.close()


if __name__ == "__main__":
    main()
