"""SerialTurntable against the in-process board emulation (no pyserial, no hardware)."""
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "coordinator"))

from capture_coordinator.turntable import SerialTurntable, FakeBoardTransport, TurntableError   # noqa: E402
from capture_coordinator.turntable import PortentaTurntable, FakePortentaTransport, make_turntable   # noqa: E402
from capture_coordinator.plan import Plan   # noqa: E402
from capture_coordinator.plan import Plan                                                        # noqa: E402


class SerialProtocol(unittest.TestCase):
    def make(self, **kw):
        board = FakeBoardTransport()
        tt = SerialTurntable(transport=board, log=lambda *a: None, **kw)
        return tt, board

    def test_connect_configures_the_board(self):
        tt, board = self.make(steps_per_rev=200, microsteps=16, gear_ratio=1.0, direction=-1, speed_deg_s=25, accel_deg_s2=50)
        info = tt.connect()
        self.assertEqual(info["version"], "1")
        self.assertEqual(board.log[0], "PING")
        self.assertEqual(board.log[1], "CFG 3200.0000 25.000 50.000 -1")
        self.assertEqual(board.log[2], "EN 1")
        self.assertEqual(board.spr, 3200.0)
        self.assertEqual(board.dir, -1)
        self.assertTrue(board.enabled)
        self.assertEqual(tt.pos_deg, 0.0)

    def test_absolute_moves_do_not_accumulate_error(self):
        # 10 deg = 88.9 microsteps at 1/16 direct drive: after 36 stops the platter is at exactly 360
        tt, board = self.make()
        tt.connect()
        tt.set_position(0.0)
        for i in range(1, 37):
            r = tt.goto(i * 10.0)
            self.assertLess(abs(r["pos_deg"] - i * 10.0), 360.0 / 3200 / 2 + 1e-6)
        self.assertEqual(board.pos_steps, 3200)
        self.assertEqual(tt.position(), 360.0)

    def test_gear_ratio_and_setpos(self):
        tt, board = self.make(steps_per_rev=200, microsteps=8, gear_ratio=3.0)
        tt.connect()
        self.assertEqual(board.spr, 4800.0)
        tt.set_position(30.0)
        self.assertEqual(board.pos_steps, 400)
        tt.goto(0.0)
        self.assertEqual(board.pos_steps, 0)

    def test_board_error_raises(self):
        tt, board = self.make()
        tt.connect()
        with self.assertRaises(TurntableError):
            tt._cmd("BOGUS")

    def test_no_reply_times_out(self):
        tt, board = self.make()
        tt.connect()
        board.write = lambda data: None          # board went silent
        with self.assertRaises(TurntableError):
            tt._cmd("POS", timeout=0.2)

    def test_close_releases_when_not_holding(self):
        tt, board = self.make(hold=False)
        tt.connect()
        self.assertFalse(board.enabled)
        tt.close()
        self.assertEqual(board.log[-1], "EN 0")


class PlanKeys(unittest.TestCase):
    def test_defaults_and_validation(self):
        p = Plan(turntable_drive="serial", turntable_gear_ratio=3.0)
        self.assertEqual(p.turntable_port, "auto")
        self.assertEqual(p.turntable_microsteps, 16)
        import tempfile, yaml
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
            yaml.safe_dump({"turntable_drive": "usb"}, f)
        with self.assertRaises(ValueError):
            Plan.load(f.name)
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
            yaml.safe_dump({"turntable_drive": "serial", "turntable_direction": 2}, f)
        with self.assertRaises(ValueError):
            Plan.load(f.name)


class PortentaDriverTest(unittest.TestCase):
    """PortentaTurntable against the in-process emulation of the user's stepper firmware protocol."""

    def make(self, **kw):
        board = FakePortentaTransport(**{k: v for k, v in kw.items() if k in ("steps_per_rev", "microsteps", "move_time")})
        tt = PortentaTurntable(transport=board, log=lambda *a: None,
                               **{k: v for k, v in kw.items() if k not in ("steps_per_rev", "microsteps", "move_time")})
        return tt, board

    def test_connect_reads_info_and_sets_speed(self):
        tt, board = self.make(speed_deg_s=18.0, accel_deg_s2=36.0)
        info = tt.connect()
        self.assertEqual(info["microsteps_per_rev"], 3200.0); self.assertEqual(info["version"], "1.0")
        self.assertEqual(board.log[:2], ["PING", "INFO"])
        self.assertIn("SPEED 160", board.log)                 # 18 deg/s * 3200 / 360
        self.assertIn("ACCEL 320", board.log)
        self.assertIn("ENABLE 1", board.log)                  # hold

    def test_absolute_moves_36_steps_no_accumulation(self):
        tt, board = self.make()
        tt.connect(); tt.set_position(0.0)
        self.assertIn("ZERO 0", board.log)
        for s in range(1, 37):
            r = tt.goto(s * 10.0)
            self.assertLess(abs(r["pos_deg"] - s * 10.0), 0.1125 / 2 + 1e-9)     # < half a microstep, always
        self.assertEqual(board.pos, 3200)                                          # one full revolution, exactly
        self.assertEqual([l for l in board.log if l.startswith("MOVETO")][-1], "MOVETO 3200")
        self.assertNotIn("MOVE ", " ".join(board.log))                             # never relative

    def test_direction_and_gear(self):
        tt, board = self.make(direction=-1, gear_ratio=2.0)
        tt.connect()
        r = tt.goto(90.0)                                     # 90 platter deg = 180 motor deg = 1600 microsteps, negative
        self.assertEqual(board.pos, -1600); self.assertAlmostEqual(r["pos_deg"], 90.0)
        tt.set_position(45.0); self.assertIn("ZERO -800", board.log)

    def test_board_values_win_over_plan(self):
        tt, board = self.make(steps_per_rev=200, microsteps=8)   # board at 1/8, plan default 16
        tt.connect()
        self.assertEqual(tt.microsteps_per_rev, 1600.0)
        tt.goto(90.0); self.assertEqual(board.pos, 400)

    def test_errors(self):
        tt, board = self.make()
        tt.connect()
        board.write(b"")                                      # nothing
        board.replies.append("ERR BUSY")                      # next command sees a rejection
        with self.assertRaises(TurntableError):
            tt.set_position(0.0)

    def test_done_arriving_early_is_kept(self):
        tt, board = self.make()
        tt.connect(); tt.set_position(0.0)
        r = tt.goto(10.0)                                     # fake replies OK then DONE immediately
        self.assertAlmostEqual(r["pos_deg"], 10.0 - (10.0 - 89 * 0.1125), places=6)   # 89 microsteps = 10.0125 deg

    def test_make_turntable_portenta(self):
        p = Plan(turntable_drive="portenta")
        tt = make_turntable(p, transport=FakePortentaTransport(), log=lambda *a: None)
        self.assertIsInstance(tt, PortentaTurntable); self.assertEqual(tt.drive, "portenta")
        with self.assertRaises(ValueError):
            make_turntable(Plan(turntable_drive="nope"))


if __name__ == "__main__":
    unittest.main()
