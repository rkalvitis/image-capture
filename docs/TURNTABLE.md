# Motorised insect turntable: coordinator ↔ Portenta stepper controller

The motor is driven by the user's controller repo
`~/Documents/repos/phd_roberts_thesis_stepper_motor_control` (Arduino Portenta H7 → A4988 →
NEMA 17 17HS4401; PlatformIO firmware, `host/stepper.py`, a PC simulator). That repo is
not finished (2026-09-30); this page is the ground work on the coordinator side and what
of that repo the coordinator relies on.

## What the coordinator relies on (the repo's serial protocol, README §5)

Plain text over USB serial, 115200 8N1, one command per line, one `OK …`/`ERR …` reply per
command, plus one asynchronous `DONE <pos>` when a motion ends. Positions are **microsteps
of the motor shaft**, kept in the board's RAM (0 after reset, no homing).

| coordinator call (`turntable.py`) | wire |
|---|---|
| `connect()` | `PING`, `INFO` (steps_per_rev, microsteps — the board's values win over the plan), `SPEED`, `ACCEL`, `STATUS`, `ENABLE 1` (hold) |
| `set_position(deg)` — the operator placed the insect | `ZERO <steps>` |
| `goto(deg)` — every rotation, ABSOLUTE platter angle | `MOVETO <steps>` → wait `DONE` → `STATUS` must say `moving=0 pos=<steps>` |
| `position()` | `POS` |
| `close()` | `ENABLE 0` only when `turntable_hold: false` |

Never `MOVE` (relative): the 360/N split is rounded once per absolute target, so the
error stays below half a microstep (0.056° at 1/16, direct drive) at every stop and the
reverse `goto(0)` at the end of a pose (loop_order rotations-per-pose) lands exactly on
the origin. Direction and gear ratio live in the plan (`turntable_direction`,
`turntable_gear_ratio`), because the board only knows the motor shaft.

Changes in that repo that would break this: renamed commands, positions in another unit,
a second reply line per command, or `DONE` sent before `OK`. Everything else (speed
limits, pins, microstepping constant, acceleration profile) is invisible here.

## Every revolution is undone backwards (user, 2026-09-30)

The insect hangs on a thread: a full forward turn changes its height, so the platter never
wraps forward. `rotations-per-pose`: after the N rotations at a pose, `goto(0)` reverses
before the arm moves on. `poses-per-rotation`: after the last rotation of the session the
coordinator sends `goto(0)` (reverse, −(N−1)·360/N degrees) before it stops; with the manual
platter it asks once for Enter instead. Event `turntable_return` (pose = k or "session").
The rotation angle itself is set once, in the plan or at the start prompt (`turntable_steps`
N → 360/N exactly), and every stop is the absolute target `s · 360/N`.

## Plan keys

    turntable_drive: portenta        # manual | portenta | serial (old sketch) | fake
    turntable_port: auto             # auto = the single Arduino USB device (VID 0x2341) or /dev/cu.usbmodem*, /dev/ttyACM*
    turntable_steps_per_rev: 200     # checked against INFO
    turntable_microsteps: 16         # checked against INFO (the firmware's MICROSTEPS constant)
    turntable_gear_ratio: 1.0        # motor revolutions per PLATTER revolution — NOT DECIDED (coupling)
    turntable_direction: 1           # -1 flips the sense — NOT DECIDED (mocap yaw sign is learned at the first move)
    turntable_speed_deg_s: 20        # platter deg/s -> SPEED in microsteps/s
    turntable_accel_deg_s2: 40       # -> ACCEL
    turntable_hold: true             # coils energised between moves (the platter cannot be nudged)
    turntable_steps: 36              # rotations per revolution; the start prompt asks and sets step = 360/N exactly
    turntable_confirm: enter|mocap|both   # with the motor: enter = no wait, mocap/both = wait (bounded) for the popillia yaw
                                          # after each move and record it — never a prompt (user, 2026-09-30)

## Tests

- `tests/test_turntable.py` `PortentaDriverTest`: the driver against `FakePortentaTransport`
  (an in-process copy of the protocol), including 36 × 10° without accumulation.
- `tests/test_dry_run.py` `test_portenta_turntable_full_loop`: the whole coordinator loop.
- End-to-end against the **real firmware** in that repo's simulator (verified 2026-09-30:
  ZERO, 10/20/30/90/180/360°, reverse to 0):

      ~/.local/bin/platformio run -e native -t exec      # in <repo>/firmware, prints SIM_PORT /dev/ttysNNN
      python -m capture_coordinator.turntable --port /dev/ttysNNN cycle 36 10 0.2

  PlatformIO was installed on the Mac with `pipx install platformio` (the `pio` on PATH is
  an unrelated broken package: use `~/.local/bin/platformio`).

## Verified on the real board (2026-09-30, branch motor-and-wiring-update)

Portenta on `/dev/cu.usbmodem2101` (USB VID 0x2341 "Envie M7"), firmware 1.0, 200 × 16
microsteps. The branch changed only the wiring (ENABLE D13, MS1-3 D12-D10, STEP D7, DIR D6,
RESET+SLEEP to 3V3) and the motor (StepperOnline 17HS19-2004S1, 2 A, 59 N·cm); the protocol
is unchanged. Tests run from the Mac with the motor turning (user: "yes it moved"):
10°/20°/0° moves; the whole coordinator loop with the real motor (fake phone + robot,
synthetic mocap: ZERO, MOVETO 10/20/30°, yaw gate, reverse to 0 at the end); 36 × 10° in
44 s landing on exactly 3200 microsteps (worst rounding 0.05°, no accumulation), reverse
360° in 18 s at 20 °/s.

## How it rotates and moves (decisions, 2026-09-30)

- **Absolute only.** Every stop is `MOVETO round(s · 360/N · 3200/360)`; the step angle is
  360/N from the plan / start prompt. N = 36 is not an integer number of microsteps (88.9),
  the rounding stays below half a microstep at every stop and is 0 at 90/180/270/360°.
- **Forward through the revolution, then backwards to 0.** Never a forward wrap: the
  insect hangs on a thread. `rotations-per-pose`: reverse after each pose's N rotations.
  `poses-per-rotation`: reverse once after the last rotation of the session.
- **The next rotation starts as soon as the last stack of a rotation is shot** (user,
  2026-09-30): in `poses-per-rotation` the motor move for step s+1 runs in a background
  thread right after the last stack of step s, while the arm returns to its initial pose and
  the next robot run starts; `_wait_turntable(s+1)` then only joins it (event
  `turntable_early`). In the first Mac run that gap was 273 s with the move inside it.
- **Positive = the platter's yaw increases** (counter-clockwise seen from above with the
  Z-up world frame). `turntable_direction: -1` flips it; the coordinator learns the sign of
  the popillia yaw per platter degree at the first move and the yaw gate (`turntable_confirm:
  mocap|both`) checks every move against it.
- **Speed 20 °/s, acceleration 40 °/s²** (`SPEED 178`, `ACCEL 356` microsteps): a 10° step
  takes 0.9 s, 180° 9.5 s, the 360° reverse 18 s — gentle for the specimen; the 10 s
  `settle_after_move_s` runs after the move ends.
- **Holding torque on between moves** (`turntable_hold: true`, `ENABLE 1`): the platter
  cannot be nudged; the A4988 then runs warm at the set current (heatsink + airflow, README
  §2 of the motor repo). `false` releases the coils at `close()` only.
- **Zero = where the platter is when the robot reaches its initial pose** (`ZERO 0`, no prompt,
  user 2026-09-30: the insect is placed before the run starts); nothing is homed. After a
  board reset the position is 0 again: `--resume --start-step N` declares the platter at stop N.
- **Gear ratio 1, direct drive** until the coupling says otherwise (`turntable_gear_ratio`).

## First real run (mactest_2026-09-30)

Commanded +180°: popillia yaw changed −140.7°; after the reverse to 0 the yaw was 17.2°
instead of the 27.1° at the start. Direction inverted, ratio ~0.78, ~10° lost over the
cycle: the coupling slipped (user: fixed with hot glue). After the fix the `turntable` /
`turntable_return` events must show ±180° per commanded 180° and the start yaw again after
the reverse; then set `turntable_direction` to the measured sign.

## Still open (hardware)

Coupling / gear ratio, direction, the microstepping the firmware is flashed with, the
current limit, and where the platter's zero is: the coordinator takes ZERO when the
operator has placed the insect and pressed Enter, nothing is homed. `hardware/turntable/`
(the older Arduino sketch, `turntable_drive: serial`) stays as a fallback only.
